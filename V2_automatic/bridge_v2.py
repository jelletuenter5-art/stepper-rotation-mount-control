import time
import threading
import serial
import serial.tools.list_ports
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

ser = None

# ─── Serial port lock ─────────────────────────────────────────────────────────
# Every route that writes a command and reads a response MUST hold _serial_lock.
# The background reader also acquires it before each readline so it never
# interleaves with a synchronous command.
_serial_lock = threading.Lock()

# ─── Motor position (tracked by bridge) ───────────────────────────────────────
_motor_pos = 0  # updated after every move and after home_seek

# ─── Background serial reader ─────────────────────────────────────────────────
_trig_lock     = threading.Lock()
_trig_position = None
_trig_seq      = 0
_reader_thread = None
_reader_stop   = threading.Event()


def _serial_reader():
    global _trig_position, _trig_seq
    while not _reader_stop.is_set():
        if ser is None:
            time.sleep(0.05)
            continue
        with _serial_lock:
            try:
                line = ser.readline().decode(errors="replace").strip()
            except Exception:
                time.sleep(0.05)
                continue
        if not line:
            continue
        if line.startswith("TRIG_STEP:"):
            try:
                pos = int(line.split(":")[1])
                with _trig_lock:
                    _trig_position = pos
                    _trig_seq     += 1
            except ValueError:
                pass
        # All other unsolicited lines are silently discarded.


def _start_reader():
    global _reader_thread, _reader_stop
    _reader_stop.clear()
    _reader_thread = threading.Thread(target=_serial_reader, daemon=True)
    _reader_thread.start()


def _stop_reader():
    _reader_stop.set()
    if _reader_thread:
        _reader_thread.join(timeout=1)


# ─── Helper: exclusive command/response exchange ──────────────────────────────
def _cmd(command_bytes, timeout=2):
    """Send command_bytes, return the first non-empty response line."""
    with _serial_lock:
        old_timeout = ser.timeout
        ser.timeout = timeout
        try:
            ser.write(b"\n")
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(command_bytes)
            line = ser.readline().decode(errors="replace").strip()
            return line
        finally:
            ser.timeout = old_timeout


def _cmd_multi(command_bytes, stop_prefixes, timeout=2, max_lines=20):
    """Send command_bytes, collect lines until a stop_prefix is seen or blank line."""
    with _serial_lock:
        old_timeout = ser.timeout
        ser.timeout = timeout
        try:
            ser.write(b"\n")
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(command_bytes)
            responses = []
            for _ in range(max_lines):
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    break
                responses.append(line)
                if any(line.startswith(p) for p in stop_prefixes):
                    # Try to grab one trailing line (e.g. POS: after Done.)
                    extra = ser.readline().decode(errors="replace").strip()
                    if extra:
                        responses.append(extra)
                    break
            return responses
        finally:
            ser.timeout = old_timeout


# ─── Helper: move steps via serial (used by auto-tracker) ────────────────────
def _move_steps(direction, steps):
    """Send a move command, update _motor_pos and cumulative step counters."""
    global _motor_pos, _auto_steps_taken, _auto_steps_cw, _auto_steps_ccw
    timeout = max(10, int(steps) * 0.002)
    cmd = f"{direction} {steps}\n".encode()
    stop_prefixes = ["Done.", "Stopped.", "ERR:", "AT_HOME", "AT_CW_LIMIT", "POS:"]
    with _serial_lock:
        ser.write(b"\n")
        time.sleep(0.05)
        ser.reset_input_buffer()
    responses = _cmd_multi(cmd, stop_prefixes=stop_prefixes, timeout=timeout)
    pos_line = next((l for l in responses if l.startswith("POS:")), None)
    if pos_line:
        try:
            _motor_pos = int(pos_line.split(":")[1])
        except ValueError:
            pass
    else:
        delta = steps if direction == "CW" else -steps
        _motor_pos += delta
    # Accumulate step counts for the UI
    with _auto_lock:
        _auto_steps_taken += steps
        if direction == "CW":
            _auto_steps_cw  += steps
        else:
            _auto_steps_ccw += steps
    return responses


# ─── Auto-tracking state ──────────────────────────────────────────────────────
_auto_lock          = threading.Lock()
_auto_running       = False
_auto_state         = "idle"      # idle | searching | tracking
_auto_intensity     = 0
_auto_peak_intensity= 0           # current tracking baseline (resets per search)
_auto_best_ever     = 0           # all-time highest sum seen (never decreases)
_auto_direction     = "none"      # CW | CCW | none
_auto_steps_taken   = 0           # cumulative total steps (never resets)
_auto_steps_cw      = 0           # cumulative CW steps
_auto_steps_ccw     = 0           # cumulative CCW steps
_auto_last_pixels   = []          # latest pixel frame for canvas (set by tracker)
_auto_thread        = None
_auto_stop_event    = threading.Event()

# Auto-tracker parameters (set by /auto_start)
_auto_dither_steps    = 50
_auto_track_steps     = 20
_auto_drop_threshold  = 0.05
_auto_poll_interval   = 0.100     # seconds


def _read_ccd_intensity():
    """Read CCD intensity via serial. Returns (sum, peak, pixel) or raises."""
    line = _cmd(b"CCD_INTENSITY\n", timeout=5)
    # Expected: INTENSITY sum=NNNNN peak=NNN pixel=MMMM
    parts = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                parts[k] = int(v)
            except ValueError:
                pass
    if "sum" not in parts:
        raise RuntimeError(f"Unexpected CCD_INTENSITY response: {line!r}")
    corrected_sum = max(0, parts["sum"] - _dark_sum)
    return corrected_sum, parts.get("peak", 0), parts.get("pixel", 0)


def _read_intensity_avg(n=2):
    """Average n CCD intensity readings to smooth out sensor noise."""
    total = 0
    for _ in range(n):
        s, _, _ = _read_ccd_intensity()
        total += s
    return total // n


def _update_best(val):
    """Update _auto_intensity and _auto_best_ever under lock."""
    global _auto_best_ever
    with _auto_lock:
        _auto_intensity = val
        if val > _auto_best_ever:
            _auto_best_ever = val


def _wide_scan(dither, n_scan=6):
    """
    Phase 1 — Wide scan.
    Step dither-sized steps in CW direction (n_scan times), then come back and
    scan CCW (n_scan times), recording intensity at every position.
    Navigate to the position with the highest intensity reading.
    Returns (best_intensity, best_motor_pos) or (None, None) on error.
    """
    global _auto_direction
    positions = []

    # Baseline at current position
    try:
        val = _read_intensity_avg()
        positions.append((_motor_pos, val))
        _update_best(val)
    except Exception:
        return None, None

    start_pos = _motor_pos

    # Scan CW
    with _auto_lock:
        _auto_direction = "CW"
    for _ in range(n_scan):
        if _auto_stop_event.is_set():
            break
        try:
            _move_steps("CW", dither)
            time.sleep(0.04)
            val = _read_intensity_avg()
            positions.append((_motor_pos, val))
            _update_best(val)
        except Exception:
            break

    # Return to start
    if _motor_pos != start_pos:
        try:
            _move_steps("CCW", abs(_motor_pos - start_pos))
        except Exception:
            pass

    # Scan CCW
    with _auto_lock:
        _auto_direction = "CCW"
    for _ in range(n_scan):
        if _auto_stop_event.is_set():
            break
        try:
            _move_steps("CCW", dither)
            time.sleep(0.04)
            val = _read_intensity_avg()
            positions.append((_motor_pos, val))
            _update_best(val)
        except Exception:
            break

    # Return to start
    if _motor_pos != start_pos:
        try:
            _move_steps("CW", abs(_motor_pos - start_pos))
        except Exception:
            pass

    if not positions:
        return None, None

    best_pos, best_val = max(positions, key=lambda x: x[1])
    return best_val, best_pos


def _fine_tune(track):
    """
    Phase 2 — Fine-tune.
    Starting from the position found by wide scan, take small track-sized steps.
    Stop IMMEDIATELY when a step does not improve intensity (at or past peak).
    Returns best intensity found, or None on error.
    """
    global _auto_direction
    try:
        current_best = _read_intensity_avg()
        _update_best(current_best)
    except Exception:
        return None

    # Probe CW vs CCW with one track step each to choose direction
    try:
        _move_steps("CW", track)
        time.sleep(0.04)
        i_cw = _read_intensity_avg()
        _move_steps("CCW", track)   # back to start
        time.sleep(0.04)
        i_back = _read_intensity_avg()
    except Exception:
        return current_best

    if i_cw > current_best:
        search_dir, opp_dir = "CW", "CCW"
        current_best = i_cw
    elif i_back > current_best:
        # Coming back was better — CCW direction
        search_dir, opp_dir = "CCW", "CW"
        current_best = i_back
    else:
        # Already at peak — neither direction improved
        _update_best(current_best)
        return current_best

    with _auto_lock:
        _auto_direction = search_dir
    _update_best(current_best)

    while not _auto_stop_event.is_set():
        try:
            _move_steps(search_dir, track)
            time.sleep(0.04)
            new_i = _read_intensity_avg()
        except Exception:
            break
        _update_best(new_i)
        if new_i <= current_best:
            # Not improving — step back and stop (we just passed the peak)
            try:
                _move_steps(opp_dir, track)
            except Exception:
                pass
            break
        current_best = new_i

    return current_best


def _find_peak(dither, track, n_scan=6):
    """
    Full two-phase peak search:
      1. Wide scan (dither-step grid, n_scan positions each direction) → global max
      2. Navigate to global max position
      3. Fine-tune around it with track-sized steps, stop when no improvement
    Returns best intensity, or None on error.
    """
    global _auto_direction

    # Phase 1: wide scan
    best_val, best_pos = _wide_scan(dither, n_scan)
    if best_val is None:
        return None

    # Navigate to the best position found
    if _motor_pos != best_pos:
        diff = best_pos - _motor_pos
        with _auto_lock:
            _auto_direction = "CW" if diff > 0 else "CCW"
        try:
            if diff > 0:
                _move_steps("CW", abs(diff))
            else:
                _move_steps("CCW", abs(diff))
        except Exception:
            pass

    with _auto_lock:
        _auto_direction = "none"

    # Phase 2: fine-tune
    settled = _fine_tune(track)
    return settled if settled is not None else best_val


def _auto_tracker_thread():
    global _auto_running, _auto_state, _auto_intensity, _auto_peak_intensity
    global _auto_best_ever, _auto_direction, _auto_steps_taken
    global _auto_steps_cw, _auto_steps_ccw

    try:
        # ── Initial read ──────────────────────────────────────────────────
        try:
            intensity_sum, _, _ = _read_ccd_intensity()
        except Exception:
            with _auto_lock:
                _auto_state   = "idle"
                _auto_running = False
            return

        with _auto_lock:
            _auto_intensity      = intensity_sum
            _auto_peak_intensity = intensity_sum
            _auto_best_ever      = intensity_sum
            _auto_state          = "searching"
            _auto_steps_taken    = 0
            _auto_steps_cw       = 0
            _auto_steps_ccw      = 0
            _auto_direction      = "none"
            dither = _auto_dither_steps
            track  = _auto_track_steps

        # ── Phase 1+2: wide scan then fine-tune ───────────────────────────
        settled = _find_peak(dither, track)
        if settled is not None:
            with _auto_lock:
                _auto_peak_intensity = settled
                _auto_intensity      = settled
                if settled > _auto_best_ever:
                    _auto_best_ever = settled

        with _auto_lock:
            _auto_state     = "tracking"
            _auto_direction = "none"

        # ── Main tracking loop ────────────────────────────────────────────
        _pixel_tick = 0
        while not _auto_stop_event.is_set():
            time.sleep(_auto_poll_interval)
            if _auto_stop_event.is_set():
                break

            try:
                current_sum, _, _ = _read_ccd_intensity()
            except Exception:
                continue

            # Every 4th poll also grab full pixel frame for the canvas
            _pixel_tick += 1
            if _pixel_tick >= 4:
                _pixel_tick = 0
                try:
                    raw = _read_ccd_pixels_raw()
                    with _auto_lock:
                        global _auto_last_pixels
                        _auto_last_pixels = _apply_dark(raw)
                except Exception:
                    pass

            with _auto_lock:
                _auto_intensity = current_sum
                peak        = _auto_peak_intensity
                drop_thresh = _auto_drop_threshold
                dither      = _auto_dither_steps
                track       = _auto_track_steps

            if current_sum < peak * (1.0 - drop_thresh):
                # ── Intensity dropped — search for new peak ───────────────
                with _auto_lock:
                    _auto_state = "searching"

                settled = _find_peak(dither, track)
                if settled is None:
                    continue

                with _auto_lock:
                    _auto_peak_intensity = settled
                    _auto_intensity      = settled
                    if settled > _auto_best_ever:
                        _auto_best_ever  = settled
                    _auto_state          = "tracking"
                    _auto_direction      = "none"

            else:
                # Intensity OK — update tracking baseline and best-ever if improved
                with _auto_lock:
                    if current_sum > _auto_peak_intensity:
                        _auto_peak_intensity = current_sum
                    if current_sum > _auto_best_ever:
                        _auto_best_ever = current_sum
                    _auto_state     = "tracking"
                    _auto_direction = "none"

    finally:
        with _auto_lock:
            _auto_running = False
            _auto_state   = "idle"


# ─── Routes ──────────────────────────────────────────────────────────────────

@app.route("/ports")
def ports():
    return jsonify([p.device for p in serial.tools.list_ports.comports()])


@app.route("/connect", methods=["POST"])
def connect():
    global ser
    port = request.json.get("port")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)
        ser.reset_input_buffer()
        _start_reader()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/disconnect", methods=["POST"])
def disconnect():
    global ser
    _stop_reader()
    if ser:
        ser.close()
        ser = None
    return jsonify({"ok": True})


@app.route("/move", methods=["POST"])
def move():
    global _motor_pos
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    direction = request.json.get("direction")
    steps     = request.json.get("steps")
    timeout   = max(10, int(steps) * 0.002)
    cmd       = f"{direction} {steps}\n".encode()
    stop_prefixes = ["Done.", "Stopped.", "ERR:", "AT_HOME", "AT_CW_LIMIT", "POS:"]

    _stop_reader()
    try:
        with _serial_lock:
            # flush any stale bytes before sending
            ser.write(b"\n")
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.write(cmd)
            old_timeout = ser.timeout
            ser.timeout = timeout
            responses   = []
            retried     = False
            try:
                for _ in range(20):
                    line = ser.readline().decode(errors="replace").strip()
                    if not line:
                        break
                    responses.append(line)
                    if any(line.startswith(p) for p in stop_prefixes):
                        if line.startswith("ERR:") and not retried:
                            retried = True
                            time.sleep(0.15)
                            ser.reset_input_buffer()
                            ser.write(cmd)
                            responses = []
                            continue
                        extra = ser.readline().decode(errors="replace").strip()
                        if extra:
                            responses.append(extra)
                        break
            finally:
                ser.timeout = old_timeout
    finally:
        _start_reader()

    # Update _motor_pos from POS: line if present
    pos_line = next((l for l in responses if l.startswith("POS:")), None)
    if pos_line:
        try:
            _motor_pos = int(pos_line.split(":")[1])
        except ValueError:
            pass
    else:
        at_home  = any(l == "AT_HOME" for l in responses)
        at_cwlim = any(l == "AT_CW_LIMIT" for l in responses)
        if at_home:
            _motor_pos = 0
        elif not at_cwlim:
            delta = int(steps) if direction == "CW" else -int(steps)
            _motor_pos += delta
    return jsonify({"ok": True, "log": responses})


@app.route("/stop", methods=["POST"])
def stop():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    responses = _cmd_multi(
        b"STOP\n",
        stop_prefixes=["Stopped.", "POS:"],
        timeout=3,
        max_lines=8
    )
    return jsonify({"ok": True, "log": responses})


@app.route("/home_seek", methods=["POST"])
def home_seek():
    global _motor_pos
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})

    _stop_reader()
    time.sleep(0.2)  # let any in-flight bytes settle

    old_timeout = ser.timeout
    ser.timeout = 2
    deadline = time.time() + 120

    try:
        with _serial_lock:
            # Send a blank line first to flush any partial/stale byte sitting in
            # the Arduino UART receiver (common at 115200 on clone boards).
            ser.write(b"\n")
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.write(b"HOME_SEEK\n")
            responses = []
            homed = False
            error = None
            retried = False

            while time.time() < deadline:
                try:
                    line = ser.readline().decode(errors="replace").strip()
                except Exception as e:
                    error = f"Serial read error: {e}"
                    break
                if not line:
                    continue
                responses.append(line)
                if line == "Homed.":
                    homed = True
                    pos_line = ser.readline().decode(errors="replace").strip()
                    if pos_line:
                        responses.append(pos_line)
                    break
                if line.startswith("Stopped."):
                    error = line
                    break
                if line.startswith("ERR:") and not retried:
                    # Garbled command — flush and retry once (common at 115200)
                    retried = True
                    time.sleep(0.15)
                    ser.reset_input_buffer()
                    ser.write(b"HOME_SEEK\n")
                    responses.append("(retrying HOME_SEEK after garbled response)")
                    continue
                if line.startswith("ERR:"):
                    error = line
                    break

            if not homed and not error:
                error = "Timeout waiting for homing (120 s)"

        if homed:
            _motor_pos = 0
            return jsonify({"ok": True,  "homed": True,  "log": responses})
        else:
            return jsonify({"ok": False, "homed": False, "error": error or "Homing did not complete", "log": responses})
    finally:
        ser.timeout = old_timeout
        _reader_stop.clear()
        _start_reader()


@app.route("/trig_poll", methods=["GET"])
def trig_poll():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})

    try:
        last_seq = int(request.args.get("last_seq", -1))
    except ValueError:
        last_seq = -1

    deadline = time.time() + 2.0
    while time.time() < deadline:
        with _trig_lock:
            if _trig_seq > last_seq:
                return jsonify({"ok": True, "changed": True, "seq": _trig_seq, "position": _trig_position})
        time.sleep(0.02)

    with _trig_lock:
        return jsonify({"ok": True, "changed": False, "seq": _trig_seq, "position": _trig_position})


@app.route("/set_speed", methods=["POST"])
def set_speed():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    delay_us = request.json.get("delay_us")
    if not delay_us or int(delay_us) < 50:
        return jsonify({"ok": False, "error": "delay_us must be >= 50"})
    response = _cmd(f"SET_SPEED {int(delay_us)}\n".encode())
    return jsonify({"ok": True, "log": [response]})


@app.route("/set_mstep", methods=["POST"])
def set_mstep():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    idx = request.json.get("index")
    if idx is None or not (0 <= int(idx) <= 4):
        return jsonify({"ok": False, "error": "index must be 0-4"})
    response = _cmd(f"SET_MSTEP {int(idx)}\n".encode())
    return jsonify({"ok": True, "log": [response]})


@app.route("/set_mpins", methods=["POST"])
def set_mpins():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    m0 = int(request.json.get("m0", 0))
    m1 = int(request.json.get("m1", 0))
    m2 = int(request.json.get("m2", 0))
    response = _cmd(f"SET_MPINS {m0} {m1} {m2}\n".encode())
    return jsonify({"ok": True, "log": [response]})


@app.route("/scan_arm", methods=["POST"])
def scan_arm():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})

    spt = request.json.get("steps_per_trigger")
    if spt is None or int(spt) <= 0:
        return jsonify({"ok": False, "error": "steps_per_trigger must be > 0"})

    reversed_flag = bool(request.json.get("reversed", False))
    max_triggers  = int(request.json.get("max_triggers", 0))

    cmd_str = f"SCAN_ARM {int(spt)}"
    if reversed_flag:
        cmd_str += " R"
    if max_triggers > 0:
        cmd_str += f" MAX:{max_triggers}"
    cmd_str += "\n"

    response = _cmd(cmd_str.encode())

    if response.startswith("SCAN_ARMED"):
        return jsonify({"ok": True, "armed": True, "steps_per_trigger": int(spt),
                        "reversed": reversed_flag, "max_triggers": max_triggers, "log": [response]})
    else:
        return jsonify({"ok": False, "error": f"Unexpected response: {response}"})


@app.route("/scan_disarm", methods=["POST"])
def scan_disarm():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    response = _cmd(b"SCAN_DISARM\n")
    return jsonify({"ok": True, "log": [response]})


@app.route("/scan_status", methods=["GET"])
def scan_status():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    response = _cmd(b"SCAN_STATUS\n")
    info = {}
    for part in response.split():
        if "=" in part:
            k, v = part.split("=", 1)
            info[k] = int(v) if v.lstrip("-").isdigit() else v
    return jsonify({"ok": True, "raw": response, **info})


# ─── CCD dark-frame calibration ───────────────────────────────────────────────
# Dark frame = ambient-light reading taken with laser blocked.
# Subtracted from every subsequent pixel read so only the laser signal remains.
_dark_frame = []   # list of N calibrated dark pixel values (N = CCD pixels / CCD_STRIDE)
_dark_sum   = 0    # sum(_dark_frame) * CCD_STRIDE — used to correct CCD_INTENSITY fast path


def _read_ccd_pixels_raw():
    """Read one full CCD frame (subsampled). Returns list of raw pixel values, or raises."""
    with _serial_lock:
        old_timeout = ser.timeout
        ser.timeout = 10
        try:
            ser.write(b"\n")
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(b"CCD_READ\n")
            data_line = ""
            for _ in range(50):
                line = ser.readline().decode(errors="replace").strip()
                if line.startswith("CCD_DATA:"):
                    data_line = line[len("CCD_DATA:"):]
                elif line == "CCD_DONE":
                    break
        finally:
            ser.timeout = old_timeout
    if not data_line:
        raise RuntimeError("No CCD data received")
    return [int(v) for v in data_line.split(",") if v.strip()]


def _apply_dark(pixels):
    """Subtract dark frame from pixel list, clip at 0. Returns corrected list."""
    if not _dark_frame or len(_dark_frame) != len(pixels):
        return pixels
    return [max(0, p - d) for p, d in zip(pixels, _dark_frame)]


# ─── CCD routes ───────────────────────────────────────────────────────────────

@app.route("/ccd_raw", methods=["GET"])
def ccd_raw():
    """Read raw A0 voltage without any CCD clocking — for wiring diagnostics."""
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    line = _cmd(b"CCD_RAW\n", timeout=2)
    if line.startswith("RAW_A0="):
        try:
            val = int(line.split("=")[1])
            return jsonify({"ok": True, "raw_adc": val, "voltage_mv": round(val * 5000 / 1023)})
        except ValueError:
            pass
    return jsonify({"ok": False, "error": f"Unexpected response: {line!r}"})


@app.route("/ccd_calibrate_dark", methods=["POST"])
def ccd_calibrate_dark():
    """Take N CCD readings with laser blocked and store as dark frame."""
    global _dark_frame, _dark_sum
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    N = 3
    readings = []
    try:
        for _ in range(N):
            readings.append(_read_ccd_pixels_raw())
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})
    length = min(len(r) for r in readings)
    _dark_frame = [sum(r[i] for r in readings) // N for i in range(length)]
    _dark_sum   = sum(_dark_frame) * 8  # CCD_STRIDE = 8
    return jsonify({"ok": True, "dark_sum": _dark_sum, "pixels": length,
                    "avg_per_pixel": round(_dark_sum / (length * 8)) if length else 0})


@app.route("/ccd_dark_reset", methods=["POST"])
def ccd_dark_reset():
    global _dark_frame, _dark_sum
    _dark_frame = []
    _dark_sum   = 0
    return jsonify({"ok": True})


@app.route("/ccd_dark_status", methods=["GET"])
def ccd_dark_status():
    return jsonify({"ok": True, "calibrated": bool(_dark_frame),
                    "dark_sum": _dark_sum,
                    "pixels": len(_dark_frame)})


@app.route("/ccd_intensity", methods=["GET"])
def ccd_intensity():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    try:
        s, peak, pixel = _read_ccd_intensity()
        return jsonify({"ok": True, "sum": s, "peak": peak, "pixel": pixel})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/ccd_read", methods=["GET"])
def ccd_read():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    try:
        raw = _read_ccd_pixels_raw()
        pixels = _apply_dark(raw)
        return jsonify({"ok": True, "pixels": pixels, "dark_calibrated": bool(_dark_frame)})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/ccd_int", methods=["POST"])
def ccd_int():
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    ms = request.json.get("ms", 5)
    try:
        ms = int(ms)
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "ms must be an integer"})
    response = _cmd(f"CCD_INT {ms}\n".encode())
    return jsonify({"ok": True, "response": response})


# ─── Auto-tracking routes ─────────────────────────────────────────────────────

@app.route("/auto_status", methods=["GET"])
def auto_status():
    with _auto_lock:
        pixels = _auto_last_pixels[:]  # snapshot; JS will consume and draw
    return jsonify({
        "ok":            True,
        "running":       _auto_running,
        "state":         _auto_state,
        "intensity":     _auto_intensity,
        "peak_intensity":_auto_peak_intensity,
        "best_ever":     _auto_best_ever,
        "motor_pos":     _motor_pos,
        "direction":     _auto_direction,
        "steps_taken":   _auto_steps_taken,
        "steps_cw":      _auto_steps_cw,
        "steps_ccw":     _auto_steps_ccw,
        "pixels":        pixels if pixels else None,
    })


@app.route("/auto_start", methods=["POST"])
def auto_start():
    global _auto_running, _auto_thread, _auto_state
    global _auto_dither_steps, _auto_track_steps, _auto_drop_threshold, _auto_poll_interval

    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})

    with _auto_lock:
        if _auto_running:
            return jsonify({"ok": False, "error": "Auto-tracker already running"})

    body = request.json or {}
    _auto_dither_steps   = int(body.get("dither_steps",    50))
    _auto_track_steps    = int(body.get("track_steps",     20))
    _auto_drop_threshold = float(body.get("drop_threshold", 0.3))
    _auto_poll_interval  = float(body.get("poll_interval_ms", 200)) / 1000.0

    _auto_stop_event.clear()
    with _auto_lock:
        _auto_running = True
        _auto_state   = "idle"

    _auto_thread = threading.Thread(target=_auto_tracker_thread, daemon=True)
    _auto_thread.start()

    return jsonify({"ok": True})


@app.route("/auto_stop", methods=["POST"])
def auto_stop():
    global _auto_running, _auto_state
    _auto_stop_event.set()
    if _auto_thread:
        _auto_thread.join(timeout=5)
    with _auto_lock:
        _auto_running = False
        _auto_state   = "idle"
    return jsonify({"ok": True})


if __name__ == "__main__":
    app.run(port=5001, threaded=True)
