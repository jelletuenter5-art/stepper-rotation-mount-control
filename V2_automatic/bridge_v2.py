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
    """Send a move command and update _motor_pos. Caller must NOT hold _serial_lock."""
    global _motor_pos
    timeout = max(10, int(steps) * 0.002)
    cmd = f"{direction} {steps}\n".encode()
    stop_prefixes = ["Done.", "Stopped.", "ERR:", "AT_HOME", "AT_CW_LIMIT", "POS:"]
    with _serial_lock:
        ser.write(b"\n")
        time.sleep(0.05)
        ser.reset_input_buffer()
    responses = _cmd_multi(
        cmd,
        stop_prefixes=stop_prefixes,
        timeout=timeout
    )
    pos_line = next((l for l in responses if l.startswith("POS:")), None)
    if pos_line:
        try:
            _motor_pos = int(pos_line.split(":")[1])
        except ValueError:
            pass
    else:
        delta = steps if direction == "CW" else -steps
        _motor_pos += delta
    return responses


# ─── Auto-tracking state ──────────────────────────────────────────────────────
_auto_lock          = threading.Lock()
_auto_running       = False
_auto_state         = "idle"      # idle | searching | tracking
_auto_intensity     = 0
_auto_peak_intensity= 0
_auto_direction     = "none"      # CW | CCW | none
_auto_steps_taken   = 0
_auto_thread        = None
_auto_stop_event    = threading.Event()

# Auto-tracker parameters (set by /auto_start)
_auto_dither_steps    = 50
_auto_track_steps     = 20
_auto_drop_threshold  = 0.30
_auto_poll_interval   = 0.200     # seconds


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
    return parts["sum"], parts.get("peak", 0), parts.get("pixel", 0)


def _auto_tracker_thread():
    global _auto_running, _auto_state, _auto_intensity, _auto_peak_intensity
    global _auto_direction, _auto_steps_taken

    try:
        # Initial read to establish baseline
        try:
            intensity_sum, _, _ = _read_ccd_intensity()
        except Exception as e:
            with _auto_lock:
                _auto_state   = "idle"
                _auto_running = False
            return

        with _auto_lock:
            _auto_peak_intensity = intensity_sum
            _auto_intensity      = intensity_sum
            _auto_state          = "tracking"
            _auto_steps_taken    = 0
            _auto_direction      = "none"

        while not _auto_stop_event.is_set():
            time.sleep(_auto_poll_interval)
            if _auto_stop_event.is_set():
                break

            # Read current intensity
            try:
                current_sum, _, _ = _read_ccd_intensity()
            except Exception:
                continue

            with _auto_lock:
                _auto_intensity = current_sum
                peak = _auto_peak_intensity
                drop_thresh = _auto_drop_threshold
                dither = _auto_dither_steps
                track  = _auto_track_steps

            if current_sum < peak * (1.0 - drop_thresh):
                # ── Intensity dropped — search for new peak ───────────────
                with _auto_lock:
                    _auto_state      = "searching"
                    _auto_steps_taken = 0

                # Gradient probe: try CW dither
                try:
                    _move_steps("CW", dither)
                    i_cw_sum, _, _ = _read_ccd_intensity()
                    _move_steps("CCW", dither)   # back to start
                    i_start_sum, _, _ = _read_ccd_intensity()
                except Exception:
                    continue

                if i_cw_sum >= i_start_sum:
                    search_dir = "CW"
                    opp_dir    = "CCW"
                else:
                    search_dir = "CCW"
                    opp_dir    = "CW"

                with _auto_lock:
                    _auto_direction = search_dir

                current_best = i_start_sum

                # Walk in search_dir until intensity peaks
                while not _auto_stop_event.is_set():
                    try:
                        _move_steps(search_dir, track)
                        new_i_sum, _, _ = _read_ccd_intensity()
                    except Exception:
                        break

                    with _auto_lock:
                        _auto_intensity    = new_i_sum
                        _auto_steps_taken += track

                    if new_i_sum < current_best:
                        # Overshot — step back half
                        try:
                            _move_steps(opp_dir, track // 2)
                        except Exception:
                            pass
                        break
                    current_best = new_i_sum

                # Re-read after settling
                try:
                    settled_sum, _, _ = _read_ccd_intensity()
                except Exception:
                    settled_sum = current_best

                with _auto_lock:
                    _auto_peak_intensity = settled_sum
                    _auto_intensity      = settled_sum
                    _auto_state          = "tracking"
                    _auto_direction      = "none"

            else:
                # Intensity OK — update peak if improved
                with _auto_lock:
                    if current_sum > _auto_peak_intensity:
                        _auto_peak_intensity = current_sum
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


# ─── CCD routes ───────────────────────────────────────────────────────────────

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
            return jsonify({"ok": False, "error": "No CCD data received"})

        pixels = [int(v) for v in data_line.split(",") if v.strip()]
        return jsonify({"ok": True, "pixels": pixels})
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
        return jsonify({
            "ok":            True,
            "running":       _auto_running,
            "state":         _auto_state,
            "intensity":     _auto_intensity,
            "peak_intensity":_auto_peak_intensity,
            "motor_pos":     _motor_pos,
            "direction":     _auto_direction,
            "steps_taken":   _auto_steps_taken,
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
    app.run(port=5001)
