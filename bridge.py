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
# interleaves with a synchronous command, preventing torn / garbled responses
# like "AARE sepsprr=41" instead of "SCAN_ARMED steps_per_trig=41".
_serial_lock = threading.Lock()

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
        # Acquire the port lock before reading; yields immediately to any
        # synchronous route that needs an exclusive command/response exchange.
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
# All synchronous routes use this so the background reader cannot interleave.
def _cmd(command_bytes, timeout=2):
    """Send command_bytes, return the first non-empty response line."""
    with _serial_lock:
        old_timeout = ser.timeout
        ser.timeout = timeout
        try:
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


# ─── Routes ──────────────────────────────────────────────────────────────────

@app.route("/ports")
def ports():
    return jsonify([p.device for p in serial.tools.list_ports.comports()])


@app.route("/connect", methods=["POST"])
def connect():
    global ser
    port = request.json.get("port")
    try:
        ser = serial.Serial(port, 9600, timeout=2)
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
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})
    direction = request.json.get("direction")
    steps     = request.json.get("steps")
    timeout   = max(10, int(steps) * 0.002)
    responses = _cmd_multi(
        f"{direction} {steps}\n".encode(),
        stop_prefixes=["Done.", "Stopped.", "ERR:", "AT_HOME", "AT_CW_LIMIT", "POS:"],
        timeout=timeout
    )
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
    if not ser:
        return jsonify({"ok": False, "error": "Not connected"})

    # Stop the background reader — we need exclusive serial access for up to
    # 120 s and cannot afford any line being stolen.
    _stop_reader()

    old_timeout = ser.timeout
    ser.timeout = 2
    deadline = time.time() + 120

    try:
        with _serial_lock:
            ser.reset_input_buffer()
            ser.write(b"HOME_SEEK\n")
            responses = []
            homed = False
            error = None

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
                if line.startswith("Stopped.") or line.startswith("ERR:"):
                    error = line
                    break

            if not homed and not error:
                error = "Timeout waiting for homing (120 s)"

        if homed:
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
        return jsonify({"ok": False, "error": "index must be 0–4"})
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


if __name__ == "__main__":
    app.run(port=5000)
