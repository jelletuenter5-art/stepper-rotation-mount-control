# Stepper Rotation Mount Control

Control software for a stepper-motor-driven rotation mount used in an
optical (spectroscopy) setup. The system lets you home the mount against a
limit switch, move it by angle or steps, calibrate a wavelength-to-position
lookup table, and run automated wavelength scans driven by an external
trigger pulse (e.g. from a lab PSU/monochromator).

## Components

- **`stepper_controller.ino`** — Arduino firmware for the motor driver.
  Handles homing, CW/CCW moves with travel limits, microstepping
  configuration, motor speed, and a "scan" mode that advances the mount by a
  fixed number of steps on each external trigger pulse.
- **`bridge.py`** — Flask server that bridges the web UI to the Arduino over
  USB serial (HTTP ↔ serial command translation).
- **`index.html`** — Browser-based control panel: connection management,
  position/homing display, move controls, wavelength calibration table, and
  the continuous step-test (scan) tab.

## Hardware / pin map

| Arduino pin | Function |
|---|---|
| D2 | DIR (stepper driver) |
| D3 | STEP (stepper driver) |
| D4 | M0 (microstepping) |
| D5 | M1 (microstepping) |
| D6 | M2 (microstepping) |
| D7 | Limit switch (NC: D7 → COM → GND, idles LOW, HIGH when triggered) |
| D12 | Trigger input for scan mode (falling edge = move one scan step) |
| D13 | Built-in LED (power-on indicator only) |

Travel is hard-limited in firmware to **+35°** (102910 steps) from the home
position, which is found via a limit switch.

## Running it

1. Flash `stepper_controller.ino` to the Arduino with the Arduino IDE.
2. Install Python dependencies and start the bridge server:
   ```bash
   pip install flask flask-cors pyserial
   python bridge.py
   ```
   This starts the API on `http://localhost:5000`.
3. Open `index.html` in a browser. It talks to the bridge at
   `http://localhost:5000`, select the Arduino's serial port, and click
   **Connect**.

## Typical workflow

1. **Seek Home** — runs the homing sequence (fast sweep → backoff → slow
   precision creep) to establish the zero position.
2. Move to known wavelength positions and record them in the **Record /
   Calibrate** tab to build a wavelength-to-steps calibration table.
3. Use **Go to λ** to move directly to an interpolated wavelength, or use
   the **Continuous Step Test** tab to arm a triggered scan that steps
   through wavelengths on each external trigger pulse.
