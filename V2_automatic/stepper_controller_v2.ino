// ═══════════════════════════════════════════════════
//  STEPPER MOTOR CONTROLLER — OPTICS BUILD  V2
//
//  ── Pin map ─────────────────────────────────────
//  D2  → DIR  (stepper driver)
//  D3  → STEP (stepper driver)
//  D4  → M0   (microstepping)
//  D5  → M1   (microstepping)
//  D6  → M2   (microstepping)
//  D7  → Limit switch (NC: D7→COM→GND, idles LOW, HIGH when triggered)
//  D9  → CCD_CLK  (TCD1304DG φM, bit-bang)
//  D10 → CCD_ICG  (TCD1304DG integration clear gate)
//  D11 → CCD_SH   (TCD1304DG shift gate)
//  A0  → CCD_OS   (TCD1304DG analog output)
//  D12 → Trigger input (FALLING edge = move one scan step)
//  D13 → Built-in LED (power-on indicator, no external wiring)
//
//  ── Trigger wiring ──────────────────────────────
//  Uses internal INPUT_PULLUP — no external resistor needed.
//  D12 idles HIGH. Fires on FALLING edge when pulled LOW.
//    PSU trigger wire (negative/pulse) → D12
//    PSU GND → any Arduino GND pin
//  The pin goes LOW when the pulse fires → triggers one scan step.
//
//  ── CCD (TCD1304DG) ─────────────────────────────
//  3648 pixels linear array. Bit-bang clock on D9.
//  Integration time set via CCD_INT command (default 5 ms).
// ═══════════════════════════════════════════════════

const int dirPin      = 2;
const int stepPin     = 3;
const int m0Pin       = 4;
const int m1Pin       = 5;
const int m2Pin       = 6;
const int limitPin    = 7;   // NC switch: idles LOW, HIGH when open/triggered
const int ccdClkPin   = 9;   // CCD φM clock (bit-bang)
const int ccdIcgPin   = 10;  // CCD ICG (integration clear gate)
const int ccdShPin    = 11;  // CCD SH  (shift gate)
const int ccdOsPin    = A0;  // CCD OS  (analog output)
const int triggerPin  = 12;  // 5 V rising-edge trigger. Needs 10 kΩ pull-down to GND.
const int powerLedPin = 13;  // Built-in LED — power indicator only, not wired externally

// ─── Microstep presets: {M0, M1, M2} ─────────────
const int mstepTable[5][3] = {
  {0, 0, 0},  // 0 = Full step  (200  steps/rev)
  {1, 0, 0},  // 1 = 1/2        (400  steps/rev)
  {0, 1, 0},  // 2 = 1/8        (1600 steps/rev)
  {1, 1, 0},  // 3 = 1/16       (3200 steps/rev)
  {1, 1, 1},  // 4 = 1/32       (6400 steps/rev)
};

int  currentMstepIdx = 3;        // default 1/16
long currentPosition = 0;        // absolute step position from home
bool stopFlag        = false;
int  stepDelay       = 500;      // µs half-period

// ─── Homing constants ─────────────────────────────
const int  HOME_FAST_DELAY  = 80;   // µs — fast CCW sweep
const int  HOME_LAST        = 750;  // µs — CW offset rotation (slow/precise)
const int  HOME_CREEP_DELAY = 750;  // µs — slow CCW precision creep
const long HOME_BACKOFF     = 12800;  // steps CW to clear switch before creep

// ─── Travel limit ─────────────────────────────────
const long MAX_STEPS = 102910;      // +35° from home

// ─── Scan / trigger state ─────────────────────────
bool scanArmed        = false;
long scanStepsPerTrig = 0;
bool scanReversed     = false;
int  scanTriggered    = 0;
int  scanMaxTriggers  = 0;

// Debounce / edge detection for trigger pin
bool          lastTrigState = false;
unsigned long lastTrigTime  = 0;
const unsigned long TRIG_DEBOUNCE_MS = 50;

// ─── CCD state ────────────────────────────────────
int integrationMs = 5;

// ─── Helper: apply microstep index ────────────────
void applyMstep(int idx) {
  digitalWrite(m0Pin, mstepTable[idx][0] ? HIGH : LOW);
  digitalWrite(m1Pin, mstepTable[idx][1] ? HIGH : LOW);
  digitalWrite(m2Pin, mstepTable[idx][2] ? HIGH : LOW);
}

// ─── Helper: move N steps in a direction ──────────
// Returns true if completed cleanly, false if stopped or limit hit.
bool doMove(bool cw, long steps) {
  stopFlag = false;
  digitalWrite(dirPin, cw ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    // Check for STOP command mid-move
    if (Serial.available() > 0) {
      String incoming = Serial.readStringUntil('\n');
      incoming.trim(); incoming.toUpperCase();
      if (incoming == F("STOP")) {
        stopFlag = true;
        Serial.println(F("Stopped."));
        return false;
      }
    }
    // CCW limit check
    if (!cw && digitalRead(limitPin) == HIGH) {
      stopFlag = true; currentPosition = 0;
      Serial.println(F("AT_HOME"));
      return false;
    }
    // CW limit check
    if (cw && currentPosition >= MAX_STEPS) {
      stopFlag = true; currentPosition = MAX_STEPS;
      Serial.println(F("AT_CW_LIMIT"));
      return false;
    }
    digitalWrite(stepPin, HIGH); delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);  delayMicroseconds(stepDelay);
    currentPosition += cw ? 1 : -1;
  }
  return true;
}

// ─── CCD: single clock pulse ──────────────────────
inline void ccdClock() {
  digitalWrite(ccdClkPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ccdClkPin, LOW);
  delayMicroseconds(5);
}

// ─── CCD: integration clear + start of readout ────
// Call before either ccdStreamPixels() or ccdPeakOnly().
void ccdStartReadout() {
  // Phase 1: Integration clear
  digitalWrite(ccdIcgPin, LOW);  delayMicroseconds(1);
  digitalWrite(ccdShPin,  HIGH);
  ccdClock(); ccdClock(); ccdClock(); ccdClock();
  digitalWrite(ccdShPin,  LOW);  delayMicroseconds(1);
  digitalWrite(ccdIcgPin, HIGH);
  delay(integrationMs);

  // Phase 2: Transfer charge to shift register
  digitalWrite(ccdIcgPin, LOW);  delayMicroseconds(1);
  digitalWrite(ccdShPin,  HIGH);
  ccdClock(); ccdClock(); ccdClock(); ccdClock();
  digitalWrite(ccdShPin,  LOW);  delayMicroseconds(1);
  digitalWrite(ccdIcgPin, HIGH);

  // 32 dummy clocks to flush preamble
  for (int i = 0; i < 32; i++) ccdClock();
}

// ─── CCD: stream pixels over Serial (subsampled) ──
// Clocks all 3648 pixels (required for correct CCD operation) but only
// transmits every CCD_STRIDE-th pixel to keep serial transfer fast at 9600.
// 3648 / 8 = 456 values → ~2.5 s at 9600 baud instead of 19 s.
#define CCD_STRIDE 8
void ccdStreamPixels() {
  ccdStartReadout();
  Serial.print(F("CCD_DATA:"));
  bool first = true;
  for (int i = 0; i < 3648; i++) {
    digitalWrite(ccdClkPin, HIGH); delayMicroseconds(5);
    uint16_t v = (uint16_t)analogRead(ccdOsPin);
    digitalWrite(ccdClkPin, LOW);  delayMicroseconds(5);
    if (i % CCD_STRIDE == 0) {
      if (!first) Serial.print(',');
      Serial.print(v);
      first = false;
    }
  }
  Serial.println();
  Serial.println(F("CCD_DONE"));
}

// ─── CCD: compute peak + sum only (no buffer) ─────
// Returns result via Serial: "INTENSITY sum=X peak=Y pixel=Z"
void ccdIntensityOnly() {
  ccdStartReadout();
  unsigned long sum      = 0;
  uint16_t      peak     = 0;
  int           peakPixel = 0;
  for (int i = 0; i < 3648; i++) {
    digitalWrite(ccdClkPin, HIGH); delayMicroseconds(5);
    uint16_t v = (uint16_t)analogRead(ccdOsPin);
    digitalWrite(ccdClkPin, LOW);  delayMicroseconds(5);
    sum += v;
    if (v > peak) { peak = v; peakPixel = i; }
  }
  Serial.print(F("INTENSITY sum="));  Serial.print(sum);
  Serial.print(F(" peak="));          Serial.print(peak);
  Serial.print(F(" pixel="));         Serial.println(peakPixel);
}

// ─── setup ────────────────────────────────────────
void setup() {
  pinMode(stepPin,     OUTPUT);
  pinMode(dirPin,      OUTPUT);
  pinMode(m0Pin,       OUTPUT);
  pinMode(m1Pin,       OUTPUT);
  pinMode(m2Pin,       OUTPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(limitPin,    INPUT_PULLUP);

  // CCD pins
  pinMode(ccdClkPin, OUTPUT);
  pinMode(ccdIcgPin, OUTPUT);
  pinMode(ccdShPin,  OUTPUT);
  // CCD idle state: clock LOW, ICG HIGH, SH LOW
  digitalWrite(ccdClkPin, LOW);
  digitalWrite(ccdIcgPin, HIGH);
  digitalWrite(ccdShPin,  LOW);

  // Trigger: INPUT_PULLUP — internal pull-up holds D12 HIGH when idle.
  pinMode(triggerPin, INPUT_PULLUP);

  // Set ADC prescaler to 16 for faster analogRead (~13 µs vs 104 µs default)
  // ADPS2=1, ADPS1=0, ADPS0=0 → prescaler 16
  ADCSRA = (ADCSRA & ~0x07) | 0x04;

  digitalWrite(powerLedPin, HIGH);  // LED on = Arduino running
  applyMstep(currentMstepIdx);

  Serial.begin(115200);
  Serial.println(F("Ready."));
}

// ─── loop ─────────────────────────────────────────
void loop() {

  // ── Trigger pin: detect falling edge (HIGH → LOW) ──
  bool pinLow = (digitalRead(triggerPin) == LOW);
  unsigned long now = millis();

  if (pinLow && !lastTrigState && (now - lastTrigTime) > TRIG_DEBOUNCE_MS) {
    lastTrigTime  = now;
    lastTrigState = true;

    if (scanArmed && scanStepsPerTrig > 0) {
      if (scanMaxTriggers > 0 && scanTriggered >= scanMaxTriggers) {
        scanArmed        = false;
        scanStepsPerTrig = 0;
        scanReversed     = false;
        Serial.println(F("SCAN_DISARMED"));
      } else {
        long stepsToMove = scanStepsPerTrig;
        bool moveCW = !scanReversed;
        if (moveCW && currentPosition + stepsToMove > MAX_STEPS) {
          stepsToMove = MAX_STEPS - currentPosition;
          Serial.println(F("WARN: CW limit — steps clamped"));
        } else if (!moveCW && stepsToMove > currentPosition) {
          stepsToMove = currentPosition;
          Serial.println(F("WARN: CCW limit — steps clamped"));
        }
        if (stepsToMove > 0) {
          doMove(moveCW, stepsToMove);
        }
        scanTriggered++;
        Serial.print(F("TRIG_STEP:"));
        Serial.println(currentPosition);

        if (scanMaxTriggers > 0 && scanTriggered >= scanMaxTriggers) {
          scanArmed        = false;
          scanStepsPerTrig = 0;
          scanReversed     = false;
          Serial.println(F("SCAN_DISARMED"));
        }
      }
    }
  }

  if (!pinLow && lastTrigState) {
    lastTrigState = false;
  }

  // ── Serial command handler ─────────────────────
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // STOP
    if (input == F("STOP")) {
      stopFlag  = true;
      scanArmed = false;
      Serial.println(F("Stopped."));
      return;
    }

    // POS
    if (input == F("POS")) {
      Serial.print(F("POS:"));
      Serial.println(currentPosition);
      return;
    }

    // SCAN_DISARM
    if (input == F("SCAN_DISARM")) {
      scanArmed        = false;
      scanStepsPerTrig = 0;
      scanReversed     = false;
      scanTriggered    = 0;
      scanMaxTriggers  = 0;
      Serial.println(F("SCAN_DISARMED"));
      return;
    }

    // SCAN_STATUS
    if (input == F("SCAN_STATUS")) {
      Serial.print(F("SCAN_STATUS armed="));
      Serial.print(scanArmed ? '1' : '0');
      Serial.print(F(" reversed="));
      Serial.print(scanReversed ? '1' : '0');
      Serial.print(F(" triggered="));
      Serial.print(scanTriggered);
      Serial.print(F(" max_triggers="));
      Serial.print(scanMaxTriggers);
      Serial.print(F(" steps_per_trig="));
      Serial.println(scanStepsPerTrig);
      return;
    }

    // CCD_INTENSITY — returns peak + sum only (fast, no buffer needed)
    if (input == F("CCD_INTENSITY")) {
      ccdIntensityOnly();
      return;
    }

    // CCD_READ — streams all 3648 pixel values over serial (no buffer)
    if (input == F("CCD_READ")) {
      ccdStreamPixels();
      return;
    }

    // HOME_SEEK
    if (input == F("HOME_SEEK")) {
      scanArmed = false;

      if (digitalRead(limitPin) == HIGH) {
        Serial.println(F("Homing: clearing switch — backing off CW..."));
        digitalWrite(dirPin, HIGH);
        for (int i = 0; i < 800; i++) {
          digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_FAST_DELAY);
          digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_FAST_DELAY);
        }
      }

      Serial.println(F("Homing: fast sweep CCW..."));
      digitalWrite(dirPin, LOW);
      unsigned long t0 = millis();
      while (digitalRead(limitPin) == LOW) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_FAST_DELAY);
        digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_FAST_DELAY);
        if (millis() - t0 > 30000UL) {
          Serial.println(F("ERR: Homing timeout — limit switch not found"));
          return;
        }
      }

      Serial.println(F("Homing: backing off CW..."));
      digitalWrite(dirPin, HIGH);
      for (long i = 0; i < HOME_BACKOFF; i++) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_FAST_DELAY);
        digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_FAST_DELAY);
      }
      delay(120);

      Serial.println(F("Homing: slow creep CCW..."));
      digitalWrite(dirPin, LOW);
      t0 = millis();
      while (digitalRead(limitPin) == LOW) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_CREEP_DELAY);
        digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_CREEP_DELAY);
        if (millis() - t0 > 30000UL) {
          Serial.println(F("ERR: Homing timeout — slow creep failed"));
          return;
        }
      }

      Serial.println(F("Homing: 4 rotations CW offset..."));
      applyMstep(0);
      delay(10);
      digitalWrite(dirPin, HIGH);
      for (int i = 0; i < 12800; i++) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_LAST);
        digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_LAST);
      }
      delay(10);
      applyMstep(currentMstepIdx);
      delay(10);

      currentPosition = 0;
      Serial.println(F("Homed."));
      Serial.println(F("POS:0"));
      return;
    }

    // ── Commands with arguments ──────────────────
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println(F("ERR: bad format"));
      return;
    }
    String cmd  = input.substring(0, spaceIndex);
    cmd.toUpperCase();
    String args = input.substring(spaceIndex + 1);
    args.trim();

    // CCD_INT <ms>
    if (cmd == F("CCD_INT")) {
      int ms = args.toInt();
      if (ms < 1)   ms = 1;
      if (ms > 100) ms = 100;
      integrationMs = ms;
      Serial.print(F("CCD_INT_SET ms="));
      Serial.println(integrationMs);
      return;
    }

    // SET_SPEED <delay_us>
    if (cmd == F("SET_SPEED")) {
      int val = args.toInt();
      if (val < 50) { Serial.println(F("ERR: delay_us must be >= 50")); return; }
      stepDelay = val;
      Serial.print(F("Speed set: ")); Serial.print(stepDelay); Serial.println(F("us"));
      return;
    }

    // SET_MSTEP <index 0-4>
    if (cmd == F("SET_MSTEP")) {
      int idx = args.toInt();
      if (idx < 0 || idx > 4) { Serial.println(F("ERR: index must be 0-4")); return; }
      currentMstepIdx = idx;
      applyMstep(currentMstepIdx);
      Serial.print(F("MStep set: ")); Serial.println(idx);
      return;
    }

    // SET_MPINS <m0> <m1> <m2>
    if (cmd == F("SET_MPINS")) {
      int a = args.indexOf(' ');
      int b = args.indexOf(' ', a + 1);
      if (a == -1 || b == -1) { Serial.println(F("ERR: SET_MPINS needs 3 args")); return; }
      int m0 = args.substring(0, a).toInt();
      int m1 = args.substring(a + 1, b).toInt();
      int m2 = args.substring(b + 1).toInt();
      digitalWrite(m0Pin, m0 ? HIGH : LOW);
      digitalWrite(m1Pin, m1 ? HIGH : LOW);
      digitalWrite(m2Pin, m2 ? HIGH : LOW);
      Serial.print(F("MPins set: "));
      Serial.print(m0); Serial.print(' ');
      Serial.print(m1); Serial.print(' ');
      Serial.println(m2);
      return;
    }

    // SCAN_ARM <steps_per_trigger> [R] [MAX:<n>]
    if (cmd == F("SCAN_ARM")) {
      int spaceIdx = args.indexOf(' ');
      long spt;
      bool rev = false;
      int  maxTrig = 0;

      if (spaceIdx == -1) {
        spt = args.toInt();
      } else {
        spt = args.substring(0, spaceIdx).toInt();
        String rest = args.substring(spaceIdx + 1); rest.trim(); rest.toUpperCase();
        int start = 0;
        while (start < (int)rest.length()) {
          int sp = rest.indexOf(' ', start);
          String tok = (sp == -1) ? rest.substring(start) : rest.substring(start, sp);
          tok.trim();
          if (tok == "R" || tok == "1") {
            rev = true;
          } else if (tok.startsWith("MAX:")) {
            maxTrig = tok.substring(4).toInt();
          }
          if (sp == -1) break;
          start = sp + 1;
        }
      }
      if (spt <= 0) { Serial.println(F("ERR: steps_per_trigger must be > 0")); return; }
      scanStepsPerTrig = spt;
      scanReversed     = rev;
      scanTriggered    = 0;
      scanMaxTriggers  = (maxTrig > 0) ? maxTrig : 0;
      scanArmed        = true;
      Serial.print(F("SCAN_ARMED steps_per_trig="));
      Serial.print(scanStepsPerTrig);
      Serial.print(F(" reversed="));
      Serial.print(scanReversed ? '1' : '0');
      Serial.print(F(" max_triggers="));
      Serial.println(scanMaxTriggers);
      return;
    }

    // CW <steps>
    // CCW <steps>
    if (cmd != F("CW") && cmd != F("CCW")) {
      Serial.println(F("ERR: unknown command"));
      return;
    }

    long steps = args.toInt();
    if (steps <= 0) { Serial.println(F("ERR: steps must be positive")); return; }
    bool cw = (cmd == F("CW"));

    if (!cw) {
      if (currentPosition <= 0) {
        Serial.println(F("ERR: At CCW limit (home). Cannot move further CCW."));
        Serial.print(F("POS:")); Serial.println(currentPosition);
        return;
      }
      if (steps > currentPosition) {
        steps = currentPosition;
        Serial.println(F("WARN: Steps clamped to home."));
      }
    } else {
      if (currentPosition >= MAX_STEPS) {
        Serial.println(F("ERR: At CW limit (+35 deg). Cannot move further CW."));
        Serial.print(F("POS:")); Serial.println(currentPosition);
        return;
      }
      long remaining = MAX_STEPS - currentPosition;
      if (steps > remaining) {
        steps = remaining;
        Serial.println(F("WARN: Steps clamped to +35 deg limit."));
      }
    }

    bool ok = doMove(cw, steps);
    if (ok) Serial.println(F("Done."));
    Serial.print(F("POS:")); Serial.println(currentPosition);
  }
}
