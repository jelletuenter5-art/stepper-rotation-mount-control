// ═══════════════════════════════════════════════════
//  STEPPER MOTOR CONTROLLER — OPTICS BUILD
//
//  ── Pin map ─────────────────────────────────────
//  D2  → DIR  (stepper driver)
//  D3  → STEP (stepper driver)
//  D4  → M0   (microstepping)
//  D5  → M1   (microstepping)
//  D6  → M2   (microstepping)
//  D7  → Limit switch (NC: D7→COM→GND, idles LOW, HIGH when triggered)
//  D12 → Trigger input (RISING edge = move one scan step)
//  D13 → Built-in LED (power-on indicator, no external wiring)
//
//  ── Trigger wiring ──────────────────────────────
//  Uses internal INPUT_PULLUP — no external resistor needed.
//  D12 idles HIGH. Fires on FALLING edge when pulled LOW.
//    PSU trigger wire (negative/pulse) → D12
//    PSU GND → any Arduino GND pin
//  The pin goes LOW when the pulse fires → triggers one scan step.
// ═══════════════════════════════════════════════════

const int dirPin      = 2;
const int stepPin     = 3;
const int m0Pin       = 4;
const int m1Pin       = 5;
const int m2Pin       = 6;
const int limitPin    = 7;   // NC switch: idles LOW, HIGH when open/triggered
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
// When scanArmed == true, each falling edge on D12 moves scanStepsPerTrig
// steps (CW or CCW per scanReversed) and reports TRIG_STEP:<position>.
bool scanArmed        = false;
long scanStepsPerTrig = 0;   // steps to move per trigger (always positive)
bool scanReversed     = false; // true = CCW per trigger (high→low nm scan)
int  scanTriggered    = 0;   // triggers received since arming
int  scanMaxTriggers  = 0;   // hard cap — 0 = no cap; set by SCAN_ARM

// Debounce / edge detection for trigger pin
bool          lastTrigState = false;   // true = pin was HIGH on last check
unsigned long lastTrigTime  = 0;
const unsigned long TRIG_DEBOUNCE_MS = 50;

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

// ─── setup ────────────────────────────────────────
void setup() {
  pinMode(stepPin,     OUTPUT);
  pinMode(dirPin,      OUTPUT);
  pinMode(m0Pin,       OUTPUT);
  pinMode(m1Pin,       OUTPUT);
  pinMode(m2Pin,       OUTPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(limitPin,    INPUT_PULLUP);

  // Trigger: INPUT_PULLUP — internal pull-up holds D12 HIGH when idle.
  // Wire the PSU trigger wire to D12; PSU GND shares Arduino GND.
  // Pin goes LOW when the signal fires → detect FALLING edge.
  // No external resistor needed.
  pinMode(triggerPin, INPUT_PULLUP);

  digitalWrite(powerLedPin, HIGH);  // LED on = Arduino running
  applyMstep(currentMstepIdx);

  Serial.begin(9600);
  Serial.println(F("Ready."));
}

// ─── loop ─────────────────────────────────────────
void loop() {

  // ── Trigger pin: detect falling edge (HIGH → LOW) ──
  // With INPUT_PULLUP the pin idles HIGH; the trigger signal pulls it LOW.
  bool pinLow = (digitalRead(triggerPin) == LOW);
  unsigned long now = millis();

  if (pinLow && !lastTrigState && (now - lastTrigTime) > TRIG_DEBOUNCE_MS) {
    lastTrigTime  = now;
    lastTrigState = true;  // latch until pin returns HIGH again

    if (scanArmed && scanStepsPerTrig > 0) {
      // Hard cap: if we have already reached the maximum, self-disarm immediately
      // and ignore this trigger — even if the SCAN_DISARM serial command is still in flight.
      if (scanMaxTriggers > 0 && scanTriggered >= scanMaxTriggers) {
        scanArmed        = false;
        scanStepsPerTrig = 0;
        scanReversed     = false;
        Serial.println(F("SCAN_DISARMED"));
        // Do NOT increment scanTriggered or move.
      } else {
        long stepsToMove = scanStepsPerTrig;
        bool moveCW = !scanReversed;
        // Clamp to travel limits
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

        // Self-disarm once the hard cap is reached after this trigger
        if (scanMaxTriggers > 0 && scanTriggered >= scanMaxTriggers) {
          scanArmed        = false;
          scanStepsPerTrig = 0;
          scanReversed     = false;
          Serial.println(F("SCAN_DISARMED"));
        }
      }
    }
    // If not armed, trigger pulses are silently ignored
  }

  // Reset edge latch once pin returns HIGH
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

    // HOME_SEEK
    if (input == F("HOME_SEEK")) {
      scanArmed = false;

      // If limit pin is already HIGH, back off CW briefly to clear it
      if (digitalRead(limitPin) == HIGH) {
        Serial.println(F("Homing: clearing switch — backing off CW..."));
        digitalWrite(dirPin, HIGH);
        for (int i = 0; i < 800; i++) {
          digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_FAST_DELAY);
          digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_FAST_DELAY);
        }
      }

      // Phase 1: fast CCW sweep until limit pin goes HIGH
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

      // Phase 2: back off CW to release switch
      Serial.println(F("Homing: backing off CW..."));
      digitalWrite(dirPin, HIGH);
      for (long i = 0; i < HOME_BACKOFF; i++) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(HOME_FAST_DELAY);
        digitalWrite(stepPin, LOW);  delayMicroseconds(HOME_FAST_DELAY);
      }
      delay(120);

      // Phase 3: slow creep CCW until limit triggers again (precise zero)
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

      // Phase 4: 4 full rotations CW offset at full-step (slow)
      Serial.println(F("Homing: 4 rotations CW offset..."));
      applyMstep(0);   // force full-step: 3200 steps/rev
      delay(10);
      digitalWrite(dirPin, HIGH);
      for (int i = 0; i < 12800; i++) {   // 4 × 3200 = 12800 full steps because 200*16 is 3200, 16 because of 1/16 microstep
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
    // Arms the trigger. Each rising edge on D12 moves this many steps CW (or CCW if R).
    // MAX:<n> sets a hard trigger count cap — the Arduino self-disarms after n triggers.
    if (cmd == F("SCAN_ARM")) {
      // Format: SCAN_ARM <steps> [R] [MAX:<n>]
      int spaceIdx = args.indexOf(' ');
      long spt;
      bool rev = false;
      int  maxTrig = 0;

      if (spaceIdx == -1) {
        spt = args.toInt();
      } else {
        spt = args.substring(0, spaceIdx).toInt();
        String rest = args.substring(spaceIdx + 1); rest.trim(); rest.toUpperCase();
        // Parse each space-separated token after the step count
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
