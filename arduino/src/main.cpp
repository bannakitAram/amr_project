// Hook lift controller - Arduino Mega 2560
//
// Serial (115200, newline-terminated) from the Jetson:
//   U = raise   D = lower   S = stop   R = reset fault   ? = report state
//   L<n> = tower light pattern from the Jetson (light_state_node.py via
//          hook_bridge.py), resent every 0.5 s:
//          L0 idle        green steady
//          L1 auto        yellow blink 1 Hz (Nav2 goal active / moving)
//          L2 manual      yellow blink 2.5 Hz (joystick driving)
//          L3 obstacle    yellow steady (collision_monitor holding it)
//          L4 fault       red steady (sensor watchdog, ...)
//   O<n><0|1> = general-purpose relay n (1..3 = D30/D31/D32) off/on,
//          replies OUT <r1><r2><r3> (e.g. OUT 100). All OFF at boot.
//          Not safety-related loads only (lights, fan, ...): no link
//          watchdog - a relay stays as commanded if the Jetson goes away.
// Replies (one line): STATE <IDLE|UP|DOWN|MOVING_UP|MOVING_DOWN|FAULT> [reason]
// Unsolicited every CURRENT_PERIOD_MS:  CUR <left_A> <right_A>
//   drive wheel motor current from ACS712 sensors on A2 (left) / A3 (right),
//   mean over the period (smooths PWM ripple), signed (sign = direction).
//   Monitor only - nothing in this firmware acts on it.
// Unsolicited:  BAT <raw reply>
//   Junctek KG-F coulometer on RS485 via an auto-direction RS485-TTL module
//   on Serial1 (D18 TX1 -> module TXD/DI, D19 RX1 <- module RXD/RO). The
//   Mega sends BAT_POLL_CMD every BAT_PERIOD_MS and forwards the meter's
//   reply line unparsed (hook_bridge.py decodes it), so field order can be
//   fixed on the Jetson without reflashing. Monitor only.
//
// Tower light (24 V, R/Y/G) via a 3-channel relay module on D26/D27/D28.
// Local states override the Jetson's pattern:
//   hook FAULT           -> red steady
//   no L<n> for LIGHT_TIMEOUT_MS (Jetson/ROS hung, cable out, after boot)
//                        -> red blink 1 Hz
//   hook moving          -> yellow + green blink 1 Hz
//
// Limit switches on A1 (top) / A0 (bottom), INPUT_PULLUP.
// Local push button on D4 to board GND (0V), INPUT_PULLUP:
//   hold BUTTON_HOLD_MS -> run to the other limit (at top -> down,
//   at bottom -> up, in between -> opposite of the last move, or down
//   if there was none); any press while moving -> stop at once.
// Motor via 4-relay H-bridge: SW1+SW4 = CCW, SW2+SW3 = CW.
// SW1+SW2 or SW3+SW4 together = short circuit across the supply, so the
// firmware only ever energizes one diagonal pair, with all relays off for
// RELAY_DEADTIME_MS before any pair closes.
//
// Safety behaviour:
//   - motor is off at power-up and after any reset
//   - a move stops on its limit, on 'S', or after MOVE_TIMEOUT_MS -> FAULT
//   - both limits active at once -> FAULT (wiring / switch fault)
//   - FAULT keeps the motor off until 'R'
// This is NOT a safety function: hardware limit cut-off and E-stop must
// remove motor power independently of this firmware.

#include <Arduino.h>

// ---- Pins -------------------------------------------------------------
const uint8_t PIN_LIMIT_UP   = A1;
const uint8_t PIN_LIMIT_DOWN = A0;
const uint8_t PIN_BUTTON     = 4;
const uint8_t PIN_SW1 = 22;
const uint8_t PIN_SW2 = 23;
const uint8_t PIN_SW3 = 24;
const uint8_t PIN_SW4 = 25;
const uint8_t PIN_LAMP_RED    = 26;
const uint8_t PIN_LAMP_YELLOW = 27;
const uint8_t PIN_LAMP_GREEN  = 28;
const uint8_t PIN_OUT[3] = {30, 31, 32};   // general-purpose relays O1..O3
const uint8_t PIN_CUR_LEFT  = A2;
const uint8_t PIN_CUR_RIGHT = A3;

// ---- Config -----------------------------------------------------------
// NC switch to GND (recommended, fail-safe): pressed or broken wire = HIGH.
// NO switch to GND: pressed = LOW -> set to LOW.
const uint8_t LIMIT_ACTIVE_LEVEL = HIGH;

// Most relay modules (opto, JD-VCC) are active LOW: LOW = relay on.
const uint8_t RELAY_ON  = LOW;
const uint8_t RELAY_OFF = (RELAY_ON == LOW) ? HIGH : LOW;

// Which rotation raises the hook. Check with the motor unloaded.
const bool UP_IS_CW = true;

const uint32_t RELAY_DEADTIME_MS = 150;   // all off before closing a pair
const uint32_t MOVE_TIMEOUT_MS  = 8000;   // normal travel time + margin
const uint32_t DEBOUNCE_MS      = 15;
const uint32_t BUTTON_HOLD_MS   = 1000;
const uint32_t SERIAL_BAUD      = 115200;
const uint32_t LIGHT_TIMEOUT_MS = 2000;   // Jetson resends every 500 ms

// ACS712 current sensors (5 V supply, output ratiometric to Vcc).
// mV per A: ACS712-05B 185, -20A 100, -30A 66. Set from the chip marking.
const float    CUR_MV_PER_A      = 100.0;   // ACS712-20A (confirmed on the robot)
// ADC count at 0 A (ideal 512 = Vcc/2). Calibrate with motors stopped:
// read the CUR value, offset_counts = 512 + A * CUR_MV_PER_A / 4.883.
const float    CUR_ZERO_LEFT     = 512.0;
const float    CUR_ZERO_RIGHT    = 512.0;
const uint32_t CUR_SAMPLE_MS     = 2;
const uint32_t CUR_PERIOD_MS     = 200;   // CUR line rate 5 Hz

// KG-F coulometer (RS485 on Serial1). Baud must match the meter's setting.
const uint32_t BAT_BAUD          = 115200;
const char     BAT_POLL_CMD[]    = ":R50=1,2,1,\r\n";   // read all, address 1
const uint32_t BAT_PERIOD_MS     = 1000;

// ---- State ------------------------------------------------------------
enum State { IDLE, AT_UP, AT_DOWN, MOVING_UP, MOVING_DOWN, FAULT };
const char *STATE_NAMES[] = {"IDLE", "UP", "DOWN", "MOVING_UP", "MOVING_DOWN", "FAULT"};

State state = IDLE;
uint32_t moveStartMs = 0;
const char *faultReason = "";

struct Debounced {
  uint8_t pin;
  uint8_t activeLevel;
  bool stable;
  bool lastRaw;
  uint32_t changedMs;
};
Debounced limitUp   = {PIN_LIMIT_UP,   LIMIT_ACTIVE_LEVEL, false, false, 0};
Debounced limitDown = {PIN_LIMIT_DOWN, LIMIT_ACTIVE_LEVEL, false, false, 0};
Debounced button    = {PIN_BUTTON,     LOW,                false, false, 0};

bool lastMoveUp = false;        // direction of the last move started
uint32_t buttonPressedMs = 0;
bool buttonHandled = false;     // one action per press; release to re-arm

enum LightCmd { L_IDLE, L_AUTO, L_MANUAL, L_OBSTACLE, L_FAULT, L_COUNT };
uint8_t lightCmd = L_IDLE;
uint32_t lightCmdMs = 0;
bool lightCmdSeen = false;      // none yet since boot -> link-lost pattern

uint32_t curSumL = 0, curSumR = 0;
uint16_t curCount = 0;
uint32_t curSampleMs = 0, curReportMs = 0;
char batBuf[128];
uint8_t batLen = 0;
uint32_t batPollMs = 0;

char rxBuf[16];
uint8_t rxLen = 0;

// ---- Helpers ----------------------------------------------------------
bool readActive(const Debounced &d) {
  return digitalRead(d.pin) == d.activeLevel;
}

// Returns true on the debounced edge to active.
bool updateDebounce(Debounced &d, uint32_t now) {
  bool raw = readActive(d);
  if (raw != d.lastRaw) {
    d.lastRaw = raw;
    d.changedMs = now;
  } else if (raw != d.stable && now - d.changedMs >= DEBOUNCE_MS) {
    d.stable = raw;
    return raw;
  }
  return false;
}

void relaysOff() {
  digitalWrite(PIN_SW1, RELAY_OFF);
  digitalWrite(PIN_SW2, RELAY_OFF);
  digitalWrite(PIN_SW3, RELAY_OFF);
  digitalWrite(PIN_SW4, RELAY_OFF);
}

void motorStop() {
  relaysOff();
}

// Close one diagonal pair. Everything is opened first and held open for the
// dead time so a relay still releasing can never overlap its partner leg.
// Blocking is fine here: the motor is off during the wait.
void motorRun(bool cw) {
  relaysOff();
  delay(RELAY_DEADTIME_MS);
  if (cw) {
    digitalWrite(PIN_SW2, RELAY_ON);
    digitalWrite(PIN_SW3, RELAY_ON);
  } else {
    digitalWrite(PIN_SW1, RELAY_ON);
    digitalWrite(PIN_SW4, RELAY_ON);
  }
}

void motorUp()   { motorRun(UP_IS_CW); }
void motorDown() { motorRun(!UP_IS_CW); }

void report() {
  Serial.print(F("STATE "));
  Serial.print(STATE_NAMES[state]);
  if (state == FAULT) {
    Serial.print(' ');
    Serial.print(faultReason);
  }
  Serial.println();
}

void setState(State s) {
  if (s == state) return;
  state = s;
  report();
}

void enterFault(const char *reason) {
  motorStop();
  faultReason = reason;
  state = IDLE;  // force report even if already FAULT with another reason
  setState(FAULT);
}

// State when the motor is not running, from the limit switches.
State restingState() {
  if (limitUp.stable) return AT_UP;
  if (limitDown.stable) return AT_DOWN;
  return IDLE;
}

// ---- Tower light ------------------------------------------------------
// Same active level as the motor relays (RELAY_ON). Lamps are not part of
// the H-bridge, so no dead time is needed.
void lamps(bool red, bool yellow, bool green) {
  digitalWrite(PIN_LAMP_RED,    red    ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_LAMP_YELLOW, yellow ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_LAMP_GREEN,  green  ? RELAY_ON : RELAY_OFF);
}

// period_ms on/off 50 %, phase from millis() so no state is needed.
bool blink(uint32_t now, uint32_t period_ms) {
  return (now % period_ms) < period_ms / 2;
}

void updateLights(uint32_t now) {
  bool linkOk = lightCmdSeen && now - lightCmdMs <= LIGHT_TIMEOUT_MS;
  if (state == FAULT) { lamps(true, false, false); return; }
  if (!linkOk) { lamps(blink(now, 1000), false, false); return; }
  if (state == MOVING_UP || state == MOVING_DOWN) {
    bool on = blink(now, 1000);
    lamps(false, on, on);
    return;
  }
  switch (lightCmd) {
    case L_AUTO:     lamps(false, blink(now, 1000), false); break;
    case L_MANUAL:   lamps(false, blink(now, 400), false);  break;
    case L_OBSTACLE: lamps(false, true, false);             break;
    case L_FAULT:    lamps(true, false, false);             break;
    default:         lamps(false, false, true);             break;  // L_IDLE
  }
}

// ---- Wheel motor current (monitor only) -------------------------------
void updateCurrent(uint32_t now) {
  if (now - curSampleMs >= CUR_SAMPLE_MS) {
    curSampleMs = now;
    curSumL += analogRead(PIN_CUR_LEFT);
    curSumR += analogRead(PIN_CUR_RIGHT);
    curCount++;
  }
  if (now - curReportMs >= CUR_PERIOD_MS && curCount > 0) {
    curReportMs = now;
    const float aPerCount = (5000.0 / 1024.0) / CUR_MV_PER_A;
    float left  = ((float)curSumL / curCount - CUR_ZERO_LEFT)  * aPerCount;
    float right = ((float)curSumR / curCount - CUR_ZERO_RIGHT) * aPerCount;
    curSumL = curSumR = 0;
    curCount = 0;
    Serial.print(F("CUR "));
    Serial.print(left, 2);
    Serial.print(' ');
    Serial.println(right, 2);
  }
}

// ---- Battery coulometer (monitor only) ---------------------------------
void updateBattery(uint32_t now) {
  if (now - batPollMs >= BAT_PERIOD_MS) {
    batPollMs = now;
    Serial1.print(BAT_POLL_CMD);
  }
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (batLen > 0) {
        batBuf[batLen] = '\0';
        Serial.print(F("BAT "));
        Serial.println(batBuf);
        batLen = 0;
      }
    } else if (batLen < sizeof(batBuf) - 1) {
      if (c >= 32 && c < 127) batBuf[batLen++] = c;   // printable only
    } else {
      batLen = 0;                                      // overlong: drop
    }
  }
}

// ---- Moves ------------------------------------------------------------
void startUp() {
  if (state == FAULT) { report(); return; }
  if (limitUp.stable) { motorStop(); state = AT_UP; report(); return; }
  motorUp();
  lastMoveUp = true;
  moveStartMs = millis();
  setState(MOVING_UP);
}

void startDown() {
  if (state == FAULT) { report(); return; }
  if (limitDown.stable) { motorStop(); state = AT_DOWN; report(); return; }
  motorDown();
  lastMoveUp = false;
  moveStartMs = millis();
  setState(MOVING_DOWN);
}

void stopMove() {
  motorStop();
  if (state != FAULT) state = restingState();
  report();
}

void handleButton(uint32_t now, bool pressedEdge) {
  if (pressedEdge) {
    buttonPressedMs = now;
    buttonHandled = false;
    if (state == MOVING_UP || state == MOVING_DOWN) {
      stopMove();
      buttonHandled = true;
    }
  }
  if (!button.stable || buttonHandled) return;
  if (now - buttonPressedMs < BUTTON_HOLD_MS) return;

  buttonHandled = true;
  if (state == AT_UP) startDown();
  else if (state == AT_DOWN) startUp();
  else if (state == IDLE) { if (lastMoveUp) startDown(); else startUp(); }
  else report();  // FAULT: needs 'R' from the Jetson
}

// ---- Commands ---------------------------------------------------------
void handleCommand(char c) {
  switch (c) {
    case 'U':
      startUp();
      break;
    case 'D':
      startDown();
      break;
    case 'S':
      stopMove();
      break;
    case 'R':
      motorStop();
      if (state == FAULT) state = restingState();
      report();
      break;
    case '?':
      report();
      break;
    default:
      Serial.println(F("ERR unknown command"));
      break;
  }
}

// ---- General-purpose relays (D30..D32) -------------------------------
bool outOn[3] = {false, false, false};

void reportOut() {
  Serial.print(F("OUT "));
  for (uint8_t i = 0; i < 3; i++) Serial.print(outOn[i] ? '1' : '0');
  Serial.println();
}

void setOut(uint8_t i, bool on) {
  outOn[i] = on;
  digitalWrite(PIN_OUT[i], on ? RELAY_ON : RELAY_OFF);
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen == 1) handleCommand(toupper(rxBuf[0]));
      else if (rxLen == 2 && toupper(rxBuf[0]) == 'L' &&
               rxBuf[1] >= '0' && rxBuf[1] < '0' + L_COUNT) {
        // silent (no reply): sent every 0.5 s, would flood the STATE stream
        lightCmd = rxBuf[1] - '0';
        lightCmdMs = millis();
        lightCmdSeen = true;
      }
      else if (rxLen == 3 && toupper(rxBuf[0]) == 'O' &&
               rxBuf[1] >= '1' && rxBuf[1] <= '3' &&
               (rxBuf[2] == '0' || rxBuf[2] == '1')) {
        setOut(rxBuf[1] - '1', rxBuf[2] == '1');
        reportOut();
      }
      else if (rxLen > 1) Serial.println(F("ERR unknown command"));
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf)) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ---- Main -------------------------------------------------------------
void setup() {
  // Relays off before anything else. Write the level BEFORE pinMode(OUTPUT):
  // pinMode alone drives LOW, which energizes active-LOW relay modules.
  relaysOff();
  pinMode(PIN_SW1, OUTPUT);
  pinMode(PIN_SW2, OUTPUT);
  pinMode(PIN_SW3, OUTPUT);
  pinMode(PIN_SW4, OUTPUT);
  relaysOff();
  for (uint8_t i = 0; i < 3; i++) {  // level first, then OUTPUT (see above)
    setOut(i, false);
    pinMode(PIN_OUT[i], OUTPUT);
  }
  lamps(false, false, false);  // level first, then OUTPUT (see above)
  pinMode(PIN_LAMP_RED, OUTPUT);
  pinMode(PIN_LAMP_YELLOW, OUTPUT);
  pinMode(PIN_LAMP_GREEN, OUTPUT);

  pinMode(PIN_LIMIT_UP, INPUT_PULLUP);
  pinMode(PIN_LIMIT_DOWN, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  limitUp.stable = limitUp.lastRaw = readActive(limitUp);
  limitDown.stable = limitDown.lastRaw = readActive(limitDown);
  // A button held during boot must be released before it can act.
  button.stable = button.lastRaw = readActive(button);
  buttonHandled = true;

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(BAT_BAUD);
  state = restingState();
  Serial.println(F("HOOK READY"));
  report();
  reportOut();
}

void loop() {
  uint32_t now = millis();
  updateDebounce(limitUp, now);
  updateDebounce(limitDown, now);
  bool buttonEdge = updateDebounce(button, now);
  readSerial();
  handleButton(now, buttonEdge);
  // A command may have just started a move and set moveStartMs after `now`
  // was taken (motorRun blocks for the dead time); re-read so the timeout
  // check below cannot underflow.
  now = millis();
  updateCurrent(now);
  updateBattery(now);

  if (limitUp.stable && limitDown.stable) {
    if (state != FAULT) enterFault("BOTH_LIMITS");
    updateLights(now);
    return;
  }

  switch (state) {
    case MOVING_UP:
      if (limitUp.stable) { motorStop(); setState(AT_UP); }
      else if (now - moveStartMs > MOVE_TIMEOUT_MS) enterFault("TIMEOUT_UP");
      break;
    case MOVING_DOWN:
      if (limitDown.stable) { motorStop(); setState(AT_DOWN); }
      else if (now - moveStartMs > MOVE_TIMEOUT_MS) enterFault("TIMEOUT_DOWN");
      break;
    case FAULT:
      motorStop();
      break;
    default:
      // Resting: follow the switches (e.g. hook moved by hand).
      setState(restingState());
      break;
  }
  updateLights(now);
}
