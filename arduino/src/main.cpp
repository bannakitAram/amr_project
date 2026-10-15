// Hook lift controller - Arduino Mega 2560
//
// Serial (115200, newline-terminated) from the Jetson:
//   U = raise   D = lower   S = stop   R = reset fault   ? = report state
// Replies (one line): STATE <IDLE|UP|DOWN|MOVING_UP|MOVING_DOWN|FAULT> [reason]
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

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen == 1) handleCommand(toupper(rxBuf[0]));
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

  pinMode(PIN_LIMIT_UP, INPUT_PULLUP);
  pinMode(PIN_LIMIT_DOWN, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  limitUp.stable = limitUp.lastRaw = readActive(limitUp);
  limitDown.stable = limitDown.lastRaw = readActive(limitDown);
  // A button held during boot must be released before it can act.
  button.stable = button.lastRaw = readActive(button);
  buttonHandled = true;

  Serial.begin(SERIAL_BAUD);
  state = restingState();
  Serial.println(F("HOOK READY"));
  report();
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

  if (limitUp.stable && limitDown.stable) {
    if (state != FAULT) enterFault("BOTH_LIMITS");
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
}
