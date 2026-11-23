// =============================================================================
//  Robot hand firmware - Arduino Mega 2560
// =============================================================================
//  Drives 5 finger servos and (optionally) reads strain gauges. The PC side
//  (ROS 2 package "robothand_interface") talks to this sketch over USB serial
//  with a simple line based ASCII protocol (115200 baud, '\n' terminated):
//
//  PC -> board
//    S <a0> <a1> <a2> <a3> <a4>   set target angles of all servos [deg]
//    P <i> <a>                    set target angle of servo i [deg]
//    E <0|1>                      disable (detach, limp) / enable servos
//    V <deg_per_s>                max. servo speed, 0 = no ramping
//    R <hz>                       telemetry rate, 0 = off
//    H                            move all servos to the home position
//    T                            tare strain gauges (current value = zero)
//    Q                            send one telemetry frame now
//    I                            print board info
//
//  board -> PC
//    I ROBOTHAND <board> <fw> <n_servos> <n_gauges> <sg_source>
//    A <t_ms> <enabled> <c0> .. <c4>      current (ramped) servo angles [deg]
//    G <t_ms> <g0> .. <gN>                strain gauge values (raw, tared)
//    OK <cmd>                             command accepted (not sent for S/P)
//    ERR <message>                        command rejected
//    # <text>                             log / debug message
//
//  Finger order: 0 thumb, 1 index, 2 middle, 3 ring, 4 little.
//  Hardware settings (pins, pulse widths, limits) are in config.h.
// =============================================================================

#include <Servo.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "config.h"

// ----------------------------------------------------------------------------
//  State
// ----------------------------------------------------------------------------
static Servo servos[NUM_SERVOS];
static float targetDeg[NUM_SERVOS];
static float currentDeg[NUM_SERVOS];
static bool servosEnabled = false;
static float maxSpeedDegS = DEFAULT_MAX_SPEED_DEG_S;
static uint16_t telemetryPeriodMs = 0;

static unsigned long lastControlMs = 0;
static unsigned long lastTelemetryMs = 0;
static unsigned long lastCommandMs = 0;

#if STRAIN_GAUGE_SOURCE != SG_SOURCE_NONE
static long sgRaw[NUM_STRAIN_GAUGES];
static long sgOffset[NUM_STRAIN_GAUGES];
#endif

#define RX_BUFFER_SIZE 96
static char rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxLength = 0;
static bool rxOverflow = false;

// ----------------------------------------------------------------------------
//  Servo helpers
// ----------------------------------------------------------------------------
static float clampServoDeg(uint8_t i, float deg) {
  if (deg < SERVO_LIMIT_MIN_DEG[i]) return SERVO_LIMIT_MIN_DEG[i];
  if (deg > SERVO_LIMIT_MAX_DEG[i]) return SERVO_LIMIT_MAX_DEG[i];
  return deg;
}

static void writeServo(uint8_t i, float deg) {
  float us = SERVO_MIN_US[i] + (deg / 180.0f) * (float)(SERVO_MAX_US[i] - SERVO_MIN_US[i]);
  servos[i].writeMicroseconds((int)(us + 0.5f));
}

static void enableServos() {
  if (servosEnabled) return;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    writeServo(i, currentDeg[i]);  // set pulse before the first output edge
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US[i], SERVO_MAX_US[i]);
    writeServo(i, currentDeg[i]);
  }
  servosEnabled = true;
}

static void disableServos() {
  if (!servosEnabled) return;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) servos[i].detach();
  servosEnabled = false;
}

// Moves currentDeg towards targetDeg with the configured max. speed.
static void updateServos(float dtS) {
  float maxStep = maxSpeedDegS * dtS;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float err = targetDeg[i] - currentDeg[i];
    if (maxSpeedDegS <= 0.0f || fabs(err) <= maxStep) {
      currentDeg[i] = targetDeg[i];
    } else {
      currentDeg[i] += (err > 0.0f) ? maxStep : -maxStep;
    }
    if (servosEnabled) writeServo(i, currentDeg[i]);
  }
}

// ----------------------------------------------------------------------------
//  Strain gauges
// ----------------------------------------------------------------------------
#if STRAIN_GAUGE_SOURCE == SG_SOURCE_HX711
static void setupStrainGauges() {
  pinMode(HX711_SCK_PIN, OUTPUT);
  digitalWrite(HX711_SCK_PIN, LOW);
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) pinMode(HX711_DOUT_PINS[i], INPUT);
}

// Reads all HX711 at once (shared clock). Returns false if a conversion is not
// finished yet - the previous values are kept in that case.
static bool readStrainGauges() {
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) {
    if (digitalRead(HX711_DOUT_PINS[i]) == HIGH) return false;
  }
  long value[NUM_STRAIN_GAUGES] = {0};
  for (uint8_t bit = 0; bit < 24; bit++) {
    digitalWrite(HX711_SCK_PIN, HIGH);
    delayMicroseconds(1);
    for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) {
      value[i] = (value[i] << 1) | (digitalRead(HX711_DOUT_PINS[i]) == HIGH ? 1 : 0);
    }
    digitalWrite(HX711_SCK_PIN, LOW);
    delayMicroseconds(1);
  }
  for (uint8_t p = 0; p < HX711_GAIN_PULSES; p++) {
    digitalWrite(HX711_SCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK_PIN, LOW);
    delayMicroseconds(1);
  }
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) {
    if (value[i] & 0x800000L) value[i] |= 0xFF000000L;  // sign extend 24 -> 32 bit
    sgRaw[i] = value[i];
  }
  return true;
}
#elif STRAIN_GAUGE_SOURCE == SG_SOURCE_ANALOG
static void setupStrainGauges() {
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) pinMode(SG_ANALOG_PINS[i], INPUT);
}

static bool readStrainGauges() {
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) {
    long sum = 0;
    for (uint8_t s = 0; s < SG_ANALOG_OVERSAMPLING; s++) sum += analogRead(SG_ANALOG_PINS[i]);
    sgRaw[i] = sum / SG_ANALOG_OVERSAMPLING;
  }
  return true;
}
#else
static void setupStrainGauges() {}
#endif

static void tareStrainGauges() {
#if STRAIN_GAUGE_SOURCE != SG_SOURCE_NONE
  readStrainGauges();
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) sgOffset[i] = sgRaw[i];
#endif
}

// ----------------------------------------------------------------------------
//  Output
// ----------------------------------------------------------------------------
static void sendInfo() {
  Serial.print(F("I ROBOTHAND " BOARD_NAME " " FW_VERSION " "));
  Serial.print(NUM_SERVOS);
  Serial.print(' ');
#if STRAIN_GAUGE_SOURCE == SG_SOURCE_NONE
  Serial.print(0);
  Serial.println(F(" NONE"));
#elif STRAIN_GAUGE_SOURCE == SG_SOURCE_ANALOG
  Serial.print(NUM_STRAIN_GAUGES);
  Serial.println(F(" ANALOG"));
#else
  Serial.print(NUM_STRAIN_GAUGES);
  Serial.println(F(" HX711"));
#endif
}

static void sendTelemetry() {
  unsigned long now = millis();
  Serial.print(F("A "));
  Serial.print(now);
  Serial.print(servosEnabled ? F(" 1") : F(" 0"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(' ');
    Serial.print(currentDeg[i], 1);
  }
  Serial.println();
#if STRAIN_GAUGE_SOURCE != SG_SOURCE_NONE
  readStrainGauges();
  Serial.print(F("G "));
  Serial.print(now);
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) {
    Serial.print(' ');
    Serial.print(sgRaw[i] - sgOffset[i]);
  }
  Serial.println();
#endif
}

static void sendError(const __FlashStringHelper* msg) {
  Serial.print(F("ERR "));
  Serial.println(msg);
}

static void sendOk(char cmd) {
  Serial.print(F("OK "));
  Serial.println(cmd);
}

// ----------------------------------------------------------------------------
//  Command parsing
// ----------------------------------------------------------------------------
// Parses up to maxValues floats from str. Returns the number of values parsed
// or -1 on a syntax error.
static int parseFloats(char* str, float* values, int maxValues) {
  int n = 0;
  char* p = str;
  while (true) {
    while (*p == ' ' || *p == '\t' || *p == ',') p++;
    if (*p == '\0') break;
    if (n >= maxValues) return -1;
    char* end;
    double v = strtod(p, &end);
    if (end == p || isnan(v) || isinf(v)) return -1;
    values[n++] = (float)v;
    p = end;
  }
  return n;
}

static void handleCommand(char* line) {
  // strip leading whitespace
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;  // accept lower case
  char* args = line + 1;
  float v[NUM_SERVOS + 1];
  int n = parseFloats(args, v, NUM_SERVOS + 1);
  if (n < 0) {
    sendError(F("syntax"));
    return;
  }
  lastCommandMs = millis();

  switch (cmd) {
    case 'S':
      if (n != NUM_SERVOS) { sendError(F("S needs one angle per servo")); return; }
      for (uint8_t i = 0; i < NUM_SERVOS; i++) targetDeg[i] = clampServoDeg(i, v[i]);
      enableServos();
      break;
    case 'P': {
      if (n != 2) { sendError(F("usage: P <index> <angle>")); return; }
      int i = (int)v[0];
      if (i < 0 || i >= NUM_SERVOS) { sendError(F("servo index")); return; }
      targetDeg[i] = clampServoDeg(i, v[1]);
      enableServos();
      break;
    }
    case 'E':
      if (n != 1) { sendError(F("usage: E <0|1>")); return; }
      if (v[0] != 0.0f) enableServos(); else disableServos();
      sendOk(cmd);
      break;
    case 'V':
      if (n != 1 || v[0] < 0.0f) { sendError(F("usage: V <deg_per_s>")); return; }
      maxSpeedDegS = v[0];
      sendOk(cmd);
      break;
    case 'R':
      if (n != 1 || v[0] < 0.0f || v[0] > MAX_TELEMETRY_HZ) { sendError(F("usage: R <0..200>")); return; }
      telemetryPeriodMs = (v[0] > 0.0f) ? (uint16_t)(1000.0f / v[0] + 0.5f) : 0;
      sendOk(cmd);
      break;
    case 'H':
      for (uint8_t i = 0; i < NUM_SERVOS; i++) targetDeg[i] = clampServoDeg(i, SERVO_HOME_DEG[i]);
      enableServos();
      sendOk(cmd);
      break;
    case 'T':
      tareStrainGauges();
      sendOk(cmd);
      break;
    case 'Q':
      sendTelemetry();
      break;
    case 'I':
      sendInfo();
      break;
    default:
      sendError(F("unknown command"));
      break;
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxOverflow) {
        sendError(F("line too long"));
      } else {
        rxBuffer[rxLength] = '\0';
        handleCommand(rxBuffer);
      }
      rxLength = 0;
      rxOverflow = false;
    } else if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = c;
    } else {
      rxOverflow = true;
    }
  }
}

// ----------------------------------------------------------------------------
//  Arduino entry points
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentDeg[i] = clampServoDeg(i, SERVO_HOME_DEG[i]);
    targetDeg[i] = currentDeg[i];
  }
  telemetryPeriodMs = (DEFAULT_TELEMETRY_HZ > 0) ? (1000 / DEFAULT_TELEMETRY_HZ) : 0;
  setupStrainGauges();
#if STRAIN_GAUGE_SOURCE != SG_SOURCE_NONE
  for (uint8_t i = 0; i < NUM_STRAIN_GAUGES; i++) { sgRaw[i] = 0; sgOffset[i] = 0; }
#endif
  // Servos stay detached (limp) until the first command arrives.
  sendInfo();
  Serial.println(F("# ready"));
  lastControlMs = lastTelemetryMs = lastCommandMs = millis();
}

void loop() {
  pollSerial();

  unsigned long now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;
    updateServos(dt);
  }

#if COMMAND_TIMEOUT_MS > 0
  if (servosEnabled && (now - lastCommandMs > COMMAND_TIMEOUT_MS)) {
    disableServos();
    Serial.println(F("# command timeout - servos disabled"));
  }
#endif

  if (telemetryPeriodMs > 0 && now - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
