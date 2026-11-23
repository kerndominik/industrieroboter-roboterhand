// =============================================================================
//  Robot hand firmware - hardware configuration for the ESP32 (DevKit v1 / WROOM-32)
// =============================================================================
//  Adapt this file to your wiring. Everything else in the sketch is generic.
//
//  Servo / finger order (index 0..4) is used everywhere (firmware, serial
//  protocol, ROS 2 parameters):
//      0 = thumb, 1 = index, 2 = middle, 3 = ring, 4 = little finger
// =============================================================================
#pragma once

#include <Arduino.h>

#define FW_VERSION  "1.0.0"
#define BOARD_NAME  "ESP32"
#define SERIAL_BAUD 115200

// ----------------------------------------------------------------------------
//  Servos
// ----------------------------------------------------------------------------
#define NUM_SERVOS 5

// Signal pins (any output capable GPIO). Avoid strapping pins (0, 2, 5, 12, 15),
// flash pins (6-11) and input-only pins (34-39). Servos need their own 5-6 V
// supply - connect its GND to the ESP32 GND. The 3.3 V signal is accepted by
// most hobby servos.
static const uint8_t SERVO_PINS[NUM_SERVOS] = {18, 19, 23, 25, 26};

// Pulse width (microseconds) that corresponds to 0 deg and 180 deg.
// MG996R / SG90 type servos: roughly 500..2500 us. Check your datasheet.
static const uint16_t SERVO_MIN_US[NUM_SERVOS] = {500, 500, 500, 500, 500};
static const uint16_t SERVO_MAX_US[NUM_SERVOS] = {2500, 2500, 2500, 2500, 2500};

// Hard safety limits (deg). Commands outside are clamped by the firmware,
// independent of what the PC sends. Narrow these once the tendons are mounted.
static const float SERVO_LIMIT_MIN_DEG[NUM_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0};
static const float SERVO_LIMIT_MAX_DEG[NUM_SERVOS] = {180.0, 180.0, 180.0, 180.0, 180.0};

// Position used when servos are enabled before any target was received.
static const float SERVO_HOME_DEG[NUM_SERVOS] = {10.0, 10.0, 10.0, 10.0, 10.0};

// Default maximum servo speed (deg/s). 0 = no ramping (jump to target).
#define DEFAULT_MAX_SPEED_DEG_S 180.0

// Interval of the motion ramp generator (ms).
#define CONTROL_PERIOD_MS 10

// If > 0: servos are detached (go limp) when no command was received for this
// time (ms). 0 disables the watchdog (servos hold their last position).
#define COMMAND_TIMEOUT_MS 0

// ----------------------------------------------------------------------------
//  Telemetry
// ----------------------------------------------------------------------------
#define DEFAULT_TELEMETRY_HZ 50   // 0 = off, max 200
#define MAX_TELEMETRY_HZ     200

// ----------------------------------------------------------------------------
//  Strain gauges (optional, for future use)
// ----------------------------------------------------------------------------
#define SG_SOURCE_NONE   0  // no strain gauges, no "G" telemetry lines
#define SG_SOURCE_ANALOG 1  // amplified bridge (e.g. INA125 / HX710 analog) on ADC pins
#define SG_SOURCE_HX711  2  // one HX711 24-bit bridge ADC per gauge, shared SCK line

#ifndef STRAIN_GAUGE_SOURCE  // can be overridden with -DSTRAIN_GAUGE_SOURCE=...
#define STRAIN_GAUGE_SOURCE SG_SOURCE_ANALOG
#endif
#define NUM_STRAIN_GAUGES   5

// SG_SOURCE_ANALOG: analog input pins and oversampling (averaged samples).
// Only use ADC1 pins (32-39): ADC2 cannot be used while WiFi is active.
// Max. input voltage 3.3 V! 12 bit resolution (0..4095).
static const uint8_t SG_ANALOG_PINS[NUM_STRAIN_GAUGES] = {36, 39, 34, 35, 32};
#define SG_ANALOG_OVERSAMPLING 8

// SG_SOURCE_HX711: one DOUT pin per HX711, all share one SCK pin
// (the input-only pins 34-39 are fine for DOUT)
static const uint8_t HX711_DOUT_PINS[NUM_STRAIN_GAUGES] = {36, 39, 34, 35, 32};
#define HX711_SCK_PIN 33
// Extra clock pulses after the 24 data bits select the next conversion:
// 1 = channel A gain 128, 2 = channel B gain 32, 3 = channel A gain 64
#define HX711_GAIN_PULSES 1
