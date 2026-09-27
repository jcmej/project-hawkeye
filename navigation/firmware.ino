/*
 * Project Hawkeye: firmware for the SunFounder Zeus Car's R3 board.
 *
 * The car is "blind": it has no idea where it is. It just turns the hub's
 * velocity commands into wheel speeds, and stops if the commands stop.
 *
 *   CarVisionHub (Mac) --WebSocket--> ESP32-CAM --serial--> this sketch --> 4 motors
 *
 * The ESP32-CAM keeps SunFounder's stock firmware. It creates the Wi-Fi network,
 * runs a WebSocket server on port 8765, and turns every JSON message it gets into
 * one serial line with a field per letter A..Z. The hub sends
 *   {"A":vx,"B":vy,"C":omega,"D":seq}      (vx, vy, omega scaled to -1000..1000)
 * which arrives here as
 *   WS+<vx>;<vy>;<omega>;<seq>;;;;;;;;;;;;;;;;;;;;;;
 *
 * vx = forward, vy = left (strafe), omega = counterclockwise, the same as
 * DriveCommand in CarVisionHub.
 *
 * Optional fields E, F, G set the RGB lights (0..255 each, 0,0,0 = off), e.g.
 *   {"A":vx,"B":vy,"C":omega,"D":seq,"E":r,"F":g,"G":b}
 * Messages without them (CarVisionHub) leave the lights as they are.
 *
 * Needs the SoftPWM library (Library Manager: "SoftPWM" by Brett Hagman).
 * Board: Arduino Uno. Serial (pins 0/1) is shared with the ESP32-CAM, so
 * nothing else may be printed to Serial and the Serial Monitor shows nothing
 * useful.
 */

#include <SoftPWM.h>

// ---- Wi-Fi (hosted by the ESP32-CAM) ----
// The network shows up as "<WIFI_SSID>-XXXXXX" (the ESP32 adds part of its MAC).
// The ESP32 stores these in flash and ignores changes after the first time
// they're set, until it is factory-reset from its settings page (http://192.168.4.1).
#define WIFI_SSID "Zeus_Car"
#define WIFI_PASSWORD "12345678"
#define WS_PORT "8765"

// ---- Motors ----
// Pins and wheel order from SunFounder's car_control.h:
//   [0]--front--[1]
//    |           |
//   [3]---------[2]
// Motor i is driven by pins 2i and 2i+1; the left side is mounted mirrored.
static const uint8_t MOTOR_PINS[8] = {3, 4, 5, 6, A3, A2, A1, A0};
static const bool MOTOR_REVERSED[4] = {true, false, false, true};

// Below about this PWM the TT motors hum but don't turn, so any nonzero
// command starts here. Raise it if the car stalls at low speed.
#define PWM_MIN 50
#define PWM_MAX 255
// Commands smaller than this (fraction of full speed) count as zero.
#define DEADZONE 0.03
// SoftPWM ramp time. Softens current spikes on direction changes; short
// enough that the hub's controller doesn't notice the lag.
#define FADE_MS 40

// ---- Lights ----
// RGB LED pins (R, G, B) and brightness balance from SunFounder's rgb.h: the
// green and blue LEDs are much brighter than the red one at the same PWM.
static const uint8_t RGB_PINS[3] = {12, 13, 11};
static const float RGB_BALANCE[3] = {1.0, 0.16, 0.30};

// ---- Safety ----
// The hub sends 20 commands per second. If none arrives for this long
// (Wi-Fi dropout, hub crashed, laptop lid closed), stop the motors.
#define WATCHDOG_MS 300
// How often to echo the latest sequence number back so the hub can show
// that the car is listening and what the round-trip time is.
#define ACK_INTERVAL_MS 200

static char line[128];
static uint8_t lineLen = 0;
static bool lineOverflow = false;

static uint32_t lastCommandAt = 0;
static uint32_t lastAckAt = 0;
static bool moving = false;

// Reads whatever serial data is available without blocking. Returns true once
// a complete line is in `line`. Lines that don't fit are dropped whole.
// Non-printable bytes (like the junk the ESP32 emits while rebooting, which
// would otherwise hide the "[OK]" at the start of its reply) are skipped,
// as SunFounder's own reader does.
bool readLine() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[lineLen] = '\0';
      bool ok = !lineOverflow;
      lineLen = 0;
      lineOverflow = false;
      if (ok) return true;
      continue;
    }
    if ((uint8_t)c < 32 || (uint8_t)c > 126) continue;
    if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
  return false;
}

bool startsWith(const char *s, const char *prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

// Sends "SET+<cmd><value>" to the ESP32 and waits for a reply starting with "[OK]".
bool espCommand(const char *cmd, const char *value, uint16_t timeoutMs) {
  Serial.print(F("SET+"));
  Serial.print(cmd);
  Serial.println(value);
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (readLine() && startsWith(line, "[OK]")) return true;
  }
  return false;
}

// Same startup sequence as SunFounder's AiCamera::begin(). Unlike theirs, it
// retries instead of hanging forever if the ESP32 is still booting.
void startEsp() {
  for (;;) {
    // RESET reboots the ESP32; it answers "[OK] <version>" once it's back up.
    // Wait long enough for a full boot: sending RESET again too early could
    // restart it mid-boot, over and over. (SunFounder's code waits up to 9 s.)
    if (!espCommand("RESET", "", 10000)) continue;
    espCommand("NAME", WIFI_SSID, 1000);
    espCommand("TYPE", "Zeus_Car", 1000);
    espCommand("SSID", WIFI_SSID, 1000);
    espCommand("PSK", WIFI_PASSWORD, 1000);
    espCommand("MODE", "2", 1000);  // 2 = access point (ignored by newer ESP32 firmware)
    espCommand("PORT", WS_PORT, 1000);
    // START opens the WebSocket server and answers "[OK] <ip>".
    if (espCommand("START", "", 5000)) return;
  }
}

// power: -1..1, positive = this wheel rolls the car forward.
void setMotor(uint8_t i, float power) {
  float mag = fabs(power);
  uint8_t pwm = 0;
  if (mag >= DEADZONE) {
    pwm = PWM_MIN + (PWM_MAX - PWM_MIN) * min(mag, 1.0f);
  }
  bool forward = (power > 0) != MOTOR_REVERSED[i];
  SoftPWMSet(MOTOR_PINS[i * 2], forward ? pwm : 0);
  SoftPWMSet(MOTOR_PINS[i * 2 + 1], forward ? 0 : pwm);
}

// Mecanum mixing. Each wheel's rollers push at 45 degrees, so strafing comes
// from spinning diagonal pairs in opposite directions.
void drive(float vx, float vy, float omega) {
  float w[4] = {
    vx - vy - omega,  // 0 front-left
    vx + vy + omega,  // 1 front-right
    vx - vy + omega,  // 2 rear-right
    vx + vy - omega,  // 3 rear-left
  };
  // If any wheel would exceed full speed, scale all four down together so
  // the direction of travel is kept.
  float biggest = 1;
  for (uint8_t i = 0; i < 4; i++) biggest = max(biggest, (float)fabs(w[i]));
  for (uint8_t i = 0; i < 4; i++) setMotor(i, w[i] / biggest);
  moving = vx != 0 || vy != 0 || omega != 0;
}

void stopMotors() {
  drive(0, 0, 0);
}

// r, g, b: 0..255, or -1 to leave the lights as they are.
void setLights(long r, long g, long b) {
  static long current[3] = {0, 0, 0};
  long want[3] = {r, g, b};
  for (uint8_t i = 0; i < 3; i++) {
    if (want[i] < 0) return;
  }
  for (uint8_t i = 0; i < 3; i++) {
    if (want[i] == current[i]) continue;
    current[i] = want[i];
    SoftPWMSet(RGB_PINS[i], constrain(want[i], 0, 255) * RGB_BALANCE[i]);
  }
}

// Parses "<vx>;<vy>;<omega>;<seq>;<r>;<g>;<b>;...". The first four fields are
// required: returns false for any other WS+ message (e.g. the SunFounder phone
// app). Missing light fields come back as -1.
bool parseCommand(const char *s, long out[7]) {
  for (uint8_t i = 4; i < 7; i++) out[i] = -1;
  for (uint8_t i = 0; i < 7; i++) {
    char *end;
    long v = strtol(s, &end, 10);
    if (end != s && *end == ';') {
      out[i] = v;
    } else if (i < 4) {
      return false;
    }
    const char *next = strchr(s, ';');
    if (next == NULL) break;
    s = next + 1;
  }
  return true;
}

void handleLine() {
  if (startsWith(line, "WS+")) {
    long v[7];
    if (!parseCommand(line + 3, v)) return;
    drive(v[0] / 1000.0, v[1] / 1000.0, v[2] / 1000.0);
    setLights(v[4], v[5], v[6]);
    lastCommandAt = millis();
    if (millis() - lastAckAt >= ACK_INTERVAL_MS) {
      // "WS+" lines are forwarded by the ESP32 to every WebSocket client.
      Serial.print(F("WS+{\"ack\":"));
      Serial.print(v[3]);
      Serial.println('}');
      lastAckAt = millis();
    }
  } else if (startsWith(line, "[DISCONNECTED]") || startsWith(line, "[APPSTOP]")) {
    stopMotors();
  }
}

void setup() {
  Serial.begin(115200);
  SoftPWMBegin();
  for (uint8_t i = 0; i < 8; i++) {
    SoftPWMSet(MOTOR_PINS[i], 0);
    SoftPWMSetFadeTime(MOTOR_PINS[i], FADE_MS, FADE_MS);
  }
  for (uint8_t i = 0; i < 3; i++) {
    SoftPWMSet(RGB_PINS[i], 0);
    SoftPWMSetFadeTime(RGB_PINS[i], 100, 100);
  }
  startEsp();
}

void loop() {
  if (readLine()) handleLine();
  if (moving && millis() - lastCommandAt > WATCHDOG_MS) stopMotors();
}
