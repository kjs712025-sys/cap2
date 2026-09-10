/* ======================================================================
 *  OmniBot — STM32 motor-controller firmware
 *  Target : ST NUCLEO-F411RE   (Arduino IDE + "STM32 MCU based boards" core)
 *  Drives : 2x AM-DC2-2D dual DC motor drivers  ->  4-wheel omni base
 *
 *  ---------------------------------------------------------------------
 *  ARDUINO IDE SETUP
 *    1. Boards Manager -> install "STM32 MCU based boards" (STMicroelectronics)
 *    2. Tools > Board             : "Nucleo-64"
 *       Tools > Board part number : "Nucleo F411RE"
 *       Tools > U(S)ART support   : "Enabled (generic 'Serial')"
 *       Tools > Upload method     : "STM32CubeProgrammer (SWD)"  (or Mass Storage)
 *    3. Port: the ST-LINK Virtual COM Port. On the Raspberry Pi this enumerates
 *       as /dev/ttyACM0 — the value already in the backend .env
 *       (ROBOT_UART_PORT). Baud 115200.
 *
 *  ---------------------------------------------------------------------
 *  WIRING   (Arduino "Dx" labels printed on the Nucleo headers)
 *  +--------+-----+-----+-----+---------------------------+
 *  | motor  | PWM | DIR | EN  | AM-DC2-2D driver / chan   |
 *  +--------+-----+-----+-----+---------------------------+
 *  | M0 FL  | D3  | D4  | D5  | #1  channel 1             |
 *  | M1 FR  | D6  | D7  | D8  | #1  channel 2             |
 *  | M2 RL  | D9  | D10 | D11 | #2  channel 1             |
 *  | M3 RR  | D12 | D13 | D2  | #2  channel 2             |
 *  +--------+-----+-----+-----+---------------------------+
 *  - Tie the grounds of the Nucleo, both drivers and the motor battery together.
 *  - Never power the motors from the Nucleo 5V rail.
 *  - D13 is also LD2 (on-board LED); it will blink with M3's direction — harmless.
 *  - FL/FR/RL/RR above is just a label. After flashing, run the bench test
 *    (see CALIBRATION below) and fix MOTOR_SIGN[] / the wheel order until
 *    "V,0.2,0,0" drives straight forward.
 *
 *  ---------------------------------------------------------------------
 *  SERIAL PROTOCOL   (must match robot/stm32.py)
 *    frame = "<payload>*<HH>\n"       HH = (sum of payload bytes) & 0xFF, hex
 *    Pi  -> STM32 :  A,1 | A,0 | V,<vx>,<vy>,<wz> | H
 *    STM32 -> Pi  :  S,<state> | B,<volts> | E,<msg> | O,<x>,<y>,<yaw>
 *      vx  forward(+) / back(-)      m/s   (|vx| <= MAX_LINEAR)
 *      vy  left(+)    / right(-)     m/s   (|vy| <= MAX_LINEAR)
 *      wz  ccw/left(+)/ cw/right(-)  rad/s (|wz| <= MAX_ANGULAR)
 *    A frame typed by hand in the Serial Monitor may omit "*HH" — the
 *    checksum is only verified when a '*' is present.
 *
 *  ---------------------------------------------------------------------
 *  CALIBRATION (once, on blocks with the wheels off the ground)
 *    1. Open Serial Monitor @115200, line ending "Newline".
 *    2. Send:  A,1
 *    3. Send:  V,0.2,0,0     -> every wheel should drive the robot FORWARD.
 *              Any wheel spinning backwards: flip its MOTOR_SIGN entry.
 *    4. Send:  V,0,0,0.5     -> robot should rotate left (CCW) in place.
 *    5. Send:  V,0,0.2,0     -> robot should strafe left.
 *              If rotation/strafe are swapped or mirrored, your physical
 *              wheel order differs from M0..M3 above — reorder PWM_PIN/
 *              DIR_PIN/EN_PIN together, or the WHEEL_MIX rows.
 *    6. Send:  A,0           -> everything stops.
 * ==================================================================== */

#include <string.h>
#include <stdlib.h>
#include <math.h>

// ---------------------------------------------------------------------
//  Configuration
// ---------------------------------------------------------------------
#define N_MOTORS      4

// Pins, indexed M0..M3 (see wiring table).
const uint8_t PWM_PIN[N_MOTORS] = { D3,  D6,  D9,  D12 };
const uint8_t DIR_PIN[N_MOTORS] = { D4,  D7,  D10, D13 };
const uint8_t EN_PIN [N_MOTORS] = { D5,  D8,  D11, D2  };

// Per-wheel spin direction fix, set during CALIBRATION step 3 (+1 or -1).
int8_t MOTOR_SIGN[N_MOTORS]     = { +1,  +1,  +1,  +1  };

// AM-DC2-2D logic levels — invert here if your board is active-low.
#define EN_ACTIVE     HIGH        // level that ENABLES a channel
#define DIR_POS       HIGH        // DIR level for "positive" wheel rotation
#define DIR_NEG       LOW

// Kinematic limits — keep in sync with config.py.
#define MAX_LINEAR    0.5f        // ROBOT_MAX_LINEAR_SPEED  (m/s)
#define MAX_ANGULAR   1.0f        // ROBOT_MAX_ANGULAR_SPEED (rad/s)
#define ROT_GAIN      1.0f        // extra scale on the wz term; tune on the bench

// 4-wheel omni / mecanum mixing (X layout). Rows = M0..M3, cols = vx, vy, wz.
// Matches the model drawn on the dashboard: [vx-vy-wz, vx+vy+wz, vx+vy-wz, vx-vy+wz]
const float WHEEL_MIX[N_MOTORS][3] = {
  { +1.f, -1.f, -1.f },   // M0 FL
  { +1.f, +1.f, +1.f },   // M1 FR
  { +1.f, +1.f, -1.f },   // M2 RL
  { +1.f, -1.f, +1.f },   // M3 RR
};

// PWM shaping.
#define PWM_FREQ_HZ   20000      // 20 kHz — above hearing, easy on the H-bridge
#define PWM_BITS      8          // analogWrite range 0..255
#define PWM_MAX       255
#define PWM_MIN       28         // deadband: below this a brushed motor only buzzes
#define SLEW_STEP     12         // max |PWM| change per control tick (soft start/stop)

// Timing.
#define CONTROL_HZ    200        // motor update rate
#define CMD_TIMEOUT_MS 500       // no V/H within this -> coast to stop (Pi watchdog ~0.5s)
#define STATUS_MS     1000       // S,<state> heartbeat back to the Pi
#define BATT_MS       2000
#define SERIAL_BAUD   115200

// Battery monitoring — set to 1 once a divider is wired to BATT_PIN.
#define ENABLE_BATTERY 0
#define BATT_PIN       A0
#define BATT_DIVIDER   11.0f     // (R_top + R_bottom) / R_bottom
#define ADC_VREF       3.3f
#define ADC_MAX        4095.0f   // 12-bit

// Wheel odometry — set to 1 ONLY if quadrature encoders are wired; otherwise
// the backend does its own motion-model dead-reckoning and a fake "O" frame
// would corrupt the map.
#define USE_ENCODERS   0

// ---------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------
bool     g_armed    = false;     // A,1 / A,0
bool     g_linkOk   = false;     // a valid frame arrived within CMD_TIMEOUT_MS
float    g_vx = 0, g_vy = 0, g_wz = 0;
uint32_t g_lastRxMs = 0;
int      g_pwm[N_MOTORS] = { 0, 0, 0, 0 };   // current signed PWM actually applied

char     g_buf[80];
uint8_t  g_bufLen = 0;

// ---------------------------------------------------------------------
//  Framing helpers
// ---------------------------------------------------------------------
static uint8_t payloadChecksum(const char *s, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum += (uint8_t)s[i];
  return (uint8_t)(sum & 0xFF);
}

// Send "<payload>*<HH>\n" with an uppercase-hex checksum (as robot/stm32.py does).
static void emit(const String &payload) {
  uint8_t cs = payloadChecksum(payload.c_str(), payload.length());
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X", cs);   // integer formatting — safe under newlib-nano
  Serial.print(payload);
  Serial.print(tail);
  Serial.print('\n');
}

// ---------------------------------------------------------------------
//  Motor output
// ---------------------------------------------------------------------
static void writeMotor(int i, int signedPwm) {
  int mag = abs(signedPwm);
  if (mag < PWM_MIN) mag = 0;
  bool live = g_armed && g_linkOk && mag > 0;

  digitalWrite(DIR_PIN[i], (signedPwm >= 0) ? DIR_POS : DIR_NEG);
  digitalWrite(EN_PIN[i], live ? EN_ACTIVE : !EN_ACTIVE);
  analogWrite(PWM_PIN[i], live ? mag : 0);
}

static void hardStop() {
  for (int i = 0; i < N_MOTORS; ++i) {
    g_pwm[i] = 0;
    digitalWrite(EN_PIN[i], !EN_ACTIVE);
    analogWrite(PWM_PIN[i], 0);
  }
}

// One control tick: mix the body twist into wheel commands, slew-limit, apply.
static void updateMotors() {
  float target[N_MOTORS];

  if (g_armed && g_linkOk) {
    const float nx = constrain(g_vx / MAX_LINEAR,  -1.f, 1.f);
    const float ny = constrain(g_vy / MAX_LINEAR,  -1.f, 1.f);
    const float nw = constrain(g_wz / MAX_ANGULAR, -1.f, 1.f) * ROT_GAIN;

    float peak = 1.f;
    for (int i = 0; i < N_MOTORS; ++i) {
      target[i] = WHEEL_MIX[i][0] * nx + WHEEL_MIX[i][1] * ny + WHEEL_MIX[i][2] * nw;
      peak = max(peak, fabsf(target[i]));
    }
    for (int i = 0; i < N_MOTORS; ++i) target[i] /= peak;   // keep every wheel within unity
  } else {
    for (int i = 0; i < N_MOTORS; ++i) target[i] = 0.f;
  }

  for (int i = 0; i < N_MOTORS; ++i) {
    int want = (int)lroundf(target[i] * MOTOR_SIGN[i] * PWM_MAX);
    int cur  = g_pwm[i];
    if      (want > cur + SLEW_STEP) cur += SLEW_STEP;
    else if (want < cur - SLEW_STEP) cur -= SLEW_STEP;
    else                             cur  = want;
    g_pwm[i] = cur;
    writeMotor(i, cur);
  }
}

// ---------------------------------------------------------------------
//  Command handling
// ---------------------------------------------------------------------
static void handleCommand(char *p) {
  g_lastRxMs = millis();
  g_linkOk   = true;

  switch (p[0]) {
    case 'H':                                   // heartbeat / watchdog ping
      return;

    case 'A':                                   // A,1 / A,0
      if (p[1] == ',' && p[2] == '1') {
        g_armed = true;
        emit("S,armed");
      } else {
        g_armed = false;
        g_vx = g_vy = g_wz = 0;
        hardStop();
        emit("S,idle");
      }
      return;

    case 'V': {                                 // V,<vx>,<vy>,<wz>
      if (p[1] != ',') { emit("E,badframe"); return; }
      char *s = p + 2;
      float vx = strtod(s, &s); if (*s == ',') ++s;   // strtod works under newlib-nano
      float vy = strtod(s, &s); if (*s == ',') ++s;
      float wz = strtod(s, &s);
      g_vx = constrain(vx, -MAX_LINEAR,  MAX_LINEAR);
      g_vy = constrain(vy, -MAX_LINEAR,  MAX_LINEAR);
      g_wz = constrain(wz, -MAX_ANGULAR, MAX_ANGULAR);
      return;
    }

    default:
      emit("E,unknown");
      return;
  }
}

// Split "<payload>*<HH>", verify the checksum when present, then dispatch.
static void handleLine(char *line) {
  char *star = strrchr(line, '*');
  if (star) {
    *star = '\0';
    uint8_t want = (uint8_t)strtol(star + 1, nullptr, 16);
    if (want != payloadChecksum(line, strlen(line))) {
      emit("E,checksum");
      return;
    }
  }
  if (line[0] != '\0') handleCommand(line);
}

static void pumpSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      g_buf[g_bufLen] = '\0';
      if (g_bufLen > 0) handleLine(g_buf);
      g_bufLen = 0;
    } else if (g_bufLen < sizeof(g_buf) - 1) {
      g_buf[g_bufLen++] = c;
    } else {
      g_bufLen = 0;                 // overflow — drop the runaway line
      emit("E,overflow");
    }
  }
}

// ---------------------------------------------------------------------
//  Telemetry
// ---------------------------------------------------------------------
#if ENABLE_BATTERY
static void sendBattery() {
  int raw = analogRead(BATT_PIN);
  float volts = (raw / ADC_MAX) * ADC_VREF * BATT_DIVIDER;
  emit(String("B,") + String(volts, 2));
}
#endif

#if USE_ENCODERS
// --- fill these in for your encoder hardware ---------------------------
volatile long g_ticks[N_MOTORS] = {0, 0, 0, 0};
float g_px = 0, g_py = 0, g_pyaw = 0;

static void updateOdometry() {
  // TODO: convert per-wheel tick deltas -> body twist -> integrate g_px/py/pyaw
  //       using your wheel radius, ticks/rev and base geometry.
}
static void sendOdometry() {
  emit(String("O,") + String(g_px, 3) + "," + String(g_py, 3) + "," + String(g_pyaw, 3));
}
#endif

// ---------------------------------------------------------------------
//  Arduino entry points
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PWM_FREQ_HZ);
#if ENABLE_BATTERY
  analogReadResolution(12);
#endif

  for (int i = 0; i < N_MOTORS; ++i) {
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(EN_PIN[i],  OUTPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    digitalWrite(DIR_PIN[i], DIR_POS);
    digitalWrite(EN_PIN[i],  !EN_ACTIVE);
    analogWrite(PWM_PIN[i], 0);
  }

  g_lastRxMs = millis();
  emit("S,boot");
}

void loop() {
  pumpSerial();

  const uint32_t now = millis();

  // Comms watchdog: the Pi refreshes V at 4-7 Hz and H at 1 Hz. If both stop,
  // coast the wheels to a halt but stay armed so motion resumes on reconnect.
  // No error is raised — the next "S," heartbeat reports "armed_nolink".
  if (g_linkOk && (now - g_lastRxMs > CMD_TIMEOUT_MS)) {
    g_linkOk = false;
    g_vx = g_vy = g_wz = 0;
  }

  static uint32_t tCtrl = 0;
  if (now - tCtrl >= (1000UL / CONTROL_HZ)) {
    tCtrl = now;
#if USE_ENCODERS
    updateOdometry();
#endif
    updateMotors();
  }

  static uint32_t tStat = 0;
  if (now - tStat >= STATUS_MS) {
    tStat = now;
    emit(g_armed ? (g_linkOk ? "S,armed" : "S,armed_nolink") : "S,idle");
#if USE_ENCODERS
    sendOdometry();
#endif
  }

#if ENABLE_BATTERY
  static uint32_t tBatt = 0;
  if (now - tBatt >= BATT_MS) { tBatt = now; sendBattery(); }
#endif
}
