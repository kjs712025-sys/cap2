/* ======================================================================
 *  OmniBot — STM32 motor-controller firmware
 *  Target : ST NUCLEO-F411RE   (Arduino IDE + "STM32 MCU based boards" core)
 *  Drives : 2x AM-DC2-2D dual DC motor drivers  ->  4-wheel omni base
 *
 *  [변경] 전후좌우 동작이 확인된 데모 코드 기준으로 핀, EN 극성, 방향 부호,
 *         횡이동 패턴, PWM 속도를 맞춘 버전.
 *
 *  ---------------------------------------------------------------------
 *  ARDUINO IDE SETUP
 *    1. Boards Manager -> install "STM32 MCU based boards" (STMicroelectronics)
 *    2. Tools > Board             : "Nucleo-64"
 *       Tools > Board part number : "Nucleo F411RE"
 *       Tools > U(S)ART support   : "Enabled (generic 'Serial')"
 *       Tools > Upload method     : "STM32CubeProgrammer (SWD)"  (or Mass Storage)
 *    3. Port: the ST-LINK Virtual COM Port. On the Raspberry Pi this enumerates
 *       as /dev/ttyACM0 (set ROBOT_UART_PORT in the backend .env). Baud 115200.
 *
 *  ---------------------------------------------------------------------
 *  WIRING   (same as the verified demo sketch)
 *  +--------+-----+-----+-----+---------------------------+
 *  | motor  | PWM | DIR | EN  | AM-DC2-2D driver / chan   |
 *  +--------+-----+-----+-----+---------------------------+
 *  | M0     | D3  | D4  | D5  | #1 (front)  channel 1     |
 *  | M1     | D6  | D7  | D8  | #1 (front)  channel 2     |
 *  | M2     | A0  | A1  | A2  | #2 (rear)   channel 1     |
 *  | M3     | A3  | A4  | A5  | #2 (rear)   channel 2     |
 *  +--------+-----+-----+-----+---------------------------+
 *  - AM-DC2-2D enable input is active-LOW (/Enable): LOW = channel ON.
 *  - Tie the grounds of the Nucleo, both drivers and the motor battery together.
 *  - Never power the motors from the Nucleo 5V rail.
 *
 *  ---------------------------------------------------------------------
 *  SERIAL PROTOCOL   (must match robot/stm32.py)
 *    frame = "<payload>*<HH>\n"       HH = (sum of payload bytes) & 0xFF, hex
 *    Pi  -> STM32 :  A,1 | A,0 | V,<vx>,<vy>,<wz> | H | T
 *    STM32 -> Pi  :  S,<state> | B,<volts> | E,<msg> | O,<x>,<y>,<yaw>
 *      vx  forward(+) / back(-)      m/s   (|vx| <= MAX_LINEAR)
 *      vy  left(+)    / right(-)     m/s   (|vy| <= MAX_LINEAR)
 *      wz  ccw/left(+)/ cw/right(-)  rad/s (|wz| <= MAX_ANGULAR)
 *    A frame typed by hand in the Serial Monitor may omit "*HH" — the
 *    checksum is only verified when a '*' is present.
 *
 *  ---------------------------------------------------------------------
 *  [추가] SELF TEST  (시리얼 모니터 115200, 줄 끝 "Newline")
 *    T  입력  ->  전진 / 후진 / 좌이동 / 우이동 / 제자리 좌회전을 3초씩
 *                 데모와 같은 속도(PWM 200, M1은 255)로 실행하고 멈춘다.
 *                 워치독과 상관없이 끝까지 돈다. 중간에 멈추려면 A,0
 *    제자리 좌회전(반시계)은 데모에 없던 동작이라 여기서 방향을 확인할 것.
 *    반대로(시계 방향) 돌면 WHEEL_MIX의 wz 열 부호를 전부 뒤집으면 된다.
 * ==================================================================== */

#include <string.h>
#include <stdlib.h>
#include <math.h>

// ---------------------------------------------------------------------
//  Configuration
// ---------------------------------------------------------------------
#define N_MOTORS      4

// [변경] 데모 코드와 동일한 핀 배치
const uint8_t PWM_PIN[N_MOTORS] = { D3,  D6,  A0,  A3  };
const uint8_t DIR_PIN[N_MOTORS] = { D4,  D7,  A1,  A4  };
const uint8_t EN_PIN [N_MOTORS] = { D5,  D8,  A2,  A5  };

// [변경] 데모의 전진 패턴(M0 REV, M1 FWD, M2 FWD, M3 REV)에서 가져온 방향 부호
int8_t MOTOR_SIGN[N_MOTORS]     = { -1,  +1,  +1,  -1  };

// AM-DC2-2D logic levels (데모와 동일: /Enable = LOW일 때 동작)
#define EN_ACTIVE     LOW
#define DIR_POS       HIGH        // DIR level for "positive" wheel rotation
#define DIR_NEG       LOW

// Kinematic limits — keep in sync with config.py.
#define MAX_LINEAR    0.5f        // ROBOT_MAX_LINEAR_SPEED  (m/s)
#define MAX_ANGULAR   1.0f        // ROBOT_MAX_ANGULAR_SPEED (rad/s)
#define ROT_GAIN      1.0f        // extra scale on the wz term; tune on the bench

// Wheel mixing. Rows = M0..M3, cols = vx, vy, wz.
// Values are in "wheel forward" units (MOTOR_SIGN is applied afterwards).
// [변경] vy 열은 데모의 좌이동 패턴(M0 REV, M1 REV, M2 FWD, M3 FWD)을 그대로 옮긴 것.
//        wz 열은 그 패턴에서 유도한 값이라 SELF TEST의 좌회전으로 확인할 것.
const float WHEEL_MIX[N_MOTORS][3] = {
  { +1.f, +1.f, +1.f },   // M0 (front)
  { +1.f, -1.f, -1.f },   // M1 (front)
  { +1.f, +1.f, -1.f },   // M2 (rear)
  { +1.f, -1.f, +1.f },   // M3 (rear)
};

// PWM shaping.
#define PWM_FREQ_HZ   20000      // 데모와 동일
#define PWM_BITS      8          // analogWrite range 0..255
#define PWM_MAX       255
// [변경] 명령 크기(0~1)를 PWM_START ~ PWM_TOP 구간으로 매핑한다.
//        예전처럼 0부터 선형으로 매핑하면 V,0.2(40%)나 순항(30%)에서 듀티가 너무 낮다.
//        최대 명령(|v| = MAX)일 때 데모와 똑같은 PWM이 나온다.
//        저속에서 여전히 안 도는 바퀴가 있으면 PWM_START를 올릴 것.
#define PWM_START     150
const int PWM_TOP[N_MOTORS] = { 200, 255, 200, 200 };   // 데모 RUN_SPEED / RUN_SPEED_M1
#define CMD_DEADZONE  0.02f      // |command| below this -> wheel off
#define PWM_MIN       28         // deadband during ramps
#define SLEW_STEP     12         // max |PWM| change per control tick (soft start/stop)

// Timing.
#define CONTROL_HZ    200        // motor update rate
#define CMD_TIMEOUT_MS 500       // no V/H within this -> coast to stop (Pi watchdog)
#define STATUS_MS     1000       // S,<state> heartbeat back to the Pi
#define BATT_MS       2000
#define SERIAL_BAUD   115200

// [추가] SELF TEST timing
#define TEST_STEP_MS  3000
#define TEST_GAP_MS   700

// Battery monitoring — set to 1 once a divider is wired to BATT_PIN.
#define ENABLE_BATTERY 0
#define BATT_PIN       PC2       // A0 is used for M2 PWM
#define BATT_DIVIDER   11.0f     // (R_top + R_bottom) / R_bottom
#define ADC_VREF       3.3f
#define ADC_MAX        4095.0f   // 12-bit

// Wheel odometry — set to 1 ONLY if quadrature encoders are wired.
#define USE_ENCODERS   0

// ---------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------
bool     g_armed    = false;     // A,1 / A,0
bool     g_linkOk   = false;     // a valid frame arrived within CMD_TIMEOUT_MS
float    g_vx = 0, g_vy = 0, g_wz = 0;
uint32_t g_lastRxMs = 0;
int      g_pwm[N_MOTORS] = { 0, 0, 0, 0 };        // current signed PWM (after slew)
int      g_pwmWritten[N_MOTORS] = { -1, -1, -1, -1 };  // last value sent to analogWrite

char     g_buf[80];
uint8_t  g_bufLen = 0;

struct TestStep { const char *status; float vx, vy, wz; };
const TestStep TEST_STEPS[] = {
  { "S,test_forward",    +MAX_LINEAR, 0.f, 0.f },
  { "S,test_backward",   -MAX_LINEAR, 0.f, 0.f },
  { "S,test_left",       0.f, +MAX_LINEAR, 0.f },
  { "S,test_right",      0.f, -MAX_LINEAR, 0.f },
  { "S,test_rotate_ccw", 0.f, 0.f, +MAX_ANGULAR },
};
const int N_TEST_STEPS = sizeof(TEST_STEPS) / sizeof(TEST_STEPS[0]);
int      g_testIdx = -1;         // -1 = not running
uint32_t g_testT0  = 0;

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
  snprintf(tail, sizeof(tail), "*%02X", cs);
  Serial.print(payload);
  Serial.print(tail);
  Serial.print('\n');
}

// ---------------------------------------------------------------------
//  Motor output
// ---------------------------------------------------------------------
// Only touch the timer when the duty actually changes.
static void setPwm(int i, int value) {
  if (value != g_pwmWritten[i]) {
    analogWrite(PWM_PIN[i], value);
    g_pwmWritten[i] = value;
  }
}

static void writeMotor(int i, int signedPwm) {
  int mag = abs(signedPwm);
  if (mag < PWM_MIN) mag = 0;
  bool live = g_armed && g_linkOk && mag > 0;

  digitalWrite(DIR_PIN[i], (signedPwm >= 0) ? DIR_POS : DIR_NEG);
  digitalWrite(EN_PIN[i], live ? EN_ACTIVE : !EN_ACTIVE);
  setPwm(i, live ? mag : 0);
}

static void hardStop() {
  for (int i = 0; i < N_MOTORS; ++i) {
    g_pwm[i] = 0;
    digitalWrite(EN_PIN[i], !EN_ACTIVE);
    setPwm(i, 0);
  }
}

// Normalised wheel command (-1..1, already signed) -> signed PWM.
static int commandToPwm(int i, float t) {
  float a = fabsf(t);
  if (a < CMD_DEADZONE) return 0;
  if (a > 1.f) a = 1.f;
  int mag = (int)lroundf(PWM_START + (PWM_TOP[i] - PWM_START) * a);
  return (t >= 0.f) ? mag : -mag;
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
    int want = commandToPwm(i, target[i] * MOTOR_SIGN[i]);
    int cur  = g_pwm[i];
    if      (want > cur + SLEW_STEP) cur += SLEW_STEP;
    else if (want < cur - SLEW_STEP) cur -= SLEW_STEP;
    else                             cur  = want;
    g_pwm[i] = cur;
    writeMotor(i, cur);
  }
}

// ---------------------------------------------------------------------
//  Self test (T)
// ---------------------------------------------------------------------
static void stopSelfTest() {
  g_testIdx = -1;
}

static void startSelfTest() {
  g_testIdx = 0;
  g_testT0  = millis();
  g_armed   = true;
  emit(TEST_STEPS[0].status);
}

static void runSelfTest(uint32_t now) {
  if (g_testIdx < 0) return;

  // keep the watchdog satisfied while the test runs
  g_lastRxMs = now;
  g_linkOk   = true;
  g_armed    = true;

  uint32_t elapsed = now - g_testT0;
  if (elapsed >= (uint32_t)(TEST_STEP_MS + TEST_GAP_MS)) {
    ++g_testIdx;
    g_testT0 = now;
    elapsed  = 0;
    if (g_testIdx >= N_TEST_STEPS) {
      stopSelfTest();
      g_armed = false;
      g_vx = g_vy = g_wz = 0;
      hardStop();
      emit("S,test_done");
      return;
    }
    emit(TEST_STEPS[g_testIdx].status);
  }

  if (elapsed < TEST_STEP_MS) {
    g_vx = TEST_STEPS[g_testIdx].vx;
    g_vy = TEST_STEPS[g_testIdx].vy;
    g_wz = TEST_STEPS[g_testIdx].wz;
  } else {
    g_vx = g_vy = g_wz = 0;      // short pause between steps
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

    case 'T':                                   // bench self test
      startSelfTest();
      return;

    case 'A':                                   // A,1 / A,0
      if (p[1] == ',' && p[2] == '1') {
        g_armed = true;
        emit("S,armed");
      } else {
        stopSelfTest();
        g_armed = false;
        g_vx = g_vy = g_wz = 0;
        hardStop();
        emit("S,idle");
      }
      return;

    case 'V': {                                 // V,<vx>,<vy>,<wz>
      if (p[1] != ',') { emit("E,badframe"); return; }
      char *s = p + 2;
      float vx = strtod(s, &s); if (*s == ',') ++s;
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
volatile long g_ticks[N_MOTORS] = {0, 0, 0, 0};
float g_px = 0, g_py = 0, g_pyaw = 0;

static void updateOdometry() {
  // TODO: convert per-wheel tick deltas -> body twist -> integrate g_px/py/pyaw
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

  // Disable every channel first, then set DIR / PWM (no twitch on boot).
  for (int i = 0; i < N_MOTORS; ++i) {
    pinMode(EN_PIN[i], OUTPUT);
    digitalWrite(EN_PIN[i], !EN_ACTIVE);
  }
  for (int i = 0; i < N_MOTORS; ++i) {
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    digitalWrite(DIR_PIN[i], DIR_POS);
    setPwm(i, 0);
  }

  g_lastRxMs = millis();
  emit("S,boot");
}

void loop() {
  pumpSerial();

  const uint32_t now = millis();
  runSelfTest(now);

  // Comms watchdog: the Pi refreshes V at 4-7 Hz and H at 1 Hz. If both stop,
  // coast the wheels to a halt but stay armed so motion resumes on reconnect.
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
    if (g_testIdx < 0) {
      emit(g_armed ? (g_linkOk ? "S,armed" : "S,armed_nolink") : "S,idle");
    }
#if USE_ENCODERS
    sendOdometry();
#endif
  }

#if ENABLE_BATTERY
  static uint32_t tBatt = 0;
  if (now - tBatt >= BATT_MS) { tBatt = now; sendBattery(); }
#endif
}
