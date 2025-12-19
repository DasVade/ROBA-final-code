/*************************************************
 * ESP32 MASTER – 3 Modes (UDP)
 *  - JOYSTICK: discrete F/B/L/R/S over UDP
 *  - WALL_FOLLOW: UART DIST:RB,RF,F (RB/G starts, state advisory only)
 *  - VIVE: Vive pose + UDP TARGET:x,y -> rotate+drive to target
 *
 * UDP port: 4210 (same for all modes)
 *
 * Commands over UDP:
 *   J  -> JOYSTICK mode
 *   W  -> WALL_FOLLOW mode
 *   V  -> VIVE mode
 *   G  -> (RB) start wall-follow motion (ALWAYS starts; state advisory only)
 *   S  -> stop (always)
 *   F/B/L/R -> joystick motion ONLY in JOYSTICK mode
 *   TARGET:x,y -> ONLY in VIVE mode
 *
 * SERVO TOGGLE:
 *   T  -> toggle servo ONLY in JOYSTICK mode
 *************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>
#include "vive510.h"

/* =========================================================
   FIX for Arduino auto-prototypes:
   Define MedianFilter + forward-declare applyMedian up top
   ========================================================= */
#define VIVE_WINDOW_SIZE 2

struct MedianFilter {
  int buf[VIVE_WINDOW_SIZE];
  int index;
  int count;
};

int applyMedian(MedianFilter &f, int newValue);  // forward declaration

/* ===================== MODE ===================== */
enum Mode { MODE_JOYSTICK=0, MODE_WALLFOLLOW=1, MODE_VIVE=2 };
static Mode mode = MODE_JOYSTICK;

/* ===================== WiFi AP + UDP ===================== */
const char* AP_SSID = "Xbox_Joy";
WiFiUDP udp;
const unsigned int UDP_PORT = 4210;

static bool haveClient = false;
static IPAddress clientIP;
static uint16_t clientPort = 0;

static uint32_t lastRxMs = 0;
static char lastCmd = 'S';

void sendToClient(const char* msg) {
  if (!haveClient) return;
  udp.beginPacket(clientIP, clientPort);
  udp.print(msg);
  udp.endPacket();
}

/* ===================== Motors ===================== */
const int LEFT_PWM_PIN  = 3;
const int RIGHT_PWM_PIN = 7;

const int LF_IN1 = 5;
const int LF_IN2 = 6;

const int LB_IN1 = 36;
const int LB_IN2 = 35;

const int RF_IN1 = 33;
const int RF_IN2 = 38;

const int RB_IN1 = 18;
const int RB_IN2 = 17;

void setLeftDir(bool forward) {
  digitalWrite(LF_IN1, forward ? HIGH : LOW);
  digitalWrite(LF_IN2, forward ? LOW  : HIGH);
  digitalWrite(LB_IN1, forward ? HIGH : LOW);
  digitalWrite(LB_IN2, forward ? LOW  : HIGH);
}

void setRightDir(bool forward) {
  digitalWrite(RF_IN1, forward ? HIGH : LOW);
  digitalWrite(RF_IN2, forward ? LOW  : HIGH);
  digitalWrite(RB_IN1, forward ? HIGH : LOW);
  digitalWrite(RB_IN2, forward ? LOW  : HIGH);
}

void setSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  analogWrite(LEFT_PWM_PIN,  leftSpeed);
  analogWrite(RIGHT_PWM_PIN, rightSpeed);
}

void stopCar() { setSpeed(0, 0); }

// joystick wiring (your confirmed directions)
const int DRIVE_SPEED_JS = 200;
const int TURN_SPEED_JS  = 200;

void forwardJS(int speed)  { setLeftDir(false); setRightDir(false); setSpeed(speed, speed); }
void backwardJS(int speed) { setLeftDir(true);  setRightDir(true);  setSpeed(speed, speed); }
void turnLeftJS(int speed) { setLeftDir(true);  setRightDir(false); setSpeed(speed, speed); }
void turnRightJS(int speed){ setLeftDir(false); setRightDir(true);  setSpeed(speed, speed); }

void forwardMove(int leftSpeed, int rightSpeed) {
  setLeftDir(false);
  setRightDir(false);
  setSpeed(leftSpeed, rightSpeed);
}

void turnInPlaceLeft(int speed) {
  setLeftDir(true);
  setRightDir(false);
  setSpeed(speed, speed);
}

void turnInPlaceRight(int speed) {
  setLeftDir(false);
  setRightDir(true);
  setSpeed(speed, speed);
}

/* =========================================================
   SERVO (CONTINUOUS SWING ON TOGGLE)
   ========================================================= */
const int SERVO_PIN = 11;

const int SERVO_PWM_FREQ = 50;
const int SERVO_PWM_BITS = 12;
const int SERVO_PWM_MAX  = (1 << SERVO_PWM_BITS) - 1;

const int SERVO_US_MIN = 600;
const int SERVO_US_MAX = 2400;

// swing tuning (speed)
const int SERVO_STEP_US = 20;
const int SERVO_STEP_INTERVAL_MS = 20;

static bool servoSwingOn = false;
static int  servoPulseUs = SERVO_US_MIN;
static int  servoDir = +1;
static unsigned long lastServoStepMs = 0;

static inline int servoPulseUsToDuty(int pulseUs) {
  const int periodUs = 1000000 / SERVO_PWM_FREQ; // 20000
  long duty = ((long)pulseUs * (long)SERVO_PWM_MAX) / periodUs;  // Cast BOTH operands
  duty = constrain(duty, 0, SERVO_PWM_MAX);
  return (int)duty;
}

static inline void servoWriteUs(int pulseUs) {
  analogWrite(SERVO_PIN, servoPulseUsToDuty(pulseUs));
}

static inline void servoInit() {
  analogWriteFrequency(SERVO_PIN, SERVO_PWM_FREQ);
  analogWriteResolution(SERVO_PIN, SERVO_PWM_BITS);

  servoSwingOn = false;
  servoPulseUs = SERVO_US_MIN;
  servoDir = +1;
  servoWriteUs(SERVO_US_MIN);
  Serial.println("[SERVO] init -> 600us");
}

static inline void servoServiceSwing() {
  if (!servoSwingOn) return;

  unsigned long now = millis();
  if (now - lastServoStepMs < (unsigned long)SERVO_STEP_INTERVAL_MS) return;
  lastServoStepMs = now;

  servoPulseUs += servoDir * SERVO_STEP_US;

  if (servoPulseUs >= SERVO_US_MAX) {
    servoPulseUs = SERVO_US_MAX;
    servoDir = -1;
  } else if (servoPulseUs <= SERVO_US_MIN) {
    servoPulseUs = SERVO_US_MIN;
    servoDir = +1;
  }

  servoWriteUs(servoPulseUs);
}

static inline void servoToggle() {
  servoSwingOn = !servoSwingOn;

  if (servoSwingOn) {
    servoPulseUs = SERVO_US_MIN;
    servoDir = +1;
    lastServoStepMs = millis();
    Serial.println("[SERVO] SWING ON");
  } else {
    servoWriteUs(SERVO_US_MIN);
    servoPulseUs = SERVO_US_MIN;
    servoDir = +1;
    Serial.println("[SERVO] SWING OFF -> 600us");
  }
}

/* ===================== OPTIONAL I2C packet count (JOYSTICK ONLY) ===================== */
#define ENABLE_JOYSTICK_I2C_COUNT 1

#if ENABLE_JOYSTICK_I2C_COUNT
#define I2C_SLAVE_ADDR 0x28
#define SDA_PIN 8
#define SCL_PIN 9

void send_I2C_u16(uint16_t packetCount) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(packetCount & 0xFF);
  Wire.write((packetCount >> 8) & 0xFF);
  uint8_t err = Wire.endTransmission();
  if (err != 0) Serial.printf("I2C Send Error: %d\n", err);
}

static uint32_t pktCount = 0;

int flushUdpCountEvery500ms() {
  static uint32_t windowStartMs = 0;
  uint32_t now = millis();
  if (windowStartMs == 0) windowStartMs = now;

  if ((uint32_t)(now - windowStartMs) >= 500) {
    uint32_t result = pktCount;
    pktCount = 0;
    do { windowStartMs += 500; }
    while ((uint32_t)(now - windowStartMs) >= 500);
    return (int)result;
  }
  return -1;
}
#endif

/* ===================== WALL FOLLOW (UART DIST) ===================== */
HardwareSerial SlaveSerial(1);
const int SLAVE_RX_PIN = 44;
const int SLAVE_TX_PIN = 43;

uint16_t rawRightBackCm  = 0, rawRightFrontCm = 0, rawFrontCm = 0;
uint16_t filtRightBackCm = 0, filtRightFrontCm = 0, filtFrontCm = 0;

// median-of-3
const int WF_FILTER_WINDOW = 3;
uint16_t rbHist[WF_FILTER_WINDOW]    = {0,0,0};
uint16_t rfHist[WF_FILTER_WINDOW]    = {0,0,0};
uint16_t frontHist[WF_FILTER_WINDOW] = {0,0,0};
int histIndex = 0; bool histFilled = false;

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}

void updateFilters() {
  rbHist[histIndex]    = rawRightBackCm;
  rfHist[histIndex]    = rawRightFrontCm;
  frontHist[histIndex] = rawFrontCm;

  histIndex = (histIndex + 1) % WF_FILTER_WINDOW;
  if (histIndex == 0) histFilled = true;

  if (histFilled) {
    filtRightBackCm  = median3(rbHist[0], rbHist[1], rbHist[2]);
    filtRightFrontCm = median3(rfHist[0], rfHist[1], rfHist[2]);
    filtFrontCm      = median3(frontHist[0], frontHist[1], frontHist[2]);
  } else {
    filtRightBackCm  = rawRightBackCm;
    filtRightFrontCm = rawRightFrontCm;
    filtFrontCm      = rawFrontCm;
  }
}

void handleSlaveUart() {
  while (SlaveSerial.available()) {
    String line = SlaveSerial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("DIST:")) {
      String payload = line.substring(5);
      int c1 = payload.indexOf(',');
      int c2 = payload.indexOf(',', c1 + 1);

      if (c1 > 0 && c2 > c1) {
        rawRightBackCm  = (uint16_t)payload.substring(0, c1).toInt();
        rawRightFrontCm = (uint16_t)payload.substring(c1 + 1, c2).toInt();
        rawFrontCm      = (uint16_t)payload.substring(c2 + 1).toInt();

        updateFilters();

        Serial.print("Raw RB: ");   Serial.print(rawRightBackCm);
        Serial.print("  Raw RF: "); Serial.print(rawRightFrontCm);
        Serial.print("  Raw F: ");  Serial.print(rawFrontCm);
        Serial.print("  |  Filt RB: "); Serial.print(filtRightBackCm);
        Serial.print("  Filt RF: ");    Serial.print(filtRightFrontCm);
        Serial.print("  Filt F: ");     Serial.println(filtFrontCm);
      } else {
        Serial.print("Malformed DIST line: ");
        Serial.println(line);
      }
    } else {
      Serial.print("Slave says: ");
      Serial.println(line);
    }
  }
}

static bool wallArmed = false;

static unsigned long lastWallStateMs = 0;
const unsigned long WALL_STATE_INTERVAL_MS = 1500;

int BASE_SPEED = 225;
const int TURN_SPEED_WF = 255;

const float RIGHT_TARGET_CM = 10.0f;
float RIGHT_BACK_OFFSET_CM = 3.0f;

const float WALL_OK_MIN_CM = 9.0f;
const float WALL_OK_MAX_CM = 11.0f;

const float ANGLE_STRAIGHT_MIN_CM = 3.0f;
const float ANGLE_STRAIGHT_MAX_CM = 6.0f;

const float HARD_RIGHT_ERR_CM = 5.0f;

const float EFFECTIVE_FRONT_THRESHOLD = 25.0f;
const int   FRONT_CLOSE_TICKS_REQUIRED = 3;
int frontCloseStreak = 0;

const unsigned long WALL_CONTROL_INTERVAL_MS = 10;
unsigned long lastWallControlTime = 0;

enum WallStartStatus { WS_UNKNOWN=0, WS_CANT_NO_RIGHT, WS_CANT_HARD_RIGHT_GAP, WS_GOOD };

WallStartStatus computeWallAdvisory(float* out_rbRaw=nullptr, float* out_rf=nullptr, float* out_mean=nullptr) {
  bool noRightWall = (filtRightBackCm == 0 && filtRightFrontCm == 0);
  if (noRightWall) {
    if (out_rbRaw) *out_rbRaw = 0.0f;
    if (out_rf)    *out_rf    = 0.0f;
    if (out_mean)  *out_mean  = 0.0f;
    return WS_CANT_NO_RIGHT;
  }

  float rbRaw = (float)filtRightBackCm;
  float rf    = (float)filtRightFrontCm;

  if (filtRightBackCm == 0 && filtRightFrontCm > 0) rbRaw = rf;
  if (filtRightFrontCm == 0 && filtRightBackCm > 0) rf    = rbRaw;

  float rbAdj = rbRaw + RIGHT_BACK_OFFSET_CM;
  float mean  = 0.5f * (rbAdj + rf);

  bool hardGap =
    (mean  > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
     rbAdj > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
     rf    > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM);

  if (out_rbRaw) *out_rbRaw = rbRaw;
  if (out_rf)    *out_rf    = rf;
  if (out_mean)  *out_mean  = mean;

  if (hardGap) return WS_CANT_HARD_RIGHT_GAP;
  return WS_GOOD;
}

void reportWallStateUdpAndSerial() {
  unsigned long now = millis();
  if (now - lastWallStateMs < WALL_STATE_INTERVAL_MS) return;
  lastWallStateMs = now;

  bool noRightWall = (filtRightBackCm == 0 && filtRightFrontCm == 0);

  float rbRaw = (float)filtRightBackCm;
  float rf    = (float)filtRightFrontCm;

  if (!noRightWall) {
    if (filtRightBackCm == 0 && filtRightFrontCm > 0) rbRaw = rf;
    if (filtRightFrontCm == 0 && filtRightBackCm > 0) rf    = rbRaw;
  }

  float rbAdj = rbRaw + RIGHT_BACK_OFFSET_CM;
  float mean  = 0.5f * (rbAdj + rf);
  float diff  = rbAdj - rf;

  bool inBand = (mean >= WALL_OK_MIN_CM && mean <= WALL_OK_MAX_CM);

  bool hardGap =
    (!noRightWall) &&
    (mean  > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
     rbAdj > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
     rf    > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM);

  if (noRightWall) hardGap = true;

  bool validFront    = (filtFrontCm > 8 && filtFrontCm < 150);
  bool frontCloseNow = (validFront && filtFrontCm < EFFECTIVE_FRONT_THRESHOLD);

  const char* reason = "GOOD_TO_START";
  if (noRightWall)    reason = "CANT_START_NO_RIGHT_WALL";
  else if (hardGap)   reason = "CANT_START_HARD_RIGHT_GAP";

  char msg[300];
  snprintf(msg, sizeof(msg),
    "STATE: %s | armed=%s | RawRB=%u RawRF=%u RawF=%u | FiltRB=%u FiltRF=%u FiltF=%u | rbAdj=%.1f mean=%.1f diff=%.1f inBand=%c | noRight=%c hardGap=%c frontClose=%c",
    reason,
    wallArmed ? "Y" : "N",
    rawRightBackCm, rawRightFrontCm, rawFrontCm,
    filtRightBackCm, filtRightFrontCm, filtFrontCm,
    rbAdj, mean, diff, inBand ? 'Y':'N',
    noRightWall ? 'Y':'N',
    hardGap  ? 'Y':'N',
    frontCloseNow ? 'Y':'N'
  );

  Serial.println(msg);

  if (haveClient) {
    udp.beginPacket(clientIP, clientPort);
    udp.print(msg);
    udp.print("\n");
    udp.endPacket();
  }
}

void previewWallDebug() {
  unsigned long now = millis();
  static unsigned long lastPreviewMs = 0;
  if (now - lastPreviewMs < 300) return;
  lastPreviewMs = now;

  if (filtRightBackCm == 0 && filtRightFrontCm == 0) {
    Serial.println("[WF PREVIEW] NO RIGHT WALL (RB=0 RF=0)");
    return;
  }

  float rbRaw = (float)filtRightBackCm;
  float rf    = (float)filtRightFrontCm;
  if (filtRightBackCm == 0 && filtRightFrontCm > 0) rbRaw = rf;
  if (filtRightFrontCm == 0 && filtRightBackCm > 0) rf    = rbRaw;

  float rbAdj = rbRaw + RIGHT_BACK_OFFSET_CM;
  float mean  = 0.5f * (rbAdj + rf);
  float diff  = rbAdj - rf;

  bool inBand = (mean >= WALL_OK_MIN_CM && mean <= WALL_OK_MAX_CM);

  bool hardGap =
    (mean  > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
     rbAdj > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
     rf    > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM);

  Serial.print("[WF PREVIEW] armed=");
  Serial.print(wallArmed ? "Y" : "N");
  Serial.print(" mean=");
  Serial.print(mean,1);
  Serial.print(" rbAdj=");
  Serial.print(rbAdj,1);
  Serial.print(" rf=");
  Serial.print(rf,1);
  Serial.print(" diff=");
  Serial.print(diff,1);
  Serial.print(" inBand=");
  Serial.print(inBand ? "Y" : "N");
  Serial.print(" hardGap=");
  Serial.println(hardGap ? "Y" : "N");
}

void controlStepWallFollow() {
  bool validFront    = (filtFrontCm > 8 && filtFrontCm < 150);
  bool frontCloseNow = (validFront && filtFrontCm < EFFECTIVE_FRONT_THRESHOLD);

  static uint16_t lastFrontForClose = 0;
  bool stableEnough = true;
  if (lastFrontForClose != 0) {
    int d = (int)filtFrontCm - (int)lastFrontForClose;
    if (abs(d) > 5) stableEnough = false;
  }
  lastFrontForClose = filtFrontCm;

  if (frontCloseNow && stableEnough) {
    if (frontCloseStreak < FRONT_CLOSE_TICKS_REQUIRED) frontCloseStreak++;
  } else {
    frontCloseStreak = 0;
  }

  if (frontCloseStreak >= FRONT_CLOSE_TICKS_REQUIRED) {
    Serial.print("TURN LEFT: front blocked, FiltF=");
    Serial.println(filtFrontCm);
    turnInPlaceLeft(TURN_SPEED_WF);
    return;
  }

  int leftSpeed  = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  if (filtRightBackCm > 0 || filtRightFrontCm > 0) {
    float rbRaw = (float)filtRightBackCm;
    float rf    = (float)filtRightFrontCm;

    if (filtRightBackCm == 0 && filtRightFrontCm > 0) rbRaw = rf;
    if (filtRightFrontCm == 0 && filtRightBackCm > 0) rf    = rbRaw;

    float rbAdj = rbRaw + RIGHT_BACK_OFFSET_CM;
    float mean  = 0.5f * (rbAdj + rf);
    float diff  = rbAdj - rf;

    bool inBand = (mean >= WALL_OK_MIN_CM && mean <= WALL_OK_MAX_CM);

    // HARD RIGHT GAP
    if (mean  > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
        rbAdj > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM &&
        rf    > RIGHT_TARGET_CM + HARD_RIGHT_ERR_CM) {
      
      leftSpeed  = min(255, BASE_SPEED + 60);
      rightSpeed = max(0,   BASE_SPEED - 130);

      Serial.print("HARD RIGHT GAP: mean=");
      Serial.print(mean);
      Serial.print(" rbAdj=");
      Serial.print(rbAdj);
      Serial.print(" RF=");
      Serial.println(rf);
    }
    else {
      float angleUse = diff;
      if (diff >= ANGLE_STRAIGHT_MIN_CM && diff <= ANGLE_STRAIGHT_MAX_CM) angleUse = 0.0f;

      float distErr = mean - RIGHT_TARGET_CM;
      float distUse = distErr;
      if (inBand) distUse = 0.0f;

      const float Kp_dist  = 15.0f;
      const float Kp_angle = 8.0f;

      float deltaCmd = (-Kp_dist * distUse) + (Kp_angle * angleUse);

      const float DELTA_CLAMP = 100.0f;
      if (deltaCmd >  DELTA_CLAMP) deltaCmd =  DELTA_CLAMP;
      if (deltaCmd < -DELTA_CLAMP) deltaCmd = -DELTA_CLAMP;

      static float deltaFilt = 0.0f;
      const float ALPHA = 0.25f;
      deltaFilt = (1.0f - ALPHA) * deltaFilt + ALPHA * deltaCmd;

      static float deltaOut = 0.0f;
      const float MAX_STEP = 12.0f;
      float step = deltaFilt - deltaOut;
      if (step >  MAX_STEP) step =  MAX_STEP;
      if (step < -MAX_STEP) step = -MAX_STEP;
      deltaOut += step;

      int deltaSpeed = (int)deltaOut;
      leftSpeed  = BASE_SPEED - deltaSpeed;
      rightSpeed = BASE_SPEED + deltaSpeed;

      Serial.print("rightMean:");
      Serial.print(mean);
      Serial.print(" diff(rbAdj-RF):");
      Serial.print(diff);
      Serial.print(" inBand:");
      Serial.print(inBand ? "Y" : "N");
      Serial.print(" distUse:");
      Serial.print(distUse);
      Serial.print(" angleUse:");
      Serial.print(angleUse);
      Serial.print(" deltaCmd:");
      Serial.print(deltaCmd);
      Serial.print(" deltaOut:");
      Serial.println(deltaSpeed);
    }
  } else {
    Serial.println("No right readings -> straight");
  }

  forwardMove(constrain(leftSpeed,0,255), constrain(rightSpeed,0,255));
}

/* ===================== VIVE MODE ===================== */
#define SIGNALPIN1 12
#define SIGNALPIN2 14

const float CAR_HEIGHT = 0.0f;

const float MAX_JUMP_POS = 500000.0f;
const float MAX_JUMP_HDG = 50000.0f;
const float ALPHA_POS    = 0.15f;
const float ALPHA_HDG    = 0.15f;

// TOF-based final alignment
bool needTofAlignment = false;
const float TOF_TARGET_DISTANCE = 20.0f;  // Stop when TOF reads under 20cm

MedianFilter x1Filter = {{0},0,0};
MedianFilter y1Filter = {{0},0,0};
MedianFilter x2Filter = {{0},0,0};
MedianFilter y2Filter = {{0},0,0};

int applyMedian(MedianFilter &f, int newValue) {
  f.buf[f.index] = newValue;
  f.index = (f.index + 1) % VIVE_WINDOW_SIZE;
  if (f.count < VIVE_WINDOW_SIZE) f.count++;

  int temp[VIVE_WINDOW_SIZE];
  for (int i = 0; i < f.count; i++) temp[i] = f.buf[i];

  for (int i = 0; i < f.count - 1; i++) {
    for (int j = 0; j < f.count - 1 - i; j++) {
      if (temp[j] > temp[j + 1]) {
        int t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
    }
  }
  return temp[f.count / 2];
}

Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

bool  carCenterValid = false;
float carX = 0.0f, carY = 0.0f, headingDeg = 0.0f;

bool  haveFiltered = false;
float carX_f = 0.0f, carY_f = 0.0f;
float hX_f = 1.0f, hY_f = 0.0f;

const unsigned long VIVE_UPDATE_INTERVAL_MS = 50;
unsigned long lastViveUpdateTime = 0;

float targetX = 0.0f, targetY = 0.0f;
bool  targetValid = false;

const int   DRIVE_SPEED_VIVE = 255;
const int   ROTATE_SPEED_MAX = 255;
const int   ROTATE_SPEED_MIN = 120;
const float HEADING_TOL_DEG  = 3.0f;
const float POSITION_TOL     = 100.0f;

const unsigned long VIVE_CONTROL_INTERVAL_MS = 50;
unsigned long lastViveControlTime = 0;

enum MoveState { STATE_IDLE=0, STATE_ROTATE, STATE_DRIVE };
MoveState moveState = STATE_IDLE;

// ====== HARD-CODED ATTACK SEQUENCES (NO VIVE) ======

// wrappers for your attack functions
static inline void forward(int speed) { forwardMove(speed, speed); }
static inline void backward(int speed) { setLeftDir(true); setRightDir(true); setSpeed(speed, speed); }
static inline void turnLeft(int speed){ turnInPlaceLeft(speed); }
static inline void turnRight(int speed){ turnInPlaceRight(speed); }

const int DRIVE_SPEED = 200;
const int TURN_SPEED  = 200;

void attack_low_tower(){
  Serial.println("[ATTACK] Low Tower - Starting");
  
  forward(DRIVE_SPEED);
  delay(12800);
  turnLeft(TURN_SPEED);
  delay(2800);
  forward(DRIVE_SPEED);
  delay(1700);
  turnLeft(TURN_SPEED);
  delay(2800);
  forward(DRIVE_SPEED);
  delay(2300);
  stopCar();

  Serial.println("[ATTACK] Low Tower - Complete");
}

void attack_nexus(){
  Serial.println("[ATTACK] Nexus - Starting");
  
  forward(DRIVE_SPEED);
  delay(10000);
  forward(DRIVE_SPEED);
  delay(500);
  backward(DRIVE_SPEED);
  delay(500);
  forward(DRIVE_SPEED);
  delay(700);
  backward(DRIVE_SPEED);
  delay(500);
  forward(DRIVE_SPEED);
  delay(700);
  backward(DRIVE_SPEED);
  delay(500);
  forward(DRIVE_SPEED);
  delay(700);
  backward(DRIVE_SPEED);
  delay(500);
  stopCar();

  Serial.println("[ATTACK] Nexus - Complete");
}

void attack_high_tower(){
  Serial.println("[ATTACK] High Tower - Starting");
  
  forward(DRIVE_SPEED);
  delay(15400);
  turnRight(TURN_SPEED);
  delay(2700);
  forward(DRIVE_SPEED);
  delay(3000);
  stopCar();

  Serial.println("[ATTACK] High Tower - Complete");
}

// Multi-target queue system
float targetQueue[2][2];  // [target_index][x,y]
int targetQueueCount = 0;
int currentTargetIndex = 0;

void checkAndLoadNextTarget() {
  if (targetQueueCount > 0 && currentTargetIndex < targetQueueCount) {
    Serial.print("[QUEUE] Loading next target ");
    Serial.print(currentTargetIndex + 1);
    Serial.print(" of ");
    Serial.println(targetQueueCount);
    
    targetX = targetQueue[currentTargetIndex][0];
    targetY = targetQueue[currentTargetIndex][1];
    targetValid = true;
    moveState = carCenterValid ? STATE_ROTATE : STATE_IDLE;
    
    currentTargetIndex++;
    
    if (currentTargetIndex >= targetQueueCount) {
      Serial.println("[QUEUE] All targets completed");
      targetQueueCount = 0;
      currentTargetIndex = 0;
    }
  }
}

float wrapAngleDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void updateFilteredPose(float carX_raw, float carY_raw, float headingDeg_raw){
  float headingRad = headingDeg_raw * PI / 180.0f;
  float hx_raw = cosf(headingRad);
  float hy_raw = sinf(headingRad);

  if(!haveFiltered){
    carX_f = carX_raw;
    carY_f = carY_raw;
    hX_f   = hx_raw;
    hY_f   = hy_raw;
    haveFiltered = true;
    return;
  }

  float dx = carX_raw - carX_f;
  float dy = carY_raw - carY_f;
  float posJump = sqrtf(dx*dx + dy*dy);

  float dot = hX_f*hx_raw + hY_f*hy_raw;
  dot = constrain(dot, -1.0f, 1.0f);
  float hdgJump = acosf(dot) * 180.0f / PI;

  if(posJump > MAX_JUMP_POS || hdgJump > MAX_JUMP_HDG) return;

  carX_f = (1.0f - ALPHA_POS)*carX_f + ALPHA_POS*carX_raw;
  carY_f = (1.0f - ALPHA_POS)*carY_f + ALPHA_POS*carY_raw;

  hX_f = (1.0f - ALPHA_HDG)*hX_f + ALPHA_HDG*hx_raw;
  hY_f = (1.0f - ALPHA_HDG)*hY_f + ALPHA_HDG*hy_raw;

  float norm = sqrtf(hX_f*hX_f + hY_f*hY_f);
  if(norm > 1e-6f){ hX_f /= norm; hY_f /= norm; }
}

float getFilteredHeadingDeg(){
  float ang = atan2f(hY_f, hX_f) * 180.0f / PI;
  if(ang < 0.0f) ang += 360.0f;
  return ang;
}

void updateVivePose() {
  unsigned long now = millis();
  if (now - lastViveUpdateTime < VIVE_UPDATE_INTERVAL_MS) return;
  lastViveUpdateTime = now;

  bool recv1=false, recv2=false;
  int x1=0,y1=0,x2=0,y2=0;

  if (vive1.status() == VIVE_RECEIVING) {
    x1 = applyMedian(x1Filter, vive1.xCoord());
    y1 = applyMedian(y1Filter, vive1.yCoord());
    recv1 = true;
  } else {
    vive1.sync(5);
  }

  if (vive2.status() == VIVE_RECEIVING) {
    x2 = applyMedian(x2Filter, vive2.xCoord());
    y2 = applyMedian(y2Filter, vive2.yCoord());
    recv2 = true;
  } else {
    vive2.sync(5);
  }

  carCenterValid = false;

  if (recv1 && recv2) {
    float dx = (float)x2 - (float)x1;
    float dy = (float)y2 - (float)y1;
    float distance = sqrtf(dx*dx + dy*dy);
    if (distance <= 0.0f) return;

    float mx = ((float)x1 + (float)x2) * 0.5f;
    float my = ((float)y1 + (float)y2) * 0.5f;

    float nx = -dy / distance;
    float ny =  dx / distance;

    float carX_raw = mx + CAR_HEIGHT * nx;
    float carY_raw = my + CAR_HEIGHT * ny;

    float headingRad = atan2f(ny, nx);
    float headingDeg_raw = headingRad * 180.0f / PI;
    if (headingDeg_raw < 0.0f) headingDeg_raw += 360.0f;

    updateFilteredPose(carX_raw, carY_raw, headingDeg_raw);

    if (haveFiltered) {
      carX = carX_f;
      carY = carY_f;
      headingDeg = getFilteredHeadingDeg();
      carCenterValid = true;

      Serial.print("[VIVE] CarCenter: (");
      Serial.print(carX, 2);
      Serial.print(", ");
      Serial.print(carY, 2);
      Serial.print(")  Heading: ");
      Serial.print(headingDeg, 2);
      Serial.println(" deg");
    }
  }
}

void parseTargetLine(const String &line) {
  String payload = line.substring(7);
  payload.trim();
  int c = payload.indexOf(',');
  if (c < 0) {
    Serial.print("[TARGET] Bad format: ");
    Serial.println(line);
    return;
  }
  targetX = payload.substring(0, c).toFloat();
  targetY = payload.substring(c + 1).toFloat();
  targetValid = true;

  moveState = carCenterValid ? STATE_ROTATE : STATE_IDLE;

  Serial.print("[TARGET] New target: (");
  Serial.print(targetX, 3);
  Serial.print(", ");
  Serial.print(targetY, 3);
  Serial.println(")");
}

int calcRotateSpeedFromError(float headingErrorDeg) {
  float err = fabs(headingErrorDeg);
  const float ERR_MAX_DEG = 90.0f;

  if (err >= ERR_MAX_DEG) return ROTATE_SPEED_MAX;
  if (err <= HEADING_TOL_DEG) return ROTATE_SPEED_MIN;

  float k = (ROTATE_SPEED_MAX - ROTATE_SPEED_MIN) / (ERR_MAX_DEG - HEADING_TOL_DEG);
  int speed = (int)(ROTATE_SPEED_MIN + (err - HEADING_TOL_DEG) * k);
  return constrain(speed, ROTATE_SPEED_MIN, ROTATE_SPEED_MAX);
}

void controlStepVive() {
  if (!carCenterValid || !targetValid) {
    if (moveState != STATE_IDLE) Serial.println("[STATE] IDLE (no pose/target)");
    moveState = STATE_IDLE;
    stopCar();
    return;
  }

  float dx = targetX - carX;
  float dy = targetY - carY;
  float dist = sqrtf(dx*dx + dy*dy);

  // CHECK IF POSITION REACHED
  if (dist < POSITION_TOL) {
    Serial.print("[POSITION REACHED] dist=");
    Serial.println(dist, 3);
    
    // Position is good, stop moving
    stopCar();
    targetValid = false;
    moveState = STATE_IDLE;

    checkAndLoadNextTarget();  // ← ADD THIS LINE
    
    // NOW check if we need TOF alignment (separate from position navigation)
    if (needTofAlignment) {
      Serial.println("[STARTING TOF ALIGNMENT]");
      needTofAlignment = false;  // Clear flag
      
      // Start a simple TOF alignment loop
      unsigned long tofStartTime = millis();
      const unsigned long TOF_TIMEOUT = 10000;  // 10 second timeout
      
      while (millis() - tofStartTime < TOF_TIMEOUT) {
        handleSlaveUart();  // Update TOF readings
        
        bool validFront = (filtFrontCm > 8 && filtFrontCm < 150);
        
        Serial.print("[TOF] Front=");
        Serial.print(filtFrontCm);
        Serial.println(" cm");
        
        if (validFront && filtFrontCm < TOF_TARGET_DISTANCE) {
          Serial.print("[TOF ALIGNED] Found tower at ");
          Serial.print(filtFrontCm);
          Serial.println(" cm");
          stopCar();
          break;
        }
        
        // Keep turning right
        turnInPlaceRight(150);
        delay(50);
      }
      
      stopCar();
      Serial.println("[TOF ALIGNMENT COMPLETE]");
      
      // NOW DRIVE FORWARD TO HIT THE TOWER
      Serial.println("[DRIVING TO TOWER]");
      forward(200);  // Drive forward at medium speed
      delay(3000);   // Drive for 3 seconds (adjust this time as needed)
      stopCar();
      Serial.println("[TOWER HIT COMPLETE]");
    }
    
    return;
  }

  // NOT AT POSITION YET - do normal navigation
  float targetHeadingRad = atan2f(dy, dx);
  float targetHeadingDeg = targetHeadingRad * 180.0f / PI;
  float headingError = wrapAngleDeg(targetHeadingDeg - headingDeg);

  Serial.print("Target angle ");
  Serial.println(targetHeadingDeg);

  switch (moveState) {
    case STATE_IDLE:
      Serial.println("[STATE] IDLE -> ROTATE");
      moveState = STATE_ROTATE;
      stopCar();
      break;

    case STATE_ROTATE: {
      Serial.print("[STATE] ROTATE  headingErr=");
      Serial.print(headingError, 2);
      Serial.print(" deg, dist=");
      Serial.println(dist, 3);

      if (fabs(headingError) > HEADING_TOL_DEG) {
        int rs = calcRotateSpeedFromError(headingError);
        if (headingError > 0) turnInPlaceLeft(rs);
        else turnInPlaceRight(rs);
      } else {
        Serial.println("[STATE] ROTATE -> DRIVE");
        moveState = STATE_DRIVE;
        stopCar();
      }
    } break;

    case STATE_DRIVE: {
      Serial.print("[STATE] DRIVE  headingErr=");
      Serial.print(headingError, 2);
      Serial.print(" deg, dist=");
      Serial.println(dist, 3);

      // Current error absolute value
      float absErr = fabs(headingError);

      // The closer you are, the larger the allowed error
      float headingErrLimit;
      if (dist > 600.0f) {
        headingErrLimit = 18.0f;       // Far: need to be accurate
      } else if (dist <= 100.0f) {
        headingErrLimit = 45.0f;       // Very close: allow steep angles
      } else {
        // 600 -> 100 linearly from 18° to 45°
        float t = (600.0f - dist) / (600.0f - 100.0f);  // dist=600 -> t=0, dist=100 -> t=1
        headingErrLimit = 18.0f + t * (45.0f - 18.0f);
      }

      if (absErr > headingErrLimit) {
        Serial.print("[STATE] DRIVE -> ROTATE (heading too off, limit=)");
        Serial.println(headingErrLimit, 2);
        moveState = STATE_ROTATE;
        stopCar();
      } else {
        // Angle within allowed range, keep going straight
        forwardMove(DRIVE_SPEED_VIVE, DRIVE_SPEED_VIVE);
      }
    } break;
  }
}

/* ===================== Mode switching ===================== */
void setMode(Mode m) {
  mode = m;
  stopCar();

  // reset wall-follow
  wallArmed = false;
  frontCloseStreak = 0;
  lastWallStateMs = 0;

  // reset vive
  targetValid = false;
  moveState = STATE_IDLE;

  if (mode == MODE_WALLFOLLOW) sendToClient("MODE: WALL_FOLLOW\n");
  if (mode == MODE_JOYSTICK)   sendToClient("MODE: JOYSTICK\n");
  if (mode == MODE_VIVE)       sendToClient("MODE: VIVE (send TARGET:x,y)\n");
}

/* ===================== JOYSTICK cmd handler ===================== */
void handleCommandJoystick(char cmd) {
  cmd = toupper((unsigned char)cmd);

  Serial.printf("Parsed CMD: '%c'\n", cmd);

  switch (cmd) {
    case 'F': Serial.println("Action: FORWARD");  forwardJS(DRIVE_SPEED_JS); break;
    case 'B': Serial.println("Action: BACKWARD"); backwardJS(DRIVE_SPEED_JS); break;
    case 'L': Serial.println("Action: LEFT");     turnLeftJS(TURN_SPEED_JS); break;
    case 'R': Serial.println("Action: RIGHT");    turnRightJS(TURN_SPEED_JS); break;
    case 'S':
    default:  Serial.println("Action: STOP");     stopCar(); break;
  }
}

/* ===================== UDP handler ===================== */
void handleUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  while (packetSize > 0) {
#if ENABLE_JOYSTICK_I2C_COUNT
    if (mode == MODE_JOYSTICK) pktCount++;
#endif
    lastRxMs = millis();

    IPAddress rip = udp.remoteIP();
    uint16_t rport = udp.remotePort();

    clientIP = rip;
    clientPort = rport;
    haveClient = true;

    char buffer[240];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    if (len < 0) len = 0;
    buffer[len] = '\0';

    String msg = String(buffer);
    msg.trim();
    if (msg.length() == 0) { packetSize = udp.parsePacket(); continue; }

    bool verboseUdp = (mode == MODE_JOYSTICK);

    if (verboseUdp) {
      Serial.printf("[UDP RX] %lu ms | from %s:%u | size=%d | raw='%s'\n",
                    (unsigned long)millis(),
                    rip.toString().c_str(),
                    rport,
                    packetSize,
                    msg.c_str());
    }

    if (mode == MODE_VIVE && msg.startsWith("TARGET:")) {
      Serial.print("[UDP RX] TARGET raw='");
      Serial.print(msg);
      Serial.println("'");
      parseTargetLine(msg);
      packetSize = udp.parsePacket();
      continue;
    }

    // Add this in handleUdp() after the existing TARGET: handler
    if (mode == MODE_VIVE && msg.startsWith("TARGET2:")) {
      Serial.print("[UDP RX] TARGET2 raw='");
      Serial.print(msg);
      Serial.println("'");
      
      String payload = msg.substring(8);
      payload.trim();
      
      int c1 = payload.indexOf(',');
      int c2 = payload.indexOf(',', c1 + 1);
      int c3 = payload.indexOf(',', c2 + 1);
      
      if (c1 > 0 && c2 > c1 && c3 > c2) {
        float x1 = payload.substring(0, c1).toFloat();
        float y1 = payload.substring(c1 + 1, c2).toFloat();
        float x2 = payload.substring(c2 + 1, c3).toFloat();
        float y2 = payload.substring(c3 + 1).toFloat();
        
        Serial.print("[TARGET2] Queuing: (");
        Serial.print(x1, 3);
        Serial.print(", ");
        Serial.print(y1, 3);
        Serial.print(") -> (");
        Serial.print(x2, 3);
        Serial.print(", ");
        Serial.print(y2, 3);
        Serial.println(")");
        
        targetQueue[0][0] = x1;
        targetQueue[0][1] = y1;
        targetQueue[1][0] = x2;
        targetQueue[1][1] = y2;
        targetQueueCount = 2;
        currentTargetIndex = 0;
        
        checkAndLoadNextTarget();
      }
      
      packetSize = udp.parsePacket();
      continue;
    }

    if (mode == MODE_VIVE && msg == "AL") {
      Serial.println("[UDP RX] AL -> attack_low_tower()");
      attack_low_tower();
      packetSize = udp.parsePacket();
      continue;
    }

    if (mode == MODE_VIVE && msg == "AH") {
      Serial.println("[UDP RX] AH -> attack_high_tower()");
      attack_high_tower();
      packetSize = udp.parsePacket();
      continue;
    }

    if (mode == MODE_VIVE && msg == "AN") {
      Serial.println("[UDP RX] AN -> attack_nexus()");
      attack_nexus();
      packetSize = udp.parsePacket();
      continue;
    }

    char c0 = toupper((unsigned char)msg.charAt(0));
    lastCmd = c0;

    if (c0 == 'J') {
      setMode(MODE_JOYSTICK);
    } else if (c0 == 'W') {
      setMode(MODE_WALLFOLLOW);
    } else if (c0 == 'V') {
      setMode(MODE_VIVE);
    } else if (c0 == 'G') {
      if (mode == MODE_WALLFOLLOW) {
        wallArmed = true;
        sendToClient("STARTING (STATE ADVISORY ONLY)\n");
        Serial.println("[WALL] START armed by RB (G).");
      }
    } else if (c0 == 'T') {
      if (mode == MODE_JOYSTICK) servoToggle();
    } else if (c0 == 'S') {
      stopCar();
      if (verboseUdp) Serial.println("Action: STOP");
    } else {
      if (mode == MODE_JOYSTICK) handleCommandJoystick(c0);
    }

    packetSize = udp.parsePacket();
  }
}

/* ===================== Setup ===================== */
void setup() {
  Serial.begin(115200);

#if ENABLE_JOYSTICK_I2C_COUNT
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(40000);
  delay(50);
#endif

  // motor pins
  pinMode(LF_IN1, OUTPUT); pinMode(LF_IN2, OUTPUT);
  pinMode(LB_IN1, OUTPUT); pinMode(LB_IN2, OUTPUT);
  pinMode(RF_IN1, OUTPUT); pinMode(RF_IN2, OUTPUT);
  pinMode(RB_IN1, OUTPUT); pinMode(RB_IN2, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  stopCar();

  servoInit();

  SlaveSerial.begin(115200, SERIAL_8N1, SLAVE_RX_PIN, SLAVE_TX_PIN);
  SlaveSerial.setTimeout(10);
  Serial.println("UART to slave started (RX=44, TX=43).");

  vive1.begin();
  vive2.begin();
  lastViveUpdateTime = millis();
  Serial.println("Vive trackers begin().");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID);

  Serial.print("AP Started — SSID: ");
  Serial.println(AP_SSID);
  Serial.print("ESP32 AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);
  Serial.print("UDP listening on port ");
  Serial.println(UDP_PORT);

  setMode(MODE_JOYSTICK);

  lastWallControlTime = millis();
  lastViveControlTime = millis();
}

/* ===================== Loop ===================== */
void loop() {
  handleUdp();

  // ALWAYS update TOF sensors from slave UART
  handleSlaveUart();

  if (mode == MODE_JOYSTICK) {
    servoServiceSwing();
  }

#if ENABLE_JOYSTICK_I2C_COUNT
  if (mode == MODE_JOYSTICK) {
    int cnt = flushUdpCountEvery500ms();
    if (cnt >= 0) {
      Serial.print("UDP packets in last 0.5s: ");
      Serial.println(cnt);
      send_I2C_u16((uint16_t)constrain(cnt, 0, 65535));
      delay(1);
    }
  } else {
    flushUdpCountEvery500ms();
  }
#endif

  if (mode == MODE_WALLFOLLOW) {
    reportWallStateUdpAndSerial();
    previewWallDebug();

    if (!wallArmed) {
      stopCar();
    } else {
      unsigned long now = millis();
      if (now - lastWallControlTime >= WALL_CONTROL_INTERVAL_MS) {
        lastWallControlTime = now;
        controlStepWallFollow();
      }
    }
  }

  if (mode == MODE_VIVE) {
    updateVivePose();
    unsigned long now = millis();
    if (now - lastViveControlTime >= VIVE_CONTROL_INTERVAL_MS) {
      lastViveControlTime = now;
      controlStepVive();
    }
  }

  delay(5);
}