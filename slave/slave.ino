/*************************************************
 * M5Stamp C3 SLAVE
 *  - RB ultrasonic: TRIG=4  ECHO=5
 *  - RF ultrasonic: TRIG=8  ECHO=10
 *  - Front ToF: VL53L0X over I2C (SDA=7, SCL=6)
 *
 * UART output:
 *   DIST:<RB_cm>,<RF_cm>,<F_cm>\n
 *
 * Key fix:
 *  - ToF uses a DIFFERENT stabilizer (no ultrasonic "tiny spike reject")
 *************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

HardwareSerial MasterSerial(0);

// ---------- Ultrasonic Pins ----------
const int TRIG_RB = 4;
const int ECHO_RB = 5;

const int TRIG_RF = 8;
const int ECHO_RF = 10;

// ---------- I2C (ToF) ----------
const int I2C_SDA = 7;
const int I2C_SCL = 6;

// ---------- UART ----------
const int UART_RX_PIN = 20;
const int UART_TX_PIN = 21;

// ---------- Timing ----------
const unsigned long MEAS_INTERVAL_MS = 70;     // ~14 Hz
const unsigned long PULSE_TIMEOUT_US = 10000;  // ultrasonic timeout //was 20000 ///////////////////////////////////////////
const int BETWEEN_SENSORS_MS = 5; //////////////////////////////////////////was 35////////////////////////////////////////

// ---------- Ranges ----------
const uint16_t MIN_VALID_CM = 2;
const uint16_t MAX_VALID_CM = 200;

// ---------- Miss handling ----------
const uint8_t MISS_LIMIT = 1;

// ---------- ToF ----------
Adafruit_VL53L0X tof;

// ---------- Ultrasonic helpers ----------
uint16_t pingOnceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long dur = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) return 0;

  uint16_t cm = (uint16_t)(dur / 58);
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return 0;
  return cm;
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}

uint16_t readUltrasonicRobust(int trigPin, int echoPin) { ////////////////////////////////////changed to diff func///////////
  uint16_t a = pingOnceCm(trigPin, echoPin);
  delay(3);
  uint16_t b = pingOnceCm(trigPin, echoPin);
  if (a == 0) return b;
  if (b == 0) return a;
  return (a + b) / 2;
}

// ---------- ToF read (cm) ----------
uint16_t readToFCm() {
  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);

  if (m.RangeStatus != 0) return 0;

  uint16_t cm = (uint16_t)(m.RangeMilliMeter / 10);
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return 0;
  return cm;
}

// ---------- Stabilize for ULTRASONIC (keeps your spike reject) ----------
uint16_t stabilizeUltrasonic(uint16_t newV,
                             uint16_t &lastV,
                             uint8_t &missCount) {
  if (newV == 0) {
    if (missCount < MISS_LIMIT) {
      missCount++;
      return lastV;
    } else {
      lastV = 0;
      return 0;
    }
  }

  missCount = 0;

  // ultrasonic-only: reject sudden tiny spikes
  if (lastV != 0 && newV <= 10 && lastV >= 20) {
    return lastV;
  }

  lastV = newV;
  return newV;
}

// ---------- Stabilize for ToF (NO spike reject) ----------
uint16_t stabilizeToF(uint16_t newV,
                      uint16_t &lastV,
                      uint8_t &missCount) {
  if (newV == 0) {
    if (missCount < MISS_LIMIT) {
      missCount++;
      return lastV;   // short hold is ok
    } else {
      lastV = 0;
      return 0;
    }
  }

  missCount = 0;

  // NO ultrasonic-style spike reject here.
  lastV = newV;
  return newV;
}

// last-good + miss counters
uint16_t lastRB = 0, lastRF = 0, lastF = 0;
uint8_t missRB = 0, missRF = 0, missF = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG_RB, OUTPUT); pinMode(ECHO_RB, INPUT);
  pinMode(TRIG_RF, OUTPUT); pinMode(ECHO_RF, INPUT);
  digitalWrite(TRIG_RB, LOW);
  digitalWrite(TRIG_RF, LOW);

  MasterSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  MasterSerial.setTimeout(10);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); //was 100000 //////////////////////////////////////////////////////////////////////

  if (!tof.begin()) {
    Serial.println("VL53L0X NOT FOUND (check SDA=7 SCL=6, VIN=3.3V, GND)");
  } else {
    Serial.println("VL53L0X OK");
    //tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  }
}

void loop() {
  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT < MEAS_INTERVAL_MS) return;
  lastT = now;

  uint16_t rbNew = readUltrasonicRobust(TRIG_RB, ECHO_RB);
  uint16_t rb    = stabilizeUltrasonic(rbNew, lastRB, missRB);
  delay(BETWEEN_SENSORS_MS);

  uint16_t rfNew = readUltrasonicRobust(TRIG_RF, ECHO_RF);
  uint16_t rf    = stabilizeUltrasonic(rfNew, lastRF, missRF);
  delay(BETWEEN_SENSORS_MS);

  uint16_t fNew  = readToFCm();
  uint16_t f     = stabilizeToF(fNew, lastF, missF);

  MasterSerial.printf("DIST:%u,%u,%u\n", rb, rf, f);
}
