/*
  Muhammad Edgar Caesarryza
  NIM: 26583061PA24470

  Studi Kasus Magang Hari 3 - Servo Pengikut Sumbu dan Deteksi Gerakan
  Simulasi Wokwi menggunakan MPU6050 sebagai pengganti BNO055.

  Roll dan pitch dihitung dari akselerometer. Yaw diperkirakan dengan
  mengintegrasikan laju giroskop sumbu Z. MPU6050 tidak memiliki magnetometer,
  sehingga estimasi yaw dapat mengalami penyimpangan seiring waktu.

  Saat mulai, letakkan MPU6050 rata dan diam. Sketch mengkalibrasi titik nol
  roll/pitch serta bias giroskop sumbu Z. Servo 1 dan Servo 2 menerima perintah
  roll yang sama; pasang keduanya secara bercermin jika arah gerak fisiknya
  harus berlawanan. PIR mengambil alih sementara: semua servo bergerak ke pose
  khusus, kembali ke posisi awal, lalu mengikuti sensor lagi.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>
#include <math.h>

#define SERVO1_PIN 25
#define SERVO2_PIN 26
#define SERVO3_PIN 27
#define SERVO4_PIN 14
#define SERVO5_PIN 32
#define PIR_PIN 33

#define SDA_PIN 21
#define SCL_PIN 22

const float SERVO_ANGLE_LIMIT = 90.0f;
const float YAW_RATE_THRESHOLD_RAD_S = 0.04f;
const float RAD_TO_DEGREES = 57.2957795f;
const float TEST_ANGLE_TOLERANCE_DEG = 1.5f;
const unsigned long SENSOR_PERIOD_MS = 20;
const unsigned long DEBUG_PERIOD_MS = 250;
const unsigned long YAW_HOME_DELAY_MS = 1000;
const unsigned long PIR_CUSTOM_DURATION_MS = 900;
const unsigned long PIR_RETURN_DURATION_MS = 900;
const unsigned int CALIBRATION_SAMPLES = 200;
const unsigned long CALIBRATION_SAMPLE_DELAY_MS = 5;

Adafruit_MPU6050 mpu;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

float rollOffset = 0.0f;
float pitchOffset = 0.0f;
float gyroZBias = 0.0f;
float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;
float yawRateZ = 0.0f;

unsigned long lastSensorTimeMs = 0;
unsigned long lastDebugTimeMs = 0;
unsigned long yawStopTimeMs = 0;
unsigned long pirSequenceStartMs = 0;

bool pirInputWasHigh = false;
bool caseTestPrinted = false;

enum YawState
{
  YAW_IDLE,
  YAW_MOVING,
  YAW_WAITING_TO_HOME
};

enum PirState
{
  PIR_IDLE,
  PIR_CUSTOM_POSE,
  PIR_RETURN_HOME
};

YawState yawState = YAW_IDLE;
PirState pirState = PIR_IDLE;

float clampAngle(float angle)
{
  return constrain(angle, -SERVO_ANGLE_LIMIT, SERVO_ANGLE_LIMIT);
}

int angleToPwmMicroseconds(float angle)
{
  const float limitedAngle = clampAngle(angle);
  const float pulseWidth =
      1000.0f + ((limitedAngle + 90.0f) * (1000.0f / 180.0f));
  return constrain((int)lroundf(pulseWidth), 1000, 2000);
}

void setServoToAngle(Servo &servo, float angle)
{
  servo.writeMicroseconds(angleToPwmMicroseconds(angle));
}

void attachServos()
{
  servo1.attach(SERVO1_PIN, 1000, 2000);
  servo2.attach(SERVO2_PIN, 1000, 2000);
  servo3.attach(SERVO3_PIN, 1000, 2000);
  servo4.attach(SERVO4_PIN, 1000, 2000);
  servo5.attach(SERVO5_PIN, 1000, 2000);
}

void homeAllServos()
{
  setServoToAngle(servo1, 0.0f);
  setServoToAngle(servo2, 0.0f);
  setServoToAngle(servo3, 0.0f);
  setServoToAngle(servo4, 0.0f);
  setServoToAngle(servo5, 0.0f);
}

void setCustomPirPose()
{
  setServoToAngle(servo1, 45.0f);
  setServoToAngle(servo2, -45.0f);
  setServoToAngle(servo3, 30.0f);
  setServoToAngle(servo4, -30.0f);
  setServoToAngle(servo5, 60.0f);
}

float calculateRoll(float ay, float az)
{
  return atan2f(ay, az) * RAD_TO_DEGREES;
}

float calculatePitch(float ax, float ay, float az)
{
  return atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEGREES;
}

void calibrateMpu6050()
{
  float rollTotal = 0.0f;
  float pitchTotal = 0.0f;
  float gyroZTotal = 0.0f;

  Serial.println("Kalibrasi: jaga MPU6050 tetap rata dan diam.");

  for (unsigned int sample = 0; sample < CALIBRATION_SAMPLES; ++sample)
  {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&accel, &gyro, &temperature);

    rollTotal += calculateRoll(
        accel.acceleration.y,
        accel.acceleration.z);
    pitchTotal += calculatePitch(
        accel.acceleration.x,
        accel.acceleration.y,
        accel.acceleration.z);
    gyroZTotal += gyro.gyro.z;

    delay(CALIBRATION_SAMPLE_DELAY_MS);
  }

  rollOffset = rollTotal / CALIBRATION_SAMPLES;
  pitchOffset = pitchTotal / CALIBRATION_SAMPLES;
  gyroZBias = gyroZTotal / CALIBRATION_SAMPLES;

  Serial.println("Kalibrasi selesai.");
}

void readMpu6050(float deltaTimeSeconds)
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(&accel, &gyro, &temperature);

  const float ax = accel.acceleration.x;
  const float ay = accel.acceleration.y;
  const float az = accel.acceleration.z;

  currentRoll = clampAngle(calculateRoll(ay, az) - rollOffset);
  currentPitch = clampAngle(calculatePitch(ax, ay, az) - pitchOffset);
  yawRateZ = gyro.gyro.z - gyroZBias;

  if (fabsf(yawRateZ) < YAW_RATE_THRESHOLD_RAD_S)
  {
    yawRateZ = 0.0f;
  }

  if (pirState == PIR_IDLE && yawRateZ != 0.0f)
  {
    currentYaw = clampAngle(
        currentYaw + yawRateZ * RAD_TO_DEGREES * deltaTimeSeconds);
    yawState = YAW_MOVING;
    yawStopTimeMs = millis();
  }
  else if (pirState == PIR_IDLE && yawState == YAW_MOVING)
  {
    yawState = YAW_WAITING_TO_HOME;
    yawStopTimeMs = millis();
  }
  else if (pirState == PIR_IDLE &&
           yawState == YAW_WAITING_TO_HOME &&
           millis() - yawStopTimeMs >= YAW_HOME_DELAY_MS)
  {
    currentYaw = 0.0f;
    yawState = YAW_IDLE;
  }
}

void handlePirTrigger(unsigned long nowMs)
{
  const bool pirIsHigh = digitalRead(PIR_PIN) == HIGH;

  if (pirIsHigh && !pirInputWasHigh && pirState == PIR_IDLE)
  {
    pirState = PIR_CUSTOM_POSE;
    pirSequenceStartMs = nowMs;
    currentYaw = 0.0f;
    yawState = YAW_IDLE;
    Serial.println("Gerakan terdeteksi: menjalankan pose khusus, lalu kembali ke posisi awal.");
  }

  pirInputWasHigh = pirIsHigh;
}

void updatePirSequence(unsigned long nowMs)
{
  if (pirState == PIR_CUSTOM_POSE)
  {
    if (nowMs - pirSequenceStartMs < PIR_CUSTOM_DURATION_MS)
    {
      setCustomPirPose();
    }
    else
    {
      pirState = PIR_RETURN_HOME;
      pirSequenceStartMs = nowMs;
      homeAllServos();
    }
  }
  else if (pirState == PIR_RETURN_HOME)
  {
    homeAllServos();

    if (nowMs - pirSequenceStartMs >= PIR_RETURN_DURATION_MS)
    {
      pirState = PIR_IDLE;
      currentYaw = 0.0f;
      yawState = YAW_IDLE;
      Serial.println("Urutan PIR selesai: kendali sensor dilanjutkan.");
    }
  }
}

void applySensorDrivenMotion()
{
  const float rollTarget = -currentRoll;
  const float pitchTarget = currentPitch;

  setServoToAngle(servo1, rollTarget);
  setServoToAngle(servo2, rollTarget);
  setServoToAngle(servo3, pitchTarget);
  setServoToAngle(servo4, pitchTarget);
  setServoToAngle(servo5, currentYaw);
}

void printRollPitchCaseTest()
{
  if (caseTestPrinted ||
      pirState != PIR_IDLE ||
      fabsf(currentRoll - 30.0f) > TEST_ANGLE_TOLERANCE_DEG ||
      fabsf(currentPitch + 20.0f) > TEST_ANGLE_TOLERANCE_DEG)
  {
    return;
  }

  const float servo1Angle = -currentRoll;
  const float servo2Angle = -currentRoll;
  const float servo3Angle = currentPitch;
  const float servo4Angle = currentPitch;

  Serial.printf(
      "Kasus Uji 1: Roll %.2f derajat, Pitch %.2f derajat\n",
      currentRoll,
      currentPitch);
  Serial.printf("Servo 1 = %d mikrodetik\n", angleToPwmMicroseconds(servo1Angle));
  Serial.printf("Servo 2 = %d mikrodetik\n", angleToPwmMicroseconds(servo2Angle));
  Serial.printf("Servo 3 = %d mikrodetik\n", angleToPwmMicroseconds(servo3Angle));
  Serial.printf("Servo 4 = %d mikrodetik\n", angleToPwmMicroseconds(servo4Angle));
  caseTestPrinted = true;
}

void printSensorDebug(unsigned long nowMs)
{
  if (nowMs - lastDebugTimeMs < DEBUG_PERIOD_MS)
  {
    return;
  }

  lastDebugTimeMs = nowMs;
  Serial.printf(
      "Roll: %.2f | Pitch: %.2f | Yaw: %.2f derajat | Laju giroskop Z: %.3f rad/detik\n",
      currentRoll,
      currentPitch,
      currentYaw,
      yawRateZ);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(PIR_PIN, INPUT);

  if (!mpu.begin())
  {
    Serial.println("MPU6050 tidak terdeteksi. Periksa koneksi I2C.");
    while (true)
    {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateMpu6050();
  attachServos();
  homeAllServos();
  lastSensorTimeMs = millis();
}

void loop()
{
  const unsigned long nowMs = millis();

  if (nowMs - lastSensorTimeMs < SENSOR_PERIOD_MS)
  {
    return;
  }

  const float deltaTimeSeconds =
      (nowMs - lastSensorTimeMs) / 1000.0f;
  lastSensorTimeMs = nowMs;

  readMpu6050(deltaTimeSeconds);
  handlePirTrigger(nowMs);

  if (pirState == PIR_IDLE)
  {
    applySensorDrivenMotion();
    printRollPitchCaseTest();
  }
  else
  {
    updatePirSequence(nowMs);
  }

  printSensorDebug(nowMs);
}
