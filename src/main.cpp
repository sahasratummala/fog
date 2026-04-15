#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DRV2605.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_DRV2605 drv_left;   // 0x5A on Wire  (SDA=21, SCL=22)
Adafruit_DRV2605 drv_right;  // 0x5A on Wire1 (SDA=18, SCL=19)

// ---- FOG Detection Parameters ----
#define STEP_THRESHOLD 11.5   // m/s^2 - slightly below gravity+impact peak (~12-14 typical)
#define FOG_CADENCE_DROP 0.65 // FOG if cadence drops below 65% of normal (more realistic than 50%)
#define CADENCE_WINDOW 8      // more steps = more stable normal cadence baseline
#define CALIBRATION_STEPS 8   // must complete this many steps before FOG detection begins
#define FOG_TIMEOUT 4000      // ms - stop cueing 4s after last FOG-level step
#define DEBOUNCE_MS 250       // ignore steps faster than 250ms (max ~240 steps/min)
#define MIN_WALKING_STEPS 3   // need at least 3 steps before computing cadence at all

// ---- State ----
unsigned long stepTimes[CADENCE_WINDOW];
int stepIndex = 0;
int stepCount = 0;
float normalCadence = 0;
bool calibrated = false;
bool fogActive = false;
unsigned long lastFOGTime = 0;
unsigned long lastCueTime = 0;
bool cueLeft = true;
float cueInterval = 500;

// ---- DRV2605 register write (bus-aware) ----
void writeDRV(TwoWire &bus, uint8_t reg, uint8_t val)
{
  bus.beginTransmission(0x5A);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
}

void buzzMotorDirect(TwoWire &bus, uint8_t effect)
{
  writeDRV(bus, 0x04, effect);
  writeDRV(bus, 0x05, 0);
  writeDRV(bus, 0x0C, 1);
}

void buzzLeft(uint8_t effect)  { buzzMotorDirect(Wire,  effect); }
void buzzRight(uint8_t effect) { buzzMotorDirect(Wire1, effect); }

// ---- Step Detection ----
bool detectHeelStrike(float accelMag)
{
  static float lastAccelMag = 0;
  static unsigned long lastStepTime = 0;

  if (millis() - lastStepTime < DEBOUNCE_MS)
  {
    lastAccelMag = accelMag;
    return false;
  }

  bool crossedUp = (accelMag > STEP_THRESHOLD && lastAccelMag <= STEP_THRESHOLD);
  lastAccelMag = accelMag;

  if (crossedUp)
  {
    lastStepTime = millis();
    return true;
  }
  return false;
}

// ---- Cadence (steps per minute) ----
float computeCadence()
{
  int samples = min(stepCount, CADENCE_WINDOW);
  if (samples < 2)
    return 0;

  float totalTime = 0;
  int counted = 0;
  for (int i = 0; i < samples - 1; i++)
  {
    int curr = (stepIndex - 1 - i + CADENCE_WINDOW) % CADENCE_WINDOW;
    int prev = (stepIndex - 2 - i + CADENCE_WINDOW) % CADENCE_WINDOW;
    long diff = (long)stepTimes[curr] - (long)stepTimes[prev];
    if (diff > 0 && diff < 3000)
    {
      totalTime += diff;
      counted++;
    }
  }
  if (counted == 0)
    return 0;
  return 60000.0 / (totalTime / counted);
}

// ---- Init DRV2605 on a given I2C bus ----
bool initDRV(TwoWire &bus)
{
  bus.beginTransmission(0x5A);
  if (bus.endTransmission() != 0)
    return false;
  writeDRV(bus, 0x01, 0x00);
  writeDRV(bus, 0x03, 0x01);
  writeDRV(bus, 0x16, 0x00);
  return true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(100);

  // ---- IMU ----
  if (!mpu.begin())
  {
    Serial.println("❌ IMU FAILED - check SDA/SCL wiring");
    while (1)
      delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("✅ IMU ready");

  // ---- LEFT MOTOR (0x5A) ----
  if (!drv_left.begin())
  {
    Serial.println("❌ LEFT haptic driver FAILED");
    while (1)
      delay(10);
  }
  drv_left.selectLibrary(1);
  drv_left.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("✅ LEFT driver ready at 0x5A");

  // ---- RIGHT MOTOR (Wire1: SDA=18, SCL=19) ----
  Wire1.begin(18, 19);
  if (!drv_right.begin(&Wire1))
  {
    Serial.println("❌ RIGHT haptic driver FAILED - check SDA/SCL on GPIO 18/19");
    while (1)
      delay(10);
  }
  drv_right.selectLibrary(1);
  drv_right.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("✅ RIGHT driver ready on Wire1 (GPIO 18/19)");

  // ---- Motor test ----
  Serial.println("Testing motors...");
  buzzLeft(47);
  delay(600);
  buzzRight(47);
  delay(600);
  Serial.println("✅ Both motors tested");
  Serial.println("--- Walk normally for 8 steps to calibrate ---");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelMag = sqrt(
      a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y +
      a.acceleration.z * a.acceleration.z);

  // ---- Step Detection ----
  if (detectHeelStrike(accelMag))
  {
    stepTimes[stepIndex] = millis();
    stepIndex = (stepIndex + 1) % CADENCE_WINDOW;
    stepCount++;

    float cadence = computeCadence();

    // ---- Calibration ----
    if (!calibrated && stepCount >= CALIBRATION_STEPS)
    {
      normalCadence = cadence;
      calibrated = true;
      Serial.print("✅ Calibrated. Normal cadence: ");
      Serial.print(normalCadence, 1);
      Serial.println(" steps/min — FOG detection active");
    }

    if (stepCount >= MIN_WALKING_STEPS)
    {
      Serial.print("Step #");
      Serial.print(stepCount);
      Serial.print(" | Cadence: ");
      Serial.print(cadence, 1);
      Serial.print(" steps/min");
      if (!calibrated)
        Serial.print(" (calibrating...)");
      Serial.println();
    }

    // ---- FOG Detection ----
    if (calibrated && cadence > 0)
    {
      if (cadence < normalCadence * FOG_CADENCE_DROP)
      {
        if (!fogActive)
        {
          Serial.println("🚨 FOG DETECTED - starting vibration cues");
        }
        fogActive = true;
        lastFOGTime = millis();
        cueInterval = (60000.0 / normalCadence) / 2.0;
      }
    }
  }

  // ---- Stop cueing if walking resumes ----
  if (fogActive && (millis() - lastFOGTime > FOG_TIMEOUT))
  {
    fogActive = false;
    Serial.println("✅ FOG resolved - stopping cues");
  }

  // ---- Alternating L/R Cues ----
  if (fogActive && (millis() - lastCueTime > cueInterval))
  {
    lastCueTime = millis();
    if (cueLeft)
    {
      buzzLeft(47);
      Serial.println("← LEFT cue");
    }
    else
    {
      buzzRight(47);
      Serial.println("→ RIGHT cue");
    }
    cueLeft = !cueLeft;
  }

  delay(10);
}