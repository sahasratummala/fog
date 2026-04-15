#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DRV2605.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_DRV2605 drv_left;  // 0x5A (default)
Adafruit_DRV2605 drv_right; // 0x5B (after ADDR pad soldered)

// ---- FOG Detection Parameters ----
#define STEP_THRESHOLD 12.0  // m/s^2 - heel strike detection
#define FOG_CADENCE_DROP 0.5 // if cadence drops below 50% of normal = FOG
#define CADENCE_WINDOW 5     // number of steps to average
#define FOG_TIMEOUT 3000     // ms - stop cueing after 3s if no FOG

// ---- State ----
unsigned long stepTimes[CADENCE_WINDOW];
int stepIndex = 0;
int stepCount = 0;
float normalCadence = 0;
bool fogActive = false;
unsigned long lastFOGTime = 0;
unsigned long lastCueTime = 0;
bool cueLeft = true;
float cueInterval = 500; // ms between alternating cues

void buzzMotor(Adafruit_DRV2605 &drv, uint8_t effect)
{
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}

bool detectHeelStrike(float accelY)
{
  static float lastAccelY = 0;
  static bool wasHigh = false;
  static unsigned long lastStepTime = 0;

  // Debounce - ignore steps faster than 200ms apart
  if (millis() - lastStepTime < 200)
    return false;

  bool isHigh = accelY > STEP_THRESHOLD;
  bool strikeDetected = isHigh && !wasHigh;
  wasHigh = isHigh;

  if (strikeDetected)
  {
    lastStepTime = millis();
    return true;
  }
  return false;
}

float computeCadence()
{
  if (stepCount < 2)
    return 0;
  int samples = min(stepCount, CADENCE_WINDOW);

  // Average time between last N steps
  float totalTime = 0;
  for (int i = 1; i < samples; i++)
  {
    int curr = (stepIndex - i + CADENCE_WINDOW) % CADENCE_WINDOW;
    int prev = (stepIndex - i - 1 + CADENCE_WINDOW) % CADENCE_WINDOW;
    totalTime += stepTimes[curr] - stepTimes[prev];
  }
  float avgInterval = totalTime / (samples - 1);
  return (avgInterval > 0) ? (60000.0 / avgInterval) : 0; // steps per minute
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);

  // ---- IMU ----
  if (!mpu.begin())
  {
    Serial.println("❌ IMU FAILED");
    while (1)
      delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.println("✅ IMU ready");

  // ---- LEFT MOTOR (0x5A) ----
  if (!drv_left.begin())
  {
    Serial.println("❌ LEFT driver FAILED");
    while (1)
      delay(10);
  }
  drv_left.selectLibrary(1);
  drv_left.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("✅ LEFT driver ready");

  // ---- RIGHT MOTOR (0x5B) ----
  TwoWire *wire2 = &Wire;
  wire2->beginTransmission(0x5B);
  if (wire2->endTransmission() != 0)
  {
    Serial.println("❌ RIGHT driver FAILED - check ADDR solder pad");
    while (1)
      delay(10);
  }
  drv_right.begin();
  drv_right.selectLibrary(1);
  drv_right.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("✅ RIGHT driver ready");

  // ---- Quick motor test ----
  Serial.println("Testing motors...");
  buzzMotor(drv_left, 47);
  delay(600);
  buzzMotor(drv_right, 47);
  delay(600);
  Serial.println("✅ Motors tested - starting FOG detection");
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

    // Calibrate normal cadence from first 5 steps
    if (stepCount == CADENCE_WINDOW)
    {
      normalCadence = cadence;
      Serial.print("✅ Normal cadence calibrated: ");
      Serial.print(normalCadence);
      Serial.println(" steps/min");
    }

    Serial.print("Step detected | Cadence: ");
    Serial.print(cadence);
    Serial.println(" steps/min");

    // ---- FOG Detection ----
    if (normalCadence > 0 && stepCount > CADENCE_WINDOW)
    {
      if (cadence < normalCadence * FOG_CADENCE_DROP)
      {
        if (!fogActive)
        {
          Serial.println("🚨 FOG DETECTED - starting vibration cues");
        }
        fogActive = true;
        lastFOGTime = millis();
        // Set cue interval based on normal cadence
        cueInterval = (60000.0 / normalCadence) / 2.0;
      }
    }
  }

  // ---- Stop FOG if walking resumes ----
  if (fogActive && millis() - lastFOGTime > FOG_TIMEOUT)
  {
    fogActive = false;
    Serial.println("✅ FOG resolved - stopping cues");
  }

  // ---- Alternating L/R Vibration Cues ----
  if (fogActive && millis() - lastCueTime > cueInterval)
  {
    lastCueTime = millis();
    if (cueLeft)
    {
      buzzMotor(drv_left, 47);
      Serial.println("← LEFT cue");
    }
    else
    {
      buzzMotor(drv_right, 47);
      Serial.println("→ RIGHT cue");
    }
    cueLeft = !cueLeft;
  }

  delay(10);
}