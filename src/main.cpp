#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // Try to connect to the IMU over I2C - if it fails, check SDA/SCL wiring (GPIO 21/22)
  if (!mpu.begin())
  {
    Serial.println("Failed to find IMU chip! Check SDA/SCL wiring.");
    while (1)
    {
      delay(10);
    }
  }

  // Bump range to ±4g so heel strikes (~3g) don't clip the signal
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

  Serial.println("IMU Found! Communication Confirmed.");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print all 3 acceleration axes (m/s^2) - Y is gravity/vertical, useful for step detection
  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(a.acceleration.z);

  // Print gyroscope axes (rad/s) - will help detect rotation during gait later
  Serial.print(" | Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print(" Y: ");
  Serial.print(g.gyro.y);
  Serial.print(" Z: ");
  Serial.println(g.gyro.z);

  delay(200);
}