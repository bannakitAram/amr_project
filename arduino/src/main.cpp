#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // รอการเชื่อมต่อ Serial

  // ตรวจสอบโมดูล MPU6050 (ใช้ Pin 20 SDA, Pin 21 SCL อัตโนมัติบน Mega 2560)
  if (!mpu.begin()) {
    Serial.println("ไม่พบโมดูล MPU6050 กรุณาตรวจสอบสายเชื่อมต่อ!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("พบโมดูล MPU6050 พร้อมใช้งาน");

  // ตั้งค่า Scale การวัด
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // แสดงค่าความเร่ง (Accelerometer)
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(", Y: "); Serial.print(a.acceleration.y);
  Serial.print(", Z: "); Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  // แสดงค่าความเร็วรอบหมุน (Gyroscope)
  Serial.print("Gyro X: "); Serial.print(g.gyro.x);
  Serial.print(", Y: "); Serial.print(g.gyro.y);
  Serial.print(", Z: "); Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  // แสดงอุณหภูมิ
  Serial.print("Temp: "); Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("------------------------------------");
  delay(500);
}