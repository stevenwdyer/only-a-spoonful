#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Wire.h>

Servo servo;

MPU6050 mpu = 0x68;

const float Kp = 3.0;
const float Ki = 0.05;
const float Kd = 2.0;

float previousError = 0;
float integral = 0;

float angle = 0;

const float targetAngle = 0.0;

const int servoPin = 13;
const int switchPin = 23;
const int i2cPin = 35;

const int servoCenter = 90;

unsigned long lastTime = 0;
float deltaTime = 0;



void setup() {

  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);

  servo.attach(servoPin);
  servo.write(servoCenter);

  pinMode(switchPin, INPUT);
  pinMode(i2cPin, OUTPUT);
  digitalWrite(i2cPin, HIGH);

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection faild");
    while(1);
  }

  mpu.CalibrateGyro(6);

  lastTime = millis();

}



void loop() {
  
  if (digitalRead(switchPin) == HIGH) {
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accelAngle = atan2(ay, az) * RAD_TO_DEG; // tilt angle from accelerometer

    static float angle = 0; 
    float alpha = 0.96; // Weight for gyro data

    angle = alpha * (angle + gx * deltaTime / 131.0) + (1 - alpha) * accelAngle; // Fuse gyro and accelerometer data to get tilt angle

    // PID controller
    float error = angle - targetAngle;
    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    int servoPos = servoCenter - angle;
 
    servo.write(servoPos);

    // Serial monitoring
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" | Servo Position: ");
    Serial.println(servoPos);
  }

  delay(10);

}