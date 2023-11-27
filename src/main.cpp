#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <PWMServo.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_BMP280.h>

// Sensor Set up
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
PWMServo pitchServo;
PWMServo yawServo;
PWMServo parachuteServo;

// Adafruit_BMP280 bmp = Adafruit_BMP280(0x77);

// Global variables
float launchAltitude;
float currentAltitude;
float previousAltitude = -1;
int decreaseCounter = 0;
const int threshold = 3;
float z_position = 0;
const float accelerationThreshold = 10;
float negativeAccelCounter = 0;
float z_accel;
float dt;

// Constants
const int PITCH_SERVO_PIN = 10;
const int YAW_SERVO_PIN = 37;
const float MAX_PID_OUTPUT = 90;
const int LED_GREEN = 15;
const int LED_RED = 14;
const int PARACHUTE_SERVO_PIN = 9;

// PID parameters
const float Kp = 1.55;
const float Ki = 0.05;
const float Kd = 0.75;

// Setpoints
float pitchAngle;
float yawAngle;
const float setPointPitch = 0;
const float setPointYaw = 0;
int pitchServoHome = 0;
int yawServoHome = 0;
int command;
// PID variables
float integralPitch = 0;
float integralYaw = 0;
float prevErrorPitch = 0;
float prevErrorYaw = 0;
unsigned long lastTime = 0;


void servoStartSequence() {
    digitalWrite(LED_GREEN, HIGH);
    pitchServo.write(90);
    yawServo.write(90);
    delay(150);
    digitalWrite(LED_GREEN, LOW);
    Serial.println("Ready for launch");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    if (!bno.begin()) {
        Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(10);
    
    // if (!bmp.begin()) {
    // Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
    //                   "try a different address!"));
    // while (1) delay(10);
    // }
    pitchServo.attach(PITCH_SERVO_PIN);
    yawServo.attach(YAW_SERVO_PIN);
    parachuteServo.attach(PARACHUTE_SERVO_PIN);
    servoStartSequence();
    delay(100);
}

float pidControl(float currentAngle, float desiredAngle, float &prevError, float &integral, float dt) {
    float error = desiredAngle - currentAngle;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

int mapPIDToServo(float pidOutput) {
    int servoMidpoint = 90;  // midpoint
    int servoRange = 45;  // Servo ROM from midpoint
    command = constrain(map(pidOutput, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, servoMidpoint - servoRange, servoMidpoint + servoRange), servoMidpoint - servoRange, servoMidpoint + servoRange);
    return command;
}

bool launchDetected = false;
bool checkForLaunch() {
    if (launchDetected) {
        return true;
    }
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if (abs(acceleration.z()) > accelerationThreshold) {
        launchDetected = true;
        return true;
    }
    return false;
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    if (dt <= 0.01) dt = 0.01;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    pitchAngle = euler.y();
    yawAngle = euler.z();
    float accel_z = accel.z();
    
    // Check for launch
    if (!launchDetected && !checkForLaunch()) {
        Serial.print("accel data: "); Serial.println(accel_z);
        delay(100);
      return;
    }
    digitalWrite(LED_RED, HIGH);
    float pitchControl = pidControl(pitchAngle, setPointPitch, prevErrorPitch, integralPitch, dt);
    float yawControl = pidControl(yawAngle, setPointYaw, prevErrorYaw, integralYaw, dt);
    int pitch_command = mapPIDToServo(pitchControl);
    int yaw_command = mapPIDToServo(yawControl);
    pitchServo.write(pitch_command);
    yawServo.write(yaw_command);

    Serial.print("Pitch: "); Serial.print(pitchAngle);
    Serial.print(" Pitch Control: "); Serial.print(pitch_command);
    Serial.print(", Yaw: "); Serial.print(yawAngle);
    Serial.print(", Yaw Control: "); Serial.println(yaw_command);

    // Check altitude
    if (accel_z < 0){
        negativeAccelCounter++;
        if (negativeAccelCounter > 10) {
            parachuteServo.write(90);
        } else { 
            negativeAccelCounter = 0;
        }
    }
    previousAltitude = currentAltitude;
    lastTime = currentTime;
    delay(10);
}
