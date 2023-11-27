// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <SPI.h>
// #include <PWMServo.h>
// #include <Adafruit_BusIO_Register.h>
// #include <Adafruit_BMP280.h>
// #include <tvcPID.h>

// // Servos
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
// PWMServo pitchServo; 
// PWMServo yawServo; 
// PWMServo parachuteServo;

// Adafruit_BMP280 bmp = Adafruit_BMP280(0x77);

// // Global variables
// float launchAltitude;
// float currentAltitude;
// float previousAltitude = -1;
// int decreaseCounter = 0;
// const int threshold = 3;
// float z_position = 0;
// const float accelerationThreshold = 12;
// float negativeAccelCounter = 0;
// float z_accel;
// unsigned long lastTime = 0;
// float dt;

// // Constants
// const int PITCH_SERVO_PIN = 10;
// const int YAW_SERVO_PIN = 37;
// const float MAX_PID_OUTPUT = 90;
// const int LED_GREEN = 15;
// const int PARACHUTE_SERVO_PIN = 9;

// // PID parameters
// tvcPID tvc(1.5, 0.05, 0.75);
// float integralPitch = 0; float integralYaw = 0;
// float prevErrorPitch = 0; float prevErrorYaw = 0;

// // Setpoints
// const float setPointPitch = 0; const float setPointYaw = 0;
// int pitchServoHome = 0.0; int yawServoHome = 0.0;

// float servoStartSequence(){
//     digitalWrite(LED_GREEN, LOW);
//     pitchServo.write(pitchServoHome);
//     yawServo.write(yawServoHome);
//     delay(150);
//     pitchServo.write(pitchServoHome);
//     yawServo.write(yawServoHome);
//     digitalWrite(LED_GREEN, HIGH);
//     Serial.println("Ready for launch");
//     delay(50);
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();
//     if (!bno.begin()) {
//         Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
//         while (1);
//     }
//     delay(10);
//     if (!bmp.begin()) {
//     Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
//                       "try a different address!"));
//     while (1) delay(10);
//     }
//     pitchServo.attach(PITCH_SERVO_PIN);
//     yawServo.attach(YAW_SERVO_PIN);
//     parachuteServo.attach(PARACHUTE_SERVO_PIN);
//     // float launchAltitude = bmp.readAltitude(1013.25);
//     servoStartSequence();
//     delay(100);
// }

// int mapPIDToServo(float pidOutput) {
//     return constrain(map(pidOutput, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, 0, 180), 0, 180);
// }

// bool launchDetected = false;
// bool checkForLaunch() {
//     if (launchDetected) {
//         return true;
//     }
//     imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//     if (abs(acceleration.z()) > accelerationThreshold) {
//         launchDetected = true;
//         return true;
//     }
//     return false;
// }

// void loop() {
//     unsigned long currentTime = millis();
//     dt = (currentTime - lastTime) / 1000.0;
//     if (dt <= 0.01) dt = 0.01;

//     // Check for launch
//     if (!launchDetected && !checkForLaunch()) {
//         imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//         imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//         Serial.print("accel data: "); Serial.print(acceleration.z());
//         Serial.print("Pitch: "); Serial.print(euler.x());
//         Serial.print(", Roll: "); Serial.print(euler.y());
//         Serial.print(", Yaw: "); Serial.println(euler.z());
//         delay(100);
//       return;
//     }
//     // Launch detected, turn off green LED
//     digitalWrite(LED_GREEN, LOW);
//     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//     float pitchControl = tvc.pidControl(euler.y(), setPointPitch, prevErrorPitch, integralPitch, dt);
//     float yawControl = tvc.pidControl(euler.z(), setPointYaw, prevErrorYaw, integralYaw, dt);
//     pitchServo.write(mapPIDToServo(pitchControl));
//     yawServo.write(mapPIDToServo(yawControl));

//     Serial.print("Pitch: "); Serial.print(euler.y());
//     Serial.print(" Pitch Control: "); Serial.print(pitchControl);
//     Serial.print(", Yaw: "); Serial.print(euler.z());
//     Serial.print(", Yaw Control: "); Serial.println(yawControl);

//     // Check altitude
//     imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//     float z_accel = acceleration.z(); 
//     // float currentAltitude = bmp.readAltitude(1013.25);
//     // Serial.print(bmp.readAltitude(1013.25));
//     if (z_accel < 0){
//         negativeAccelCounter++;
//         if (negativeAccelCounter > 10) {
//             parachuteServo.write(90);
//         } else { 
//             negativeAccelCounter = 0;
//         }
//     }
//     previousAltitude = currentAltitude;
//     lastTime = currentTime;
//     delay(10);
// }
