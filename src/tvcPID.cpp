#include "tvcPID.h"

tvcPID::tvcPID(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
float tvcPID::pidControl(float currentAngle, float desiredAngle, float &prevError, float &integral, float dt) {
    float error = desiredAngle - currentAngle;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return Kp * error + Ki * integral + Kd * derivative;
}
