#ifndef TVCPID_H
#define TVCPID_H

class tvcPID {
public:
    tvcPID(float Kp, float Ki, float Kd);
    float pidControl(float currentAngle, float desiredAngle, float &prevError, float &integral, float dt);

private:
    float Kp, Ki, Kd;
};

#endif
