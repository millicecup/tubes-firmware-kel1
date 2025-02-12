#ifndef PID_H
#define PID_H

class PIDController {
private:
    float kp, ki, kd;
    float previousError;
    float integral;
    unsigned long lastTime;

public:
    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d), previousError(0), integral(0), lastTime(0) {}

    void setTunings(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float compute(float currentValue, float setpoint);
    void reset();
};

#endif
