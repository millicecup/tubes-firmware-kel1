#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {
      reset();
    }

    void setGains(float Kp, float Ki, float Kd) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
      reset();
    }

    float compute(float setpoint, float input) {
      float error = setpoint - input;
      integral += error * dt;
      float derivative = (error - prevError) / dt;
      prevError = error;
      return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset() {
      integral = 0;
      prevError = 0;
    }

  private:
    float Kp, Ki, Kd;
    float integral = 0;
    float prevError = 0;
    const float dt = 0.1;
};

#endif
