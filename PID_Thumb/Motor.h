#ifndef MOTOR_H
#define MOTOR_H

#include <Encoder.h>

class Motor {
private:
  int lastError, cap;

  // Digital pins
  byte D_PWM, D1, D2;

  // Encoder associated with motor
  Encoder enc;

  elapsedMillis sinceCtrl;

  // PID constants (INTEGRAL ADDED)
  float Kp = 2.0f, Ki = 0.001f, Kd = 0.02f;
  float integral;

  void setPWM(int pwm_in) {
    if (pwm_in > cap) pwm_in = cap;
    else if (pwm_in < -cap) pwm_in = -cap;

    if (pwm_in == 0) {
      digitalWrite(D1, LOW);
      digitalWrite(D2, LOW);
      analogWrite(D_PWM, 0);
    } else if (pwm_in > 0) {
      digitalWrite(D1, LOW);
      digitalWrite(D2, HIGH);
      analogWrite(D_PWM, pwm_in);
    } else {
      digitalWrite(D1, HIGH);
      digitalWrite(D2, LOW);
      analogWrite(D_PWM, -pwm_in);
    }
  }

public:
  Motor() = delete;

  Motor(byte _D_PWM, byte _D1, byte _D2, byte _en1, byte _en2)
    : enc(_en2, _en1) {

    Kp = 2.0f, Ki = 0.001f, Kd = 0.02f;
    cap = 255;

    D_PWM = _D_PWM;
    D1 = _D1;
    D2 = _D2;
    lastError = 0;

    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D_PWM, OUTPUT);

    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    digitalWrite(D_PWM, 0);

    enc.write(0);
    integral = 0;                    // <-- ADDED
  }

  Motor(byte _D_PWM, byte _D1, byte _D2, byte _en1, byte _en2, float _Kp, float _Ki, float _Kd, int _cap)
    : enc(_en2, _en1) {

    Kp = _Kp, Ki = _Ki, Kd = _Kd;
    cap = _cap;

    D_PWM = _D_PWM;
    D1 = _D1;
    D2 = _D2;
    lastError = 0;

    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D_PWM, OUTPUT);

    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    digitalWrite(D_PWM, 0);

    enc.write(0);
    integral = 0;                    // <-- ADDED
  }

  void spinToTarget(int newTarget) {
    int pos = enc.read() / 4;
    int error = newTarget - pos;

    float dt_ms = sinceCtrl;
    float dt = dt_ms / 1000.0f;      // convert ms → sec
    if (dt < 0.0001f) dt = 0.0001f;  // avoid divide-by-zero

    float derivative = (error - lastError) / dt;

    // ---- INTEGRAL TERM ----
    integral += error * dt;          // <-- ADDED

    // Anti-windup            // <-- ADDED
    if (integral > 3000) integral = 3000;
    else if (integral < -3000) integral = -3000;

    // PID OUTPUT
    int power = (int)(
      Kp * error +
      Ki * integral +               // <-- ADDED
      Kd * derivative
    );

    setPWM(power);

    lastError = error;
    sinceCtrl = 0;
  }

  int getPosition() {
    return enc.read() / 4;
  }
};

#endif
