#ifndef MOTOR_H
#define MOTOR_H

#include <Encoder.h>

class Motor {
private:
  // Last recorded position error
  int lastError;

  // Digital pins
  byte D_PWM, D1, D2;

  // Encoder associated with motor
  Encoder enc;

  // Time elapsed, used for PID
  elapsedMillis sinceCtrl;

  // Integral currently not used
  static constexpr float Kp = 2.0f, Kd = 0.02f;

  /**
    Sets the PWM power
    @param pwm_in signed pwm power, will be capped between -255 and 255
    */
  void setPWM(int pwm_in) {
    if (pwm_in > 255) {
      pwm_in = 255;
    } else if (pwm_in < -255) {
      pwm_in = -255;
    }

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
      analogWrite(D_PWM, 0 - pwm_in);
    }
  }

public:
  // We don't want the default constructor
  Motor() = delete;

  /**
    @param _D_PWM the PWM pin
    @param _D1 digital pin 1
    @param _D2 digital pin 2
    @param _en1 digital pin 1
    @param _en2 encoder pin 2
    */
  Motor(byte _D_PWM, byte _D1, byte _D2, byte _en1, byte _en2)
    : enc(_en2, _en1) {
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
  }

  /**
    Spins the motor to the target position
    @param newTarget the target position in pulses
    */
  void spinToTarget(int newTarget) {
    int pos = enc.read() / 4;

    int error = newTarget - pos;

    // Get inverse since multiplication is cheaper than division
    float dt_inverse = 1e3f / sinceCtrl;

    float derivative = (error - lastError) * dt_inverse;

    int power = (int) (Kp * error + Kd * derivative);

    setPWM(power);

    lastError = error;
    sinceCtrl = 0;
  }
};

#endif