#ifndef MOTOR_H
#define MOTOR_H
//Outputs
#define AIN1 0
#define AIN2 1
#define A_PWM 2

#define BIN1 5
#define BIN2 6
#define B_PWM 7

#define CIN1 10
#define CIN2 11
#define C_PWM 12

#define DIN1 21
#define DIN2 22
#define D_PWM 23

void setupMotors() {
  //Output setting
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(A_PWM, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(C_PWM, OUTPUT);

  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(D_PWM, OUTPUT);

  //First set all the pins to low
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(A_PWM, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(B_PWM, 0);

  digitalWrite(CIN1, LOW);
  digitalWrite(CIN2, LOW);
  analogWrite(C_PWM, 0);

  digitalWrite(DIN1, LOW);
  digitalWrite(DIN2, LOW);
  analogWrite(D_PWM, 0);
}

void checkBounds(int& pwm) {
  if (pwm > 255) {
    pwm = 255;
  }

  if (pwm < -255) {
    pwm = -255;
  }
}

//Set the speed of motor A
void Set_PWMA(int pwm) {
  checkBounds(pwm);

  if(pwm>0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(A_PWM, pwm);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(A_PWM, 0 - pwm);
  }
}

//Set the speed of motor B
void Set_PWMB(int pwm) {
  checkBounds(pwm);

  if(pwm>0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(B_PWM, pwm);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(B_PWM, 0 - pwm);
  }
}

//Set the speed of motor C
void Set_PWMC(int pwm) {
  checkBounds(pwm);

  if(pwm>0)
  {
    digitalWrite(CIN1, HIGH);
    digitalWrite(CIN2, LOW);
    analogWrite(C_PWM, pwm);
  }
  else
  {
    digitalWrite(CIN1, LOW);
    digitalWrite(CIN2, HIGH);
    analogWrite(C_PWM, 0 - pwm);
  }
}

//Set the speed of motor D
void Set_PWMD(int pwm) {
  checkBounds(pwm);

  if(pwm>0)
  {
    digitalWrite(DIN1, HIGH);
    digitalWrite(DIN2, LOW);
    analogWrite(D_PWM, pwm);
  }
  else
  {
    digitalWrite(DIN1, LOW);
    digitalWrite(DIN2, HIGH);
    analogWrite(D_PWM, 0 - pwm);
  }
}
#endif