#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include "Motor.h"

// Motor outputs
#define AIN1 5
#define AIN2 4
#define A_PWM 3

#define BIN1 2
#define BIN2 1
#define B_PWM 0

#define CIN1 11
#define CIN2 10
#define C_PWM 9

#define DIN1 8
#define DIN2 7
#define D_PWM 6

// Encoder inputs
#define E1A 22
#define E1B 23
#define E2A 18
#define E2B 19
#define E3A 14
#define E3B 15
#define E4A 12
#define E4B 13

Motor m1(A_PWM, AIN1, AIN2, E1A, E1B, 2.0f, 0.001f, 0.02f, 170);
Motor m2(B_PWM, BIN2, BIN1, E2B, E2A, 10.0f, 0.001f, 0.02f, 255); // All pins must be reversed
Motor m3(C_PWM, CIN1, CIN2, E3A, E3B);
Motor m4(D_PWM, DIN1, DIN2, E4A, E4B);

int motorTargets[4] = { 0 };
const int ranges[4] = {1800, 130, 2900, 3300}; // CMC, Pronation, MCP, IP

extern "C" uint32_t set_arm_clock(uint32_t frequency);

void receiveEvent(int bytesRead) {
  float data[4] = { 0.0 };

  if (bytesRead >= 16) {
    Wire1.readBytes((byte*)data, 16);

    for (int i = 0; i < 4; i++) {
      motorTargets[i] = (int)(data[i] * ranges[i]);
    }
  }
}

void setup() {
  // Reduce clock
  set_arm_clock(30000000);

  // For Serial debugging
  Serial.begin(9600);

  //I2C
  Wire1.begin(10);
  Wire1.setClock(100000);
  Wire1.onReceive(&receiveEvent);
}

void loop() {
  // m1.spinToTarget(motorTargets[0]);   //
  // m3.spinToTarget(motorTargets[2]);   // MCP
  // m4.spinToTarget(motorTargets[3]);   // IP
  
  Serial.println(m2.getPosition());
  // m1.spinToTarget(0.5*ranges[0]);   // CMC
  m2.spinToTarget(130);   //Pronation
  // m3.spinToTarget(0.3*ranges[2]);   // MCP
  // m4.spinToTarget(-0.5*ranges[3]);   // IP

  delay(10);
}
