#include <Wire.h>
#include "Motor.h"
#include "Position.h"

//Pulses from each motor
extern volatile int numPulsesA;
extern volatile int numPulsesB;
extern volatile int numPulsesC;
extern volatile int numPulsesD;

//Targets for each motor
extern volatile int aTarget;
extern volatile int bTarget;
extern volatile int cTarget;
extern volatile int dTarget;

//Set the max and min number of pulses, which equates to number of rotations
const int maxPulses = 3000; //Index and middle can do 3100

extern "C" uint32_t set_arm_clock(uint32_t frequency);

void receiveEvent(int bytesRead) {
  float data[4] = {0.0};

  if (bytesRead >= 16) {
    Wire1.readBytes((byte*) data, 16);

    aTarget = (int) (maxPulses * data[0]);
    bTarget = (int) (maxPulses * data[1]);
    cTarget = (int) (maxPulses * data[2]);
    dTarget = (int) (maxPulses * data[3]);
  }
}

void setup() {
  set_arm_clock(30000000);

  Serial.begin(9600);
  
  //I2C
  Wire1.begin(11);
  Wire1.setClock(100000);
  Wire1.onReceive(&receiveEvent);

  setupPosition();
  setupMotors();
}

void loop() {
   setPosition();
   delay(10);
}
