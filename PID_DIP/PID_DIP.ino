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

float percentage = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(4);

  //I2C
  Wire.onReceive(receiveEvent);

  setupPosition();
  setupMotors();
}

void receiveEvent(int bytesRead) {
  float data[4];

  if (bytesRead == sizeof(data)) {
    Wire.readBytes((uint8_t*) data, sizeof(data));

    aTarget = (int) (maxPulses * data[0]);
    bTarget = (int) (maxPulses * data[1]);
    cTarget = (int) (maxPulses * data[2]);
    dTarget = (int) (maxPulses * data[3]);
  }
}

void loop() {
  // setPosition();
  Set_PWMB(100);
  Serial.println(numPulsesB);
}