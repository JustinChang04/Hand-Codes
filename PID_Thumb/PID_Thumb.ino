#include <Wire.h>
#include "Motor.h"
#include "Position.h"

//Track the number of pulses from each motor
extern volatile int numPulsesA;
extern volatile int numPulsesB;
extern volatile int numPulsesC;

//Targets for each motor
extern volatile int aTarget;
extern volatile int bTarget;
extern volatile int cTarget;

//Set the max and min number of pulses, which equates to number of rotations
const int maxPulsesCMC = 1800; // Should be 3600 but we increment by 2 each time for some reason
const int maxPulsesMCP = 2900;
const int maxPulsesIP = 3300;

float percentage = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(1);

  //I2C
  Wire.onReceive(receiveEvent);

  setupPosition();
  setupMotors();
}

void receiveEvent(int bytesRead) {
  float data[3];

  if (bytesRead == sizeof(data)) {
    Wire.readBytes((uint8_t*) data, sizeof(data));
    aTarget = (int) (maxPulsesCMC * data[0]);
    bTarget = (int) (maxPulsesMCP * data[1]);
    cTarget = (int) (maxPulsesIP * data[2]);
  }
}

void loop() {
  setPosition();
}
