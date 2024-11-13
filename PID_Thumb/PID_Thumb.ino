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
const int maxPulsesCMC = 1800;
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

void receiveEvent(int howMany) {
  if (howMany > 0) {
    uint8_t* byteArray = (uint8_t*) &percentage;

    for (int i = 0; i < 4; i++) {
      byteArray[i] = Wire.read();
    }
    aTarget = (int) (maxPulsesCMC * percentage);

    for (int i = 0; i < 4; i++) {
      byteArray[i] = Wire.read();
    }
    bTarget = (int) (maxPulsesMCP * percentage);

    for (int i = 0; i < 4; i++) {
      byteArray[i] = Wire.read();
    }
    cTarget = (int) (maxPulsesIP * percentage);

    Serial.println(percentage);
  }
}

void loop() {
  setPosition();
}