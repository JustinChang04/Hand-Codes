#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include "Motor.h"
#include "Position.h"

//Track the number of pulses from each motor
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
const int maxPulsesCMC = 1800; // Should be 3600 but we increment by 2 each time for some reason
const int maxPulsesNew = 300;
const int maxPulsesMCP = 2900;
const int maxPulsesIP = 3300;

// Downclock
extern "C" uint32_t set_arm_clock(uint32_t frequency);

void receiveEvent(int bytesRead) {
  float data[4] = {0.0};

  if (bytesRead >= 16) {
    Wire1.readBytes((byte*) data, 16);

    aTarget = (int) (maxPulsesCMC * data[0]);
    bTarget = (int) (maxPulsesMCP * data[1]);
    cTarget = (int) (maxPulsesIP * data[2]);
    dTarget = (int) (maxPulsesIP * data[3]);
  }
}

void setup() {
  set_arm_clock(30000000);

  Serial.begin(9600);
  
  //I2C
  Wire1.begin(8);
  Wire1.setClock(100000);
  Wire1.onReceive(&receiveEvent);

  setupPosition();
  setupMotors();
}

void loop() {
  setPosition();
  delay(10);
}
