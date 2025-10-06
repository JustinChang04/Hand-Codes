#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include <PA12.h>

PA12 servoRight(&Serial3, 23, 27);
PA12 servoLeft(&Serial5, 24, 21);

volatile int leftPosition = 1076;
volatile int rightPosition = 1076;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

void receiveEvent(int numReceived) {
  if (numReceived >= 8) {
    Wire.readBytes((byte*) &leftPosition, sizeof(int));
    Wire.readBytes((byte*) &rightPosition, sizeof(int));
  }
  
  Serial.println(leftPosition);
}

void setup() {
  set_arm_clock(30000000);

  Serial.begin(9600);

  // I2C
  Wire1.begin(13);
  Wire1.setClock(100000);
  Wire1.onReceive(&receiveEvent);

  // Servo setup
  servoRight.begin(32);
  servoLeft.begin(32);
}

bool forward() {
  servoLeft.goalPosition(0, 2900);
  servoRight.goalPosition(0, 2900);

  return abs(servoLeft.presentPosition(0) - 2900) <= 5 && abs(servoRight.presentPosition(0) - 2900) <= 5;
}

bool partialForward() {
  servoLeft.goalPosition(0, 2152);
  servoRight.goalPosition(0, 2152);

  return abs(servoLeft.presentPosition(0) - 2152) <= 5 && abs(servoRight.presentPosition(0) - 2152) <= 5;
}

bool level() {
  servoLeft.goalPosition(0, 1076);
  servoRight.goalPosition(0, 1076);

  return abs(servoLeft.presentPosition(0) - 1076) <= 5 && (servoRight.presentPosition(0) - 1076) <= 5;
}

bool back() {
  servoLeft.goalPosition(0, 0);
  servoRight.goalPosition(0, 0);

  return abs(servoLeft.presentPosition(0)) <= 5 && abs(servoRight.presentPosition(0)) <= 5;
}

bool left() {
  servoLeft.goalPosition(0, 1000);
  servoRight.goalPosition(0, 3152);

  return abs(servoLeft.presentPosition(0) - 1000) <= 5 && abs(servoRight.presentPosition(0) - 3152) <= 5;
}

bool right() {
  //Forward
  servoLeft.goalPosition(0, 3152);
  servoRight.goalPosition(0, 1000);

  return abs(servoLeft.presentPosition(0) - 3152) <= 5 && abs(servoRight.presentPosition(0) - 1000) <= 5;
}

uint8_t state = 0;
 
//Max forward is 3200
void loop() {
  // switch (state) {
  //   case 0:
  //     if (partialForward()) state++;
  //     break;
  //   case 1:
  //     if (left()) state++;
  //     break;
  //   case 2:
  //     if (back()) state++;
  //     break;
  //   case 3:
  //     if (right()) state = 0;
  //     break;
  // }
  // level();
  servoRight.ledOn(0, GREEN);
  servoLeft.ledOn(0, GREEN);
}
