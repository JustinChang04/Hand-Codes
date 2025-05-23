#include <PA12.h>
#include <Wire.h>

PA12 servoRight(&Serial2, 2, 16);
PA12 servoLeft(&Serial3, 3, 14);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  servoRight.begin(32);
  servoLeft.begin(32);

  Wire.begin(6);
  Wire.onReceive(receiveEvent);
}

volatile int leftPosition = 1076;
volatile int rightPosition = 1076;

void receiveEvent(int numReceived) {
  if (numReceived > 0) {
    uint8_t* byteArray = (uint8_t*) &left;

    for (int i = 0; i < 4; i++) {
      byteArray[i] = Wire.read();
    }

    byteArray = (uint8_t*) &right;
    for (int i = 0; i < 4; i++) {
      byteArray[i] = Wire.read();
    }
  }
  
  Serial.println(leftPosition);
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
