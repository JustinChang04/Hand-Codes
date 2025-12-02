#include <i2c_driver.h>
#include <i2c_driver_wire.h>

//Limits (1.0 / limit)
const float THUMB_CMC = 1.0 / 1.8545;
const float THUMB_NEW = 1.0;
const float THUMB_MCP = 1.0 / 0.9203;
const float THUMB_IP = 1.0 / 0.7858;

const float MCP_INDEX = 1.0 / 1.8002;
const float PIP_INDEX = 1.0 / 1.3104;
const float DIP_INDEX = 1.0 / 1.1885; // USE AT MOST 0.9

const float MCP_MIDDLE = 1.0 / 1.7791;
const float PIP_MIDDLE = 1.0 / 1.2823;
const float DIP_MIDDLE = 1.0 / 1.275;

const float MCP_RING = 1.0 / 1.7553;
const float PIP_RING = 1.0 / 1.273;
const float DIP_RING = 1.0 / 1.2842;

const float MCP_PINKY = 1.0 / 1.7269;
const float PIP_PINKY = 1.0 / 1.2573;
const float DIP_PINKY = 1.0 / 1.2577;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

void setup() {
  set_arm_clock(30000000);

  Wire.begin();
  Wire.setClock(100000);

  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
}

void normalize(float* arr, int size) {
  for (int i = 0; i < size; i++) {
    if (arr[i] > 1) {
      arr[i] = 1;
    }
    else if (arr[i] < 0) {
      arr[i] = 0;
    }
  }
}

void transmitThumb(float CMC, float Supination, float MCP, float IP) {
  float data[4] = {CMC * THUMB_CMC, Supination * THUMB_NEW, MCP * THUMB_MCP, IP * THUMB_IP};
  normalize(data, 4);

  Wire.beginTransmission(8);
  Wire.write((uint8_t*) data, 16);
  Wire.endTransmission();
}

void transmitMCP(float m1, float m2, float m3, float m4) {
  float data[4] = {m1 * MCP_INDEX, m2 * MCP_MIDDLE, m3 * MCP_RING, m4 * MCP_PINKY};
  normalize(data, 4);
  
  Wire.beginTransmission(9);
  Wire.write((byte*) data, 16);
  Wire.endTransmission();
}

void transmitPIP(float p1, float p2, float p3, float p4) {
  float data[4] = {p1 * PIP_INDEX, p2 * PIP_MIDDLE, p3 * PIP_RING, p4 * PIP_PINKY};
  normalize(data, 4);

  Wire.beginTransmission(10);
  Wire.write((uint8_t*) data, 16);
  Wire.endTransmission();
}

void transmitDIP(float d1, float d2, float d3, float d4) {
  float data[4] = {d1 * DIP_INDEX, d2 * DIP_MIDDLE, d3 * DIP_RING, d4 * DIP_PINKY};
  normalize(data, 4);

  Wire.beginTransmission(11);
  Wire.write((uint8_t*) data, 16);
  Wire.endTransmission();
}

void transmitAbduction(float percent) {
  float copy = percent;
  normalize(&copy, 1);

  Wire.beginTransmission(12);
  Wire.write((byte*) &copy, sizeof(float));
  Wire.endTransmission();
}

void transmitWrist(int left, int right) {
  Wire.beginTransmission(13);

  int copy = left;
  Wire.write((byte*) &copy, sizeof(int));

  copy = right;
  Wire.write((byte*) &copy, sizeof(int));

  Wire.endTransmission();
}


void loop() {
  // Can set to angles between 0 radians and the maximum number of radians specified, all set to 0 for now
  // transmitThumb(0, 0, 0, 0);
  // transmitMCP(0, 0, 0, 0);
  // transmitPIP(0, 0, 0, 0);
  // transmitDIP(0, 0, 0, 0);
  // transmitAbduction(0.6);
  // transmitWrist(2000, 2000); // Not currently working with wrist

  delay(3000);

  transmitMCP(0, 0, 0, 0);
  delay(1);
  transmitPIP(0, -1, 0, 0);
  delay(1);
  transmitDIP(0, 0, 0, 0);

  delay(3000);

  transmitMCP(0, 0, 0, 0);
  delay(1);
  transmitPIP(0, 0, 0, 0);
  delay(1);
  transmitDIP(0, 0, 0, 0);
}

