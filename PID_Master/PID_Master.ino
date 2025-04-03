#include <Wire.h>

//Limits (1.0 / limit)
const float THUMB_CMC = 1.0 / 1.8545;
const float THUMB_MCP = 1.0 / 0.9203;
const float THUMB_IP = 1.0 / 0.7858;

const float MCP_INDEX = 1.0 / 1.8002;
const float PIP_INDEX = 1.0 / 1.3104;
const float DIP_INDEX = 1.0 / 1.1885;

const float MCP_MIDDLE = 1.0 / 1.7791;
const float PIP_MIDDLE = 1.0 / 1.2823;
const float DIP_MIDDLE = 1.0 / 1.275;

const float MCP_RING = 1.0 / 1.7553;
const float PIP_RING = 1.0 / 1.273;
const float DIP_RING = 1.0 / 1.2842;

const float MCP_PINKY = 1.0 / 1.7269;
const float PIP_PINKY = 1.0 / 1.2573;
const float DIP_PINKY = 1.0 / 1.2577;

void setup() {
  Wire.begin();
}

void normalize(float& value, float limit) {
  if (value > limit) {
    value = limit;
  }
  else if (value < 0) {
    value = 0;
  }
}

void transmitThumb(float CMC, float MCP, float IP) {
  normalize(CMC, THUMB_CMC);
  normalize(MCP, THUMB_MCP);
  normalize(IP, THUMB_IP);

  Wire.beginTransmission(1);
  float data[3] = {CMC * THUMB_CMC, MCP * THUMB_MCP, IP * THUMB_IP};

  Wire.write((uint8_t*) data, sizeof(data));

  Wire.endTransmission();
}

void transmitMCP(float m1, float m2, float m3, float m4) {
  normalize(m1, MCP_INDEX);
  normalize(m2, MCP_MIDDLE);
  normalize(m3, MCP_RING);
  normalize(m4, MCP_PINKY);
  
  Wire.beginTransmission(2);
  float data[4] = {m1 * MCP_INDEX, m2 * MCP_MIDDLE, m3 * MCP_RING, m4 * MCP_PINKY};

  Wire.write((uint8_t*) data, sizeof(data));

  Wire.endTransmission();
}

void transmitPIP(float p1, float p2, float p3, float p4) {
  normalize(p1, PIP_INDEX);
  normalize(p2, PIP_MIDDLE);
  normalize(p3, PIP_RING);
  normalize(p4, PIP_PINKY);

  Wire.beginTransmission(3);
  float data[4] = {p1 * PIP_INDEX, p2 * PIP_MIDDLE, p3 * PIP_RING, p4 * PIP_PINKY};

  Wire.write((uint8_t*) data, sizeof(data));
  Serial.println(data[0]);

  Wire.endTransmission();
}

void transmitDIP(float d1, float d2, float d3, float d4) {
  normalize(d1, DIP_INDEX);
  normalize(d2, DIP_MIDDLE);
  normalize(d3, DIP_RING);
  normalize(d4, DIP_PINKY);

  Wire.beginTransmission(4);
  float data[4] = {d1 * DIP_INDEX, d2 * DIP_MIDDLE, d3 * DIP_RING, d4 * DIP_PINKY};

  Wire.write((uint8_t*) data, sizeof(data));

  Wire.endTransmission();
}

void transmitAbduction(float angle) {
  Wire.beginTransmission(5);

  float copy = angle;
  byte* byteArray = (byte*) &angle;

  for (int i = 0; i < 4; i++) {
    Wire.write(byteArray[i]);
  }

  Wire.endTransmission(5);
}

void transmitWrist(int left, int right) {
  Wire.beginTransmission(6);

  int copy = left;
  byte* byteArray = (byte*) &left;

  for (int i = 0; i < 4; i++) {
    Wire.write(byteArray[i]);
  }

  copy = right;
  for (int i = 0; i < 4; i++) {
    Wire.write(byteArray[i]);
  }

  Wire.endTransmission(6);
}

void loop() {
  // Can set to angles between 0 radians and the maximum number of radians specified, all set to 0 for now
  transmitThumb(0, 0, 0);
  transmitMCP(0, 0, 0, 0);
  transmitPIP(0, 0, 0, 0);
  transmitDIP(0, 0, 0, 0);
  transmitAbduction(0);
//  transmitWrist(2000, 2000); Not currently working with wrist
  
  delay(100);
}
