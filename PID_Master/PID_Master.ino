#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <Wire.h>

//Limits
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

volatile float jointArray[10] = {0};

ros::NodeHandle nh;

void messageCb(const sensor_msgs::JointState& msg) {
  for (int i = 0; i < 10; i++) {
    jointArray[i] = msg.position[i];
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("joints", &messageCb);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  nh.initNode();
  nh.subscribe(sub);
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
  transmitThumb(jointArray[0], jointArray[1], jointArray[2]);
  transmitMCP(jointArray[4], jointArray[8] , jointArray[12], jointArray[16]);
  transmitPIP(jointArray[5], jointArray[9] , jointArray[13], jointArray[17]);
  transmitDIP(jointArray[6], jointArray[10], jointArray[14], jointArray[18]);
  transmitAbduction(0);
  // transmitWrist(2000, 2000);

  // Grapsing scissors
  // transmitMCP(100, 100, 0, 0);
  // transmitPIP(30, 30, 0, 0);
  // transmitDIP(0, 0, 0, 0);
  // transmitThumb(0, 0, 20);
  // transmitAbduction(0);

  // Grasping large disk
  // transmitMCP(60, 20, 30, 60);
  // transmitPIP(50, 70, 70, 70);
  // transmitDIP(10, 20, 10, 0);
  // transmitThumb(40, 30, 0);
  // transmitAbduction(0.9);

  // Credit card tripod grip
  // transmitMCP(104, 101.93, 0, 0);
  // transmitPIP(60, 70, 0, 0);
  // transmitDIP(0, 10, 0, 0);
  // transmitThumb(20, 52.73, 30);
  // transmitAbduction(0);

  // Holding electrical tape
  // transmitMCP(70, 70, 0, 0);
  // transmitPIP(40, 40, 0, 0);
  // transmitDIP(0, 10, 0, 0);
  // transmitThumb(20, 30, 0);
  // transmitAbduction(0.3);

  // Holding a nut
  // transmitMCP(80, 0, 0, 0);
  // transmitPIP(73, 0, 0, 0);
  // transmitDIP(0, 0, 0, 0);
  // transmitThumb(0, 70, 35);
  // transmitAbduction(0);

  //Holding scissors
  // transmitMCP(104, 101.93, 0, 0);
  // transmitPIP(40, 40, 0, 0);
  // transmitDIP(0, 0, 0, 0);
  // transmitThumb(30, 30, 0);
  // transmitAbduction(0);

  // Holding a paperclip
  // transmitMCP(104, 0, 0, 0);
  // transmitPIP(0, 0, 0, 0);
  // transmitDIP(50, 0, 0, 0);
  // transmitThumb(10, 0, 40);
  // transmitAbduction(0);

  // Handshake
  // transmitMCP(60, 70, 70, 70);
  // transmitPIP(30, 50, 50, 70);
  // transmitDIP(30, 30, 50, 70);
  // transmitThumb(30, 30, 30);
  // transmitAbduction(0);

  // 2 kg weight
  // transmitMCP(60, 40, 60, 60);
  // transmitPIP(40, 30, 30, 70);
  // transmitDIP(40, 55, 65, 30);
  // transmitThumb(70, 20, 20);
  // transmitAbduction(0.8);

  // Peace sign
  // transmitMCP(0, 0, 100, 100);
  // transmitPIP(0, 0, 70, 70);
  // transmitDIP(0, 0, 20, 30);
  // transmitThumb(80, 30, 30);
  // transmitAbduction(0.3);

  // nh.spinOnce();
  delay(100);
}
