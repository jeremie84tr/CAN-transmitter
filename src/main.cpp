#include <Arduino.h>
#include "../include/can.h"

// put function declarations here:
void readInformation(CANFrame* frame);

CAN* can;

void setup() {
  can = new CAN(12,13);
  can->listen(readInformation);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
void readInformation(CANFrame* frame) {
  Serial.print("test");
}