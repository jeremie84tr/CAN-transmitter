#include <Arduino.h>
#include "../include/can.h"

// put function declarations here:
void readInformation(CANFrame* frame);
void printFrames();
int boolsToInt(bool* bools, int start);

int rxGPIO = 12;
int txGPIO = 13;

CAN* can;
int nbFrames;
CANFrame* frames;

void setup() {
  Serial.begin(9600);
  Serial.print("Hello setup");
  can = new CAN(rxGPIO, txGPIO, 125000);
  frames = new CANFrame[32];
  usleep(100000);
  can->listen(readInformation);
  usleep(100000);
}

void loop() {
  // put your main code here, to run repeatedly:
  usleep(1000000);
  printFrames();
}

// put function definitions here:
void readInformation(CANFrame* frame) {
  bool found = false;

  for (int i = 0; i < nbFrames; i++) {
    if (frame->arbitrationID == frames[i].arbitrationID) {
      frames[i].control = frame->control;
      frames[i].data = frame->data;
      found = true;
    }
  }

  if (!found && nbFrames < 32) {
    frames[nbFrames].arbitrationID = frame->arbitrationID;
    frames[nbFrames].control = frame->control;
    frames[nbFrames].data = frame->data;
    nbFrames++;
  }
}

void printFrames() {
  Serial.flush();
  Serial.print("there are ");
  Serial.print(nbFrames);
  Serial.print(" Frames\n");
  int dataLength;
  for (int i = 0; i < nbFrames; i++) {
    dataLength = frames[i].control & 0b1111;
    Serial.print("frame ID :");
    Serial.print(frames[i].arbitrationID);
    Serial.print(",");
    if (frames[i].arbitrationID < 1000) {
      Serial.print("   ");
    }
    Serial.print(" \tdataLength :");
    Serial.print(dataLength);
    for (int index = 0; index < dataLength; index++) {
      Serial.print(" ");
      Serial.print(char(boolsToInt(frames[i].data, index * 8)));
    }
    if (i % 2 == 1) {
      Serial.print("\n");
    } else {
      Serial.print("\t");
    }
  }
  Serial.print("\n");
}

int boolsToInt(bool* bools, int start) {
  int ret = 0;
  for (int i = start; i < start + 8; i++) {
    ret = ret << 1;
    ret |= 0b00000001 & bools[i];
  }
  return ret;
}