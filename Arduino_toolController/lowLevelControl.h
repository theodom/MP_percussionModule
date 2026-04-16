#ifndef LOW_LEVEL_CONTROL_H
#define LOW_LEVEL_CONTROL_H

#include <Arduino.h>

enum IN_MSG_TYPE {
  RETRACT_EM,  // Enable electromagnet to retract hammer
  HAMMER_REQ,  // Request hammer action
  IND_VALUES,  // ROS requests the inductive sensor values
  FAN_TEST,
};

enum OUT_MSG_TYPE {
  OUT_IND_VALUES,  // Send inductive sensor values to ROS
  HAMMER_DONE,
};

struct parsedMessage {
  IN_MSG_TYPE type;
  String data;
  String extra;
  bool valid;
};

struct messageToParse {
  String type;
  String state;
  String msg;
};

parsedMessage readROSSerial();
int hammerCycle(int cycleLength = 6);
int writeROSSerial(messageToParse message);

#endif
