#include "lowLevelControl.h"

// Serial message format: MSG_TYPE|data|extra
// All fields are separated by '|'
// MSG_TYPE must match an IN_MSG_TYPE enum value (as integer)

parsedMessage readROSSerial() {
  parsedMessage result = { (IN_MSG_TYPE)0, "", "", false };

  if (!Serial.available()) {
    return result;
  }

  String raw = Serial.readStringUntil('\n');
  raw.trim();

  int first = raw.indexOf('|');
  if (first == -1) return result;

  int second = raw.indexOf('|', first + 1);

  String typeStr = raw.substring(0, first);
  result.data  = (second == -1) ? raw.substring(first + 1) : raw.substring(first + 1, second);
  result.extra = (second == -1) ? "" : raw.substring(second + 1);

  int typeInt = typeStr.toInt();
  // toInt() returns 0 on failure — verify it was actually "0" if that's the result
  if (typeInt == 0 && typeStr != "0") return result;

  result.type  = (IN_MSG_TYPE)typeInt;
  result.valid = true;
  return result;
}

int writeROSSerial(messageToParse message) {


  return 0;
}

int hammerCycle(int cycleLength) {
  analogWrite(fan, 255);

  if (!EMState) {
    return 1;
  }
  for (int i = 0; i < (cycleLength); i++) {
    EMState = !EMState;
    digitalWrite(EM, EMState);
    delay(1000);
  }
  EMState = true;
  digitalWrite(EM, EMState);

  return 0;
}
