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

  // Match string MSG_TYPE to enum value
  if (typeStr == "RETRACT_EM") {
    result.type = RETRACT_EM;
  } else if (typeStr == "HAMMER_REQ") {
    result.type = HAMMER_REQ;
  } else if (typeStr == "IND_VALUES") {
    result.type = IND_VALUES;
  } else {
    return result;
  }

  result.valid = true;
  return result;
}

int writeROSSerial(messageToParse message) {
  Serial.print(message.type);
  Serial.print("|");
  Serial.print(message.state);
  Serial.print("|");
  Serial.print(message.msg);
  Serial.print("\n");
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
