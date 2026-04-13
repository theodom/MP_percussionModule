#define EM D2
#define fan D3
#define inductive_1 D9
#define inductive_2 D10

#include "lowLevelControl.h"

// --------- Global Variables ----------
bool EMState = false;


void setup() {

  // Inductive sensors are connected 3.3V -> sensor -> input.
  pinMode(inductive_1, INPUT_PULLDOWN);
  pinMode(inductive_2, INPUT_PULLDOWN);

  // Digital outputs
  pinMode(EM, OUTPUT);
  pinMode(fan, OUTPUT);

  Serial.begin(115200);
  // while (!Serial.available());
}

void loop() {
  parsedMessage request;
  request = readROSSerial();

  switch (request.type)
  {
    int actionState;
    case RETRACT_EM:  // Retract EM and enable fan max speed.
      analogWrite(fan, 255);
      EMState = true;
      digitalWrite(EM, EMState);
      actionState = 0;
      break;
    case HAMMER_REQ: // Perform hammering cycle. 
      actionState = hammerCycle();
      break;
    default:
      actionState = -1;
      break;
  }



}
