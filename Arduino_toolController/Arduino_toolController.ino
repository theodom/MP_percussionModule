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

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  Serial.println("setup finished.");
}

void loop() {
  parsedMessage request;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  request = readROSSerial();

  if (!request.valid) {
    return;
  }

  int ind_a = digitalRead(inductive_1);
  int ind_b = digitalRead(inductive_2);

  digitalWrite(LED_BUILTIN, LOW);
  messageToParse msg_out;
  String msg_str;
  switch (request.type)
  {
    int actionState;
    case RETRACT_EM:  // Retract EM and enable fan max speed.
      analogWrite(fan, 255);
      EMState = true;
      digitalWrite(EM, EMState);

      break;
    case HAMMER_REQ: // Perform hammering cycle. 
      msg_out.state = "DONE";
      msg_out.type = "HAMMER_REQ";
      actionState = -1;
      actionState = hammerCycle();

      while (actionState != 0){
        delay(1);
      }
      writeROSSerial(msg_out);
      break;
    case IND_VALUES:

      
      msg_out.type = "IND_VALUES";
      msg_out.state = "DONE";
      msg_str = "(" + String(ind_a) + ";" +  String(ind_b) + ")";
      msg_out.msg = msg_str;
      writeROSSerial(msg_out);
      break;
    default:
      actionState = -1;
      break;
  }



}
