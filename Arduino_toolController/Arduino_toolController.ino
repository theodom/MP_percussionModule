#define EM D2
#define fan D3
#define inductive_1 D9
#define inductive_2 D10


#include "lowLevelControl.h"
bool EMState = false;
const int cooldownTime = 10;
bool cooldownActive = false;
int cooldownStart;


void setup() {

  // Inductive sensors are connected 3.3V -> sensor -> input.
  pinMode(inductive_1,INPUT_PULLDOWN);
  pinMode(inductive_2, INPUT_PULLDOWN);

  // Digital outputs
  pinMode(EM, OUTPUT);
  pinMode(fan, OUTPUT);
  digitalWrite(EM, HIGH);
  Serial.begin(115200);
  // while (!Serial.available());


  analogWrite(fan, 200);
  delay(1000);
  analogWrite(fan, 100);
  delay(1000);
  analogWrite(fan, 255);
  Serial.println("setup finished.");
}

void loop() {
  digitalWrite(EM, HIGH);

  if (cooldownActive){
    Serial.println("coolign down");
    int now = millis();
    if (now - cooldownStart > (cooldownTime * 1000)){
      analogWrite(fan, 255);
      cooldownActive = false;
    }
  }
  else {
    Serial.println("not");
    Serial.println(cooldownActive);
  }
  parsedMessage request;
  request = readROSSerial();

  if (!request.valid) {
    return;
  }

  int ind_a = digitalRead(inductive_1);
  int ind_b = digitalRead(inductive_2);
  messageToParse msg_out;
  String msg_str;
  switch (request.type)
  {
    int actionState;
    case HAMMER_REQ: // Perform hammering cycle. 
      analogWrite(fan, 0);
      msg_out.state = "DONE";
      msg_out.type = "HAMMER_REQ";
      actionState = -1;
      hammerCycle();
      writeROSSerial(msg_out);
      // analogWrite(fan, 255);
      cooldownActive = true;
      cooldownStart = millis();
      break;
    case IND_VALUES:
      msg_out.type = "IND_VALUES";
      msg_out.state = "DONE";
      msg_str = "(" + String(ind_a) +  ";" +  String(ind_b) + ")";
      msg_out.msg = msg_str;
      writeROSSerial(msg_out);
      break;
    default:
      actionState = -1;
      break;
  }
}
