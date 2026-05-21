#define EM 2
#define fan 3
#define inductive_1 9
#define inductive_2 10


#include "lowLevelControl.h"
bool EMState = false;
const int cooldownTime = 10;
bool cooldownActive = false;
int cooldownStart;


void setup() {

  // Inductive sensors are connected 3.3V -> sensor -> input.
  pinMode(inductive_1,INPUT_PULLUP);
  pinMode(inductive_2, INPUT_PULLUP);

  // Digital outputs
  pinMode(EM, OUTPUT);
  pinMode(fan, OUTPUT);
  analogWrite(EM, 255);
  Serial.begin(115200);
  Serial.println("setup finished.");
}

void loop() {
  analogWrite(EM, 255);

  if (cooldownActive){
    int now = millis();
    if (now - cooldownStart > (cooldownTime * 1000)){
      analogWrite(fan, 255);
      cooldownActive = false;
    }
  }
  int ind_a = digitalRead(inductive_1);
  //Serial.println("Inductive 2:");
  int ind_b = digitalRead(inductive_2);
  //Serial.println(ind_b);
  
  parsedMessage request;
  request = readROSSerial();
  
  if (!request.valid) {
    return;
  }

  messageToParse msg_out;
  String msg_str;
  switch (request.type)
  {
    int actionState;
    case FAN_TEST:
      // digitalWrite(EM, LOW);
      // delay(1000);
      // digitalWrite(EM, HIGH);
      Serial.println("setting fan low");
      analogWrite(fan, 0);
      delay(4000);
      Serial.println("setting fan HIGH");
      analogWrite(fan,255);
      delay(4000);
      analogWrite(fan, 127);
    case HAMMER_REQ: {// Perform hammering cycle. 
      analogWrite(fan, 0);
      msg_out.state = "DONE";
      msg_out.type = "HAMMER_REQ";
      msg_out.msg = "Hammer action completed";
      actionState = -1;
      int hammerLength = request.data.toInt();
      hammerCycle(hammerLength);
      writeROSSerial(msg_out);
      // analogWrite(fan, 255);
      cooldownActive = true;
      cooldownStart = millis();
      break;}
    case IND_VALUES:
      msg_out.type = "IND_VALUES";
      msg_out.state = "DONE";
      msg_str = String(ind_a) +  ";" +  String(ind_b) ;
      msg_out.msg = msg_str;
      writeROSSerial(msg_out);
    break;
    default:
      actionState = -1;
      break;
  }
}
