#define EM 2
#define fan 3
#define inductive_1 9
#define inductive_2 10


#include "lowLevelControl.h"
bool EMState = false;


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
  EMState = True
  analogWrite(EM, 255);
  
  parsedMessage request;
  request = readROSSerial();
  
  if (!request.valid) {
    return;
  }

  messageToParse msg_out;
  String msg_str;
  switch (request.type)
  {
    case HAMMER_REQ: {// Perform hammering cycle. 
      analogWrite(fan, 0);
      msg_out.state = "DONE";
      msg_out.type = "HAMMER_REQ";
      msg_out.msg = "Hammer action completed";
      int hammerLength = request.data.toInt();
      hammerCycle(hammerLength);
      writeROSSerial(msg_out);
      break;}
    case IND_VALUES:
      msg_out.type = "IND_VALUES";
      msg_out.state = "DONE";
      msg_str = String(ind_a) +  ";" +  String(ind_b) ;
      msg_out.msg = msg_str;
      writeROSSerial(msg_out);
    break;
    default:
      break;
  }
}
