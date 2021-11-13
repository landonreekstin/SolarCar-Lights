#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <canAddresses.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port

IntervalTimer timer;

#define led 13

// ---Light Modules---
// Front:
// - left turn signal
// - right turn signal
// - DTRL
// Rear:
// - left turn signal
// - right turn signal
// - brake
// Strobe:
// - strobe
// - brake

//Lights Pins
#define Right_Pin 21 // 21 IN 4
#define Left_Pin 2  // 2 IN 2
#define DTRL_right 17 // 17 IN 3 this is only set for strobe lights lights, //11
#define DTRL_left 0 // 0 IN 1 Powers lights which require seperate power signal

//CAN Commands
#define leftTurnCmd 0x4C
#define rightTurnCmd 0x52
#define DTRLCmd 0x44
//turn signal blink frequency
#define blink_frequency 1
#define duty_cycle 125
//Setting the blinking time before cancellation
#define blinktime 10000//10 second blink time cahnge to 60 second

//Light Signals
bool leftsig = false;
bool rightsig = false;
bool DTRLsig = false;
bool hazardsig = false;
bool brakelightsig = false;
bool StrobeSig = false;

// Timer variables
unsigned int leftTimerStart = 0;
unsigned int leftTimerEnd = 0;
unsigned int rightTimerStart = 0;
unsigned int rightTimerEnd = 0;

// Horn state
int hornWorking = 0;

void lightTest() {
   digitalWrite(Left_Pin, HIGH);
   delay(1000);
   digitalWrite(Right_Pin, HIGH);
   delay(1000);
   digitalWrite(DTRL_left, HIGH);
   delay(1000);
   digitalWrite(DTRL_right, HIGH);
   delay(1000);
   digitalWrite(Left_Pin, LOW);
   delay(1000);
   digitalWrite(Right_Pin, LOW);
   delay(1000);
   digitalWrite(DTRL_left, LOW);
   delay(1000);
   digitalWrite(DTRL_right, LOW);
   delay(1000);
}

void lightTest2() {
   digitalWrite(Left_Pin, HIGH);
   digitalWrite(Right_Pin, HIGH);
   digitalWrite(DTRL_right, HIGH);
   digitalWrite(DTRL_left, HIGH);
}

void updateLights() {
   // turn signals
   
   if (millis() >= leftTimerEnd) {
      digitalWrite(Left_Pin, LOW);
      leftsig = false;
   }
   
   
   if (millis() >= rightTimerEnd) {
      digitalWrite(Right_Pin, LOW);
      rightsig = false;
   }

   // hazards
   if (hazardsig) {
      digitalWrite(Left_Pin, HIGH);
      digitalWrite(Right_Pin, HIGH);
   }
   else if (!hazardsig && !(leftsig || rightsig)) {
      digitalWrite(Left_Pin, LOW);
      digitalWrite(Right_Pin, LOW);
   }
   

   /*
   // control lights based on flags
   // on left signal
   if (leftsig)
   {
      Serial.println("Left ON");
      tone(Left_Pin, blink_frequency, blinktime);
   }
   // on right signal
   if (rightsig)
   {
      Serial.println("Right ON");
      tone(Right_Pin, blink_frequency, blinktime);
   } */
   // on DTRL
   if (DTRLsig)   // DTLights are active LO
   {
      Serial.println("Day time  light ON");
      digitalWrite(DTRL_right, HIGH);
      digitalWrite(DTRL_left, HIGH);
   }
   else
   {
      //Serial.println("Day time OFF");
      digitalWrite(DTRL_right, LOW);
      digitalWrite(DTRL_left, LOW);
   }
}

/**
* Sends the state of the front lights to the rear module. 
The rear module then sends the state of the front and rear lights to the strobe module.
Lastly, the strobe sends the state of all lights in one message to the steering wheel.
Final message format:
Byte 0:
0 - leftsig
1 - rightsig
2 - DTRLsig
3 - hazardsig
4 - brakelightsig
5 - strobesig
*/
void sendLights()
{
   // Light states needed to send to screen
   CAN_message_t F_Light_State_MSG;
   F_Light_State_MSG.id = F_LIGHTS;      // message to be sent to rear lights for light states to be added
   bitWrite(F_Light_State_MSG.buf[0], 0, leftsig);
   bitWrite(F_Light_State_MSG.buf[0], 1, rightsig);
   bitWrite(F_Light_State_MSG.buf[0], 4, DTRLsig);
   bitWrite(F_Light_State_MSG.buf[0], 5, hazardsig);
   for (size_t i = 1; i < 8; i++)
   {
      F_Light_State_MSG.buf[i] = 0x00;
   }
}

void sendframe()
{
   sendLights(); //Send light status
   updateLights();
}

/**
* decodes a message and reacts depending on the message ID.
* the "printRawCanMessage" variable is a debug option to output
* raw form of can messages for checkup. should be set to "false"
* unless needed.
*/
void ReadCanBus(const CAN_message_t &msg)
{ // global callback
   // for DEBUG purposes
   bool printRawCanMessage = true;
   if (printRawCanMessage)
   {
      Serial.print(" BUS ");
      Serial.println(msg.bus, HEX);
      Serial.print(" TS: ");
      Serial.println(msg.timestamp);
      Serial.print(" ID: ");
      Serial.println(msg.id, HEX);
      Serial.print(" Buffer: ");
      for (uint8_t i = 0; i < msg.len; i++)
      {
         Serial.print(msg.buf[i], HEX);
         Serial.print(" ");
      }
   }

   // lights flag activation switch
   switch (msg.id)
   {
      case DC_TURN_SIGNALS:
         Serial.println("Receiving turn signal");
         // turn lights
         if (msg.buf[0] == leftTurnCmd)
         {
            leftsig = true;
            leftTimerStart = millis();
            leftTimerEnd = leftTimerStart + blinktime;
            digitalWrite(Left_Pin, HIGH);
            digitalWrite(Right_Pin, LOW);
            Serial.println("Left turn signal true.");
         }
         else if (msg.buf[0] == rightTurnCmd)
         {
            rightsig = true;
            rightTimerStart = millis();
            rightTimerEnd = rightTimerStart + blinktime;
            digitalWrite(Right_Pin, HIGH);
            digitalWrite(Left_Pin, LOW);
            Serial.println("Right turn signal true.");
         }
         // strobe light are a rear function
         break;

      case SWITCH_PANEL:
         Serial.println("Receiving switch Panel");
         // DTRL
         if (bitRead(msg.buf[0], 6) == 1) {     
            DTRLsig = true;
            Serial.println("DTRL on.");
         }
         else {
            DTRLsig = false;
         }
         // Hazards
         if (bitRead(msg.buf[0], 4) == 1) {    
            hazardsig = true;
            Serial.println("Hazards on.");
         }
         else {
            hazardsig = false;
         }
         
         break;
   }// end of SWITCH
}

// Teensy runtime config
void setup()
{
  // front lights pin config
  Serial.println("Front lights");
  
  // initialize the digital pins as an output.
  pinMode(DTRL_left, OUTPUT);
  pinMode(Right_Pin, OUTPUT);
  pinMode(Left_Pin, OUTPUT);
  pinMode(DTRL_right, OUTPUT);
  pinMode(led, OUTPUT);

  // can2 channel config
  can1.begin();
  can1.setBaudRate(500000); // 500kbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(FIFO, ReadCanBus);
  can1.mailboxStatus();

  timer.begin(sendframe, 50000); // Send frame every 50ms--100ms
}

void loop()
{
   //lightTest2();
  analogWrite(led, 25);   // status led
  can1.events();
  //lightTest();
}