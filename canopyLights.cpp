#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <canAddresses.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; // can1 port

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
#define Brake_Pin 11 //IN 4 This is only set for rear lights, same as horn pin
#define Strobe_Pin 5 // Same as DTRL since only used in Rear, //11

#define duty_cycle 125

//CAN Commands
#define brakeCmd 0x42
#define StrobeCMD 0x77
//turn signal blink frequency
#define blink_frequency 1
//Setting the blinking time before cancellation
#define blinktime 10000//10 second blink time cahnge to 60 second

//Light Signals
bool leftsig = false;
bool rightsig = false;
bool DTRLsig = false;
bool hazardsig = false;
bool brakelightsig = false;
bool StrobeSig = true;

// Light states from Front lights
bool F_leftsig = false;
bool F_rightsig = false;
bool F_hazardsig = false;
bool F_DTRLsig = false;
// Light states from Rear lights
bool R_leftsig = false;
bool R_rightsig = false;
bool R_hazardsig = false;
bool R_brakelightsig = false;

// Horn state
int hornWorking = 0;

// save state
uint32_t last_message = 0x00;
int times = 0;

void lightTest() {         // turns all lights on
   digitalWrite(Strobe_Pin, HIGH);
   //digitalWrite(Brake_Pin, HIGH);
   // digitalWrite(5, HIGH);
   //digitalWrite(7, HIGH);
   //digitalWrite(11, HIGH);
   delay(3000);
   digitalWrite(Strobe_Pin, LOW);
   digitalWrite(Brake_Pin, HIGH);
   delay(3000);
   digitalWrite(Brake_Pin, LOW);
   // digitalWrite(5, LOW);
   // digitalWrite(7, LOW);
}

void updateLights() {
   // control lights based on flags
   // on brake signal
   if (brakelightsig)
   {
      Serial.println("Brake light ON");
      digitalWrite(Brake_Pin, HIGH);
   }
   else
   {
      //Serial.println("Brakelight OFF");
      digitalWrite(Brake_Pin, LOW);
   }
   // on strobe signal
   if (StrobeSig)
   {
      Serial.println("Strobe ON");
      tone(Strobe_Pin, blink_frequency);
   }
   else
   {
      noTone(Strobe_Pin);
   }
}

/**
* Sends the state of the front lights to the rear module. 
The rear module then sends the state of the front and rear lights to the strobe module.
Lastly, the strobe sends the state of all lights in one message to the steering wheel.
Final message format:
Byte 0:
0 - F leftsig 
1 - F rightsig
2 - R leftsig 
3 - R rightsig
4 - DTRLsig
5 - F hazardsig
6 - R hazardsig
7 - R brakelightsig
Byte 1:
0 - S brakelightsig
1 - strobesig
*/
void sendLights()
{
   CAN_message_t Lights_State_MSG;
   Lights_State_MSG.id = LIGHTS_STATE;
   // turn signals and hazards
   bitWrite(Lights_State_MSG.buf[0], 0, F_leftsig);
   bitWrite(Lights_State_MSG.buf[0], 1, F_rightsig);
   bitWrite(Lights_State_MSG.buf[0], 2, R_leftsig);
   bitWrite(Lights_State_MSG.buf[0], 3, R_rightsig);
   bitWrite(Lights_State_MSG.buf[0], 4, F_DTRLsig);
   bitWrite(Lights_State_MSG.buf[0], 5, F_hazardsig);
   bitWrite(Lights_State_MSG.buf[0], 6, R_hazardsig);
   bitWrite(Lights_State_MSG.buf[0], 7, R_brakelightsig);

   bitWrite(Lights_State_MSG.buf[1], 0, brakelightsig);
   bitWrite(Lights_State_MSG.buf[1], 1, StrobeSig);

   for (size_t i = 2; i < 8; i++) {
      bitWrite(Lights_State_MSG.buf[1], i, 0);
   }

   for (size_t i = 2; i < 8; i++)
   {
      Lights_State_MSG.buf[i] = 0x00;
   }
   can2.write(Lights_State_MSG);
}

void sendframe()
{
   sendLights();
}

/**
* decodes a message and reacts depending on the message ID.
* the "printRawCanMessage" variable is a debug option to output
* raw form of can messages for checkup. should be set to "false"
* unless needed.
*/
void ReadCanBus(const CAN_message_t &msg)
{ // global callback
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
      case R_LIGHTS:
      F_leftsig = bitRead(msg.buf[0], 0);
         F_rightsig = bitRead(msg.buf[0], 1);
         R_leftsig = bitRead(msg.buf[0], 2);
         R_rightsig = bitRead(msg.buf[0], 3);
         F_DTRLsig  = bitRead(msg.buf[0], 4);
         F_hazardsig  = bitRead(msg.buf[0], 5);
         R_hazardsig = bitRead(msg.buf[0], 6);
         R_brakelightsig = bitRead(msg.buf[0], 7);
         sendLights(); //Send light status
      break;
     case DC_SWITCH: // brakes signal
         brakelightsig = msg.buf[1] == brakeCmd ? true : false;
         break;
      case BRAKES_REAR: // brakes signal from pedal
         brakelightsig = msg.buf[0] == brakeCmd ? true : false;
         if(brakelightsig)
            Serial.println("Brake signal received");
         break;
      case SWITCH_PANEL: // Hazards
         hazardsig = bitRead(msg.buf[0], 4) ? true : false;
         if (hazardsig) 
            Serial.println("Hazards signal read");
         break;
      /*case STROBE: // strobe signal
         StrobeSig = msg.buf[0] == StrobeCMD ? true : false;
         if (StrobeSig)
            Serial.println("Receiving Strobe");
      break;
      */
      case BMS_PACK_1:
      {
         Serial.println("battery message");
         byte relayStatus = msg.buf[6];
         bool dsgOn = bitRead(relayStatus, 0) == 1;
         bool cgOn = bitRead(relayStatus, 1) == 1;
         StrobeSig = (dsgOn && cgOn) ? false : true;
         Serial.println("dsgOn and cgOn ? " + String(dsgOn && cgOn));
         break;
      }
      case EMERGENCY: // emergency broadcast
         if (msg.buf[0] == 0x32) {
            while(32) // everlasting loop
               StrobeSig = true;
         }
      break;
   }// end of SWITCH
   last_message = msg.id; // record id of last message
}

void resetStrobe(){
   times = last_message != BMS_PACK_1 ? times + 1 : 0;
   if (times >= 100){
      StrobeSig = true; // reset strobe since BMS unheard of
   }
}


// Teensy runtime config
void setup()
{
   // initialize the digital pins as an output.
   pinMode(Brake_Pin, OUTPUT);
   pinMode(Strobe_Pin, OUTPUT);
   pinMode(led, OUTPUT);
   analogWriteFrequency(Strobe_Pin, duty_cycle);
   
   // can1 channel config
   can2.begin();
   can2.setBaudRate(500000); // 500kbps data rate
   can2.enableFIFO();
   can2.enableFIFOInterrupt();
   can2.onReceive(FIFO, ReadCanBus);
   can2.mailboxStatus();

   timer.begin(sendframe, 50000); // Send frame every 50ms--100ms
   //imer.begin(resetStrobe, 2000000);

   analogWrite(led, 15);
}

void loop()
{
   can2.events();
   updateLights();
   resetStrobe();
   //lightTest();
} 