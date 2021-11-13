#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <canAddresses.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port. Lights MUST be on Can1

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
#define Left_Pin 2  //2 IN 3
#define Brake_Pin1 0 //0 IN 1
#define Brake_Pin2 17  // 17 IN 2

//CAN Commands
#define leftTurnCmd 0x4C
#define rightTurnCmd 0x52
#define brakeCmd 0x42
#define HarzardONCmd 0x48
#define HarzardOFFCmd 0x50
//turn signal blink frequency
#define blink_frequency 2
// analogWrite value for light blinking
#define duty_cycle 125
//Setting the blinking time before cancellation
#define blinktime 10000 //10 second blink time cahnge to 60 second

//Light Signals
bool leftsig = false;
bool rightsig = false;
int leftSignal = 0, rightSignal = 0;
bool hazardsig = false;
bool brakelightsig = false;

// Light states from Front lights
bool F_leftsig = false;
bool F_rightsig = false;
bool F_hazardsig = false;
bool F_DTRLsig = false;

// Horn state
int hornWorking = 0;

void lightTest() {  
   digitalWrite(Left_Pin, HIGH);
   delay(1000);
   digitalWrite(Left_Pin, LOW);
   delay(1000);
   digitalWrite(Right_Pin, HIGH);
   delay(1000);
   digitalWrite(Right_Pin, LOW);
   delay(1000);
   digitalWrite(Brake_Pin1, HIGH);
   delay(1000);
   digitalWrite(Brake_Pin1, LOW);
   delay(1000);
}

void lightTest2() {
   digitalWrite(Left_Pin, HIGH);
   digitalWrite(Right_Pin, HIGH);
   digitalWrite(Brake_Pin1, HIGH);
}

void turnSignalTest() {
   digitalWrite(Left_Pin, HIGH);
   digitalWrite(Right_Pin, HIGH);
}

void updateLights() {
   // on hazard signal
   if (hazardsig)
   {
      Serial.print("Hazards on");
      //analogWrite(Right_Pin, duty_cycle);
      tone(Right_Pin, blink_frequency, blinktime);
      analogWrite(Left_Pin, duty_cycle);
      Serial.print("Hazards should be blinking");
   }else{ // no hazard so we check for turn signals before shutting
      // stuff down
      if (!rightsig)
         noTone(Right_Pin);
         rightSignal = 0;
      if (!leftsig)
         analogWrite(Left_Pin, 0);
         leftSignal = 0;
   }
  
   // control lights based on flags
   // on brake signal
   if (brakelightsig)
   {
      Serial.println("Brake light ON");
      digitalWrite(Brake_Pin1, HIGH);
      digitalWrite(Brake_Pin2, HIGH);
   }
   else
   {
      //Serial.println("Brakelight OFF");
      digitalWrite(Brake_Pin1, LOW);
      digitalWrite(Brake_Pin2, LOW);
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
   CAN_message_t R_Lights_MSG;
   R_Lights_MSG.id = R_LIGHTS;
   // turn signals and hazards
   bitWrite(R_Lights_MSG.buf[0], 0, F_leftsig);
   bitWrite(R_Lights_MSG.buf[0], 1, F_rightsig);
   bitWrite(R_Lights_MSG.buf[0], 2, leftsig);
   bitWrite(R_Lights_MSG.buf[0], 3, rightsig);
   bitWrite(R_Lights_MSG.buf[0], 4, F_DTRLsig);
   bitWrite(R_Lights_MSG.buf[0], 5, F_hazardsig);
   bitWrite(R_Lights_MSG.buf[0], 6, hazardsig);
   bitWrite(R_Lights_MSG.buf[0], 7, brakelightsig);

   for (size_t i = 1; i < 8; i++)
   {
      R_Lights_MSG.buf[i] = 0x00;
   }
   can1.write(R_Lights_MSG);
}

void sendframe()
{
   //updateLights(); // Update light states
}

/**
* decodes a message and reacts depending on the message ID.
* the "printRawCanMessage" variable is a debug option to output
* raw form of can messages for checkup. should be set to "false"
* unless needed.
*/
void ReadCanBus(const CAN_message_t &msg)
{ // global callback
   bool printRawCanMessage = false;
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
      case F_LIGHTS:
         // Forward front light states with rear states to strobe
         F_leftsig = bitRead(msg.buf[0], 0);
         F_rightsig = bitRead(msg.buf[0], 1);
         F_DTRLsig  = bitRead(msg.buf[0], 4);
         F_hazardsig  = bitRead(msg.buf[0], 5);
         sendLights(); //Send light status
      break;

      case DC_TURN_SIGNALS:
         // turn lights
         switch(msg.buf[0]){
            case leftTurnCmd:
               leftSignal++;
               if(leftSignal == 1){
                  leftsig = true;
                  tone(Left_Pin, blink_frequency, blinktime);
                  Serial.println("Left ON");
               }else{ // leftSignal > 1
                  leftsig = false;
                  leftSignal = 0;
               }
               break;
            case rightTurnCmd:
               rightSignal++;
               if(rightSignal == 1){
                  rightsig = true;
                  tone(Right_Pin, blink_frequency, blinktime);
                  Serial.println("Right ON");
               }else{ // rightSignal > 1
                  rightsig = false;
                  rightSignal = 0;
               }
               break;
         }
         break;
         
      case DC_SWITCH: // brakes signal
         brakelightsig = msg.buf[1] == brakeCmd ? true : false;
         break;
      case BRAKES_REAR: // brakes signal from pedal
         brakelightsig = msg.buf[0] == brakeCmd ? true : false;
         if(brakelightsig)
            Serial.println("Brake signal received");
         break;

      case SWITCH_PANEL:
         // Hazards
         hazardsig = bitRead(msg.buf[0], 4) == HIGH ? true : false;
         if (hazardsig)
            Serial.println("Hazards signal received");
         break;

      
   }// end of SWITCH
}

// Teensy runtime config
void setup()
{
   // rear lights pin config
   Serial.println("Rear lights");

   // initialize the digital pins as an output.
   pinMode(Brake_Pin1, OUTPUT);
   pinMode(Brake_Pin2, OUTPUT);
   pinMode(Right_Pin, OUTPUT);
   pinMode(Left_Pin, OUTPUT);
   pinMode(led, OUTPUT);
   analogWriteFrequency(Left_Pin, blink_frequency);
   analogWriteFrequency(Right_Pin, blink_frequency);
   
   // can1 channel config
   can1.begin();
   can1.setBaudRate(500000); // 500kbps data rate
   can1.enableFIFO();
   can1.enableFIFOInterrupt();
   can1.onReceive(FIFO, ReadCanBus);
   can1.mailboxStatus();

   timer.begin(sendframe, 50000); // Send frame every 50ms--100ms
   Serial.println("Setup run");
   //digitalWrite(Left_Pin, HIGH);
   //lightTest();
}

void loop()
{
   //Serial.println("test");
   //lightTest2();
   analogWrite(led, 10);  // status led
   can1.events();
   //lightTest();
   updateLights();
}