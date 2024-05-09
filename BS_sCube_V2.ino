/* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
// Get libraries at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/
#include <SPI.h>
#include <SPIFlash.h>    
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     110  // The same on all nodes that talk to each other
#define BASEID        0  // broadcast
#define NODEID        255  // BS
#define POLL_TIMEOUT  42 //original is 60 --HD
//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "neews_key_060820" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
 
//*********************************************************************************************
#define SERIAL_BAUD   115200
// Setup changed for adafruit Feather 32u4 RFM69HCW
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4
#define RFM69_RST     4

#define LED           13  // onboard blinky
#define NUMBER_OF_BLOCKS           13 

#define SPC_RX_PIN    10
#define SPC_TX_PIN    15

#define SERIAL_ENABLE

#ifdef SERIAL_ENABLE
  #define SERIALOUT(s) {s;}
#else
  #define SERIALOUT(s)
#endif

enum CscRfTypes {
  POLL=0,
  RESPONSE=1
};

#pragma packed
typedef struct {
  byte src;
  byte dst;
  byte type;
} CscRfHeader;

#pragma packed
typedef struct {
  CscRfHeader header;
  byte data[24];
} CscRfPacket;

#pragma packed
typedef struct {
  byte csc_id;
  byte face_id;
} NeighborTableEntry;

#pragma packed
typedef struct {
  byte batt;
  long last_poll;
  long elapse;
  byte acc[3];
} StatusEntry;

#pragma packed
NeighborTableEntry tables[17][6];
bool pollResponse[17];
StatusEntry blockStatus [17];

int16_t packetnum = 0;  // packet counter, we increment per xmission
 
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);
SoftwareSerial speech = SoftwareSerial(SPC_RX_PIN, SPC_TX_PIN);

volatile bool sqrWaveState;

void sqrWaveA2() {
  if(sqrWaveState) {
    digitalWrite(A2, !digitalRead(A2));
  } else {
    digitalWrite(A2, false);
  }
}

#define SPC_RETRY_CNT 300

void emitSpeech(char* str) {
  int ct = SPC_RETRY_CNT;
  while(1) {
    if(speech.read()!=':'){
      ct--;
      if (ct==0) {
        ct = SPC_RETRY_CNT;
        speech.write('\n');
      }
    } else {
      break;
    }
  }
  speech.print('S');
  speech.print(str);
  speech.print('\n');
  delay(10);
  speech.flush();
}

void setup() {
  SERIALOUT(
    while (!Serial); // wait until serial console is open, remove if not tethered to computer
    Serial.begin(SERIAL_BAUD);
    // def baud rate of speech module
    pinMode(SPC_RX_PIN, INPUT);
    pinMode(SPC_TX_PIN, OUTPUT);
    //speech.begin(9600);
    //emitSpeech("Welcome to iCube");
    
  )
  // 1ms
  Timer1.initialize(700);
  sqrWaveState = false;
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  Timer1.attachInterrupt(sqrWaveA2);
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);

  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  
  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.writeReg(REG_BITRATEMSB,RF_BITRATEMSB_200KBPS);
  radio.writeReg(REG_BITRATELSB,RF_BITRATELSB_200KBPS);
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);
  
  pinMode(LED, OUTPUT);
  memset(pollResponse,0, sizeof(pollResponse)*sizeof(bool));
}
byte block_id=6; // original value is 0 --BH
long timeo = 0;
long recvd = 0;
void sendEscSeq(const char* cmd) {
  Serial.write(27);
  Serial.print(cmd);
}
void loop() {
  block_id=(block_id+1)%(NUMBER_OF_BLOCKS + 1); // currently 10 + 1 = 11
  if (block_id==0)  {
    SERIALOUT(
        Serial.flush(); //make sure all serial data is clocked out
        sendEscSeq("[H");// go to home position
    )
    sendEscSeq("[2J");
    bool contact = false;
    bool TOH_Violation_1 = false;
    int bottom_face, top_face;
    
    for (int j = 1; j < NUMBER_OF_BLOCKS + 1; j++) {
      SERIALOUT(
        if (pollResponse[j]) {
//          switch (j-1) {
//            case 0:
//              Serial.print("Black");
//              Serial.print('\t');
//              break;
//            case 1:
//              Serial.print("Green");
//              Serial.print('\t');
//              break;
//            case 2:
//              Serial.print("Blue");
//              Serial.print('\t');
//              break;
//            case 5:
//              Serial.print("White");
//              Serial.print('\t');
//              break;
//          }
//          
          bottom_face = 64;

//          if (j==11){
//            if ((blockStatus[j].acc[0] < 4 || blockStatus[j].acc[0] > 252 ) && blockStatus[j].acc[1] > 248 && blockStatus[j].acc[1] < 253 && blockStatus[j].acc[2] > 191 && blockStatus[j].acc[2] < 195){
//              Serial.print("Face: Horizontal");
//              Serial.print('\t');
//            } else{
//              sendEscSeq("[1;31;40m");
//              Serial.print("Face: NOT Horizontal");
//              Serial.print('\t');
//              sendEscSeq("[0;37;40m");              
//              }           
//          }
//          if (j==12 || j==11){
//            if ((blockStatus[j].acc[0] < 3|| blockStatus[j].acc[0] > 254) && blockStatus[j].acc[1] > 240 && blockStatus[j].acc[1] < 245 && blockStatus[j].acc[2] > 193 && blockStatus[j].acc[2] < 199){
//              Serial.print("Face: Horizontal");
//              Serial.print('\t');
//            } else{
//              sendEscSeq("[1;31;40m");
//              Serial.print("Face: NOT Horizontal");
//              Serial.print('\t');
//              sendEscSeq("[0;37;40m");              
//              }           
//          }
//          if (j<11){            
//            if (blockStatus[j].acc[2] > 200 && blockStatus[j].acc[2] < 230 && blockStatus[j].acc[1] > 200 && blockStatus[j].acc[1] < 230) {bottom_face=3; top_face=0;}
//            if (blockStatus[j].acc[1] > 200 && blockStatus[j].acc[1] < 230 && blockStatus[j].acc[0] > 200 && blockStatus[j].acc[0] < 230) {bottom_face=1; top_face=2;}
//            if (blockStatus[j].acc[2] > 200 && blockStatus[j].acc[2] < 230 && blockStatus[j].acc[0] > 35 && blockStatus[j].acc[0] < 55) {bottom_face=2; top_face=1;}
//            if (blockStatus[j].acc[2] > 35 && blockStatus[j].acc[2] < 55 && blockStatus[j].acc[1] > 35 && blockStatus[j].acc[1] < 55)   {bottom_face=0; top_face=3;}
//            if (blockStatus[j].acc[1] > 200 && blockStatus[j].acc[1] < 230 && blockStatus[j].acc[0] > 35 && blockStatus[j].acc[0] < 55) {bottom_face=4; top_face=5;}
//            if (blockStatus[j].acc[2] > 200 && blockStatus[j].acc[2] < 230 && blockStatus[j].acc[0] > 200 && blockStatus[j].acc[0] < 230)   {bottom_face=5; top_face=4;}
//            Serial.print("On Face: ");
//            Serial.print(bottom_face);
//            Serial.print('\t');
//            }



          if (blockStatus[j].acc[2] > 200 && blockStatus[j].acc[2] < 230 && blockStatus[j].acc[1] > 200 && blockStatus[j].acc[1] < 230) {bottom_face=3; top_face=0;}
          if (blockStatus[j].acc[1] > 200 && blockStatus[j].acc[1] < 230 && blockStatus[j].acc[0] > 200 && blockStatus[j].acc[0] < 230) {bottom_face=1; top_face=2;}
          if (blockStatus[j].acc[2] > 200 && blockStatus[j].acc[2] < 230 && blockStatus[j].acc[0] > 35 && blockStatus[j].acc[0] < 55) {bottom_face=2; top_face=1;}
          if (blockStatus[j].acc[2] > 35 && blockStatus[j].acc[2] < 55 && blockStatus[j].acc[1] > 35 && blockStatus[j].acc[1] < 55)   {bottom_face=0; top_face=3;}
          if (blockStatus[j].acc[1] > 200 && blockStatus[j].acc[1] < 230 && blockStatus[j].acc[0] > 35 && blockStatus[j].acc[0] < 55) {bottom_face=4; top_face=5;}
          if (blockStatus[j].acc[2] > 200 && blockStatus[j].acc[2] < 230 && blockStatus[j].acc[0] > 200 && blockStatus[j].acc[0] < 230)   {bottom_face=5; top_face=4;}
          Serial.print("On Face: ");
          Serial.print(bottom_face);
          Serial.print('\t');



          

          Serial.print("Block ID: ");
          Serial.print(j-1);
          Serial.print('\t');
          Serial.print(blockStatus[j].last_poll);
          Serial.print('\t');
          Serial.print(((float)blockStatus[j].batt)*3.3f/127.0f);
          Serial.print('\t');
          Serial.print(blockStatus[j].acc[0]);
          Serial.print('\t');
          Serial.print(blockStatus[j].acc[1]);
          Serial.print('\t');
          Serial.print(blockStatus[j].acc[2]);
          Serial.print('\t');
          Serial.print(blockStatus[j].elapse);
          Serial.print('\t');
          for(int i = 0; i < 6; i++) {
             if (tables[j][i].csc_id!=31) { // contact found
                // contact = true; 
                sendEscSeq("[1;31;40m"); // color red

                if (i==top_face && tables[j][i].csc_id != 31 && tables[j][i].csc_id > j){ // another cube with higher id. is on top
                  TOH_Violation_1 = true;
                  
                }
             }
             Serial.print(tables[j][i].csc_id!=31?tables[j][i].csc_id-1:31);
             Serial.print('-');
             Serial.print(tables[j][i].face_id);
             Serial.print('\t');
             if (tables[j][i].csc_id!=31) 
                sendEscSeq("[0;37;40m");
          }
          Serial.println();
        } else {
          Serial.println("------ NO POLL RESPONSE -------");
        }
      )
      // sqrWaveState = contact;
      sqrWaveState = TOH_Violation_1;
    }
  }
  CscRfPacket poll;
  poll.header.src=NODEID;
  poll.header.dst=block_id+1;
  poll.header.type=POLL;
  poll.data[0]=block_id%2;
  radio.send(BASEID, &poll, sizeof(poll));
  radio.receiveDone(); //put radio in RX mode
  long poll_start = millis();
  long elapse = 0;
  delayMicroseconds(300);
  while ((elapse=millis()-poll_start) < POLL_TIMEOUT && !radio.receiveDone()){}
  blockStatus[block_id+1].elapse=elapse;
  CscRfPacket* in_pkt = (CscRfPacket*) radio.DATA;
  if (elapse< POLL_TIMEOUT && in_pkt->header.type==RESPONSE) {
      //TODO: process response packet
      pollResponse[in_pkt->header.src] = true;
      blockStatus[in_pkt->header.src].last_poll = elapse;  //CHANGE BACK TO JUST millis()
      for(int i = 0; i < 6; i++) {
        tables[in_pkt->header.src][i].csc_id = (in_pkt->data[i]>>3);
        tables[in_pkt->header.src][i].face_id = (in_pkt->data[i]&7);
      }
      // TODO: read battery and ACC
      blockStatus[in_pkt->header.src].batt = in_pkt->data[6];
      blockStatus[in_pkt->header.src].acc[0] = in_pkt->data[7];
      blockStatus[in_pkt->header.src].acc[1] = in_pkt->data[8];
      blockStatus[in_pkt->header.src].acc[2] = in_pkt->data[9];
  } else {
      pollResponse[block_id+1]=false;
  }
  // wait until the next time slot
  radio.receiveDone();
  while ((millis()-poll_start < POLL_TIMEOUT)){}
}
