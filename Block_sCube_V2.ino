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

#define BLOCK_ID 9

// ++ SAP - ACC
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
// -- SAP - ACC
#include <SPI.h>
#include <SPIFlash.h>
#include <RFM69registers.h>
//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#define NETWORKID     110  //the same on all nodes that talk to each other 
#define NODEID        (BLOCK_ID+1)
#define BASEID        255 

//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY      RF69_915MHZ
#define ENCRYPTKEY     "neews_key_060820" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW    true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200

#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  

#define LED           5  // blinky
#define VBATT       A11  // battery indicator

#define SERIAL_ENABLE

#ifdef SERIAL_ENABLE
  //#define F0_DISABLE
  #define SERIALOUT(s) {s;}
#else
  #define SERIALOUT(s)
#endif

#define TX_Pulse_width 200
#define RX_Pulse_width 300
#define Carrier_Pulse_width 17

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
  unsigned long timestamp;
} NeighborTableEntry;

#pragma packed
NeighborTableEntry table[6];

#define I2C_DEV_ENABLE

#ifdef I2C_DEV_ENABLE
#define I2C_CALL(s) {s;}
#else
#define I2C_CALL(s)
#endif

byte acc_x=0,acc_y=0,acc_z=0;
volatile bool i2c_scheduled = false;
// ++ SAP - ACC
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
// -- SAP - ACC

int16_t packetnum = 0;  // packet counter, we increment per xmission

byte Data[10] = {0};
byte Encoded_Data[20] = {0};
byte count_p1 = 0;
byte count_p2 = 0;
byte check_p1, check_p2;

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

void init_radio() {
  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }

  SERIALOUT(Serial.println("Radio Initilaized"));
  radio.writeReg(REG_BITRATEMSB,RF_BITRATEMSB_200KBPS);
  radio.writeReg(REG_BITRATELSB,RF_BITRATELSB_200KBPS);
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY);
}

void setup() {
  memset(table, 0xFF, sizeof(table));
  delay(1000);
  SERIALOUT(
    //while (!Serial); // wait until serial console is open, remove if not tethered to computer
    Serial.begin(SERIAL_BAUD);
    Serial.println("Feather RFM69HCW Receiver")
  )
  I2C_CALL(
      // ++ SAP - ACC
      lis.begin(0x18);
      SERIALOUT(Serial.println("LIS3DH found!");)
      lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
      SERIALOUT(
          Serial.print("Range = "); Serial.print(2 << lis.getRange());  
          Serial.println("G");
      )
      // -- SAP - ACC
    )
  init_radio();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  SERIALOUT(Serial.println("System up"));
}

#define NFMI_TIMESTAMP_LEN 40
#define NFMI_PINS_LEN 6
#define NFMI_RX_TIMEOUT 28
const byte NFMI_RX_pins[] ={15,16,14,11,10,9};

const byte tx_pins[6] = {A0, A1, A2, A3, A4, A5};
volatile byte tx_pin;

volatile byte NFMI_RX_mode = false;
volatile byte NFMI_RX_finish = false;

volatile unsigned long timestamps[NFMI_TIMESTAMP_LEN];
volatile int timestamp_ptr = 0;
volatile byte hface_id;

byte Box_id = 0;
byte Face_id = 0;


void NFMI_RX_INT_enable() {
  cli();
  SPI.end();
  for (int i = 0; i < NFMI_PINS_LEN; i++) {
    digitalWrite(NFMI_RX_pins[i],0);
    pinMode(NFMI_RX_pins[i], INPUT);
  }
  PCMSK0 |=  0xEE; //(11101110b)
  PCICR |= 1;
  sei();
}
void NFMI_RX_INT_disable() {
  cli();
  PCMSK0 &= 0x11;
  PCICR &= ~1;
  SPI.begin();
  sei();  
}

ISR(PCINT0_vect,ISR_BLOCK)
{
  // find the smoking gun
  // Serial.println("ISR begin");
  byte host_face_id;
  if (timestamp_ptr >= NFMI_TIMESTAMP_LEN) return;
  for(host_face_id = 0; host_face_id < NFMI_PINS_LEN; host_face_id++) {
    if (digitalRead(NFMI_RX_pins[host_face_id])) {
      hface_id = host_face_id;
      break;
    }
  }
//  digitalWrite(A4,!digitalRead(A4));
  timestamps[timestamp_ptr++]=micros();
  if (timestamp_ptr >= NFMI_TIMESTAMP_LEN) {
      NFMI_RX_finish = true;
      NFMI_RX_INT_disable();
  }
  delayMicroseconds(100);
}

void fillNeighbor() {
  Box_id = 0;
  Face_id = 0;
  for (int j = 0; j < 4; j++)
  {
    Box_id = Box_id + Data[j + 1] * int(pow(2, j) + 0.05); // Added 0.05 to take care of floating point rounding issues
  }
  for (int m = 0; m < 3; m++)
  {
    Face_id = Face_id + Data[m + 5] * int(pow(2, m) + 0.05); // Added 0.05 to take care of floating point rounding issues
  }
  count_p1 = 0;
  count_p2 = 0;
  check_p1 = 0;
  check_p2 = 0;
  for (int c = 1; c < 5; c++)
  {
    if (Data[c] == 1)
    {
      count_p1 = count_p1 + 1;
    }
    if (Data[c + 3] == 1)
    {
      count_p2 = count_p2 + 1;
    }
  }
  if (count_p1%2 == 0)
  {
    check_p1 = 0; 
  }
  else
  {
    check_p1 = 1;
  }
  if (count_p2%2 == 0)
  {
    check_p2 = 0;
  }
  else
  {
    check_p2 = 1;
  }
  if (check_p1 == Data[8] && check_p2 == Data[9])
  {
    SERIALOUT(Serial.print("R "));
  }
  else
  {
    SERIALOUT(Serial.print("W ")); 
    return;
  }
  table[hface_id].timestamp = millis();
  table[hface_id].csc_id = Box_id+1;
  table[hface_id].face_id = Face_id;
  
  SERIALOUT(
    Serial.print(hface_id);
    Serial.print(' ');
    Serial.print(table[hface_id].csc_id);
    Serial.print(" ");
    Serial.println(table[hface_id].face_id);
  )
}

void nf_send_bit(byte box_id, byte face_id)
{
  Data[0] = 1;
  for (int m = 0; m < 4; m++) //m is the 4-digit Box_id
  {
    if (bitRead(box_id, m) == 1)      //starting at 0 for the least-significant (rightmost) bit
      Data[m + 1] = 1;
    else
      Data[m + 1] = 0;
  }
  for (int n = 0; n < 3; n++) //n is the 3-digit face_id
  {
    if (bitRead(face_id, n) == 1)
      Data[n + 5] = 1;
    else
      Data[n + 5] = 0;
  }

  count_p1 = 0;
  count_p2 = 0;
  for (int l = 1; l < 5; l++)
  {
    if (Data[l] == 1)
    {
      count_p1 = count_p1 + 1;
    }
    if (Data[l + 3] == 1)
    {
      count_p2 = count_p2 + 1;
    }
  }
  if (count_p1 % 2 == 0)
  {
    Data[8] = 0;
  }
  else
  {
    Data[8] = 1;
  }
  if (count_p2 % 2 == 0)
  {
    Data[9] = 0;
  }
  else
  {
    Data[9] = 1;
  }

  //  if((box_id+face_id+1)%2==1)      //even parity
  //    Data[8]=0;
  //  else
  //    Data[8]=1;

  for (int w = 0; w < 10; w++)
  {
    //Serial.print(Data[w]);
    //Serial.print(" ");
  }
  //Serial.println();

  for (int i = 0; i < 10; i++)
  {
    if (Data[i] == 1)
    {
      //        digitalWrite(TX_pin, HIGH);
      for (int k = 0; k < int(TX_Pulse_width / (Carrier_Pulse_width * 2 + 10)) + 1; k++)
      {
        digitalWrite(tx_pin, HIGH);
        delayMicroseconds(Carrier_Pulse_width);
        digitalWrite(tx_pin, LOW);
        delayMicroseconds(Carrier_Pulse_width);
      }
      digitalWrite(tx_pin, LOW);
      delayMicroseconds(TX_Pulse_width*2);
//      for (int k = 0; k < int(TX_Pulse_width / (Carrier_Pulse_width * 2 + 10)) + 1; k++)
//      {
//        digitalWrite(tx_pin, LOW);
//        delayMicroseconds(Carrier_Pulse_width);
//        digitalWrite(tx_pin, LOW);
//        delayMicroseconds(Carrier_Pulse_width);
//      }
    }
    else
    {
      digitalWrite(tx_pin, LOW);
      delayMicroseconds(TX_Pulse_width*2);
//      for (int k = 0; k < int(TX_Pulse_width / (Carrier_Pulse_width * 2 + 10)) + 1; k++)
//      {
//        digitalWrite(tx_pin, LOW);
//        delayMicroseconds(Carrier_Pulse_width);
//        digitalWrite(tx_pin, LOW);
//        delayMicroseconds(Carrier_Pulse_width);
//      }
       for (int k = 0; k < int(TX_Pulse_width / (Carrier_Pulse_width * 2 + 10)) + 1; k++)
      {
        digitalWrite(tx_pin, HIGH);
        delayMicroseconds(Carrier_Pulse_width);
        digitalWrite(tx_pin, LOW);
        delayMicroseconds(Carrier_Pulse_width);
      }
    }
    //      else
    //        digitalWrite(tx_pin, LOW);
    //      delayMicroseconds(Pulse_width);
  }
  digitalWrite(tx_pin, LOW);
}

void loop() {
  //SERIALOUT(ln("Entering reception loop"));
  packetnum=(++packetnum)%(6);
  digitalWrite(LED, !digitalRead(LED));
  //memset(table, 0xFF, sizeof(table));
  unsigned long n = millis();
  for (int j = 0; j < 6; j++) {
    if (table[j].csc_id !=31 && (n-table[j].timestamp) > 1000) {
      table[j].csc_id=31;
      table[j].face_id=6;
    }
  }
  //Wait if aven't receive packet
  while (!radio.receiveDone()) {}
  //check if received message contains Hello Cube (from BS)
  CscRfPacket *volatile in_pkt = (CscRfPacket*) radio.DATA;
  //SERIALOUT(Serial.print("packet received|");Serial.println(in_pkt->header.dst));
  byte pktlength = radio.DATALEN;
  if (radio.SENDERID==0) {// packet from base station
      if (in_pkt->header.type==POLL) {
        if (in_pkt->header.dst == NODEID) {
            CscRfPacket out_packet;
            out_packet.header.src=NODEID;
            out_packet.header.dst=0;
            out_packet.header.type=RESPONSE;
            memset(out_packet.data, 0, sizeof(out_packet.data));
            // send neighbor table
            for (int j=0; j < 6; j++) {
              out_packet.data[j] = table[j].csc_id << 3 | (table[j].face_id & 7);
              Serial.print(j);
              Serial.print("=");
              Serial.print(out_packet.data[j]);
              Serial.print(',');
            }
            Serial.println();
            I2C_CALL(
              i2c_scheduled = true;
              out_packet.data[7] = acc_x;
              out_packet.data[8] = acc_y;
              out_packet.data[9] = acc_z;
            )
            // TODO: voltage value
            out_packet.data[6] = (analogRead(VBATT)>>2);
            radio.send(BASEID, &out_packet, sizeof(out_packet.header)+10);
            //TODO: turn off interrupts, start NFC send mode
            cli();
            delayMicroseconds(2000);
            for(byte face_id = 0;face_id < NFMI_PINS_LEN; face_id++) {
              tx_pin = tx_pins[face_id];
              pinMode(tx_pin, OUTPUT); // ENABLE OUTPUT MODE ON TX PIN
              nf_send_bit(BLOCK_ID, face_id); // TRANSMIT PROXIMITY SIGNAL FOR FACE ID face_id
              pinMode(tx_pin, INPUT); // ENABLE INPUT MODE ON TX PIN
              delayMicroseconds(50);
            }
            sei();
            I2C_CALL(
              if (i2c_scheduled) {
                i2c_scheduled = false;
                // TODO: acc value
                // ++ SAP - ACC
                lis.begin(0x18);
                lis.read();      // get X Y and Z data at once
                acc_x = lis.x >> 8;
                acc_y = lis.y >> 8;
                acc_z = lis.z >> 8;
                // -- SAP - ACC
              }
            )
        } else {
            NFMI_RX_mode = false;
            NFMI_RX_finish = false;
            unsigned long start = millis();
            unsigned long elapse = 0;
            timestamp_ptr=0;
            hface_id = -1;
            NFMI_RX_INT_enable();
            while(!NFMI_RX_finish && (elapse=millis()-start)< 48);
            NFMI_RX_INT_disable();
            byte data_ptr = 0;
            byte ERR_flag = 0;
            SERIALOUT(
            if (timestamp_ptr > 0) {
              Serial.print(timestamp_ptr);
              Serial.print('|');
            })
            for(int j = 0; j < (timestamp_ptr-1);j++) {
              SERIALOUT(Serial.print(
                timestamps[j+1]-timestamps[j]);
                Serial.print(','););
              unsigned long runLen = (timestamps[j+1]-timestamps[j])/(j%2==0?300:350);
              runLen += (j%2);
              if (runLen > 1) {
                runLen=2;
              } else if (runLen == 0) {
                runLen = 1;
              }
              byte sign = (j%2)==0;
              for(;runLen > 0;runLen--) {
                Encoded_Data[data_ptr++]=sign;
                if (data_ptr > 19) break;
              }
              if (data_ptr > 19) break;
            }
            while(data_ptr < 20) {
              Encoded_Data[data_ptr++]=0;
            }
            if(timestamp_ptr > 0)SERIALOUT(Serial.println(data_ptr));
            if (data_ptr > 0 && !ERR_flag) {
              //TODO: decode
              byte decode_ptr=0;
              for(byte k=1; k < data_ptr; k=k+2) {
                if (Encoded_Data[k] && !Encoded_Data[k-1]) {
                  // zero
                  Data[decode_ptr++]=0;
                } else if (!Encoded_Data[k] && Encoded_Data[k-1]) {
                  // one
                  Data[decode_ptr++]=1;
                } else {
                   return;
                }
              }
              fillNeighbor();
            }
        }
     }// END polling
  }
}
