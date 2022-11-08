// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>

//for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

//callsign for this station
#define CALLSIGN "N3VEM"

//buffer setup for messages.  limit them like old-school tweets
const int MSG_BUFFER_SIZE = 140;

//
int messageType = 1; //1 relay, 2 message

//-----setup----------------------------------------------------

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa Ground Station");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

//----------main loop------------------------------------

void loop()
{
  listenForPacket();
  
}

//----------functions-------------------------------------
void listenForPacket()
{
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      buf[len]=0;
      digitalWrite(LED, HIGH);
      //RH_RF95::printBuffer("Received: ", buf, len);
      //Serial.print("Got: ");
      Serial.println((char*)buf);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      digitalWrite(LED, LOW);
      
      // Send a reply
      if (Serial.available() > 0)
      {
        
        sendFromSerial(messageType);
        
        
      }else{
        autoReply();
      }
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

//-----------------

void sendFromSerial(int type)
{
  char* packetType;
  char msgBuf[MSG_BUFFER_SIZE];
  int rlen = Serial.readBytesUntil('\n', msgBuf, MSG_BUFFER_SIZE);
        msgBuf[rlen] = 0;

  if (type == 1)
  {
    packetType = "RELAY";
   }else{
    packetType = "MESSAGE";
   }

  //Serial.println(msgBuf);

  char packetFromSerial[RH_RF95_MAX_MESSAGE_LEN];
  snprintf(packetFromSerial,
           RH_RF95_MAX_MESSAGE_LEN,
           "{"
           "\"pktType\":\"%s\","
           "\"call\":\"%s\","
           "\"msg\":\"%s\""
           "}",
           packetType,
           CALLSIGN,
           (char*)msgBuf);

  packetFromSerial[sizeof(packetFromSerial)] = 0;
  
  //Serial.print("Sending "); Serial.println(packetFromSerial);
  //Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)packetFromSerial, strlen((char*)packetFromSerial));
 
  //Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();

  
}

void autoReply()
{
  uint8_t data[] = "{\"pktType\":\"AUTOREPLY\",\"call\":\"N3VEM\",\"msg\":\"Confirmed\"}";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      //Serial.print("Sent Reply");
}
