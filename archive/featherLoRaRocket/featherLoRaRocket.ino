#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADT7410.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_LIS2MDL.h>
#include <ArduinoJson.h>

//for feather32u4.  check Adafruit's example code for other models 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define VBATPIN A9

// Create the ADT7410 temperature sensor object
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();

// Create the ADXL343 accelerometer sensor object
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

// Create the LIS2MDL magnetomoter sensor object
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12346);

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// set transmitter power from 5 to 23 dBm (3.16mW to 199.53 mW) 
#define txPower 23

// set callsign.  Use APRS suffixes
#define CALLSIGN  "N3VEM-11"

// initial comments
char COMMENTS[20] = "I'm a rocket!";

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

StaticJsonDocument<200> doc;

//---setup------------------------------------------

void setup() 
{
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);
  Serial.println("Feather LoRa Rocket Radio Starting Up");
  radioon();
  initializeMotionTempFeather();
  initializeMagnetometer();
 
  
}


//----main loop---------------------------

void loop()
{
  delay(1); // Wait between transmits, could also 'sleep' here!
  beacon();
  delay(1);// in case we want a delay between beconing and receiveing messages
  listenForMessage();


}


//----functions----------------------------------------------------------------

void radioon()
{
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
  Serial.print("Set Power to: "); Serial.println(txPower);
  Serial.print("Max packet length: "); Serial.println(RH_RF95_MAX_MESSAGE_LEN); //251
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // use rf95 fuctions to change any defualts here
  rf95.setTxPower(txPower, false);
}

//------------------

void beacon(){
  //sensor event
  sensors_event_t event;
  
  //get power supply voltage
  float vcc=voltage();

  //get acceleromter data
  float tempC, accelX, accelY, accelZ;
  accel.getEvent(&event);
  accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  accelZ = event.acceleration.z;

  //get temperature data
  tempC = tempsensor.readTempC();

  //get magnetometer data
  float magX, magY, magZ;
  mag.getEvent(&event);
  magX = event.magnetic.x;
  magY = event.magnetic.y;
  magZ = event.magnetic.z;
    
  //Serial.println("Transmitting..."); // Send a message to rf95_server
  
  char radiopacket[RH_RF95_MAX_MESSAGE_LEN];
  snprintf(radiopacket,
           RH_RF95_MAX_MESSAGE_LEN,
           "{"
           "\"pktType\":\"BEACON\","
           "\"call\":\"%s\","
           "\"comnts\":\"%s\","
           "\"VCC\":%d.%03d,"
           "\"accel\":{\"x\":%d.%03d,\"y\":%d.%03d,\"z\":%d.%03d},"
           "\"mag\":{\"x\":%d.%03d,\"y\":%d.%03d,\"z\":%d.%03d},"
           "\"tempC\":%d.%03d,"
           "\"pktId\":%d"
           "}",
           CALLSIGN,
           COMMENTS,
           (int) vcc,
           (int) (vcc*1000)%1000,
           (int) accelX,
           abs((int) (accelX*1000)%1000),
           (int) accelY,
           abs((int) (accelY*1000)%1000),
           (int) accelZ,
           abs((int) (accelZ*1000)%1000),
           (int) magX,
           abs((int) (magX*1000)%1000),
           (int) magY,
           abs((int) (magY*1000)%1000),
           (int) magZ,
           abs((int) (magZ*1000)%1000),
           (int) tempC,
           (int) (tempC*1000)%1000,
           packetnum);

  radiopacket[sizeof(radiopacket)] = 0;

  //Serial.print("Sending "); Serial.println(radiopacket);
  //Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, strlen((char*) radiopacket));
 
  //Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  packetnum++;
}

//---------------------------------------------

void listenForMessage(){
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //Serial.println("Checking Messages...");
  if (rf95.waitAvailableTimeout(250))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      buf[len]=0;
      const char* receivedMessage = (char*)buf;
      char modifiedMessage[RH_RF95_MAX_MESSAGE_LEN];
      
      //Serial.print("Got reply: ");
      Serial.println(receivedMessage);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);  

      //mainuplating what should be the incoming json
      DeserializationError error = deserializeJson(doc, receivedMessage);
      if (error) 
          {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
          }
      else
        {
          const char* packetType = doc["pktType"];
         
          if(strcmp(packetType,"RELAY")==0)
          {
            //Serial.println(packetType);
            //Serial.println(receivedMessage);
            doc["pktType"]="MESSAGE";
            serializeJson(doc,modifiedMessage);
            message(modifiedMessage);
          }
        }
      


        
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No Messages");
  }
  
  
}

//-----------------

void message(char messageToSend[]){
   
  //Serial.print("Sending "); Serial.println(messageToSend);
  //Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)messageToSend, strlen((char*) messageToSend));
 
  //Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  packetnum++;
}

//-----------------
void initializeMotionTempFeather()
{
    Serial.println("Adafruit IO - ADT7410 + ADX343");

  /* Initialise the ADXL343 */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("no ADXL343 accelerometer detected");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_16_G);

  /* Initialise the ADT7410 */
  if (!tempsensor.begin())
  {
    Serial.println("no ADT7410 temp sesnor detected");
    while (1)
      ;
  }

  // sensor takes 250 ms to get first readings
  delay(250);
}

//-----------------

void initializeMagnetometer()
{
  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if (!mag.begin()) {  // I2C mode
    Serial.println("no LIS2MDL magnetomoter detected");
    while (1) delay(10);
  }
  mag.printSensorDetails();
}

//-----------------

long int uptime(){
  static unsigned long rollover=0;
  static unsigned long lastmillis=millis();

  //Account for rollovers, every ~50 days or so.
  if(lastmillis>millis()){
    rollover+=(lastmillis>>10);
    lastmillis=millis();
  }

  return(rollover+(millis()>>10));
}

//------------------

float voltage(){

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}
