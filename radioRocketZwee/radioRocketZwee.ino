#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BME680.h>
#include <Adafruit_APDS9960.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <avr/wdt.h>
#include <SD.h>


#define SEALEVELPRESSURE_HPA (1013.25)

#define LAUNCH_DETECT_THRESHOLD 19.61 // m/s^2 that would indicate enough accleration to assume lanuch has occured.  19.61 = ~2g
#define RECOVERY_HEIGHT_THRESHOLD 100 //height in feet to reach, before pyrotenic recovery will be armed
#define PRIMARY_RECOVERY_DEPLOY_ALT 300 //height in feet at which primary recovery should deploy 

#define RELAY_A_PIN 5
#define RELAY_B_PIN 6

#define RFM95_RST     9
#define RFM95_CS      10
#define RFM95_INT     4
#define RF95_FREQ 434.0 //set frequency of radio
#define txPower 23 // set transmitter power from 5 to 23 dBm (3.16mW to 199.53 mW)

#define CALLSIGN "N3VEM"
#define MESSAGE_WAIT 250 //100 seems to be functional minimum, but ground station can't quite keep up at that rate.  250 seems to be where ground station can keep up. change this to set how long to wait for received messages. longer receives better, but delays loop. If messages seem to be getting missed, increasing this will probably solve it.

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver

Adafruit_BMP3XX bmp;
  double bmp_temp = 0.00;
  double bmp_hPa = 0.00;
  int bmp_altitude = 0;
  double baseline_pressure = 0.0;

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
  double highMPSSx, highMPSSy, highMPSSz, highGx, highGy, highGz = 0.0;

Adafruit_LSM6DSOX lsm6ds;
  double lowMPSSx, lowMPSSy, lowMPSSz, lowGx, lowGy, lowGz, gyroX, gyroY, gyroZ = 0.0;
  
Adafruit_LIS3MDL lis3mdl;
  double magX, magY, magZ = 0.0;

Adafruit_BME680 bme; //this one doesn't seem to want to work? Might be a hardware issue as it won't work even when it's the only connected device, and running only the example sketch
  double bme_temp = 0.00;
  double bme_hPa = 0.00;
  double bme_humid = 0.00;
  double bme_gas = 0.0;
  double bme_altitude = 0.0;

Adafruit_APDS9960 apds;
  int proximityValue = 0;
  uint8_t gesture;

int packetId = 0;

double maxMPSS = 0.0;
int maxAltitude = 0;

bool launchDetected = false;
bool apogeeDetected = false;
bool seperationDetected = false;

char datafile[15] = "dataLog.txt";

StaticJsonDocument<500> dataPacket;

StaticJsonDocument<100> tinyPacket; //idea with this one is to basically clear it, put only a sensors worth of data in, and then send it.

StaticJsonDocument<500> receivedPacket;

StaticJsonDocument<500> eventPacket;

File dataLog;

void setup() {
  Serial.begin(115200);
  //while (!Serial);

  Serial.println("initializing...");

  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("SD Card Initialization Failed!");
  }else{
    Serial.println("SD Card Ready");
    dataLog = SD.open(datafile, FILE_WRITE); 
  }
  
  setupRadio();  
  setupBMP390();
  setupADXL343();
  setupLSM6DSOXandLIS3MDL();
  //setupBME680();
  setupAPDS9960();

  pinMode(RELAY_A_PIN, OUTPUT);
  pinMode(RELAY_B_PIN, OUTPUT);

  digitalWrite(RELAY_A_PIN, LOW);
  digitalWrite(RELAY_A_PIN, LOW);

  initializeAltitude();

  dataPacket["id"] = packetId;
  dataPacket["sz"] = measureJson(dataPacket);
  dataPacket["call"] = CALLSIGN;
  dataPacket["evnt"]["lnch"] = launchDetected;
  dataPacket["evnt"]["apg"] = apogeeDetected;

  dataLog.println("Initialization Complete");
  sendPktChar("Initialization Complete, ready for launch");
  dataLog.flush();
}

void loop() {
  
  //char packet[RH_RF95_MAX_MESSAGE_LEN];
  char charArray[80];
  char vxSign = '+';
  char vySign = '+';
  char vzSign = '+';
  char mxSign = '+';
  char mySign = '+';
  char mzSign = '+';
  char gxSign = '+';
  char gySign = '+';
  char gzSign = '+';
  
 
  getAllValues();
  checkEvents();

  if(launchDetected && apogeeDetected)
  {
  tone(2,2500);
  }

  if(highMPSSx >= 0.00){vxSign = '+';}else{vxSign = '-';}
  if(highMPSSy >= 0.00){vySign = '+';}else{vySign = '-';}
  if(highMPSSz >= 0.00){vzSign = '+';}else{vzSign = '-';}

  if(gyroX >= 0.00){gxSign = '+';}else{gxSign = '-';}
  if(gyroY >= 0.00){gySign = '+';}else{gySign = '-';}
  if(gyroZ >= 0.00){gzSign = '+';}else{gzSign = '-';}

  if(magX >= 0.00){mxSign = '+';}else{mxSign = '-';}
  if(magY >= 0.00){mySign = '+';}else{mySign = '-';}
  if(magZ >= 0.00){mzSign = '+';}else{mzSign = '-';}
  
  sprintf(charArray, "%s|%05d|D|%05d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%c%02d.%02d|%03d|%02d.%02d|%04d.%02d|%1d|%1d|%1d|%02d.%02d|%05d"
    ,CALLSIGN
    ,packetId
    ,bmp_altitude
    ,vxSign
    ,abs((int)highMPSSx)
    ,abs((int)(highMPSSx*100)%100)
    ,vySign
    ,abs((int)highMPSSy)
    ,abs((int)(highMPSSy*100)%100)
    ,vzSign
    ,abs((int)highMPSSz)
    ,abs((int)(highMPSSz*100)%100)
    ,gxSign
    ,abs((int)gyroX)
    ,abs((int)(gyroX*100)%100)
    ,gySign
    ,abs((int)gyroY)
    ,abs((int)(gyroY*100)%100)
    ,gzSign
    ,abs((int)gyroZ)
    ,abs((int)(gyroZ*100)%100)
    ,mxSign
    ,abs((int)magX)
    ,abs((int)(magX*100)%100)
    ,mySign
    ,abs((int)magY)
    ,abs((int)(magY*100)%100)
    ,mzSign
    ,abs((int)magZ)
    ,abs((int)(magZ*100)%100)
    ,proximityValue
    ,abs((int)bmp_temp)
    ,abs((int)(bmp_temp*100)%100)
    ,abs((int)bmp_hPa)
    ,abs((int)(bmp_hPa*100)%100)
    ,(int)launchDetected
    ,(int)apogeeDetected
    ,(int)seperationDetected
    ,abs((int)maxMPSS)
    ,abs((int)(maxMPSS*100)%100)
    ,maxAltitude
    );

  //Serial.println(charArray);
  
  
  sendPktChar(charArray);
  dataLog.flush();
 
}

//-----------------------------------------------------------------------------
//---------- Setup Functions --------------------------------------------------
//-----------------------------------------------------------------------------
//---------- Setup LoRa radio -------------------------------------------------
void setupRadio()
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
  dataLog.println("LoRa Radio Ready");

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
  sendPktChar("Initializing");
}
//---------- Setup BMP390 Precision Barometric Pressure & Altimeter & Temp ----
void setupBMP390()
{
  Serial.println("Adafruit BMP390 setup");
  sendPktChar("Adafruit BMP390 setup");

  if (!bmp.begin_I2C(119)) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  dataLog.println("BMP390 ready");
}
//------------ Setup ADXL375 High G Accelerometer -----------------------------
void setupADXL343()
{
  Serial.println("ADXL375 Accelerometer Setup");
  sendPktChar("ADXL375 Accelerometer Setup");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("could not find a valid ADXL375 sensor, check wiring!");
    while(1);
  }

  // Range is fixed at +-200g

  /* Display some basic information on this sensor */
  accel.printSensorDetails();
  Serial.println("");  

  // init offsets to zero
  accel.setTrimOffsets(0, 0, 0);
  
  Serial.println("Hold accelerometer flat to set offsets to 0, 0, and -1g...");
  delay(1000);
  int16_t x, y, z;
  x = accel.getX();
  y = accel.getY();
  z = accel.getZ();
  Serial.print("Raw X: "); Serial.print(x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(z); Serial.print("  ");Serial.println(" counts");

  // the trim offsets are in 'multiples' of 4, we want to round, so we add 2
  accel.setTrimOffsets(-(x+2)/4, 
                       -(y+2)/4, 
                       -(z-20+2)/4);  // Z should be '20' at 1g (49mg per bit)
  
  int8_t x_offset, y_offset, z_offset;
  accel.getTrimOffsets(&x_offset, &y_offset, &z_offset);
  Serial.print("Current trim offsets: ");
  Serial.print(x_offset);  Serial.print(", ");
  Serial.print(y_offset);  Serial.print(", ");
  Serial.println(z_offset);

  dataLog.println("ADXL375 ready");
}
//--------- Setup LSM6DS and LIS3MDL -----------------------------------------
void setupLSM6DSOXandLIS3MDL()
{
    Serial.println("Adafruit LSM6DS+LIS3MDL Setup");
    sendPktChar("Adafruit LSM6DS+LIS3MDL Setup");

  bool lsm6ds_success, lis3mdl_success;

  // hardware I2C mode, can pass in address & alt Wire

  lsm6ds_success = lsm6ds.begin_I2C();
  lis3mdl_success = lis3mdl.begin_I2C();

  if (!lsm6ds_success){
    Serial.println("Failed to find LSM6DS chip");
  }
  if (!lis3mdl_success){
    Serial.println("Failed to find LIS3MDL chip");
  }
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS and LIS3MDL Found!");

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (lsm6ds.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }
  // lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Magnetometer data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Magnetometer performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Magnetometer operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  dataLog.println("lis3mdl & LSM6DS ready");
}
//---------- Setup BME680 temp, press, humid, gas -----------------------------
void setupBME680()
{
  Serial.println("BME680 Setup");
  sendPktChar("BME680 Setup");

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  dataLog.println("BME680 ready");
}
//---------- Setup APDS9960 prox/gesture/color --------------------------------
void setupAPDS9960()
{
  sendPktChar("APDS9960 Setup");
  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
    while(1);
  }

  //enable modes as needed
  apds.enableProximity(true);
  //apds.enableColor(true);
  //apds.enableGesture(true);
  dataLog.println("APDS9960 ready");
}

//-----------------------------------------------------------------------------
//---------- Initialize Sensors -----------------------------------------------
//-----------------------------------------------------------------------------
void initializeAltitude()
{
 sendPktChar("Calibrating Pressure Sensor (Altitude)"); 
 double pressureAvg = 0.0;
 
 if (! bmp.performReading()) 
    {
      Serial.println("first pressure read failed");
      while(1);
    }
 else
    {
      delay(5000);
      bmp.performReading();
      pressureAvg = bmp.pressure / 100.0;
    }
 
 for (int i = 0; i<30; i++)
  {
    if (! bmp.performReading()) 
    {
      Serial.println("pressure reading failed");
    }
    else
    {
      pressureAvg = (pressureAvg + (bmp.pressure / 100.0)) / 2;
    }
    delay(100);
    Serial.print("baselining pressure: ");
    Serial.println(pressureAvg);
  }
 baseline_pressure = pressureAvg;
 dataLog.println("Presure sensor ready / altitude calibrated");
}

//-----------------------------------------------------------------------------
//---------- Sensor Reading Functions -----------------------------------------
//-----------------------------------------------------------------------------

void getAllValues()
{
  getBMP390values();
  getADXL375values();
  getLSM6DSandLIS3MDLvaluesGyro();
  getLSM6DSandLIS3MDLvaluesMag();
  getAPDS9960values();  

  //to send json to terminal
  //Serial.print("Internal: "); serializeJson(dataPacket, Serial); Serial.println();
}

void checkEvents()
{
  //code here to check for events and set the variables
  
  //launch
  if (maxMPSS > LAUNCH_DETECT_THRESHOLD)
  {
    launchDetected = true;    
  }
  
  //apogee
  if (launchDetected && bmp_altitude < maxAltitude - 5)
  {
    apogeeDetected = true;  
  }

  if (launchDetected && apogeeDetected &&(bmp_altitude > RECOVERY_HEIGHT_THRESHOLD)) //this would fire a drouge chute in theory.  only fire if launch has occured, apogee has happened, and you're above the recovery threshhold.  This is for safety!!
  {
    digitalWrite(RELAY_A_PIN, HIGH);
  }
  
  if (proximityValue < 50)
  {
    seperationDetected = true;
  }
  // other events?
   
}

void sendEventData()
{
  char packet[RH_RF95_MAX_MESSAGE_LEN];
  
   eventPacket["lnchDet"] = launchDetected;
   eventPacket["apgDet"] = apogeeDetected;
   eventPacket["maxV"] = round2(maxMPSS);
   eventPacket["maxAlt"] = maxAltitude;
   eventPacket["id"] = packetId;

   serializeJson(eventPacket, packet);
   sendPkt(packet);

}

void getBMP390values()
{
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
   
   bmp_temp = (bmp.temperature * 1.8)+32;
   bmp_hPa = bmp.pressure / 100.0;
   bmp_altitude = (int)((bmp.readAltitude(baseline_pressure))*3.28084)+1;

   dataPacket["degF"] = round2(bmp_temp);
   dataPacket["hPa"] = round2(bmp_hPa);
   dataPacket["altF"] = round2(bmp_altitude);

   tinyPacket.clear();
   tinyPacket["degF"] = round2(bmp_temp);
   tinyPacket["hPa"] = round2(bmp_hPa);
   tinyPacket["altF"] = round2(bmp_altitude);

   if(bmp_altitude > maxAltitude)
   {
    maxAltitude = bmp_altitude;
   }
  
}

void getADXL375values()
{
  sensors_event_t event;
  accel.getEvent(&event);

  highMPSSx = event.acceleration.x;
  highMPSSy = event.acceleration.y;
  highMPSSz = event.acceleration.z;
  highGx = highMPSSx / 9.80665;
  highGy = highMPSSy / 9.80665;
  highGz = highMPSSz / 9.80665;

  dataPacket["aclH"]["x"] = round2(highMPSSx);
  dataPacket["aclH"]["y"] = round2(highMPSSy);
  dataPacket["aclH"]["z"] = round2(highMPSSz);

  tinyPacket.clear();
  tinyPacket["aclH"]["x"] = round2(highMPSSx);
  tinyPacket["aclH"]["y"] = round2(highMPSSy);
  tinyPacket["aclH"]["z"] = round2(highMPSSz);

  if (highMPSSx > maxMPSS)
  {
    maxMPSS = highMPSSx;
  }
  if (highMPSSy > maxMPSS)
  {
    maxMPSS = highMPSSy;
  }
  if (highMPSSz > maxMPSS)
  {
    maxMPSS = highMPSSz;
  }
}

void getLSM6DSandLIS3MDLvalues()
{
    sensors_event_t accel, gyro, mag, temp;

  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  lowMPSSx = accel.acceleration.x;
  lowMPSSy = accel.acceleration.y;
  lowMPSSz = accel.acceleration.z;
  lowGx = lowMPSSx / 9.80665;
  lowGy = lowMPSSy / 9.80665;
  lowGz = lowMPSSz / 9.80665;
  
  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;
  
  magX = mag.magnetic.x;
  magY = mag.magnetic.y;
  magZ = mag.magnetic.z;

  dataPacket["gyr"]["x"] = round2(gyroX);
  dataPacket["gyr"]["y"] = round2(gyroY);
  dataPacket["gyr"]["z"] = round2(gyroZ);

  dataPacket["mag"]["x"] = round2(magX);
  dataPacket["mag"]["y"] = round2(magY);
  dataPacket["mag"]["z"] = round2(magZ);
  
}

void getLSM6DSandLIS3MDLvaluesGyro()
{
    sensors_event_t accel, gyro, temp;

  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp);
  
  gyroX = gyro.gyro.x;
  gyroY = gyro.gyro.y;
  gyroZ = gyro.gyro.z;

  
  dataPacket["gyr"]["x"] = round2(gyroX);
  dataPacket["gyr"]["y"] = round2(gyroY);
  dataPacket["gyr"]["z"] = round2(gyroZ);

  tinyPacket.clear();
  tinyPacket["gyr"]["x"] = round2(gyroX);
  tinyPacket["gyr"]["y"] = round2(gyroY);
  tinyPacket["gyr"]["z"] = round2(gyroZ);
  
}

void getLSM6DSandLIS3MDLvaluesMag()
{
    sensors_event_t mag;

  //  /* Get new normalized sensor events */
  lis3mdl.getEvent(&mag);

  magX = mag.magnetic.x;
  magY = mag.magnetic.y;
  magZ = mag.magnetic.z;

  dataPacket["mag"]["x"] = round2(magX);
  dataPacket["mag"]["y"] = round2(magY);
  dataPacket["mag"]["z"] = round2(magZ);

  tinyPacket.clear();
  tinyPacket["mag"]["x"] = round2(magX);
  tinyPacket["mag"]["y"] = round2(magY);
  tinyPacket["mag"]["z"] = round2(magZ);
  
}

void getBME680values()
{
  bme_temp = bme.temperature;
  bme_hPa = bme.pressure / 100.0;
  bme_humid = bme.humidity;
  bme_gas = bme.gas_resistance / 1000.0;
  bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  //add to dataPacket
  //add to tinyPacket
}

void getAPDS9960values()
{

  //gesture = apds.readGesture();
  proximityValue = apds.readProximity();

  dataPacket["prx"] = proximityValue;

  tinyPacket.clear();
  tinyPacket["prx"] = proximityValue;
    
}
//-----------------------------------------------------------------------------
//---------- Radio Stuff ------------------------------------------------------
//-----------------------------------------------------------------------------

//---------- Send Packet from Character Array----------------------------------
void sendPkt(char packetToSend[])
{
  Serial.print("Sending: "); Serial.println(packetToSend);
  //Serial.println("Sending..."); delay(10);
  packetToSend[measureJson(dataPacket)] = 0;
  
  rf95.send((uint8_t *)packetToSend, strlen((char*) packetToSend));
  
  //Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  packetId++;
  listenForPackets();
}

//---------- Send Packet from Character Array, but not the JSON!-----------------
void sendPktChar(char packetToSend[])
{
  int len = strlen((char*) packetToSend);
  Serial.print("Sending: "); Serial.println(packetToSend);
  dataLog.print("Sending: "); dataLog.println(packetToSend);
  //Serial.println("Sending..."); delay(10);
  packetToSend[len] = 0;
  
  rf95.send((uint8_t *)packetToSend, strlen((char*) packetToSend));
  
  //Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  packetId++;
  dataLog.flush();
  listenForPackets();
}

//---------- Send the Tiny Packet --------------------------------------------
void sendTinyPacket()
{
  tinyPacket["call"] = CALLSIGN;
  tinyPacket["id"] = packetId;

  char radioPacket[70];
  serializeJson(tinyPacket, radioPacket);
  //Serial.print("Sending: "); Serial.println(radioPacket);
  radioPacket[measureJson(tinyPacket)] = 0;
  rf95.send((uint8_t *)radioPacket, strlen((char*) radioPacket));
  
  //Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
  packetId++;
  listenForPackets();
}

//---------- Listen -----------------------------------------------------------
void listenForPackets()
{
  // waiting for a packet
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //Serial.println("checking messages");
  if (rf95.waitAvailableTimeout(MESSAGE_WAIT))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      buf[len]=0;
      
      Serial.print("Received: "); Serial.print((char*)buf); Serial.println();
      dataLog.print("Received: "); dataLog.println((char*)buf);
      //deserializeJson(receivedPacket, buf);
      //Serial.print("Received: "); serializeJson(receivedPacket, Serial); Serial.println();
      //Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC);    

      if(strstr(buf,"@relay"))
      {
        str_replace(buf,"@relay","[relayed via N3VEM-11]");
        dataLog.print("Relaying: "); dataLog.println((char*)buf);
        sendPktChar(buf);
      }
      else if (strstr(buf, "@reset")) //main use is to reset everything while on the pad, before launch, so that all the various sensors can freshly intitialzie to '0' state.
      {
        if(!launchDetected)
        {
          sendPktChar("Resetting");
          dataLog.println("Rebooting");
          reboot();
        }else
        {
          dataLog.println("Reboot attempted after launch. Denied.");
          sendPktChar("Can't reset after launch detection!");
        }
        
      }else
      {
        //other stuff?
      }
      
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    //Serial.println("No messages");
  }
  dataLog.flush();
}
//---------- handle a relay message -------------------------------------------
void handleRelay(char relayMessage[])
{

}
//-----------------------------------------------------------------------------
//---------- Misc Functions ---------------------------------------------------
//-----------------------------------------------------------------------------

double round2(double value)
{
  return (int)(value * 100 + 0.5) / 100.0;
}
//-----------------------------------------------------------------------------
void str_replace(char *src, char *oldchars, char *newchars) { // utility string function from lastchancename on arduino forum
  char *p = strstr(src, oldchars);
  char buf[RH_RF95_MAX_MESSAGE_LEN];
  do {
    if (p) {
      memset(buf, '\0', strlen(buf));
      if (src == p) {
        strcpy(buf, newchars);
        strcat(buf, p + strlen(oldchars));
      } else {
        strncpy(buf, src, strlen(src) - strlen(p));
        strcat(buf, newchars);
        strcat(buf, p + strlen(oldchars));
      }
      memset(src, '\0', strlen(src));
      strcpy(src, buf);
    }
  } while (p && (p = strstr(src, oldchars)));
}
//---------------------------------------------------------------------------
void reboot() {
  SCB_AIRCR = 0x05FA0004;
  asm volatile ("dsb");
  while (1) {;}
}
