// Copyright Loggerhead Instruments, 2017
// David Mann

// OpenTag2 is an underwater motion datalogger
// designed around the ATSAMD21G that is used by the Arduino Zero

// OpenTag2 supports the following sensors:
// Pressure/Temperature
// IMU
// RGB light
// GPS

// sample rate settings
// 100 Hz IMU/ 1 Hz pressure
// 10 Hz IMU / 1 Hz pressure
// 1/sec all sensors
// 1/min all sensors

// To Do
// Watchdog timer
// record no motion file at beginning for accelerometer and gyro offsets
// save spin me file
// check priority of imu timer interrupt (dropping samples during file write)?
// --branch--try FIFO mode with interrupt

#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTCZero.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <ArduinoLowPower.h>

//
// DEV SETTINGS
//
float codeVer = 1.01;
int printDiags = 1;
int dd = 1; // dd=0 to disable display
int recDur = 60;
int recInt = 0;
int led2en = 1; //enable green LEDs flash 1x per second. Can be disabled from script.
int skipGPS = 1; // skip GPS for getting time and lat/lon
int logGPS = 0; // if not logging, turn off GPS after get time
long gpsTimeOutThreshold = 60 * 15; //if longer then 15 minutes at start without GPS time, just start
int spinMeTimeOut = 30000;
#define HWSERIAL Serial1
#define MS5803_30bar // Pressure sensor. Each sensor has different constants.
//
//

#ifdef MS5837_30bar
  #define MS58xx_constant 8192.0
  #define pressAddress 0x76
#endif
#ifdef MS5803_01bar
  #define MS58xx_constant 32768.0
  #define pressAddress 0x77
#endif
#ifdef MS5803_05bar
  #define MS58xx_constant 32768.0
  #define pressAddress 0x77
#endif
#ifdef MS5803_30bar
  #define MS58xx_constant 8192.0
  #define pressAddress 0x77
#endif


// pin assignments
#define LED1        31  // PB23  //INSIDE RING
#define LED2        26  // PA27  //OUTSIDE RING
#define LED_RED     30  // PB22
#define chipSelect  10
#define vSense      A4  // PA05
#define displayPow  3   //PA09
#define vhfPow      25  // PB03
#define burnWire    19
#define BUTTON1     5   //PA15
#define BUTTON2     11  //PA16
#define GPS_EN      6   // PA20 
#define MPU_INTERRUPT 42  //PA03

//From variant.cpp
// C:\Users\David Mann\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.6.15\variants\arduino_zero
//pin19 PB10 = digital pin 23
//pin20 PB11 = digital pin 24
//pin21 PA12 = digital pin 21
//pin22 PA13 = digital pin 38
//pin37 PB22 = digital pin 30
//pin38 PB23 = digital pin 31
//pin39 PA27 = digital pin 26
//pin41 PA28 = digital pin 27
//pin45 PA30 x not defined, used for SWD programming/debug
//pin46 PA31 x not defined, used for SWD programming/debug
//pin48 PB3 = digital pin 25

#define displayLine1 0
#define displayLine2 8
#define displayLine3 16
#define displayLine4 25
Adafruit_FeatherOLED display = Adafruit_FeatherOLED();

// SD file system
SdFat sd;
File dataFile;
int fileCount; 

int ssCounter; // used to get different sample rates from one timer based on imu_srate

char myID[33]; // ATSAM chipID

//
// SENSORS
//
int imuSrate = 100; // must be integer for timer. If change sample rate, need to adjust buffer size
int sensorSrate = 1; // must divide into imuSrate. If change sample rate, need to adjust buffer size

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity
uint16_t POFF;  //Pressure offset
uint16_t TCSENS; //Temp coefficient of pressure sensitivity
uint16_t TCOFF; //Temp coefficient of pressure offset
uint16_t TREF;  //Ref temperature
uint16_t TEMPSENS; //Temperature sensitivity coefficient
byte Tbuff[3];
byte Pbuff[3];
volatile float depth, temperature, pressure_mbar;
boolean togglePress = 0; // flag to toggle conversion of temperature and pressure

// Pressure/Temp and RGB sensor buffers need to sample so they full half-buf at same time
// because writing to file is only done by checking Pressure/Temp
// Pressure, Temp double buffer
#define PTBUFFERSIZE 4
volatile float PTbuffer[PTBUFFERSIZE];
byte time2writePT = 0; 
volatile byte bufferposPT=0;
byte halfbufPT = PTBUFFERSIZE/2;
boolean firstwrittenPT;

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;

// RGB buffer
// RGBBUFFERSIZE = 2 * 3 channels * 1/sec
#define RGBBUFFERSIZE 6
volatile int16_t RGBbuffer[RGBBUFFERSIZE];
byte time2writeRGB=0; 
int RGBCounter = 0;
volatile byte bufferposRGB=0;
byte halfbufRGB = RGBBUFFERSIZE/2;
boolean firstwrittenRGB;

// YEAR, MONTH, DAY, HOUR, MINUTE, SECOND
// TIMEBUFFERSIZE = 2 * 6 * 1/sec
#define TIMEBUFFERSIZE 12
volatile int16_t timeBuffer[TIMEBUFFERSIZE];
volatile byte bufferposTime = 0;
byte halfbufTime = TIMEBUFFERSIZE / 2;
boolean firstwrittenTime;

// IMU
//MPU9250_DMP imu;
// IMUBUFFERSIZE = 2 * 9 channels * 100 / sec
#define IMUBUFFERSIZE 1800 
volatile int16_t imuBuffer[IMUBUFFERSIZE]; // buffer used to store IMU sensor data before writes in bytes
volatile byte time2writeIMU=0; 
volatile int IMUCounter = 0;
volatile int bufferposIMU = 0;
int halfbufIMU = IMUBUFFERSIZE/2;
volatile boolean firstwrittenIMU;
volatile uint8_t imuTempBuffer[22];
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t mag_x;
int16_t mag_y;
int16_t mag_z;
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
float gyro_temp;
int accel_scale = 16;
float pitch, roll, yaw;
int magAdjX = 1; 
int magAdjY = 1; 
int magAdjZ = 1;
int magXoffset;
int magYoffset;
int magZoffset;


// GPS
double latitude, longitude;
char latHem, lonHem;
int goodGPS = 0;
long gpsTimeout; //increments every GPRMC line read; about 1 per second
int gpsYear = 0, gpsMonth = 1, gpsDay = 1, gpsHour = 0, gpsMinute = 0, gpsSecond = 0;
int gpsStatus = 0; // 0 = standby; 1 = awake

// System Modes and Status
int mode = 0; //standby = 0; running = 1
volatile float voltage;

// Time
RTCZero rtc;
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

time_t t, startTime, endTime, burnTime;
int burnFlag = 0;
long burnSeconds;

void setup() {
  SerialUSB.begin(115200);
  delay(5000);
  SerialUSB.println("On");
  
  HWSERIAL.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(vhfPow, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED_RED, LOW);
  digitalWrite(vhfPow, LOW);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(burnWire, OUTPUT);
  digitalWrite(burnWire, LOW);
  pinMode(MPU_INTERRUPT, INPUT_PULLUP);
  delay(5000);

  SerialUSB.println("pinMode setup");

  Wire.begin();
  Wire.setClock(400000);  // set I2C clock to 400 kHz

  delay(1000);

  if(dd){
    SerialUSB.println("Display power on");
    pinMode(displayPow, OUTPUT);
    digitalWrite(displayPow, HIGH);
    delay(200);
    displayOff();
    delay(200);
    displayOn();
    delay(200);
    cDisplay();
    display.display();    
  }

  SerialUSB.println("Init microSD");
  // see if the card is present and can be initialized:
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    SerialUSB.println("Card failed");
    digitalWrite(LED_RED, HIGH);
    delay(200);
    digitalWrite(LED_RED, LOW);
    delay(100);

    if(dd){
      cDisplay();
      display.println("Card: fail");
      display.display();
    }
  }
  rtc.begin();
  loadScript(); // do this early to set time
  
  if(dd){
    displayOn();
    cDisplay();
    display.println("Loggerhead");
    display.println("OpenTag 2");
    display.display();
  }

  SerialUSB.println("GPS power on");
  if(!skipGPS) gpsOn(); // get GPS on so can get fix while init sensors
  initSensors();
 
// GPS configuration
//

  gpsTimeout = 0;
  if(!skipGPS){
   gpsSpewOff();
   waitForGPS();
   SerialUSB.println();
   SerialUSB.println("GPS Status");
   gpsStatusLogger();
   waitForGPS();

  if(logGPS){
   // if any data in GPSlogger, download it to microSD
   SerialUSB.println();
   SerialUSB.println("Dump GPS");
   cDisplay();
   
   display.println("GPS Log");
   display.println("Downloading...");
   display.display();
   if(gpsDumpLogger()==1){
     // erase data if download was good
     SerialUSB.println();
     SerialUSB.println("Erase GPS");
     gpsEraseLogger();
     waitForGPS();
   }
    
   // start GPS logger
   SerialUSB.println();
   SerialUSB.println("Start logging");
   gpsStartLogger();
   SerialUSB.println();
   SerialUSB.println("GPS Status");
   gpsStatusLogger();
   SerialUSB.println();
  }

  gpsSpewOn();
  cDisplay();
  display.println("GPS Fix");
  display.print("searching...");
  display.display();
   
   while(!goodGPS){
     byte incomingByte;
     digitalWrite(LED1, LOW);
     if(gpsTimeout >= gpsTimeOutThreshold) break;
     while (HWSERIAL.available() > 0) {    
      digitalWrite(LED1, HIGH);
      incomingByte = HWSERIAL.read();
      SerialUSB.write(incomingByte);
      gps(incomingByte);  // parse incoming GPS data
      }
    }
    if(gpsTimeout <  gpsTimeOutThreshold){
      rtc.setTime(gpsHour, gpsMinute, gpsSecond);
      rtc.setDate(gpsDay, gpsMonth, gpsYear);
      cDisplay();
      display.println("FIX");
      displayGPS();
      display.display();
    }
    gpsSpewOff();
    waitForGPS();
  } // skip GPS
  
  if(logGPS==0) gpsOff();
  
  getTime();
  readVoltage();
  getChipId();
  logFileWrite();
  
  t = rtc.getEpoch();
  if(burnFlag==2){
    burnTime = t + burnSeconds;
    SerialUSB.print("Burn time set");
    SerialUSB.println(burnTime);
  }
  if(startTime==0) startTime = t + 21; // wait a couple of seconds if no delay start set from script
  SerialUSB.print("Time:"); SerialUSB.println(t);
  SerialUSB.print("Start Time:"); SerialUSB.println(startTime);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  // wait here so can see settings
  int startWait = millis();
  while (millis()-startWait < 20000){
      t = rtc.getEpoch();
      getTime();
      cDisplay();
      displaySettings();
      displayClock(displayLine4);
      display.display();
      delay(500);
  }

  cDisplay();
  display.print("Display Off");
  display.display();
  delay(1000);
}

void loop() {
  // Waiting: check if time to start
  while(mode==0){
    t = rtc.getEpoch();
    getTime();
    checkBurn();

    // sleep if more than 10 seconds
    if(startTime - t > 10){
      //displayOff();
      int alarmSeconds = second + 10;
      if(alarmSeconds>60) alarmSeconds-=60;
      rtc.setAlarmSeconds(alarmSeconds);
      rtc.enableAlarm(rtc.MATCH_SS);
      rtc.attachInterrupt(alarmMatch);
      rtc.standbyMode();
      delay(10);
      digitalWrite(LED_RED, LOW);
      rtc.disableAlarm();
    }

    if(t >= startTime){
      endTime = startTime + recDur;
      startTime += recDur + recInt;  // this will be next start time for interval record
      fileInit();
      updateTemp();  // get first reading ready
      mode = 1;
      resetGyroFIFO();
      displayOff();
    }
  } // mode = 0

  // Recording: check if time to end
  int downTime = millis();
  while(mode==1){
    t = rtc.getEpoch();

  // - Test FIFO overflow and not losing track of sensors
  // - Write FIFO after read (delay 100 ms, read FIFO, write FIFO--every second read other sensors)
  // - Sleep for x ms, then read and write FIFO
    int fifoPoints = getImuFifo();
    if(fifoPoints > 512) digitalWrite(LED_RED, HIGH); // red LED if overflow
    if(fifoPoints>100){
      SerialUSB.print(fifoPoints);
      SerialUSB.print(" ");
      SerialUSB.println(millis()- downTime);
      while(fifoPoints>20){
        sampleSensors();
        writeData(); // check to see if double buffers ready to be written
        fifoPoints = getImuFifo();
      }
      downTime = millis();
      //printImu();
    }   
    
  
    if(t >= endTime){
      dataFile.close(); // close file
      if(recInt==0){  // no interval between files
        endTime += recDur;  // update end time
        fileInit();
        break;
      }
      mode = 0;
      break;
    }
    else{  // don't sleep if just made a new file
      LowPower.sleep(10);
    }

    // Check if stop button pressed
    if(digitalRead(BUTTON1)==0){
      delay(10); // simple deBounce
      if(digitalRead(BUTTON1)==0){
        digitalWrite(LED_RED, HIGH);
        dataFile.close();
        if(dd){
          displayOn();
          cDisplay();
          display.println("Stopped");
          display.println();
          display.print("Safe to turn off");
          display.display();
        }
        delay(58000); // if don't power off in 60s, restart
        if(dd){
          cDisplay();
          display.print("Restarting");
          display.display();
          delay(2000);
          displayOff();
        }
        fileInit();
        resetGyroFIFO();
        digitalWrite(LED_RED, LOW);
      }
    }
  }
}

void alarmMatch(){
  digitalWrite(LED_RED, HIGH);
}

void initSensors(){
  // Pressure/Temperature
  pressInit();
  updatePress();
  delay(20);
  readPress();
  updateTemp();
  delay(20);
  readTemp();
  calcPressTemp();
  SerialUSB.print(" press:"); SerialUSB.print(pressure_mbar);
  SerialUSB.print(" depth:"); SerialUSB.print(depth);
  SerialUSB.print(" temp:"); SerialUSB.println(temperature);

  cDisplay();
  display.println("OpenTag2");
  display.print("Press:"); display.println(pressure_mbar);
  display.print("Depth:"); display.println(depth);
  display.print("Temp:"); display.println(temperature);
  display.display();

  delay(6000);
  
  mpuInit(1);



  resetGyroFIFO();
  int downTime = millis();
  int fifoPoints = 0;
  for(int i=0; i<10; i++){
    fifoPoints = getImuFifo();
    if(fifoPoints>200){
      SerialUSB.println(millis()-downTime);
      while(fifoPoints>10){
        readImu();
        calcImu();
        fifoPoints = getImuFifo();
        //printImu();
      }
      downTime = millis();
      printImu();
    }   
  }


 // for getting offset
  int minMagX = mag_x;
  int maxMagX = mag_x;
  int minMagY = mag_y;
  int maxMagY = mag_y;
  int minMagZ = mag_z;
  int maxMagZ = mag_z;
  int i=0; 
  int mXrange = maxMagX - minMagX;
  int mYrange = maxMagY - minMagY;
  int mZrange = maxMagZ - minMagZ;

  SerialUSB.println(mXrange);
  SerialUSB.println(mYrange);
  SerialUSB.println(mZrange);

  long startCalTime = millis();
  while((mXrange<580) | (mYrange<580) | (mZrange<580) ){
      if((millis() - startCalTime) > spinMeTimeOut) {
        magXoffset = 0;
        magYoffset = 0;
        magZoffset = 0;
        break;
      }
      i++;

      readImu();
      calcImu();
      euler();

      
       // update min and max Mag
      if (mag_x<minMagX) minMagX = mag_x;
      if (mag_x>maxMagX) maxMagX = mag_x;
      if (mag_y<minMagY) minMagY = mag_y;
      if (mag_y>maxMagY) maxMagY = mag_y;
      if (mag_z<minMagZ) minMagZ = mag_z;
      if (mag_z>maxMagZ) maxMagZ = mag_z;

      mXrange = maxMagX - minMagX;
      mYrange = maxMagY - minMagY;
      mZrange = maxMagZ - minMagZ;
     if(printDiags){
      SerialUSB.print("a/g/m: \t");
      SerialUSB.print(accel_x); SerialUSB.print("\t");
      SerialUSB.print(accel_y); SerialUSB.print("\t");
      SerialUSB.print(accel_z); SerialUSB.print("\t");
      SerialUSB.print(gyro_x); SerialUSB.print("\t");
      SerialUSB.print(gyro_y); SerialUSB.print("\t");
      SerialUSB.print(gyro_z); SerialUSB.print("\t");
      SerialUSB.print(mag_x); SerialUSB.print("\t");
      SerialUSB.print(mag_y); SerialUSB.print("\t");
      SerialUSB.println(mag_z);
      SerialUSB.print(pitch); SerialUSB.print("\t");
      SerialUSB.print(roll); SerialUSB.print("\t");
      SerialUSB.println(yaw); 
    }
     
      cDisplay();
      display.println("SPIN ME");
      display.println("Magnetometer");
      display.print("Range:");
      display.print(mXrange); display.print(" ");
      display.print(mYrange); display.print(" ");
      display.println(mZrange);
      display.print(pitch); display.print(" ");
      display.print(roll); display.print(" "); 
      display.print(yaw);
      display.display();
      
    magXoffset = ((maxMagX - minMagX) / 2) + minMagX;
    magYoffset = ((maxMagY - minMagY) / 2) + minMagY;
    magZoffset = ((maxMagZ - minMagZ) / 2) + minMagZ;

    delay(8);
  }

  if((mag_z==0) & (mag_x==0) & (mag_y==0)){
    cDisplay();
    display.println("IMU ERROR");
    display.println();
    display.print("Turn off and on");
    display.display();
    delay(60000);
  }

  cDisplay();
  display.println("Mag Offset");
  display.print("X ");display.println(magXoffset); 
  display.print("Y "); display.println(magYoffset); 
  display.print("Z "); display.print(magZoffset); 
  display.display();
  delay(10000);

  for(int i=1; i<100; i++){
    if ( digitalRead(MPU_INTERRUPT) == LOW )
    {
      //imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      readImu();
      calcImu();
      euler();
      if(printDiags){
        SerialUSB.print(pitch); SerialUSB.print("\t");
        SerialUSB.print(roll); SerialUSB.print("\t");
        SerialUSB.println(yaw); 
      }
      cDisplay();
      display.println("  ");
      display.print("A:");
      display.print(accel_x); display.print(" ");
      display.print(accel_y); display.print(" ");
      display.println(accel_z);
      display.print("M:");
      display.print(mag_x); display.print(" ");
      display.print(mag_y); display.print(" ");
      display.println(mag_z);
      display.print(pitch); display.print(" ");
      display.print(roll); display.print(" "); 
      display.print(yaw);
      display.display();
    }
    delay(50);
  }
  
  // RGB
  SerialUSB.print("RGBinit: ");
  SerialUSB.println(islInit()); 
  for(int n=0; n<20; n++){
      islRead();
      SerialUSB.print("R:"); SerialUSB.print(islRed); SerialUSB.print("\t");
      SerialUSB.print("G:"); SerialUSB.print(islGreen); SerialUSB.print("\t");
      SerialUSB.print("B:"); SerialUSB.println(islBlue);

      cDisplay();
      display.println("Light");
      display.print("Red:  "); display.println(islRed);
      display.print("Green:"); display.println(islGreen);
      display.print("Blue: "); display.println(islBlue);
      display.display();
      delay(200);
  }
  if((islRed==0) & (islGreen==0) & (islBlue==0)){
    cDisplay();
    display.println("LIGHT SENSOR ERROR");
    display.println();
    display.print("Turn off and on");
    display.display();
    delay(60000);
  }
}

void printImu(){
      SerialUSB.print("a/g/m: \t");
      SerialUSB.print(accel_x); SerialUSB.print("\t");
      SerialUSB.print(accel_y); SerialUSB.print("\t");
      SerialUSB.print(accel_z); SerialUSB.print("\t");
      SerialUSB.print(gyro_x); SerialUSB.print("\t");
      SerialUSB.print(gyro_y); SerialUSB.print("\t");
      SerialUSB.print(gyro_z); SerialUSB.print("\t");
      SerialUSB.print(mag_x); SerialUSB.print("\t");
      SerialUSB.print(mag_y); SerialUSB.print("\t");
      SerialUSB.println(mag_z);
}

void logFileWrite()
{
   t = rtc.getEpoch();
   getTime();
   File logFile = sd.open("log.txt", O_WRITE | O_CREAT | O_APPEND);
   logFile.print("ID:"); logFile.println(myID);
   logFile.print("Code version:"); logFile.println(codeVer);
   logFile.print(year);  logFile.print("-");
   logFile.print(month); logFile.print("-");
   logFile.print(day); logFile.print("T");
   logFile.print(hour); logFile.print(":");
   logFile.print(minute); logFile.print(":");
   logFile.println(second);

   logFile.println("Magnetometer Offsets");
   logFile.print("X:"); logFile.println(magXoffset);  
   logFile.print("Y:"); logFile.println(magYoffset); 
   logFile.print("Z:"); logFile.println(magZoffset);

   logFile.close();
}


void fileInit()
{
   char filename[60];
   t = rtc.getEpoch();
   getTime();
   sprintf(filename,"%02d%02d%02dT%02d%02d%02d_%s.csv", year, month, day, hour, minute, second, myID);  //filename is DDHHMM
   dataFile = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
   while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.amx",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    SerialUSB.println(filename);
    delay(100);
   }
   dataFile.println("accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,datetime,red,green,blue,pressure(mBar),temperature,pitch,roll,yaw,latitude,latHem,longitude,lonHem,V");
   SdFile::dateTimeCallback(file_date_time);
   SerialUSB.println(filename);
}

void getTime(){
  day = rtc.getDay();
  month = rtc.getMonth();
  year = rtc.getYear();
  hour = rtc.getHours();
  minute = rtc.getMinutes();
  second = rtc.getSeconds();
}

void sampleSensors(void){  

  ssCounter++;
  readImu();
  calcImu();
  incrementIMU();

  // MS58xx start temperature conversion half-way through
  if((ssCounter>=(0.5 * imuSrate / sensorSrate)) & togglePress){ 
    readPress();   
    updateTemp();
    togglePress = 0;
  //  SerialUSB.print("t");
  }
  
  if(ssCounter>=(int)(imuSrate/sensorSrate)){
    ssCounter = 0;
    // MS58xx pressure and temperature
    readTemp();
    updatePress();
    togglePress = 1;
   // SerialUSB.print("p");

    // RGB
    islRead(); 
    incrementRGBbufpos(islRed);
    incrementRGBbufpos(islGreen);
    incrementRGBbufpos(islBlue);

    SerialUSB.print("l");

    getTime();
    incrementTimebufpos();
    checkBurn();

    SerialUSB.print("b");

    if(led2en) digitalWrite(LED1, HIGH);
    if(led2en) digitalWrite(LED2, HIGH);
  
    calcPressTemp(); // MS58xx pressure and temperature
    incrementPTbufpos(pressure_mbar);
    incrementPTbufpos(temperature);
    writeData();
    
    if(depth<1.0) {
      digitalWrite(vhfPow, HIGH);
      if(logGPS & (gpsStatus==0)){
        gpsWake();
        gpsStatus=1;
      }
    }
    else{
      if((depth>1.5)) {
        digitalWrite(vhfPow, LOW);
        if(logGPS & (gpsStatus==1)){
          gpsStandby();
          gpsStatus = 0;
        }
      }
    }
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }
}

void calcImu(){
  accel_x = (int16_t) (((int16_t)imuTempBuffer[0] << 8) | imuTempBuffer[1]);    
  accel_y = (int16_t) (((int16_t)imuTempBuffer[2] << 8) | imuTempBuffer[3]);   
  accel_z = (int16_t) -(((int16_t)imuTempBuffer[4] << 8) | imuTempBuffer[5]);    

  gyro_temp = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);   
 
  gyro_x = (int16_t) (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);  
  gyro_y = (int16_t) (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]);   
  gyro_z = (int16_t) -(((int16_t)imuTempBuffer[12] << 8) | imuTempBuffer[13]);  
  
  mag_y = (int16_t)  (((int16_t)imuTempBuffer[14] << 8) | imuTempBuffer[15]);   
  mag_x = (int16_t)  (((int16_t)imuTempBuffer[16] << 8) | imuTempBuffer[17]);     
  mag_z = (int16_t) (((int16_t)imuTempBuffer[18] << 8) | imuTempBuffer[19]); 

// NED orientation

//  gyro_x = imu.gx;
//  gyro_y = imu.gy;
//  gyro_z = -imu.gz;
//  accel_x = imu.ax;
//  accel_y = imu.ay;
//  accel_z = -imu.az;
//  mag_x = imu.my - magYoffset;
//  mag_y = imu.mx - magXoffset;
//  mag_z = imu.mz - magZoffset;
  
}

void readVoltage(){
  float vDivider = 100.0/133.0;
  float vReg = 3.3;
  voltage = (float) analogRead(vSense) * vReg / (vDivider * 1024.0);
}



void getChipId() {
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;
  sprintf(myID, "%8x%8x%8x%8x", val1, val2, val3, val4);
  SerialUSB.println(myID);
}
//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  *date=FAT_DATE(year + 2000,month,day);
  *time=FAT_TIME(hour,minute,second);
}

int checkBurn(){
  if((t>=burnTime) & (burnFlag>0)){
    digitalWrite(burnWire, HIGH);
    digitalWrite(vhfPow, HIGH);
  }
}

void gpsOn(){
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
}

void gpsOff(){
  digitalWrite(GPS_EN, LOW);
}

