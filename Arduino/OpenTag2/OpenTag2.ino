// Copyright Loggerhead Instruments, 2017
// David Mann

// OpenTag2 is an underwater motion datalogger
// designed around the ATSAMD21G that is used by the Arduino Zero

// OpenTag2 supports the following sensors:
// Pressure/Temperature
// IMU
// RGB light
// GPS

// - LED
// - RTC set
// - MPU
// - PT
// - Burn
// - VHF
// - RGB
// - Display
// - GPS
// - Low power

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTCZero.h>

// pin assignments
#define LED1 20 // PB23
#define LED2 21 // PA27
#define LED3 22 // PB22
#define chipSelect 10

// SD file system
SdFat sd;
File dataFile;
int fileCount; 

// Time
RTCZero rtc;
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;


void setup() {
  SerialUSB.begin(115200);
  delay(2000);
  SerialUSB.println("Loggerhead OpenTag2");

  Wire.begin();
  Wire.setClock(400);  // set I2C clock to 400 kHz
  rtc.begin();

  year=2017; month=6; day=12;
  hour=0; minute=0; second=0;
  rtc.setTime(hour, minute, second);
  rtc.setDate(day, month, year);
  
  // see if the card is present and can be initialized:
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    SerialUSB.println("Card failed");
  }

  initSensors();
  fileInit();

  for(int i=0; i<10; i++){
    writeSensors();
    delay(1000);
  }

  dataFile.close();
  SerialUSB.println("Done");
}

void loop() {



//  for(int i=0; i<10; i++){
//    digitalWrite(LED2, HIGH);
//    delay(100);
//    digitalWrite(LED2, LOW);
//    delay(100);
//  }
//    for(int i=0; i<10; i++){
//    digitalWrite(LED3, HIGH);
//    delay(100);
//    digitalWrite(LED3, LOW);
//    delay(100);
//  }
}

void initSensors(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
}

void fileInit()
{
   char filename[20];
   getTime();
   sprintf(filename,"%02d%02d%02d%02d.csv", day, hour, minute, second);  //filename is DDHHMM
   dataFile = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
   while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.amx",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    
   }
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

void writeSensors(){
  char sensorLine[255];
  getTime();
  sprintf(sensorLine,"%04d-%02d-%02dT%02d:%02d:%02d.AMX", year, month, day, hour, minute, second);
  dataFile.println(sensorLine);
}

