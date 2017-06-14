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

// DEFAULT SETTINGS
int printDiags = 1;
#define MS5803_30bar // Pressure sensor. Each sensor has different constants.


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

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;

void setup() {
  SerialUSB.begin(115200);
  delay(2000);
  SerialUSB.println("Loggerhead OpenTag2");
  delay(8000);
  

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
  initSensors();
  initSensors();
  fileInit();

  dataFile.close();
  SerialUSB.println("Done");
}

void loop() {



}

void initSensors(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

    // RGB
  
  SerialUSB.print("RGBinit: ");
  SerialUSB.println(islInit()); 
  for(int n=0; n<4; n++){
      islRead();
      SerialUSB.print("R:"); SerialUSB.print(islRed); SerialUSB.print("\t");
      SerialUSB.print("G:"); SerialUSB.print(islGreen); SerialUSB.print("\t");
      SerialUSB.print("B:"); SerialUSB.println(islBlue);
      delay(200);
  }

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

