// Copyright Loggerhead Instruments, 2017
// David Mann

// OpenTag2 is an underwater motion datalogger
// designed around the ATSAMD21G that is used by the Arduino Zero

// OpenTag2 supports the following sensors:
// Pressure/Temperature
// IMU
// RGB light
// GPS


// buffer system so don't write to card when file closed

// - write ASCII and speed test (can it keep up, what is idle time?)
// - if ASCII works, write header
// - LED
// - RTC set
// - MPU
// - PT
// - Burn
// - VHF
// - Display
// - GPS
// - Low power
// - Delay start

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTCZero.h>

// DEFAULT SETTINGS
int printDiags = 1;
int recDur = 30;
int recInt = 0;
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
#define vSense 18  // PA05

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

// SD file system
SdFat sd;
File dataFile;
int fileCount; 

int ssCounter; // used to get different sample rates from one timer based on imu_srate

// Time
RTCZero rtc;
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

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
// RGBBUFFERSIZE = 20 * 3 channels * 1/sec / 10
#define RGBBUFFERSIZE 6
volatile int16_t RGBbuffer[RGBBUFFERSIZE];
byte time2writeRGB=0; 
int RGBCounter = 0;
volatile byte bufferposRGB=0;
byte halfbufRGB = RGBBUFFERSIZE/2;
boolean firstwrittenRGB;

// IMU
// IMUBUFFERSIZE = 20 * 9 channels * 100 / sec / 10
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
int accel_scale = 0;

int mode = 0; //standby = 0; running = 1

uint32_t t, startTime, endTime;

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
  t = rtc.getEpoch();
  startTime = t + 2;
  SerialUSB.print("Time:"); SerialUSB.println(t);
  SerialUSB.print("Start Time:"); SerialUSB.println(startTime);
}

void loop() {
  // Waiting: check if time to start
  while(mode==0){
    t = rtc.getEpoch();
    if(t >= startTime){
      endTime = startTime + recDur;
      startTime += recDur + recInt;  // this will be next start time for interval record
      fileInit();
      startTimer((int) imuSrate); // start timer
      updateTemp();  // get first reading ready
      mode = 1;
    }
  }

  // Recording: check if time to end
  while(mode==1){
    t = rtc.getEpoch();
    writeData();
    if(t >= endTime){
      dataFile.close(); // close file
      if(recInt==0){  // no interval between files
        endTime += recDur;  // update end time
        fileInit();
        break;
      }
      stopTimer(); // interval between files
      mode = 0;
      break;
    }
  }
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

  // IMU
  SerialUSB.println(mpuInit(1));
  for(int i=0; i<10; i++){
    readImu();
    readCompass();
    calcImu();
 
    SerialUSB.print("a/g/m/t: \t");
    SerialUSB.print(accel_x); SerialUSB.print("\t");
    SerialUSB.print(accel_y); SerialUSB.print("\t");
    SerialUSB.print(accel_z); SerialUSB.print("\t");
    SerialUSB.print(gyro_x); SerialUSB.print("\t");
    SerialUSB.print(gyro_y); SerialUSB.print("\t");
    SerialUSB.print(gyro_z); SerialUSB.print("\t");
    SerialUSB.print(mag_x); SerialUSB.print("\t");
    SerialUSB.print(mag_y); SerialUSB.print("\t");
    SerialUSB.print(mag_z); SerialUSB.print("\t");
    SerialUSB.print("FIFO pts:"); SerialUSB.println(getImuFifo()); //check FIFO is working
    delay(100);
  }
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

void sampleSensors(void){  //interrupt at update_rate
  ssCounter++;
  
  readImu();
  readCompass();
  calcImu();
  incrementIMU();

  // MS58xx start temperature conversion half-way through
  if((ssCounter>=(0.5 * imuSrate / sensorSrate)) & togglePress){ 
    readPress();   
    updateTemp();
    togglePress = 0;
  }
  
  if(ssCounter>=(int)(imuSrate/sensorSrate)){
    ssCounter = 0;
    // MS58xx pressure and temperature
    readTemp();
    updatePress();
    togglePress = 1;

    // RGB
    islRead(); 
    incrementRGBbufpos(islRed);
    incrementRGBbufpos(islGreen);
    incrementRGBbufpos(islBlue);
    
    calcPressTemp(); // MS58xx pressure and temperature
    
    incrementPTbufpos(pressure_mbar);
    incrementPTbufpos(temperature);
  }
}

void calcImu(){
  accel_x = (int16_t) ((int16_t)imuTempBuffer[0] << 8 | imuTempBuffer[1]);    
  accel_y = (int16_t) ((int16_t)imuTempBuffer[2] << 8 | imuTempBuffer[3]);   
  accel_z = (int16_t) ((int16_t)imuTempBuffer[4] << 8 | imuTempBuffer[5]);    

  gyro_x = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);  
  gyro_y = (int16_t)  (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);   
  gyro_z = (int16_t)  (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]);  
  
  mag_x = (int16_t)  (((int16_t)imuTempBuffer[13] << 8) | imuTempBuffer[12]);   
  mag_y = (int16_t)  (((int16_t)imuTempBuffer[15] << 8) | imuTempBuffer[14]);   
  mag_z = (int16_t)  (((int16_t)imuTempBuffer[17] << 8) | imuTempBuffer[16]); 
}

float readVoltage(){
  float vDivider = 0.8333;
  float vReg = 3.3;
  float voltage = (float) analogRead(vSense) * vReg / (vDivider * 1024.0);
  return voltage;
}


/*
This is a slightly modified version of the timer setup found at:
https://github.com/maxbader/arduino_tools
 */
void startTimer(int frequencyHz) {
  //GCLK->GENCTRL.reg |= GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_RUNSTDBY;
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  TcCount16* TC = (TcCount16*) TC3;
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  //TC->CTRLA.reg |= TC_CTRLA_RUNSTDBY;
  
  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    sampleSensors();
  }
}

void stopTimer(){
  // TcCount16* TC = (TcCount16*) TC3; // get timer struct
   NVIC_DisableIRQ(TC3_IRQn);
}
