// from https://github.com/ROHMUSDC/ROHM_SensorPlatform_Multi-Sensor-Shield/blob/master/Platform%20Code/Arduino_UNO_FirmwareExample/ROHM_SENSORSHLD1-EVK-101_10-20-2016/ROHM_SENSORSHLD1-EVK-101_TerminalDemo_11-10-2016/ROHM_SENSORSHLD1-EVK-101_TerminalDemo_11-10-2016.ino

int KMX62_DeviceAddress = 0x0E; 

#define KMX_COTR (0x3C)
#define KMX_CNTL1 (0x39)
#define KMX_CNTL2 (0x3A)
#define KMX_WHO_AM_I (0x00)

//Accel Portion
int MEMS_Accel_Xout_highByte = 0;
int MEMS_Accel_Xout_lowByte = 0;
int MEMS_Accel_Yout_highByte = 0;
int MEMS_Accel_Yout_lowByte = 0;
int MEMS_Accel_Zout_highByte = 0;
int MEMS_Accel_Zout_lowByte = 0;

float MEMS_Accel_Conv_Xout = 0;
float MEMS_Accel_Conv_Yout = 0;
float MEMS_Accel_Conv_Zout = 0;
//Mag Sensor Portion
int MEMS_Mag_Xout_highByte = 0;
int MEMS_Mag_Xout_lowByte = 0;
int MEMS_Mag_Yout_highByte = 0;
int MEMS_Mag_Yout_lowByte = 0;
int MEMS_Mag_Zout_highByte = 0;
int MEMS_Mag_Zout_lowByte = 0;
 
float MEMS_Mag_Conv_Xout = 0;
float MEMS_Mag_Conv_Yout = 0;
float MEMS_Mag_Conv_Zout = 0;

 //----- Start Initialization for KMX62 Digital Accel/Mag Sensor -----
 //KMX62 Init Sequence
  // 1. Standby Register (0x29), write 0x03 (Turn Off)
  // 2. Self Test Register (0x60), write 0x00
  // 3. Control Register 1 (0x2A), write 0x13
  // 4. Control Register 2 (0x2B), write 0x00
  // 5. ODCNTL Register (0x2C), write 0x00
  // 6. Temp EN Register (0x4C), write 0x01
  // 7. Buffer CTRL Register 1 (0x78), write 0x00
  // 8. Buffer CTRL Register 2 (0x79), write 0x00
  // 9. Standby Register (0x29), write 0x0u

  //KMX62 Init Sequence
  // 1. CNTL2 (0x3A), write (0x5F): 4g, Max RES, EN temp mag and accel

void kmx62Reset(){
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(0x7F);
  Wire.write(0x00);
  Wire.endTransmission();
  delay( 2);
  
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(KMX_CNTL2); // CNTL2
  Wire.write(0x00);
  Wire.endTransmission();
  delay(2);
  
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(KMX_CNTL1);
  Wire.write(0x80); //software reset
  Wire.endTransmission();
  delay(2);

  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(KMX_WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(KMX62_DeviceAddress, 1);
  Serial.print("WAI:");
  Serial.println(Wire.read()); // should be 0x18 = 24
  Wire.endTransmission();
  delay(2);
}

int kmx62Init(){
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(KMX_CNTL2); // CNTL2
  Wire.write(0x5F);
  Wire.endTransmission();
  delay(5);

  // read it back
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(0x3A);
  Wire.endTransmission();
  Wire.requestFrom(KMX62_DeviceAddress, 1);
  int val = Wire.read();
  Wire.endTransmission();
  return val;
}

int kmx62TestResponse(){  // should return 0x55
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(KMX_COTR);
  Wire.endTransmission();
  
  Wire.requestFrom(KMX62_DeviceAddress, 1);
  return(Wire.read());
  Wire.endTransmission();
}

 //---------- Start Code for Reading KMX62 Kionix Accelerometer+Mag Sensor ----------  
  // -- Notes on ROHM KMX62 Accel/Mag Sensor --
  //Device Address = 0x0Eu
  //12 Bit Return Value
  
  //Intialization Routines (See Setup function above)
  // 1. Standby Register (0x29), write 0x03 (Turn Off)
  // 2. Self Test Register (0x60), write 0x00
  // 3. Control Register 1 (0x2A), write 0x13
  // 4. Control Register 2 (0x2B), write 0x00
  // 5. ODCNTL Register (0x2C), write 0x00
  // 6. Temp EN Register (0x4C), write 0x01
  // 7. Buffer CTRL Register 1 (0x78), write 0x00
  // 8. Buffer CTRL Register 2 (0x79), write 0x00
  // 9. Standby Register (0x29), write 0x0u
  
  //Main Loop Routines
  // 1. Read 6 Bytes starting from address 0x0A.  These will be the accelerometer output. [0][1]...[5]
  // 2. Xout = ([1]<<6) | ([0]>>2)
  // 3. Yout = ([3]<<6) | ([2]>>2)
  // 4. Zout = ([5]<<6) | ([4]>>2)
  // 5. Read 6 Bytes starting from addres 0x12. These will be the magnetometer output. [0][1]...[5]  
  // 6. Xout = ([1]<<6) | ([0]>>2)
  // 7. Yout = ([3]<<6) | ([2]>>2)
  // 8. Zout = ([5]<<6) | ([4]>>2)

  void readKMX62(){
  // Start Getting Data from Accel
  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(0x0A);
  Wire.endTransmission(0);
  
  Wire.requestFrom(KMX62_DeviceAddress, 6, 0);
  //Serial.println(Wire.available());
  MEMS_Accel_Xout_lowByte = Wire.read();
  MEMS_Accel_Xout_highByte = Wire.read();
  MEMS_Accel_Yout_lowByte = Wire.read();
  MEMS_Accel_Yout_highByte = Wire.read();
  MEMS_Accel_Zout_lowByte = Wire.read();
  MEMS_Accel_Zout_highByte = Wire.read();
  Wire.endTransmission();
  
  accelX = (MEMS_Accel_Xout_highByte<<8) | (MEMS_Accel_Xout_lowByte);
  accelY = (MEMS_Accel_Yout_highByte<<8) | (MEMS_Accel_Yout_lowByte);
  accelZ = (MEMS_Accel_Zout_highByte<<8) | (MEMS_Accel_Zout_lowByte);
  
  MEMS_Accel_Conv_Xout = (float)accelX/8192;
  MEMS_Accel_Conv_Yout = (float)accelY/8192;
  MEMS_Accel_Conv_Zout = (float)accelZ/8192;

  Wire.beginTransmission(KMX62_DeviceAddress);
  Wire.write(0x10);
  Wire.endTransmission(0);
  Wire.requestFrom(KMX62_DeviceAddress, 6, 0);
  MEMS_Mag_Xout_lowByte = Wire.read();
  MEMS_Mag_Xout_highByte = Wire.read();
  MEMS_Mag_Yout_lowByte = Wire.read();
  MEMS_Mag_Yout_highByte = Wire.read();
  MEMS_Mag_Zout_lowByte = Wire.read();
  MEMS_Mag_Zout_highByte = Wire.read();
  Wire.endTransmission();
  
  magX = (MEMS_Mag_Xout_highByte<<8) | (MEMS_Mag_Xout_lowByte);
  magY = (MEMS_Mag_Yout_highByte<<8) | (MEMS_Mag_Yout_lowByte);
  magZ = (MEMS_Mag_Zout_highByte<<8) | (MEMS_Mag_Zout_lowByte);
  
  MEMS_Mag_Conv_Xout = (float)magX/27.30666619;
  MEMS_Mag_Conv_Yout = (float)magY/27.30666619;
  MEMS_Mag_Conv_Zout = (float)magZ/27.30666619;
  }  
