int imuAddress = 0x68;
int compassAddress = 0x0C;

// Need to select User Bank before writing to register in that bank
#define USR_BNK_0 (0x00)
#define USR_BNK_1 (0x10)
#define USR_BNK_2 (0x20)
#define USR_BNK_3 (0x30)

// USER BANK 0 REGISTER MAP
#define IMU_WHO_AM_I        (0x00)
#define IMU_USER_CTRL       (0x03)
#define IMU_LP_CONFIG       (0x05)
#define IMU_PWR_MGMT_1      (0x06)
#define IMU_PWR_MGMT_2      (0x07)
#define IMU_INT_PIN_CFG     (0x0F)
#define IMU_INT_ENABLE      (0x10)
#define IMU_INT_ENABLE_1    (0x11)
#define IMU_INT_ENABLE_2    (0x12)
#define IMU_INT_ENABLE_3    (0x13)
#define IMU_I2C_MST_STATUS  (0x17)
#define IMU_INT_STATUS      (0x19)
#define IMU_INT_STATUS_1    (0x1A)
#define IMU_INT_STATUS_2    (0x1B)
#define IMU_INT_STATUS_3    (0x1C)
#define IMU_DELAY_TIMEH     (0x28)
#define IMU_DELAY_TIMEL     (0x29)
#define IMU_ACCEL_XOUT_H    (0x2D)
#define IMU_ACCEL_XOUT_L    (0x2E)
#define IMU_ACCEL_YOUT_H    (0x2F)
#define IMU_ACCEL_YOUT_L    (0x30)
#define IMU_ACCEL_ZOUT_H    (0x31)
#define IMU_ACCEL_ZOUT_L    (0x32)

#define IMU_GYRO_XOUT_H (0x33)
#define IMU_GYRO_XOUT_L (0x34)
#define IMU_GYRO_YOUT_H (0x35)
#define IMU_GYRO_YOUT_L (0x36)
#define IMU_GYRO_ZOUT_H (0x37)
#define IMU_GYRO_ZOUT_L (0x38)

#define IMU_TEMP_OUT_H (0x39)
#define IMU_TEMP_OUT_L (0x3A)

#define IMU_EXT_SLV_DATA_00 (0x3B)
#define IMU_EXT_SLV_DATA_01 (0x3C)
#define IMU_EXT_SLV_DATA_02 (0x3D)
#define IMU_EXT_SLV_DATA_03 (0x3E)
#define IMU_EXT_SLV_DATA_04 (0x3F)
#define IMU_EXT_SLV_DATA_05 (0x40)
#define IMU_EXT_SLV_DATA_06 (0x41)
#define IMU_EXT_SLV_DATA_07 (0x42)
#define IMU_EXT_SLV_DATA_08 (0x43)
#define IMU_EXT_SLV_DATA_09 (0x44)
#define IMU_EXT_SLV_DATA_10 (0x45)
#define IMU_EXT_SLV_DATA_11 (0x46)
#define IMU_EXT_SLV_DATA_12 (0x47)
#define IMU_EXT_SLV_DATA_13 (0x48)
#define IMU_EXT_SLV_DATA_14 (0x49)
#define IMU_EXT_SLV_DATA_15 (0x4A)
#define IMU_EXT_SLV_DATA_16 (0x4B)
#define IMU_EXT_SLV_DATA_17 (0x4C)
#define IMU_EXT_SLV_DATA_18 (0x4D)
#define IMU_EXT_SLV_DATA_19 (0x4E)
#define IMU_EXT_SLV_DATA_20 (0x4F)
#define IMU_EXT_SLV_DATA_21 (0x50)
#define IMU_EXT_SLV_DATA_22 (0x51)
#define IMU_EXT_SLV_DATA_23 (0x52)

#define IMU_FIFO_EN_1       (0x66)
#define IMU_FIFO_EN_2       (0x67)
#define IMU_FIFO_RST        (0x68)
#define IMU_FIFO_MODE       (0x69)
#define IMU_FIFO_COUNTH     (0x70)
#define IMU_FIFO_COUNTL     (0x71)
#define IMU_FIFO_R_W        (0x72)
#define IMU_DATA_RDY_STATUS (0x74)
#define IMU_FIFO_CFG        (0x76)
#define IMU_REG_BANK_SEL    (0x7F)

// User Bank 1
#define IMU_SELF_TEST_X_GYRO  (0x02)
#define IMU_SELF_TEST_Y_GYRO  (0x03)
#define IMU_SELF_TEST_Z_GYRO  (0x04)
#define IMU_SELF_TEST_X_ACCEL (0x0E)
#define IMU_SELF_TEST_Y_ACCEL (0x0F)
#define IMU_SELF_TEST_Z_ACCEL (0x10)
#define IMU_XA_OFFS_H         (0x14)  
#define IMU_XA_OFFS_L         (0x15)
#define IMU_YA_OFFS_H         (0x17)  
#define IMU_YA_OFFS_L         (0x18)
#define IMU_ZA_OFFS_H         (0x1A)  
#define IMU_ZA_OFFS_L         (0x1B)
#define IMU_TIMEBASE_CORRECTION_PLL (0x28)

// User Bank 2
#define IMU_GYRO_SMPLRT_DIV     (0x00)
#define IMU_GYRO_CONFIG_1       (0x01)
#define IMU_GYRO_CONFIG_2       (0x02)
#define IMU_XG_OFFS_USRH        (0x03)
#define IMU_XG_OFFS_USRL        (0x04)
#define IMU_YG_OFFS_USRH        (0x05)
#define IMU_YG_OFFS_USRL        (0x06)
#define IMU_ZG_OFFS_USRH        (0x07)
#define IMU_ZG_OFFS_USRL        (0x08)
#define IMU_ODR_ALIGN_EN        (0x09)
#define IMU_ACCEL_SMPLRT_DIV_1  (0x10)
#define IMU_ACCEL_SMPLRT_DIV_2  (0x11)
#define IMU_ACCEL_INTEL_CTRL    (0x12)
#define IMU_ACCEL_WOM_THR       (0x13)
#define IMU_ACCEL_CONFIG        (0x14)
#define IMU_ACCEL_CONFIG_2      (0x15)
#define IMU_FSYNC_CONFIG        (0x52)
#define IMU_TEMP_CONFIG         (0x53)
#define IMU_MOD_CTRL_USR        (0x54)

// User Bank 3
#define IMU_I2C_MST_ODR_CONFIG  (0x00)
#define IMU_I2C_MST_CTRL        (0x01)
#define IMU_I2C_MST_DELAY_CTRL  (0x02)
#define IMU_I2C_SLV0_ADDR       (0x03)
#define IMU_I2C_SLV0_REG        (0x04)
#define IMU_I2C_SLV0_CTRL       (0x05)
#define IMU_I2C_SLV0_DO         (0x06)
#define IMU_I2C_SLV1_ADDR       (0x07)
#define IMU_I2C_SLV1_REG        (0x08)
#define IMU_I2C_SLV1_CTRL       (0x09)
#define IMU_I2C_SLV1_DO         (0x0A)
#define IMU_I2C_SLV2_ADDR       (0x0B)
#define IMU_I2C_SLV2_REG        (0x0C)
#define IMU_I2C_SLV2_CTRL       (0x0D)
#define IMU_I2C_SLV2_DO         (0x0E)
#define IMU_I2C_SLV3_ADDR       (0x0F)
#define IMU_I2C_SLV3_REG        (0x10)
#define IMU_I2C_SLV3_CTRL       (0x11)
#define IMU_I2C_SLV3_DO         (0x12)
#define IMU_I2C_SLV4_ADDR       (0x13)
#define IMU_I2C_SLV4_REG        (0x14)
#define IMU_I2C_SLV4_CTRL       (0x15)
#define IMU_I2C_SLV4_DO         (0x16)
#define IMU_I2C_SLV4_DI         (0x17)

// Magnetometer
#define AKM_WIA2        (0x01)
#define AKM_ST1         (0x10)
#define AKM_HXL         (0x11)
#define AKM_ST2         (0x18)
#define AKM_CNTL2       (0x31)
#define AKM_CNTL3       (0x32)

// ST1: Status 1 Read Only
#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)

// ST2: Status 2 Read Only
#define AKM_OVERFLOW        (0x08)

// CNTL2: Control2 Read/Write
#define AKM_POWER_DOWN                    (0x00)
#define AKM_SINGLE_MEASUREMENT_MODE       (0x01)
#define AKM_CONTINUOUS_MEASUREMENT_MODE1  (0x02) // 10 Hz
#define AKM_CONTINUOUS_MEASUREMENT_MODE2  (0x04) // 20 Hz
#define AKM_CONTINUOUS_MEASUREMENT_MODE3  (0x06) // 40 Hz
#define AKM_CONTINUOUS_MEASUREMENT_MODE4  (0x08) // 100 Hz
#define AKM_MODE_SELF_TEST                (0x10)

// CNTL3: Control 3 Read/Write
#define AKM_SOFT_RESET (0x01)

#define BIT_I2C_READ        (0x80)
#define BIT_SLAVE_EN        (0x80)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)

int mpuInit(boolean mode)
{
  int ecode;
   if (printDiags) {
    SerialUSB.println("MPU Init");
   }

  SerialUSB.print("IMU ID:");
  SerialUSB.println(I2Cread8(imuAddress, IMU_WHO_AM_I));
  if(mode==0){
     ecode = I2Cwrite(imuAddress, IMU_PWR_MGMT_1, 0x49);  //Sleep mode, temp sensor off, auto select clock
     return ecode;
  }
  
  I2Cwrite(imuAddress, IMU_PWR_MGMT_1, 0x01); // get out of sleep mode, make sure LP_EN disabled so can write to all registers
  I2Cwrite(imuAddress, IMU_PWR_MGMT_2, 0x00); // accelerometer and gyroscope all axes on

  //I2Cwrite(imuAddress, IMU_LP_CONFIG, 0x00); // disable duty cycled mode
  
  //I2Cwrite(imuAddress, IMU_USER_CTRL, 0x00); // DMP disabled, FIFO disabled, I2C pass through mode to set up magnetometer
  I2Cwrite(imuAddress, IMU_USER_CTRL, 0x20); // DMP disabled, FIFO disabled, I2C master mode so talk to magnetometer through ICM
  
  I2Cwrite(imuAddress, IMU_INT_PIN_CFG,0x12); //active high, push-pull, 50 us interrupt pulse, clear on any read, FSYNC disable interrupt, bypass enable 
  //I2Cwrite(imuAddress, IMU_INT_PIN_CFG,0x10); //active high, push-pull, 50 us interrupt pulse, clear on any read, FSYNC disable interrupt

  setupMag();

  I2Cwrite(imuAddress, IMU_INT_ENABLE, 0x00); // no interrupts from FSYNC, wake on motion, PLL RDY, DMP, or I2C master
  I2Cwrite(imuAddress, IMU_INT_ENABLE_1, 0x00); // no interrupt from raw data ready
  I2Cwrite(imuAddress, IMU_INT_ENABLE_2, 0x00); // no interrupt from FIFO overflow
  I2Cwrite(imuAddress, IMU_INT_ENABLE_3, 0x00); // no interrupt from FIFO watermark

  byte sampleRateDiv = 0x09;
  // Gyroscope setup USR BANK 2
  I2Cwrite(imuAddress, IMU_REG_BANK_SEL, USR_BNK_2); // select register bank 2
  I2Cwrite(imuAddress, IMU_GYRO_SMPLRT_DIV, sampleRateDiv);   // GYRO_SMPLRT_DIV=0x09=9; 110 Hz sample rate = 1100 Hz / (1+GYRO_SMPLRT_DIV)
  I2Cwrite(imuAddress, IMU_GYRO_CONFIG_1, 0x1F); //Full scale = 2000 dps; FCHOICE=1 (LPF ON), DLPFCFG=3; 3DB BW = 51.2 Hz
  I2Cwrite(imuAddress, IMU_GYRO_CONFIG_2, 0x02); // 4x averaging 

  I2Cwrite(imuAddress, IMU_ODR_ALIGN_EN, 0x01); //ODR start time alignment when gyro or accel sample rate registers written
  
  // Accelerometer setup USR BANK 2
  I2Cwrite(imuAddress, IMU_ACCEL_SMPLRT_DIV_1, 0x00);
  I2Cwrite(imuAddress, IMU_ACCEL_SMPLRT_DIV_2, sampleRateDiv); // same sample rate as Gyro
  I2Cwrite(imuAddress, IMU_ACCEL_CONFIG, 0x1F); // DLPF=3; Accelerometer full scale = +/-16g; enable DLPF
  I2Cwrite(imuAddress, IMU_ACCEL_CONFIG_2, 0x00); // 4x averaging
  I2Cwrite(imuAddress, IMU_ACCEL_INTEL_CTRL, 0x00); // no wake on motion
  
  // start taking readings
  I2Cwrite(imuAddress, IMU_REG_BANK_SEL, USR_BNK_0); // back to User bank 0
  I2Cwrite(imuAddress, IMU_FIFO_EN_1, 0x01); // EXT_SENS_DATA registers associated to SLV_0 to the FIFO
  I2Cwrite(imuAddress, IMU_FIFO_EN_2, 0x10); // accel and gyro to FIFO, not temperature
  I2Cwrite(imuAddress, IMU_FIFO_MODE, 0x00); // Stream data to FIFO
  I2Cwrite(imuAddress, IMU_PWR_MGMT_1, 0x01); // power on, auto-select best clock source, temperature sensor enabled
  I2Cwrite(imuAddress, IMU_USER_CTRL, 0xE0); // DMP enabled, FIFO enabled, I2C Master enabled to get magnetometer
  resetGyroFIFO();
  return ecode;
}

int setupMag(){
  // AKM is magnetometer
  I2Cwrite(imuAddress, IMU_REG_BANK_SEL, USR_BNK_3); // User bank 3 for I2C commands through gyro

  I2Cwrite(imuAddress, IMU_I2C_MST_CTRL, 0x07);  // set I2C master clock frequency to 7 (recommended for max 400 kHz i2c)
  
  // Use Slave 4 to talk to compass via ICM to configure and start conversions
  // try to read ID
  I2Cwrite(imuAddress, IMU_I2C_SLV4_ADDR, BIT_I2C_READ | compassAddress);
  I2Cwrite(imuAddress, IMU_I2C_SLV4_REG, AKM_WIA2); // get who am i of compass
  I2Cwrite(imuAddress, IMU_I2C_SLV4_CTRL, BIT_SLAVE_EN | 0x01); // conversion of 1 byte
  delay(1);
  SerialUSB.print("Magnetometer ID:");
  SerialUSB.println(I2Cread8(imuAddress, IMU_I2C_SLV4_DI));
  
  I2Cwrite(imuAddress, IMU_I2C_SLV4_ADDR, compassAddress); // transfer is a write, address of compass slave
  I2Cwrite(imuAddress, IMU_I2C_SLV4_REG, AKM_CNTL2); // I2C slave 4 register address from where to begin transfer
 // I2Cwrite(imuAddress, IMU_I2C_SLV4_DO, AKM_SINGLE_MEASUREMENT_MODE); // load data out on IMU to send to AKM
  I2Cwrite(imuAddress, IMU_I2C_SLV4_DO, AKM_CONTINUOUS_MEASUREMENT_MODE4); // load data out on IMU to send to AKM
  // read back register to see if set
  SerialUSB.print("SLV4_DO:");
  SerialUSB.println(I2Cread8(imuAddress, IMU_I2C_SLV4_DO));
  
  I2Cwrite(imuAddress, IMU_I2C_SLV4_CTRL, BIT_SLAVE_EN);  // enable transfer to AKM slave

  // Slave 0 will handle getting data from magnetometer
  I2Cwrite(imuAddress, IMU_I2C_SLV0_ADDR, BIT_I2C_READ | compassAddress); //request read
  I2Cwrite(imuAddress, IMU_I2C_SLV0_REG, AKM_HXL); // I2c slave 0 register address from where to begin data transfer
  // need to read 8 bytes so read through ST2 register, so frees compass registers for new data writes
  I2Cwrite(imuAddress, IMU_I2C_SLV0_CTRL, BIT_SLAVE_EN  | BIT_SLAVE_GROUP | BIT_SLAVE_BYTE_SW | 0x08); // store data to ext_sens_data register; swap bytes; even numbered register ends group; read 8 bytes (0x08) 

   
  I2Cwrite(imuAddress, IMU_REG_BANK_SEL, USR_BNK_0); // back to User bank 0
//  I2Cwrite(imuAddress, IMU_I2C_MST_DELAY_CTRL, 0x11); // slave 0 and slave 4 delay--triggers them together?
  return 1;
}
void resetGyroFIFO(){
    I2Cwrite(imuAddress, IMU_FIFO_RST, 0x01); // reset FIFO
    I2Cwrite(imuAddress, IMU_FIFO_RST, 0x00); // deassert
}

byte I2Cwrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  return ecode;
}

byte I2Cread8(byte addr, byte reg){
  Wire.beginTransmission(addr);  
  Wire.write(reg);  
  Wire.endTransmission();
  Wire.beginTransmission(addr); 
  Wire.requestFrom(addr,(uint8_t)1);
  uint8_t data = Wire.read();
  Wire.endTransmission();
  return data;
}

void readImu()
{
  int i = 0;
  Wire.beginTransmission(imuAddress); 
  Wire.write(IMU_ACCEL_XOUT_H); //sends address to read from IMU_ACCEL_XOUT_H is direct read; IMU_FIFO_R_W is FIFO
  Wire.endTransmission(0); //send restart to keep connection alive
  Wire.requestFrom(imuAddress, 20, 0); //send restart to keep alive // read 20 because need 2 additional bytes to reset magnetometer
  while(Wire.available()){
    imuTempBuffer[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
}

int getImuFifo()
{
 int fifopts; 
 // read FIFO size
  Wire.beginTransmission(imuAddress); 
  Wire.write(IMU_FIFO_COUNTH);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(imuAddress, 2);  
  
  byte FIFO_CNT[2];
  int i=0;
 
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    FIFO_CNT[i]= Wire.read();  // receive one byte
    i++;
  }
  fifopts=((FIFO_CNT[0]&0x1F)<<8)|FIFO_CNT[1];
   
 return fifopts; 
}

// direct read of compass
void readCompass(){
  Wire.beginTransmission(compassAddress);
  Wire.write(AKM_HXL);
  Wire.endTransmission();
  Wire.requestFrom(compassAddress, 8);
  int i = 12;
  while(Wire.available()){
    imuTempBuffer[i] = Wire.read();
    i++;
  }
}

int intStatus(){
  byte intStatus; 
  Wire.beginTransmission(imuAddress); 
  Wire.write(IMU_INT_STATUS);        
  Wire.endTransmission();           
  Wire.requestFrom(imuAddress, 1); 
  if(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    intStatus = Wire.read();  // receive one byte
  }
 return intStatus; 
}
