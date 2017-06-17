int icmAddress = 0x68;
int icmMagAddress = 0x0C;

// Need to select User Bank before writing to register in that bank
#define USR_BNK_0 (0x00)
#define USR_BNK_1 (0x10)
#define USR_BNK_2 (0x20)
#define USR_BNK_3 (0x30)

// USER BANK 0 REGISTER MAP
#define ICM_WHO_AM_I        (0x00)
#define ICM_USER_CTRL       (0x03)
#define ICM_LP_CONFIG       (0x05)
#define ICM_PWR_MGMT_1      (0x06)
#define ICM_PWR_MGMT_2      (0x07)
#define ICM_INT_PIN_CFG     (0x0F)
#define ICM_INT_ENABLE      (0x10)
#define ICM_INT_ENABLE_1    (0x11)
#define ICM_INT_ENABLE_2    (0x12)
#define ICM_INT_ENABLE_3    (0x13)
#define ICM_I2C_MST_STATUS  (0x17)
#define ICM_INT_STATUS      (0x19)
#define ICM_INT_STATUS_1    (0x1A)
#define ICM_INT_STATUS_2    (0x1B)
#define ICM_INT_STATUS_3    (0x1C)
#define ICM_DELAY_TIMEH     (0x28)
#define ICM_DELAY_TIMEL     (0x29)
#define ICM_ACCEL_XOUT_H    (0x2D)
#define ICM_ACCEL_XOUT_L    (0x2E)
#define ICM_ACCEL_YOUT_H    (0x2F)
#define ICM_ACCEL_YOUT_L    (0x30)
#define ICM_ACCEL_ZOUT_H    (0x31)
#define ICM_ACCEL_ZOUT_L    (0x32)

#define ICM_GYRO_XOUT_H (0x33)
#define ICM_GYRO_XOUT_L (0x34)
#define ICM_GYRO_YOUT_H (0x35)
#define ICM_GYRO_YOUT_L (0x36)
#define ICM_GYRO_ZOUT_H (0x37)
#define ICM_GYRO_ZOUT_L (0x38)

#define ICM_TEMP_OUT_H (0x39)
#define ICM_TEMP_OUT_L (0x3A)

#define ICM_EXT_SLV_DATA_00 (0x3B)
#define ICM_EXT_SLV_DATA_01 (0x3C)
#define ICM_EXT_SLV_DATA_02 (0x3D)
#define ICM_EXT_SLV_DATA_03 (0x3E)
#define ICM_EXT_SLV_DATA_04 (0x3F)
#define ICM_EXT_SLV_DATA_05 (0x40)
#define ICM_EXT_SLV_DATA_06 (0x41)
#define ICM_EXT_SLV_DATA_07 (0x42)
#define ICM_EXT_SLV_DATA_08 (0x43)
#define ICM_EXT_SLV_DATA_09 (0x44)
#define ICM_EXT_SLV_DATA_10 (0x45)
#define ICM_EXT_SLV_DATA_11 (0x46)
#define ICM_EXT_SLV_DATA_12 (0x47)
#define ICM_EXT_SLV_DATA_13 (0x48)
#define ICM_EXT_SLV_DATA_14 (0x49)
#define ICM_EXT_SLV_DATA_15 (0x4A)
#define ICM_EXT_SLV_DATA_16 (0x4B)
#define ICM_EXT_SLV_DATA_17 (0x4C)
#define ICM_EXT_SLV_DATA_18 (0x4D)
#define ICM_EXT_SLV_DATA_19 (0x4E)
#define ICM_EXT_SLV_DATA_20 (0x4F)
#define ICM_EXT_SLV_DATA_21 (0x50)
#define ICM_EXT_SLV_DATA_22 (0x51)
#define ICM_EXT_SLV_DATA_23 (0x52)

#define ICM_FIFO_EN_1       (0x66)
#define ICM_FIFO_EN_2       (0x67)
#define ICM_FIFO_RST        (0x68)
#define ICM_FIFO_MODE       (0x69)
#define ICM_FIFO_COUNTH     (0x70)
#define ICM_FIFO_COUNTL     (0x71)
#define ICM_FIFO_R_W        (0x72)
#define ICM_DATA_RDY_STATUS (0x74)
#define ICM_FIFO_CFG        (0x76)
#define ICM_REG_BANK_SEL    (0x7F)

// User Bank 1
#define ICM_SELF_TEST_X_GYRO  (0x02)
#define ICM_SELF_TEST_Y_GYRO  (0x03)
#define ICM_SELF_TEST_Z_GYRO  (0x04)
#define ICM_SELF_TEST_X_ACCEL (0x0E)
#define ICM_SELF_TEST_Y_ACCEL (0x0F)
#define ICM_SELF_TEST_Z_ACCEL (0x10)
#define ICM_XA_OFFS_H         (0x14)  
#define ICM_XA_OFFS_L         (0x15)
#define ICM_YA_OFFS_H         (0x17)  
#define ICM_YA_OFFS_L         (0x18)
#define ICM_ZA_OFFS_H         (0x1A)  
#define ICM_ZA_OFFS_L         (0x1B)
#define ICM_TIMEBASE_CORRECTION_PLL (0x28)

// User Bank 2
#define ICM_GYRO_SMPLRT_DIV     (0x00)
#define ICM_GYRO_CONFIG_1       (0x01)
#define ICM_GYRO_CONFIG_2       (0x02)
#define ICM_XG_OFFS_USRH        (0x03)
#define ICM_XG_OFFS_USRL        (0x04)
#define ICM_YG_OFFS_USRH        (0x05)
#define ICM_YG_OFFS_USRL        (0x06)
#define ICM_ZG_OFFS_USRH        (0x07)
#define ICM_ZG_OFFS_USRL        (0x08)
#define ICM_ODR_ALIGN_EN        (0x09)
#define ICM_ACCEL_SMPLRT_DIV_1  (0x10)
#define ICM_ACCEL_SMPLRT_DIV_2  (0x11)
#define ICM_ACCEL_INTEL_CTRL    (0x12)
#define ICM_ACCEL_WOM_THR       (0x13)
#define ICM_ACCEL_CONFIG        (0x14)
#define ICM_ACCEL_CONFIG_2      (0x15)
#define ICM_FSYNC_CONFIG        (0x52)
#define ICM_TEMP_CONFIG         (0x53)
#define ICM_MOD_CTRL_USR        (0x54)

// User Bank 3
#define ICM_I2C_MST_ODR_CONFIG  (0x00)
#define ICM_I2C_MST_CTRL        (0x01)
#define ICM_I2C_MST_DELAY_CTRL  (0x02)
#define ICM_I2C_SLV0_ADDR       (0x03)
#define ICM_I2C_SLV0_REG        (0x04)
#define ICM_I2C_SLV0_CTRL       (0x05)
#define ICM_I2C_SLV0_DO         (0x06)
#define ICM_I2C_SLV1_ADDR       (0x07)
#define ICM_I2C_SLV1_REG        (0x08)
#define ICM_I2C_SLV1_CTRL       (0x09)
#define ICM_I2C_SLV1_DO         (0x0A)
#define ICM_I2C_SLV2_ADDR       (0x0B)
#define ICM_I2C_SLV2_REG        (0x0C)
#define ICM_I2C_SLV2_CTRL       (0x0D)
#define ICM_I2C_SLV2_DO         (0x0E)
#define ICM_I2C_SLV3_ADDR       (0x0F)
#define ICM_I2C_SLV3_REG        (0x10)
#define ICM_I2C_SLV3_CTRL       (0x11)
#define ICM_I2C_SLV3_DO         (0x12)
#define ICM_I2C_SLV4_ADDR       (0x13)
#define ICM_I2C_SLV4_REG        (0x14)
#define ICM_I2C_SLV4_CTRL       (0x15)
#define ICM_I2C_SLV4_DO         (0x16)
#define ICM_I2C_SLV4_DI         (0x17)

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

int icmInit(boolean mode)
{
  int ecode;
   if (printDiags) {
    SerialUSB.println("MPU Init");
   }

  SerialUSB.print("ICM ID:");
  SerialUSB.println(icmI2cRead8(icmAddress, ICM_WHO_AM_I));
  if(mode==0){
     ecode = icmI2cWrite(icmAddress, ICM_PWR_MGMT_1, 0x49);  //Sleep mode, temp sensor off, auto select clock
     return ecode;
  }
  
  icmI2cWrite(icmAddress, ICM_PWR_MGMT_1, 0x01); // get out of sleep mode, make sure LP_EN disabled so can write to all registers
  icmI2cWrite(icmAddress, ICM_PWR_MGMT_2, 0x00); // accelerometer and gyroscope all axes on

  //icmI2cWrite(icmAddress, ICM_LP_CONFIG, 0x00); // disable duty cycled mode
  
  //icmI2cWrite(icmAddress, ICM_USER_CTRL, 0x00); // DMP disabled, FIFO disabled, I2C pass through mode to set up magnetometer
  icmI2cWrite(icmAddress, ICM_USER_CTRL, 0x20); // DMP disabled, FIFO disabled, I2C master mode so talk to magnetometer through ICM
  
  icmI2cWrite(icmAddress, ICM_INT_PIN_CFG,0x12); //active high, push-pull, 50 us interrupt pulse, clear on any read, FSYNC disable interrupt, bypass enable 
  //icmI2cWrite(icmAddress, ICM_INT_PIN_CFG,0x10); //active high, push-pull, 50 us interrupt pulse, clear on any read, FSYNC disable interrupt

  setupMag();

  icmI2cWrite(icmAddress, ICM_INT_ENABLE, 0x00); // no interrupts from FSYNC, wake on motion, PLL RDY, DMP, or I2C master
  icmI2cWrite(icmAddress, ICM_INT_ENABLE_1, 0x00); // no interrupt from raw data ready
  icmI2cWrite(icmAddress, ICM_INT_ENABLE_2, 0x00); // no interrupt from FIFO overflow
  icmI2cWrite(icmAddress, ICM_INT_ENABLE_3, 0x00); // no interrupt from FIFO watermark

  byte sampleRateDiv = 0x09;
  // Gyroscope setup USR BANK 2
  icmI2cWrite(icmAddress, ICM_REG_BANK_SEL, USR_BNK_2); // select register bank 2
  icmI2cWrite(icmAddress, ICM_GYRO_SMPLRT_DIV, sampleRateDiv);   // GYRO_SMPLRT_DIV=0x09=9; 110 Hz sample rate = 1100 Hz / (1+GYRO_SMPLRT_DIV)
  icmI2cWrite(icmAddress, ICM_GYRO_CONFIG_1, 0x1F); //Full scale = 2000 dps; FCHOICE=1 (LPF ON), DLPFCFG=3; 3DB BW = 51.2 Hz
  icmI2cWrite(icmAddress, ICM_GYRO_CONFIG_2, 0x02); // 4x averaging 

  icmI2cWrite(icmAddress, ICM_ODR_ALIGN_EN, 0x01); //ODR start time alignment when gyro or accel sample rate registers written
  
  // Accelerometer setup USR BANK 2
  icmI2cWrite(icmAddress, ICM_ACCEL_SMPLRT_DIV_1, 0x00);
  icmI2cWrite(icmAddress, ICM_ACCEL_SMPLRT_DIV_2, sampleRateDiv); // same sample rate as Gyro
  icmI2cWrite(icmAddress, ICM_ACCEL_CONFIG, 0x1F); // DLPF=3; Accelerometer full scale = +/-16g; enable DLPF
  icmI2cWrite(icmAddress, ICM_ACCEL_CONFIG_2, 0x00); // 4x averaging
  icmI2cWrite(icmAddress, ICM_ACCEL_INTEL_CTRL, 0x00); // no wake on motion
  
  // start taking readings
  icmI2cWrite(icmAddress, ICM_REG_BANK_SEL, USR_BNK_0); // back to User bank 0
  icmI2cWrite(icmAddress, ICM_FIFO_EN_1, 0x01); // EXT_SENS_DATA registers associated to SLV_0 to the FIFO
  icmI2cWrite(icmAddress, ICM_FIFO_EN_2, 0x10); // accel and gyro to FIFO, not temperature
  icmI2cWrite(icmAddress, ICM_FIFO_MODE, 0x00); // Stream data to FIFO
  icmI2cWrite(icmAddress, ICM_PWR_MGMT_1, 0x01); // power on, auto-select best clock source, temperature sensor enabled
  icmI2cWrite(icmAddress, ICM_USER_CTRL, 0xE0); // DMP enabled, FIFO enabled, I2C Master enabled to get magnetometer
  resetGyroFIFO();
  return ecode;
}

int setupMag(){
  // AKM is magnetometer
  icmI2cWrite(icmAddress, ICM_REG_BANK_SEL, USR_BNK_3); // User bank 3 for I2C commands through gyro

  icmI2cWrite(icmAddress, ICM_I2C_MST_CTRL, 0x07);  // set I2C master clock frequency to 7 (recommended for max 400 kHz i2c)
  
  // Use Slave 4 to talk to compass via ICM to configure and start conversions
  // try to read ID
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_ADDR, BIT_I2C_READ | icmMagAddress);
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_REG, AKM_WIA2); // get who am i of compass
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_CTRL, BIT_SLAVE_EN | 0x01); // conversion of 1 byte
  delay(1);
  SerialUSB.print("Magnetometer ID:");
  SerialUSB.println(icmI2cRead8(icmAddress, ICM_I2C_SLV4_DI));
  
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_ADDR, icmMagAddress); // transfer is a write, address of compass slave
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_REG, AKM_CNTL2); // I2C slave 4 register address from where to begin transfer
 // icmI2cWrite(icmAddress, ICM_I2C_SLV4_DO, AKM_SINGLE_MEASUREMENT_MODE); // load data out on ICM to send to AKM
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_DO, AKM_CONTINUOUS_MEASUREMENT_MODE4); // load data out on ICM to send to AKM
  // read back register to see if set
  SerialUSB.print("SLV4_DO:");
  SerialUSB.println(icmI2cRead8(icmAddress, ICM_I2C_SLV4_DO));
  
  icmI2cWrite(icmAddress, ICM_I2C_SLV4_CTRL, BIT_SLAVE_EN);  // enable transfer to AKM slave

  // Slave 0 will handle getting data from magnetometer
  icmI2cWrite(icmAddress, ICM_I2C_SLV0_ADDR, BIT_I2C_READ | icmMagAddress); //request read
  icmI2cWrite(icmAddress, ICM_I2C_SLV0_REG, AKM_HXL); // I2c slave 0 register address from where to begin data transfer
  // need to read 8 bytes so read through ST2 register, so frees compass registers for new data writes
  icmI2cWrite(icmAddress, ICM_I2C_SLV0_CTRL, BIT_SLAVE_EN  | BIT_SLAVE_GROUP | BIT_SLAVE_BYTE_SW | 0x08); // store data to ext_sens_data register; swap bytes; even numbered register ends group; read 8 bytes (0x08) 

   
  icmI2cWrite(icmAddress, ICM_REG_BANK_SEL, USR_BNK_0); // back to User bank 0
//  icmI2cWrite(icmAddress, ICM_I2C_MST_DELAY_CTRL, 0x11); // slave 0 and slave 4 delay--triggers them together?
  return 1;
}
void resetICMFIFO(){
    icmI2cWrite(icmAddress, ICM_FIFO_RST, 0x01); // reset FIFO
    icmI2cWrite(icmAddress, ICM_FIFO_RST, 0x00); // deassert
}

byte icmI2cWrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  return ecode;
}

byte icmI2cRead8(byte addr, byte reg){
  Wire.beginTransmission(addr);  
  Wire.write(reg);  
  Wire.endTransmission();
  Wire.beginTransmission(addr); 
  Wire.requestFrom(addr,(uint8_t)1);
  uint8_t data = Wire.read();
  Wire.endTransmission();
  return data;
}

void readIcu()
{
  int i = 0;
  Wire.beginTransmission(icmAddress); 
  Wire.write(ICM_ACCEL_XOUT_H); //sends address to read from ICM_ACCEL_XOUT_H is direct read; ICM_FIFO_R_W is FIFO
  Wire.endTransmission(0); //send restart to keep connection alive
  Wire.requestFrom(icmAddress, 20, 0); //send restart to keep alive // read 20 because need 2 additional bytes to reset magnetometer
  while(Wire.available()){
    imuTempBuffer[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
}

int getICMFifo()
{
 int fifopts; 
 // read FIFO size
  Wire.beginTransmission(icmAddress); 
  Wire.write(ICM_FIFO_COUNTH);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(icmAddress, 2);  
  
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
void readIcmDirect(){
  Wire.beginTransmission(icmMagAddress);
  Wire.write(AKM_HXL);
  Wire.endTransmission();
  Wire.requestFrom(icmMagAddress, 8);
  int i = 12;
  while(Wire.available()){
    imuTempBuffer[i] = Wire.read();
    i++;
  }
}

int intIcmStatus(){
  byte intStatus; 
  Wire.beginTransmission(icmAddress); 
  Wire.write(ICM_INT_STATUS);        
  Wire.endTransmission();           
  Wire.requestFrom(icmAddress, 1); 
  if(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    intStatus = Wire.read();  // receive one byte
  }
 return intStatus; 
}
