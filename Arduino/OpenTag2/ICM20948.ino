int imuAddress = 0x68;
int compassAddress = 0x0C;

// USER BANK 0 REGISTER MAP
#define IMU_WHO_AM_I        (0x00)
#define IMU_USER_CTRL       (0x03)
#define IMU_LP_CONFIG       (0x05)
#define IMU_PWR_MGMT_1      (0x06)
#define IMU_PWR_MGMT_2      (0x07)
#define IMU_INT_PIN_CFG     (0x0F)
#define IMU_INT_ENABLE_1    (0x10)
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

// Magnetometer
#define AKM_REG_WHOAMI      (0x01)
#define AKM_REG_ST1         (0x10)
#define AKM_REG_HXL         (0x11)
#define AKM_REG_ST2         (0x18)
#define AKM_REG_CNTL2       (0x31)
#define AKM_REG_CNTL3       (0x32)

// ST1: Status 1 Read Only
#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)

// ST2: Status 2 Read Only
#define AKM_OVERFLOW        (0x08)

// CNTL2: Control2 Read/Write
#define AKM_POWER_DOWN                    (0x00)
#define AKM_SINGLE_MEASUREMENT            (0x01)
#define AKM_CONTINUOUS_MEASUREMENT_MODE1  (0x02)
#define AKM_CONTINUOUS_MEASUREMENT_MODE2  (0x04)
#define AKM_CONTINUOUS_MEASUREMENT_MODE3  (0x06)
#define AKM_CONTINUOUS_MEASUREMENT_MODE4  (0x08)
#define AKM_MODE_SELF_TEST                (0x10)

// CNTL3: Control 3 Read/Write
#define AKM_SOFT_RESET (0x01)



#define AKM_WHOAMI      (0x09)

#define BIT_I2C_READ        (0x80)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_MST_VDDIO   (0x80)

#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38

int mpuInit(boolean mode)
{
  int ecode;
   if (printDiags) SerialUSB.print("MPU Init\n");
   if(mode==0)
  {
     ecode = I2Cwrite(imuAddress, 0x6B, 0x40);  //Sleep mode, internal 8 MHz oscillator  //another mode is cycle where it wakes up periodically to take a value
     return ecode;
  }

    //set clock source
    ecode = I2Cwrite(imuAddress, 0x6B, 0x01);  //everything awake; clock from X gyro reference
  
    // configure frame sync and LPF
    I2Cwrite(imuAddress, 0x1A, 0x03);  //no frame sync; Gyro to sample at 1 kHz with DLPF 41 Hz (4.8 ms delay)
    
    // set gyro range
    I2Cwrite(imuAddress, 0x1B, 0x10);  // 0x10 +/- 1000 deg/s ; 0x18 +/-2000 deg/s Fchoice_b = 00 (use DLPF)
    
    // set sample rate divider
    I2Cwrite(imuAddress, 0x19, 9);  //  0x31=49=>20Hz; 1kHz/(1+4)=200; divide 1 kHz/(1+9)=100 Hz sample rate for all sensors

        // set accel range
    I2Cwrite(imuAddress, 0x1C, 0x18); // 0x18 =  +/- 16 g  DEFAULT
    if (accel_scale == 2) I2Cwrite(imuAddress, 0x1C, 0x00); // 2g
    if (accel_scale == 4) I2Cwrite(imuAddress, 0x1C, 0x08); // 4g
    if (accel_scale == 8) I2Cwrite(imuAddress, 0x1C, 0x10); // 8g
  
    // Accelerometer Configuration 2
    I2Cwrite(imuAddress, 0x1D, 0x03); // low pass filter at 41 Hz (11.8 ms delay)

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master.
     I2Cwrite(imuAddress, INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
     I2Cwrite(imuAddress, INT_ENABLE, 0x01);

    // setup compass
    SerialUSB.print("Compass ");
    SerialUSB.println(setup_compass());

    // using FIFO mode to automatically move magnetometer readings
    I2Cwrite(imuAddress, 0x66, 0x07); // reset FIFO
    I2Cwrite(imuAddress, 0x66, 0x60); // FIFO enabled, Master Mode enabled

   return ecode;
}

void resetGyroFIFO(){
    I2Cwrite(imuAddress, 0x66, 0x07); // reset FIFO
    I2Cwrite(imuAddress, 0x66, 0x60); // FIFO enabled, Master Mode enabled
}

byte I2Cwrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  // gyro scale, sample rate and LPF
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  if (printDiags) SerialUSB.print(ecode);
  delay(5);
  return ecode;
}

void readImu()
{
  int i = 0;
  Wire.beginTransmission(imuAddress); 
  Wire.write(0x3B);        //sends address to read from  0x3B is direct read; 0x74 is FIFO
  Wire.endTransmission(0); //send restart to keep connection alive
  Wire.requestFrom(imuAddress, 20, 0); //send restart to keep alive
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
  Wire.write(0x70);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(imuAddress, 2);  
  
  byte FIFO_CNT[2];
  int i=0;
 
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    FIFO_CNT[i]= Wire.read();  // receive one byte
    i++;
  }
  fifopts=FIFO_CNT[0]<<8|FIFO_CNT[1];
   
 return fifopts; 
}

 /* This initialization is similar to the one in ak8975.c. */
int setup_compass(void)
{
   byte data;
   /* Set up master mode, master clock, and ES bit. */
    data = 0x40;
    if (I2Cwrite(imuAddress, 0x24, data))
        return -6;

    /* Slave 0 reads from AKM data registers. */
    data = BIT_I2C_READ | compassAddress;
    if (I2Cwrite(imuAddress, 0x25, data))
        return -7;

    /* Compass reads start at this register. */
    data = AKM_REG_HXL;
    if (I2Cwrite(imuAddress, 0x26, data))
        return -8;

    /* Enable slave 0, 6-byte reads. */
    data = BIT_SLAVE_EN  | BIT_SLAVE_GROUP | BIT_SLAVE_BYTE_SW | 6;
    //data= BIT_SLAVE_EN | 8;
    if (I2Cwrite(imuAddress, 0x27, data))
        return -9;

    /* Slave 1 changes AKM measurement mode. */
    data = compassAddress;
    if (I2Cwrite(imuAddress, 0x28, data))
        return -10;

    /* AKM measurement mode register. */
    data = AKM_REG_CNTL;
    if (I2Cwrite(imuAddress, 0x29, data))
        return -11;

    /* Enable slave 1, 1-byte writes. */
    data = BIT_SLAVE_EN | 1;
    if (I2Cwrite(imuAddress, 0x2A, data))
        return -12;

    /* Set slave 1 data. */
    data = AKM_SINGLE_MEASUREMENT;
    if (I2Cwrite(imuAddress, 0x64, data))
        return -13;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data = 0x03;
    if (I2Cwrite(imuAddress, 0x67, data))
        return -14;

    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    data = BIT_I2C_MST_VDDIO;
    if (I2Cwrite(imuAddress, 0x01, data))
        return -15;
        
    return 0;
}

int intStatus(){
  byte intStatus; 
  Wire.beginTransmission(imuAddress); 
  Wire.write(0x3A);        //sends address to read from
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(imuAddress, 1);    // request 6 bytes from device
  
  byte FIFO_CNT[2];
  int i=0;
 
  if(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    intStatus = Wire.read();  // receive one byte
  }
   
 return intStatus; 
}

