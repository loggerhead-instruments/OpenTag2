// Double buffers for storing data sampled under interrupt
// When buffer is half full, data are written to dataFile

// increment PTbuffer position by 1 sample. This does not check for overflow, because collected at a slow rate
void incrementPTbufpos(float val){
  PTbuffer[bufferposPT] = val;
  bufferposPT++;
  if(bufferposPT==PTBUFFERSIZE)
  {
     bufferposPT=0;
     time2writePT=2;  // set flag to write second half
     firstwrittenPT=0; 
  }
  if((bufferposPT>=halfbufPT) & !firstwrittenPT)  //at end of first buffer
  {
    time2writePT=1; 
    firstwrittenPT=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementRGBbufpos(int16_t val){
  RGBbuffer[bufferposRGB] = val;
  bufferposRGB++;
  if(bufferposRGB==RGBBUFFERSIZE)
  {
   bufferposRGB = 0;
   firstwrittenRGB = 0; 
  }
  if((bufferposRGB>=halfbufRGB) & !firstwrittenRGB)  //at end of first buffer
  {
    firstwrittenRGB = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}


void incrementTimebufpos(){  
  timeBuffer[bufferposTime] = year;
  bufferposTime++;
  timeBuffer[bufferposTime] = month;
  bufferposTime++;
  timeBuffer[bufferposTime] = day;
  bufferposTime++;
  timeBuffer[bufferposTime] = hour;
  bufferposTime++;
  timeBuffer[bufferposTime] = minute;
  bufferposTime++;
  timeBuffer[bufferposTime] = second;
  bufferposTime++;
  
  if(bufferposTime==TIMEBUFFERSIZE)
  {
   bufferposTime = 0;
   firstwrittenTime = 0; 
  }
  if((bufferposTime>=halfbufTime) & !firstwrittenTime)  //at end of first buffer
  {
    firstwrittenTime = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementIMU(){
  imuBuffer[bufferposIMU] = accel_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = accel_y;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = accel_z;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = gyro_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = gyro_y;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = gyro_z; 
  bufferposIMU++;
  imuBuffer[bufferposIMU] = mag_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = mag_y; 
  bufferposIMU++;
  imuBuffer[bufferposIMU] = mag_z; 
  bufferposIMU++;

  if(bufferposIMU==IMUBUFFERSIZE)
  {
    if(time2writeIMU==2) {
      SerialUSB.println("overflow");
      digitalWrite(LED_RED, HIGH);
    }
    bufferposIMU = 0;
    time2writeIMU = 2;  // set flag to write second half
    firstwrittenIMU = 0; 
  }
  if((bufferposIMU>=halfbufIMU) & !firstwrittenIMU)  //at end of first buffer
  {
    if(time2writeIMU==1) {
      SerialUSB.println("overflow");
      digitalWrite(LED_RED, HIGH);
    }
    time2writeIMU = 1; 
    firstwrittenIMU = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void writeData(){
  // based on pressure/temperature buffer, because that is last one written
  // all buffers align
  if(time2writePT==1){  //first half
    writeSensors(0);
    time2writePT = 0;
  }
  if(time2writePT==2){  // second half
    writeSensors(1);  
    time2writePT = 0;
  }   
}
void writeDateTime(){
  char sensorLine[255];
  getTime();
  sprintf(sensorLine,"%04d-%02d-%02dT%02d:%02d:%02d", year, month, day, hour, minute, second);
  dataFile.print(sensorLine);
}

void writeImu(int lineFeed){
  char sensorLine[255];
  sprintf(sensorLine,",%d,%d,%d,%d,%d,%d,%d,%d,%d", accel_x, accel_y, accel_z,
    gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);
  if(lineFeed==1) dataFile.println(sensorLine); 
  else
   dataFile.println(sensorLine); 
}

void writeSensors(int halfBuf){
  int iPressure, iRGB, iImu, iTime, iStart, iEnd; //index into buffer
  digitalWrite(LED_RED, LOW); // clear overflow flag
  time2writeIMU = 0;
  if(halfBuf==1) {
    iStart = halfbufTime;
    iEnd = TIMEBUFFERSIZE;
    iRGB = halfbufRGB;
    iPressure = halfbufPT;
    iTime = halfbufTime;
    iImu = halfbufIMU;
  }
  else{
    iStart = 0;
    iEnd = halfbufTime;
    iRGB = 0;
    iPressure = 0;
    iTime = 0;
    iImu = 0;
  }

  int nImuVals = imuSrate/sensorSrate;
  for (int i=iStart; i<iEnd; i+=6){
    for (int j = 0; j<nImuVals; j++){ // write out imuSrate/sensorSrate (100) lines of IMU for each line of other sensor data
      for (int k = 0; k<9; k++){
        dataFile.print(imuBuffer[iImu]);
        if(k<8) dataFile.print(',');
        iImu++;
      }
      if((j<nImuVals - 1)) {
        dataFile.println();  // write line to file unless last line
      }
    }
    
    dataFile.print(','); dataFile.print(timeBuffer[iTime]);  
    dataFile.print('-');
    if(timeBuffer[iTime+1] < 10) dataFile.print('0');
    dataFile.print(timeBuffer[iTime+1]);
    dataFile.print('-');
    if(timeBuffer[iTime+2] < 10) dataFile.print('0');
    dataFile.print(timeBuffer[iTime+2]);
    dataFile.print('T');
    if(timeBuffer[iTime+3] < 10) dataFile.print('0');
    dataFile.print(timeBuffer[iTime+3]);
    dataFile.print(':');
   if(timeBuffer[iTime+4] < 10) dataFile.print('0');
   dataFile.print(timeBuffer[iTime+4]);
   dataFile.print(':');
   if(timeBuffer[iTime+5] < 10) dataFile.print('0');
   dataFile.print(timeBuffer[iTime+5]);
  dataFile.print("Z,");

    dataFile.print(RGBbuffer[iRGB]);
    dataFile.print(','); dataFile.print(RGBbuffer[iRGB+1]);
    dataFile.print(','); dataFile.print(RGBbuffer[iRGB+2]);
    dataFile.print(','); dataFile.print(PTbuffer[iPressure]);
    dataFile.print(','); dataFile.print(PTbuffer[iPressure+1]);

    euler();

    dataFile.print(','); dataFile.print(pitch);
    dataFile.print(','); dataFile.print(roll);
    dataFile.print(','); dataFile.print(yaw);

    dataFile.print(','); dataFile.print(latitude,4);
    dataFile.print(','); dataFile.print(latHem);
    dataFile.print(','); dataFile.print(longitude,4);
    dataFile.print(','); dataFile.print(lonHem);

    readVoltage();
    dataFile.print(','); 
    dataFile.println(voltage,4);
    
    iPressure += 2;
    iRGB += 3;
    iTime += 6;
  }
}

