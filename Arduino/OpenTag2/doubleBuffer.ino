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
//    cDisplay();
//    displaySettings();
//    displayClock(displayLine4);
//    display.display();
  
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
    if(time2writeIMU!=0) SerialUSB.println("overflow");
    bufferposIMU = 0;
    time2writeIMU= 2;  // set flag to write second half
    firstwrittenIMU = 0; 
  }
  if((bufferposIMU>=halfbufIMU) & !firstwrittenIMU)  //at end of first buffer
  {
    if(time2writeIMU!=0) SerialUSB.println("overflow");
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
//  SerialUSB.print("Write buf:");
//  SerialUSB.println(halfBuf);
  String sensorLine; // text to write to file
  int iPressure, iRGB, iImu, iTime, iStart, iEnd; //index into buffer
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
    sensorLine="";
    for (int j = 0; j<nImuVals; j++){ // write out imuSrate/sensorSrate (100) lines of IMU for each line of other sensor data
      for (int k = 0; k<9; k++){
        sensorLine += imuBuffer[iImu];
        if(k<8) sensorLine += ",";
        iImu++;
      }
      if((j<nImuVals - 1)) {
        dataFile.println(sensorLine);  // write line to file unless last line
        //SerialUSB.println(sensorLine);
        sensorLine = "";
      }
    }
    sensorLine += ","; sensorLine += RGBbuffer[iRGB];
    sensorLine += ","; sensorLine += RGBbuffer[iRGB+1];
    sensorLine += ","; sensorLine += RGBbuffer[iRGB+2];
    sensorLine += ","; sensorLine += PTbuffer[iPressure];
    sensorLine += ","; sensorLine += PTbuffer[iPressure+1];
    sensorLine += ","; sensorLine += timeBuffer[iTime];  // year
    sensorLine += "-"; 
    if(timeBuffer[iTime+1] < 10) sensorLine += "0";
    sensorLine += timeBuffer[iTime+1];
    sensorLine += "-"; 
    if(timeBuffer[iTime+2] < 10) sensorLine += "0";
    sensorLine += timeBuffer[iTime+2];
    sensorLine += "T"; 
    if(timeBuffer[iTime+3] < 10) sensorLine += "0";
    sensorLine += timeBuffer[iTime+3];
    sensorLine += ":"; 
    if(timeBuffer[iTime+4] < 10) sensorLine += "0";
    sensorLine += timeBuffer[iTime+4];
    sensorLine += ":"; 
    if(timeBuffer[iTime+5] < 10) sensorLine += "0";
    sensorLine += timeBuffer[iTime+5];
    
    dataFile.println(sensorLine);
    SerialUSB.println(sensorLine);
    iPressure += 2;
    iRGB += 3;
    iTime += 6;
  }
}

