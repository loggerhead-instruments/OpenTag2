// Double buffers for storing data sampled under interrupt
// When buffer is half full, data are written to dataFile

// To Do
// - IMU
// - timestamp with sensor data

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
    bufferposIMU = 0;
    time2writeIMU= 2;  // set flag to write second half
    firstwrittenIMU = 0; 
  }
  if((bufferposIMU>=halfbufIMU) & !firstwrittenIMU)  //at end of first buffer
  {
    time2writeIMU = 1; 
    firstwrittenIMU = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void writeData(){
  // write IMU values to file
  if(time2writeIMU==1)
  {
//    if(dataFile.write((uint8_t *) & sidRec[3],sizeof(SID_REC))==-1) resetFunc();
//    if(dataFile.write((uint8_t *) & imuBuffer[0], halfbufIMU)==-1) resetFunc(); 
    time2writeIMU = 0;
  }
  if(time2writeIMU==2)
  {
//    if(dataFile.write((uint8_t *) & sidRec[3],sizeof(SID_REC))==-1) resetFunc();
//    if(dataFile.write((uint8_t *) & imuBuffer[halfbufIMU], halfbufIMU)==-1) resetFunc();     
    time2writeIMU = 0;
  } 
  
  // write Pressure & Temperature to file
  if(time2writePT==1){
//    if(dataFile.write((uint8_t *)&sidRec[1],sizeof(SID_REC))==-1) resetFunc();
//    if(dataFile.write((uint8_t *)&PTbuffer[0], halfbufPT * 4)==-1) resetFunc(); 
    writeSensors(0);
    time2writePT = 0;
  }
  if(time2writePT==2){
//    if(dataFile.write((uint8_t *)&sidRec[1],sizeof(SID_REC))==-1) resetFunc();
//    if(dataFile.write((uint8_t *)&PTbuffer[halfbufPT], halfbufPT * 4)==-1) resetFunc();   
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
  String sensorLine;
  int iPressure, iRGB, iImu;
  if(halfBuf==1) {
    iPressure = halfbufPT;
    iRGB = halfbufRGB;
    iImu = halfbufIMU;
  }
  else{
    iPressure = 0;
    iRGB = 0;
    iImu = 0;
  }

  int nImuVals = imuSrate/sensorSrate;
  for (int i=iRGB; i<halfbufRGB; i+=3){
    // write out imuSrate/sensorSrate (100) lines of IMU for each line of other sensor data
    sensorLine="";
    for (int j = 0; j<nImuVals; j++){
      for (int k = 0; k<9; k++){
        sensorLine += imuBuffer[iImu];
        if(k<8) sensorLine += ",";
        iImu++;
      }
      if((j<nImuVals - 1)) {
        dataFile.println(sensorLine);  // write line to file unless last line
        SerialUSB.println(sensorLine);
        sensorLine = "";
      }
    }
    sensorLine += ","; sensorLine += RGBbuffer[i];
    sensorLine += ","; sensorLine += RGBbuffer[i+1];
    sensorLine += ","; sensorLine += RGBbuffer[i+2];
    sensorLine += ","; sensorLine += PTbuffer[iPressure];
    sensorLine += ","; sensorLine += PTbuffer[iPressure+1];
    dataFile.println(sensorLine);
    SerialUSB.println(sensorLine);
    iPressure+=2;
  }
}
