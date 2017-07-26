#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define PMTK_LOCUS_DUMP "$PMTK622,1*29"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_BAUD_19200 "$PMTK251,19200*22"
#define PMTK_BAUD_38400 "$PMTK251,38400*27"
#define PMTK_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_BAUD_115200 "$PMTK251,115200*1F"
#define PMTK_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_UPDATE_2HZ "$PMTK220,500*2B"
#define PMTK_UPDATE_5HZ "$PMTK220,200*2C"
#define PMTK_UPDATE_10HZ "$PMTK220,100*2F"

// send any byte to wake from Standby
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_NORMAL "$PMTK225,0*2B"
#define PMTK_ALWAYSLOCATE_STANDBY "$PMTK225,8*23"
#define PMTK_ALWAYSLOCATE_BACKUP "$PMTK225,9*22"

#define maxChar 256
char gpsStream[maxChar];
int streamPos;
volatile boolean endGpsLog;

int gps(byte incomingByte){
  char temp2[2];
  char temp3[3];
  char temp5[5];
  char temp7[7];
  char temp12[12];

// portions modified from Adafruit library
// https://github.com/adafruit/Adafruit_GPS/blob/master/Adafruit_GPS.cpp

  gpsStream[streamPos] = incomingByte;
  streamPos++;
  if(streamPos >= maxChar) streamPos = 0;
    
  // check for end of a message
  if((incomingByte=='\n') & (streamPos>4)) {
    //process last message
    if (gpsStream[streamPos-5] == '*') {    
      if(printDiags){
        SerialUSB.print("Check sum: ");
        SerialUSB.println(gpsStream);
      }
      
      uint16_t sum = parseHex(gpsStream[streamPos-4]) * 16;
      sum += parseHex(gpsStream[streamPos-3]);
  
      // check checksum 
      for (uint8_t i=1; i < (streamPos-5); i++) {
        sum ^= gpsStream[i];
      }
      if (sum != 0) {
        // bad checksum :(
        streamPos = 0;
        if (printDiags) SerialUSB.println("bad checksum");
        return false;
      }
      else{
        if (printDiags) SerialUSB.println("good checksum");
      }
    
      // OriginGPS
      // $GNRMC,134211.000,A,2715.5428,N,08228.7924,W,1.91,167.64,020816,,,A*62
      // Adafruit GPS
      // $GPRMC,222250.000,A,2716.6201,N,08227.4996,W,1.01,301.49,250117,,,A*7C
      char rmcCode[6 + 1];
      float rmcTime; //           225446       Time of fix 22:54:46 UTC
      char rmcValid[2]; //           A            Navigation receiver warning A = OK, V = warning
      float rmcLat; //           4916.45,N    Latitude 49 deg. 16.45 min North
      char rmcLatHem[2];
      float rmcLon; //           12311.12,W   Longitude 123 deg. 11.12 min West
      char rmcLonHem[2];
      float rmcSpeed; //           000.5        Speed over ground, Knots
      float rmcCourse;//           054.7        Course Made Good, True
      char rmcDate[6 + 1];//           191194       Date of fix  19 November 1994
      float rmcMag;//           020.3,E      Magnetic variation 20.3 deg East
      char rmcMagHem[2];
      char rmcChecksum[4 + 1]; //           *68          mandatory checksum

      // check for end of log dump  $PMTKLOX,2*47
      if(gpsStream[1]=='P' & gpsStream[2]=='M' &  gpsStream[3]=='T' &  gpsStream[4]=='K' &  gpsStream[5]=='L'  
      & gpsStream[6]=='O' &  gpsStream[7]=='X' &  gpsStream[8]==',' &  gpsStream[9]=='2' & gpsStream[10]=='*' 
      & gpsStream[11]=='4' & gpsStream[12]=='7'){
        endGpsLog = 1;
      }

      if(gpsStream[1]=='G' & gpsStream[2]=='P' &  gpsStream[3]=='R' &  gpsStream[4]=='M' &  gpsStream[5]=='C'){
       char temp[streamPos + 1];
       const char s[2] = ",";
       char *token;
       gpsTimeout += 1;  
       memcpy(&temp, &gpsStream, streamPos);
       token = strtok(temp, s);
       sprintf(rmcCode, "%s", token);
       
       token = strtok(NULL, s);
       //sscanf(token, "%f", &rmcTime);
      // SerialUSB.print("\nrmcTime: ");
       //SerialUSB.println(rmcTime);
       sscanf(token, "%2d%2d%2d", &gpsHour, &gpsMinute, &gpsSecond);
       
       token = strtok(NULL, s);
       if (printDiags) SerialUSB.println(token);
       sprintf(rmcValid, "%s", token);
       
       token = strtok(NULL, s);
       if (printDiags) SerialUSB.println(token);
       rmcLat = atof(token);
       
       token = strtok(NULL, s);
       if (printDiags) SerialUSB.println(token);
       sprintf(rmcLatHem, "%s", token);
       
       token = strtok(NULL, s);
       if (printDiags) SerialUSB.println(token);
       rmcLon = atof(token);
       
       token = strtok(NULL, s);
       if (printDiags) SerialUSB.println(token);
       sprintf(rmcLonHem, "%s", token);
       
       token = strtok(NULL, s);
       rmcSpeed = atof(token);
       
       token = strtok(NULL, s);
       rmcCourse = atof(token);
       
       token = strtok(NULL, s);
       sprintf(rmcDate, "%s", token);
       sscanf(token, "%2d%2d%2d", &gpsDay, &gpsMonth, &gpsYear);
       
       token = strtok(NULL, s);
       rmcMag = atof(token);
       sscanf(token, "%d", &rmcMag);
       
       token = strtok(NULL, s);
       sprintf(rmcMagHem, "%s", token);
       
       token = strtok(NULL, s);
       sprintf(rmcChecksum, "%s", token);
       
       if(rmcValid[0]=='A'){
           latitude = rmcLat;
           longitude = rmcLon;
           latHem = rmcLatHem[0];
           lonHem = rmcLonHem[0];
           if (printDiags){
             SerialUSB.print("\nLat:"); SerialUSB.println(latitude, 4);
             SerialUSB.print("Lon:"); SerialUSB.println(longitude, 4);
             SerialUSB.print("stream:"); SerialUSB.println(gpsStream);
           }
           goodGPS = 1;
        }
      }
    }
    streamPos = 0;
  }



  return 1;
}


// read a Hex value and return the decimal equivalent

uint8_t parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

void gpsStartLogger(){
  
  HWSERIAL.println(PMTK_LOCUS_STARTLOG);
  waitForGPS();
}


void gpsStopLogger(){
  HWSERIAL.println(PMTK_LOCUS_STOPLOG);
  waitForGPS();
}

void gpsEraseLogger(){
  HWSERIAL.println(PMTK_LOCUS_ERASE_FLASH);
  waitForGPS();
}

void gpsStatusLogger(){
  HWSERIAL.println(PMTK_LOCUS_QUERY_STATUS);
  waitForGPS();
}


void gpsHibernate(){
  HWSERIAL.println("$PMTK225,4*2F");
  HWSERIAL.flush();
}

void gpsSpewOff(){
  HWSERIAL.println(PMTK_SET_NMEA_OUTPUT_OFF);
}

void gpsSpewOn(){
  HWSERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
}

void gpsBaud(int baudRate){
  if (baudRate==9600) HWSERIAL.println(PMTK_BAUD_9600);
  if (baudRate==19200) HWSERIAL.println(PMTK_BAUD_19200);
  if (baudRate==38400) HWSERIAL.println(PMTK_BAUD_38400);
  if (baudRate==57600) HWSERIAL.println(PMTK_BAUD_57600);
  if (baudRate==115200) HWSERIAL.println(PMTK_BAUD_115200);
}

void waitForGPS(){
  for(int n=0; n<100; n++){
    delay(20);
    while (HWSERIAL.available() > 0) {    
        byte incomingByte = HWSERIAL.read();
        SerialUSB.write(incomingByte);
    }
  }
}

int gpsDumpLogger(){
  // open file for storing data; append
  endGpsLog = 0;
   if(File logFile = sd.open("GPSLOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      for(int n=0; n<8; n++){
        logFile.print(myID[n]);
      }
   HWSERIAL.println(PMTK_LOCUS_DUMP);
   int dumping = 1;
   for(int n=0; n<1000; n++){
       delay(5);
       digitalWrite(LED1, LOW);
       while (HWSERIAL.available() > 0) {    
        n=0;
        digitalWrite(LED1, HIGH);
        byte incomingByte = HWSERIAL.read();
        //gps(incomingByte);
        SerialUSB.write(incomingByte);
        logFile.write(incomingByte);
    }
   }
      logFile.close();
   }
   else{
    if(printDiags) SerialUSB.print("Log open fail.");
    return 0;
   }
   return 1;
}

void gpsUpdateRate(int frequency){
 if(frequency==1) HWSERIAL.println(PMTK_UPDATE_1HZ);
 if(frequency==2) HWSERIAL.println(PMTK_UPDATE_2HZ);
 if(frequency==5) HWSERIAL.println(PMTK_UPDATE_5HZ);
 if(frequency==10) HWSERIAL.println(PMTK_UPDATE_10HZ);
}

void gpsStandby(){
  HWSERIAL.println(PMTK_STANDBY);
  HWSERIAL.flush();
  gpsStatus = 0;
}

void gpsWake(){
  HWSERIAL.println();
  HWSERIAL.flush();
  gpsStatus = 1;
}

void gpsAlwaysLocateStandby(){
  HWSERIAL.println(PMTK_NORMAL);
  waitForGPS();
  HWSERIAL.println(PMTK_ALWAYSLOCATE_STANDBY);
  waitForGPS();
}

void gpsAlwaysLocateBackup(){
  HWSERIAL.println(PMTK_NORMAL);
  waitForGPS();
  HWSERIAL.println(PMTK_ALWAYSLOCATE_BACKUP);
  waitForGPS();
}


