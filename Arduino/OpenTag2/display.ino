
/* DISPLAY FUNCTIONS
 *  
 */

void displayOn(){
  //display.ssd1306_command(SSD1306_DISPLAYON);
  display.init();
  display.setBatteryVisible(true);
}

void displayOff(){
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void cDisplay(){
  display.clearDisplay();
  readVoltage();
  displayBattery();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
}

void displaySettings(){
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, displayLine1);
  if(mode==0) display.print("Standby");
  if(mode==1) display.print("Running");
  display.setCursor(0, displayLine2);
  display.print("Start in: ");
  display.print(startTime - t);
  display.println("s");
  display.setCursor(0, displayLine3);
  display.print("Rec:");
  display.print(recDur);
  display.print("s");
  display.print("  Sleep:");
  display.print(recInt);
  display.println("s");
}

void displayBattery(){
  display.setBattery(voltage);
  display.renderBattery();
}

void displayClock(int loc){
  display.setTextSize(1);
  display.setCursor(0,loc);
  display.print(year);
  display.print('-');
  display.print(month);
  display.print('-');
  display.print(day);
  display.print("  ");
  printZero(hour);
  display.print(hour);
  printDigits(minute);
  printDigits(second);
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  printZero(digits);
  display.print(digits);
}

void printZero(int val){
  if(val<10) display.print("0");
}

void displayGPS(){
      display.setCursor(0, displayLine2);
      display.print(gpsYear); display.print("-");
      display.print(gpsMonth); display.print("-");
      display.print(gpsDay); display.print(" ");
      display.print(gpsHour); display.print(":");
      display.print(gpsMinute); display.print(":");
      display.println(gpsSecond); 
      display.print("Lat:");
      display.print(latitude); 
      display.println(latHem);
      display.print("Lon:");
      display.print(longitude);
      display.print(lonHem);
}

