
/* DISPLAY FUNCTIONS
 *  
 */

void displayOn(){
  SerialUSB.println("Display on");
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
  display.print("Rec:");
  display.print(recDur);
  display.println("s");
  display.print("Sleep:");
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
