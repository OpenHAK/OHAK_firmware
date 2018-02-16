

void splashOLED(){
  oled.begin();    // Initialize the OLED
  Serial.println("begun");
  oled.flipHorizontal(true);
  oled.flipVertical(true);
  oled.clear(ALL); // Clear the display's internal memory
  oled.clear(PAGE); // Clear the buffer.
  oled.display();
  oled.setFontType(1);
  oled.setCursor(0, 0);
  oled.print("OpenHAK");
  oled.setCursor(0, 24);
  oled.setFontType(0);
  oled.println("version");
  oled.print(VERSION);
//  oled.flipHorizontal(true);
//  oled.flipVertical(true);
  oled.display();
}



void printOLED(String inString, boolean printTime) {
  
  
  oled.begin();    // Initialize the OLED
  oled.flipHorizontal(true);
  oled.flipVertical(true);
  oled.clear(ALL); // Clear the display's internal memory
  oled.clear(PAGE); // Clear the buffer.
  oled.display();
  oled.setFontType(2);
  oled.setCursor(0, 0);
  if(printTime){
    String timeString = "";
    if (minute(localTime) < 10) {
      timeString += "0";
    }
    timeString += hour(localTime);
    timeString += ":";
    if (minute(localTime) < 10) {
      timeString += "0";
    }
    timeString += minute(localTime);
    oled.print(timeString);
  }
  oled.setCursor(0, 24);
  oled.setFontType(0);
  oled.println(inString);
//  oled.flipHorizontal(true);
//  oled.flipVertical(true);
  oled.display();
}
