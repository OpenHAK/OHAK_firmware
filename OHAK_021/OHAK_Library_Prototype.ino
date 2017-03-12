/*
  developing a library to control the MAX30102 Sp02 Sensor
*/


void getBMI_chipID(){ // should return 0xD1
  char chipID = BMI160_readRegister(CHIP_ID);
  Serial.print("BMI chip ID: 0x"); Serial.println(chipID,HEX);
}

// reads one register from the MAX30102
char BMI160_readRegister(char reg){
  char inChar;
  Wire.beginTransmission(BMI_ADD);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI_ADD,1);
  while(Wire.available()){
    inChar = Wire.read();
  }
 return inChar;
}

short BMI160_read16bit(char startReg){
  char inChar[2];
  short shorty;
  int byteCounter = 0;
  Wire.beginTransmission(BMI_ADD);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI_ADD,2);
  while(Wire.available()){
    inChar[byteCounter] = Wire.read();
    byteCounter++;
  }
  shorty = (inChar[0]<<8) | inChar[1];
 return shorty;
}

void BMI160_writeRegister(char reg, char setting){
  Wire.beginTransmission(BMI_ADD);
  Wire.write(reg);
  Wire.write(setting);
  Wire.endTransmission(true);
}

//************************************************

void MAX_init(char sr){
  char setting;
  // reset the MAX30102
  setting = RESET;
  MAX30102_writeRegister(MODE_CONFIG,setting);
  delay(50);
  // set mode configuration put device in shutdown to set registers
  // 0x82 = shutdown, Heart Rate mode
  // 0x83 = shutdown, Sp02 mode
  setting = (SHUTDOWN | mode) & 0xFF;
  MAX30102_writeRegister(MODE_CONFIG,setting);
  // set fifo configuration
  setting = (SMP_AVE_1 | ROLLOVER_EN | 0x0F) & 0xFF;
  MAX30102_writeRegister(FIFO_CONFIG,setting);
  // set Sp02 configuration
  sampleRate = sr;
  setting = (ADC_RGE_4096 | sampleRate | PW_411) & 0xFF;
  MAX30102_writeRegister(SPO2_CONFIG,setting);
  // set LED pulse amplitude (current in mA)
  setLEDamplitude(rAmp,irAmp);
  // enable interrupts
  short interruptSettings = ( (PPG_RDY<<8) | (ALC_OVF<<8) | TEMP_RDY ) & 0xFFFF; // (A_FULL<<8) |
  MAX_setInterrupts(interruptSettings);
}

void enableMAX30102(boolean activate){
  char setting = mode;
  zeroFIFOpointers();
  if(!activate){ setting |= 0x80; }
  MAX30102_writeRegister(MODE_CONFIG,setting);
}

void zeroFIFOpointers(){
  MAX30102_writeRegister(FIFO_WRITE,0x00);
  MAX30102_writeRegister(OVF_COUNTER,0x00);
  MAX30102_writeRegister(FIFO_READ,0x00);
}

// report RevID and PartID for verification
void getMAXdeviceInfo(){
  char revID = MAX30102_readRegister(REV_ID);
  char partID = MAX30102_readRegister(PART_ID);
  Serial.print("MAX rev: 0x"); Serial.println(revID,HEX);
  Serial.print("MAX part ID: 0x"); Serial.println(partID,HEX);
}

// read interrupt flags and do the work to service them
void serviceInterrupts(){
    MAX_interrupt = false;  // reset this software flag
    interruptFlags = MAX_readInterrupts();  // read interrupt registers
    if((interruptFlags & A_FULL<<8) > 0){ // FIFO Almost Full
      Serial.println("A_FULL");
      // go do something...
    }else if((interruptFlags & (PPG_RDY<<8)) > 0){ // PPG data ready
//      Serial.println("PPG_RDY");
//      readPointers();
      readPPG();  // read the light sensor data that is available
      serialPPG(); // send the RED and/or IR data
    }else if((interruptFlags & (ALC_OVF<<8)) > 0){ // Ambient Light Cancellation Overflow
      Serial.println("ALC_OVF");
    }else if((interruptFlags & TEMP_RDY) > 0){  // Temperature Conversion Available
//      Serial.println("TEMP_RDY");
      readTemp();
      printTemp();
    }
}

void readPointers(){
  readPointer = MAX30102_readRegister(FIFO_READ);
  ovfCounter = MAX30102_readRegister(OVF_COUNTER);
  writePointer = MAX30102_readRegister(FIFO_WRITE);
  Serial.print(readPointer,HEX); printTab();
  Serial.print(ovfCounter,HEX); printTab();
  Serial.println(writePointer,HEX);
}

//  read die temperature to compansate for RED LED
char tempInteger;
char tempFraction;
void readTemp(){
  tempInteger = MAX30102_readRegister(TEMP_INT);
  tempFraction = MAX30102_readRegister(TEMP_FRAC);
  Celcius = float(tempInteger);
  Celcius += (float(tempFraction)/16);
  Fahrenheit = Celcius*1.8 + 32;
}

void printTemp(){
  if (!PRINT_ONLY_FOR_PLOTTER) {
    printTab(); // formatting...
    Serial.print(Celcius,3); Serial.print("*C");
  }
}


void readPPG(){
//  sampleTimeTest();
  sampleCounter++;
  if(sampleCounter > 200){ sampleCounter = 0; }
  // use the FIFO read and write pointers?
  readFIFOdata();

}

// send PPG value(s) via Serial port
void serialPPG(){
  if (!PRINT_ONLY_FOR_PLOTTER) {
    Serial.println();  // formatting...
    Serial.print(sampleCounter,DEC); printTab();
    Serial.print(REDvalue); printTab();
    Serial.print(IRvalue);
  } else {
    if(useFilter){
      Serial.print(HPfilterOutputRED[NUM_SAMPLES-1]); printSpace();
      Serial.print(HPfilterOutputIR[NUM_SAMPLES-1]);
    } else {
      Serial.print(REDvalue); printSpace();
      Serial.print(IRvalue);
    }
    Serial.println();
  }
}

void printSpace(){
   Serial.print(" ");
}

// read in the FIFO data three bytes per ADC result
void readFIFOdata(){

  char dataByte[6];
  int byteCounter = 0;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(FIFO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,6);
  while(Wire.available()){
    dataByte[byteCounter] = Wire.read();
    byteCounter++;
  }
  REDvalue = 0; IRvalue = 0;
//  REDvalue = (dataByte[0] << 16) | (dataByte[1] << 8) | dataByte[2];
//  IRvalue = (dataByte[3] << 16) | (dataByte[4] << 8) | dataByte[5];

  REDvalue = (dataByte[0] & 0xFF); REDvalue <<= 8;
  REDvalue |= dataByte[1]; REDvalue <<= 8;
  REDvalue |= dataByte[2];
  IRvalue = (dataByte[3] & 0xFF); IRvalue <<= 8;
  IRvalue |= dataByte[4]; IRvalue <<= 8;
  IRvalue |= dataByte[5];

  REDvalue &= 0x0003FFFF; IRvalue &= 0x0003FFFF;
//  REDvalue >>= 3; IRvalue >>= 3;
//  REDvalue &= 0x0003FFF8; IRvalue &= 0x0003FFF8;
  if(useFilter){
    filterHP(REDvalue, IRvalue);
  }
}

// set the current amplitude for the LEDs
// currently uses the same setting for both RED and IR
// should be able to adjust each dynamically...
void setLEDamplitude(int Ir, int Iir){
  Ir *= 1000; Iir *= 1000;
  Ir /= 196; Iir /= 196;
  char currentIR = Iir & 0xFF;
  char currentR = Ir & 0xFF;
  MAX30102_writeRegister(RED_PA,currentR);
  if(mode == SPO2_MODE){
    MAX30102_writeRegister(IR_PA,currentIR);
  }else{
    MAX30102_writeRegister(IR_PA,0x00);
  }
}

// measures time between samples for verificaion purposes
void sampleTimeTest(){
  thisTestTime = micros();
  Serial.print("S\t"); Serial.println(thisTestTime - thatTestTime);
  thatTestTime = thisTestTime;
}

// set the desired interrupt flags
void MAX_setInterrupts(uint16_t setting){
  char highSetting = (setting >> 8) & 0xFF;
  char lowSetting = setting & 0xFF;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(ENABLE_1);
  Wire.write(highSetting);
  Wire.write(lowSetting);
  Wire.endTransmission(true);
}



// reads the interrupt status registers
// returns a 16 bit value
uint16_t MAX_readInterrupts(){
  short inShort = MAX30102_readShort(STATUS_1);
  return inShort;
}

// writes one register to the MAX30102
void MAX30102_writeRegister(char reg, char setting){
  Wire.beginTransmission(MAX_ADD);
  Wire.write(reg);
  Wire.write(setting);
  Wire.endTransmission(true);
}

// reads one register from the MAX30102
char MAX30102_readRegister(char reg){
  char inChar;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,1);
  while(Wire.available()){
    inChar = Wire.read();
  }
 return inChar;
}

// reads two successive registers from the MAX30102
short MAX30102_readShort(char startReg){
  char inChar[2];
  short shorty;
  int byteCounter = 0;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,2);
  while(Wire.available()){
    inChar[byteCounter] = Wire.read();
    byteCounter++;
  }
  shorty = (inChar[0]<<8) | inChar[1];
 return shorty;
}

// prints out register values
void printAllRegisters(){
  Wire.beginTransmission(MAX_ADD);
  Wire.write(STATUS_1);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,7);
  readWireAndPrintHex(STATUS_1);
  Wire.endTransmission(true);
  Wire.beginTransmission(MAX_ADD);
  Wire.write(FIFO_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,11);
  readWireAndPrintHex(FIFO_CONFIG);
  Wire.endTransmission(true);
  Wire.beginTransmission(MAX_ADD);
  Wire.write(TEMP_INT);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,3);
  readWireAndPrintHex(TEMP_INT);
  Wire.endTransmission(true);

  Wire.beginTransmission(MAX_ADD);
  Wire.write(PROX_INT_THRESH);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,1);
  readWireAndPrintHex(PROX_INT_THRESH);
  Wire.endTransmission(true);

  Wire.beginTransmission(MAX_ADD);
  Wire.write(REV_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,2);
  readWireAndPrintHex(REV_ID);
  Wire.endTransmission(true);
}

// helps to print out register values
void readWireAndPrintHex(char startReg){
  char inChar;
  while(Wire.available()){
    inChar = Wire.read();
    printRegName(startReg); startReg++;
    Serial.print("0x"); Serial.println(inChar,HEX);
  }
}



// helps with verbose feedback
void printRegName(char regToPrint){

  switch(regToPrint){
    case STATUS_1:
      Serial.print("STATUS_1\t"); break;
    case STATUS_2:
      Serial.print("STATUS_2\t"); break;
    case ENABLE_1:
      Serial.print("ENABLE_1\t"); break;
    case ENABLE_2:
      Serial.print("ENABLE_2\t"); break;
    case FIFO_WRITE:
      Serial.print("FIFO_WRITE\t"); break;
    case OVF_COUNTER:
      Serial.print("OVF_COUNTER\t"); break;
    case FIFO_READ:
      Serial.print("FIFO_READ\t"); break;
    case FIFO_DATA:
      Serial.print("FIFO_DATA\t"); break;
    case FIFO_CONFIG:
      Serial.print("FIFO_CONFIG\t"); break;
    case MODE_CONFIG:
      Serial.print("MODE_CONFIG\t"); break;
    case SPO2_CONFIG:
      Serial.print("SPO2_CONFIG\t"); break;
    case RED_PA:
      Serial.print("RED_PA\t"); break;
    case IR_PA:
      Serial.print("IR_PA\t"); break;
    case PILOT_PA:
      Serial.print("PILOT_PA\t"); break;
    case MODE_CNTRL_1:
      Serial.print("MODE_CNTRL_1\t"); break;
    case MODE_CNTRL_2:
      Serial.print("MODE_CNTRL_2\t"); break;
    case TEMP_INT:
      Serial.print("TEMP_INT\t"); break;
    case TEMP_FRAC:
      Serial.print("TEMP_FRAC\t"); break;
    case TEMP_CONFIG:
      Serial.print("TEMP_CONFIG\t"); break;
    case PROX_INT_THRESH:
      Serial.print("PROX_INT_THRESH\t"); break;
    case REV_ID:
      Serial.print("REV_ID\t"); break;
    case PART_ID:
      Serial.print("PART_ID\t"); break;
    default:
      Serial.print("RESERVED\t"); break;
  }
}

// formatting...
void printTab(){
  Serial.print("\t");
}
