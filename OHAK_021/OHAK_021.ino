/*
  Pulse Patch
  This code targets a Simblee
  I2C Interface with MAX30102 Sp02 Sensor Module
  Also got a BOSH MEMS thing
  And it might actually be a MAX30101...?
*/

#include <Wire.h>
#include "OHAK_Definitions.h"
#include <BMI160Gen.h>
#include <Wire.h>

#define RED_ON 1
#define GREEN_ON 2
#define BLUE_ON 3
#define ALL_ON 4
#define DOWN 0
#define UP 1

//DEFINE IF YOU ARE USING BLE OR SERIAL
//Comment out if you want serial
#define BLE_ON

//This line added by Chip 2016-09-28 to enable plotting by Arduino Serial Plotter
const int PRINT_ONLY_FOR_PLOTTER = 0;  //Set this to zero to return normal verbose print() statements

unsigned int LED_blinkTimer;
unsigned int LED_fadeTimer;
unsigned int rainbowTimer;
int LED_blinkTime = 300;
int LED_fadeTime = 10;
int rainbowTime = 50;
int LEDpin[] = {RED,GRN,BLU};
int LEDvalue[3];
boolean rising[] = {true,true,true};
int LEDcounter = 0;
int lightInPlay;
int colorWheelDegree = 0;
boolean rainbow = false;

volatile boolean MAX_interrupt = false;
short interruptSetting;
short interruptFlags;
float Celcius;
float Fahrenheit;
char sampleCounter = 0;
int REDvalue;
int IRvalue;
char mode = SPO2_MODE;  // SPO2_MODE or HR_MODE
char readPointer;
char writePointer;
char ovfCounter;
int rAmp = 10;
int irAmp = 10;


//  TESTING
unsigned int thisTestTime;
unsigned int thatTestTime;

char sampleRate;
boolean useFilter = false;
int gain = 10;
float HPfilterInputRED[NUM_SAMPLES];
float HPfilterOutputRED[NUM_SAMPLES];
float LPfilterInputRED[NUM_SAMPLES];
float LPfilterOutputRED[NUM_SAMPLES];
float HPfilterInputIR[NUM_SAMPLES];
float HPfilterOutputIR[NUM_SAMPLES];
float LPfilterInputIR[NUM_SAMPLES];
float LPfilterOutputIR[NUM_SAMPLES];


void setup(){

  Wire.beginOnPins(SCL_PIN,SDA_PIN);
  Serial.begin(230400);
  for(int i=0; i<3; i++){
    pinMode(LEDpin[i],OUTPUT); digitalWrite(LEDpin[i],HIGH); // Enable RGB and turn them off
  }
  
  LED_blinkTimer = LED_fadeTimer = millis();
  
  pinMode(MAX_INT,INPUT);
  attachPinInterrupt(MAX_INT,MAX_ISR,LOW);
  
  if (!PRINT_ONLY_FOR_PLOTTER) Serial.println("\nPulsePatch 01\n");
  
  MAX_init(SR_100); // initialize MAX30102, specify sampleRate
  if (useFilter){ initFilter(); }
  if (!PRINT_ONLY_FOR_PLOTTER) {
    printAllRegisters();
    Serial.println("");
    printHelpToSerial();
    Serial.println("");
  } else {
    //when configured for the Arduino Serial Plotter, start the system running right away
    enableMAX30102(true);
    thatTestTime = micros();
  }
  Serial.println("OpenHAK v0.2.1");
    // initialize device
  BMI160.begin(10);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("BMI DEVICE ID: ");
  Serial.println(dev_id, HEX);
  //getBMI_chipID();    // should print 0xD1
  getMAXdeviceInfo(); // prints rev and device ID

  pulse(BLU);
  
}


void loop(){

  if(MAX_interrupt){
    serviceInterrupts(); // go see what woke us up, and do the work
    if(sampleCounter == 0x00){  // rolls over to 0 at 200
      MAX30102_writeRegister(TEMP_CONFIG,0x01); // take temperature
    }
  }


  if(rainbow){
    rainbowLEDs();
  }else{
    blinkLEDs();
  }

  eventSerial();
}

void eventSerial(){
  while(Serial.available()){
    char inByte = Serial.read();
    uint16_t intSource;
    switch(inByte){
      case 'h':
        printHelpToSerial();
        break;
      case 'b':
        Serial.println("start running");
        enableMAX30102(true);
        thatTestTime = micros();
        break;
      case 's':
        Serial.println("stop running");
        enableMAX30102(false);
        break;
      case 't':
        MAX30102_writeRegister(TEMP_CONFIG,0x01);
        break;
      case 'i':
        intSource = MAX30102_readShort(STATUS_1);
        Serial.print("intSource: 0x"); Serial.println(intSource,HEX);
        break;
      case 'v':
        getBMI_chipID();
        getMAXdeviceInfo();
        break;
      case '?':
        printAllRegisters();
        break;
      case 'f':
        useFilter = false;
        break;
      case 'F':
        useFilter = true;
        break;
      case '1':
        rAmp++; if(rAmp > 50){rAmp = 50;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case '2':
        rAmp--; if(rAmp < 1){rAmp = 0;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case '3':
        irAmp++; if(irAmp > 50){irAmp = 50;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case '4':
        irAmp--; if(irAmp < 1){irAmp = 0;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case 'r':
        LEDvalue[0] = 0; LEDvalue[1] = 255; LEDvalue[2] = 255;
        for(int i=0; i<3; i++){
          analogWrite(LEDpin[i],LEDvalue[i]);        
        }
        colorWheelDegree = 0;
        rainbow = true;
        break;
      case 'R':
        rainbow = false;
        break;
      case 'p':
        getBatteryVoltage();
        break;
      default:
        Serial.print("Serial Event got: "); Serial.write(inByte); Serial.println();
        break;
    }
  }
}


void blinkLEDs(){
  if(millis() - LED_blinkTimer > LED_blinkTime){
    LED_blinkTimer = millis();
    LEDcounter++;
    if(LEDcounter>2){ LEDcounter = 0; }
    for(int i=0; i<3; i++){
      if(i == LEDcounter){
        analogWrite(LEDpin[i],200);
      }else{
        analogWrite(LEDpin[i],255);
      }
    }
  }
}





void rainbowLEDs(){
  if(millis()-rainbowTimer > rainbowTime){
    rainbowTimer = millis();
    colorWheelDegree++;
    
    if(colorWheelDegree > 360){ colorWheelDegree = 1; }
    
    if(colorWheelDegree > 0 && colorWheelDegree < 61){
      LEDvalue[1] = map(colorWheelDegree,1,60,254,0);
      writeLEDvalues(); // g fades up while r is up b is down
    } else if(colorWheelDegree > 60 && colorWheelDegree < 121){
      LEDvalue[0] = map(colorWheelDegree,61,120,0,254);
      writeLEDvalues(); // r fades down while g is up b is down
    } else if(colorWheelDegree > 120 && colorWheelDegree < 181){
      LEDvalue[2] = map(colorWheelDegree,121,180,254,0);
      writeLEDvalues(); // b fades up while g is up r is down
    } else if(colorWheelDegree > 180 && colorWheelDegree < 241){
      LEDvalue[1] = map(colorWheelDegree,181,240,0,254);
      writeLEDvalues(); // g fades down while b is up r is down
    } else if(colorWheelDegree > 240 && colorWheelDegree < 301){
      LEDvalue[0] = map(colorWheelDegree,241,300,254,0);
      writeLEDvalues(); // r fades up while b is up g is down
    } else if(colorWheelDegree > 300 && colorWheelDegree < 361){
      LEDvalue[2] = map(colorWheelDegree,301,360,0,254);
      writeLEDvalues(); // b fades down while r is up g is down
    }
   
  } 
}

void pulse(int led){  // needs help
  for(int i=0; i<2; i++){
    for(int j=254; j>0; j-=10){
      analogWrite(led,LEDvalue[j]);
      delay(5);
    }
    for(int k=1; k<255; k+=10){
      analogWrite(led,LEDvalue[k]);
      delay(5);
    }
    analogWrite(led,255);
  }
}


void writeLEDvalues(){  
  for(int i=0; i<3; i++){
    analogWrite(LEDpin[i],LEDvalue[i]);
  }
}



//Print out all of the commands so that the user can see what to do
//Added: Chip 2016-09-28
void printHelpToSerial() {
  Serial.println(F("Commands:"));
  Serial.println(F("   'h'  Print this help information on available commands"));
  Serial.println(F("   'b'  Start the thing running at the sample rate selected"));
  Serial.println(F("   's'  Stop the thing running"));
  Serial.println(F("   't'  Initiate a temperature conversion. This should work if 'b' is pressed or not"));
  Serial.println(F("   'i'  Query the interrupt flags register. Not really useful"));
  Serial.println(F("   'v'  Verify the device by querying the RevID and PartID registers (hex 6 and hex 15 respectively)"));
  Serial.println(F("   '1'  Increase red LED intensity"));
  Serial.println(F("   '2'  Decrease red LED intensity"));
  Serial.println(F("   '3'  Increase IR LED intensity"));
  Serial.println(F("   '4'  Decrease IR LED intensity"));
  Serial.println(F("   '?'  Print all registers"));
  Serial.println(F("   'F'  Turn on filters"));
  Serial.println(F("   'p'  Query battery Voltage"));
  
}

int MAX_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
  MAX_interrupt = true;
  return 0; // gotta return something, somehow...
}

void serialAmps(){
  Serial.print("PA\t");
  Serial.print(rAmp); printTab(); Serial.println(irAmp);
}

float getBatteryVoltage(){
  int counts = analogRead(V_SENSE);
  Serial.print(counts); Serial.print("\t");
  float volts = float(counts) * (3.3/1023.0);
  Serial.print(volts,3); Serial.print("\t");
  volts *=2;
  Serial.println(volts,3);

  return volts;
}

