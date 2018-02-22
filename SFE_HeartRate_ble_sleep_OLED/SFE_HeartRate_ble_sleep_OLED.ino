

/*
   Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
   By: Nathan Seidle @ SparkFun Electronics
   Date: October 2nd, 2016
   https://github.com/sparkfun/MAX30105_Breakout

   This is a demo to show the reading of heart rate or beats per minute (BPM) using
   a Penpheral Beat Amplitude (PBA) algorithm.

   It is best to attach the sensor to your finger using a rubber band or other tightening
   device. Humans are generally bad at applying constant pressure to a thing. When you
   press your finger against the sensor it varies enough to cause the blood in your
   finger to flow differently which causes the sensor readings to go wonky.

   Hardware Connections (Breakoutboard to Arduino):
   -5V = 5V (3.3V is allowed)
   -GND = GND
   -SDA = A4 (or SDA)
   -SCL = A5 (or SCL)
   -INT = Not connected

   The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
   but it will also run at 3.3V.


   0.38925 = mAh running
   0.475 = mAh sleeping


*/
//#define DEBUG 1
//#define OLED 1
#include "OHAK_Definitions.h"

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#ifdef OLED
#include <OpenHAK_MicroOLED.h>
#endif
#include <BMI160Gen.h>

#include "QuickStats.h"
//#include <TimeLib.h>
#include <Timezone.h>
#include <Lazarus.h>
Lazarus Lazarus;

#include <ota_bootloader.h>
#include <SimbleeBLE.h>


String VERSION = "0.1.0";

// OLED stuff
#ifdef OLED
MicroOLED oled(OLED_RESET, DC);    // reset pin, I2C address
String bpm = "";
#endif


time_t localTime, utc;
int minutesOffset = 0;
signed char timeZoneOffset = 0;

MAX30105 particleSensor;

QuickStats stats; //initialize an instance of stats class


long lastTime;
long awakeTime;
//#ifndef DEBUG
//long interval = 15000; //30000 this is how long we capture hr data
//int sleepTime = 60; //600 is production
//#else
long interval = 30000; //30000 this is how long we capture hr data
int sleepTime = 600; //600 is production
//#endif

float volts = 8;

// heart beat stuff
float EMA_a = 0.4;    //initialization of EMA alpha
long sumEMA_S = 0;        //initialization of sum EMA S
int irEMA_S = 0;        //initialization of IR EMA S
int redEMA_S = 0;        //initialization of RED EMA S
int greenEMA_S = 0;        //initialization of GREEN EMA S
int sumHighpass = 0;
int irHighpass = 0;
int redHighpass = 0;
int greenHighpass = 0;
int sumSample = 0;
int irSample = 0;
int redSample = 0;
int greenSample = 0;
long lastBeatTime = 0; //Time at which the last beat occurred
bool firstBeat = true;
float beatsPerMinute;
int beat;
float delta;
uint8_t lastBeat;
uint8_t arrayBeats[256];
uint8_t beatCounter;
unsigned long startTime;
int T = 0;  // trough
int P = 0;  // peak
int amp;
boolean rising = false;

// tap stuff
bool tapFlag = false;


uint8_t mode = 10;
bool bConnected = false;

// interval between advertisement transmissions ms (range is 20ms to 10.24s) - default 20ms
int bleInterval = 675;  // 675 ms between advertisement transmissions longer time = better battery but slower scan/connect

/* DATA structures
   Sample data every 10 minutes
   Each sample
   Steps since Midnight (uint16_t) 0 - 65535
   Last BPM - rolling average of 30 seconds of caputre (uint8_t) 0 - 255
   Time ()

*/
typedef struct {
  uint32_t epoch;
  uint16_t steps;
  uint8_t hr;
  uint8_t hrDev;
  uint8_t battery;
  uint8_t aux1;
  uint8_t aux2;
  uint8_t aux3;
} Payload;

Payload samples[512];

uint16_t currentSample = 0;

uint8_t advdata[14] =
{
  13, // length // 0
  0x09, // complete local name type // 1
  0x4f, // 'O' // 2
  0x70, // 'p' // 3
  0x65, // 'e' // 4
  0x6e, // 'n' // 5
  0x48, // 'H' // 6
  0x41, // 'A' // 7
  0x4b, // 'K' // 8
  0x2D, // '-' // 9
  0x54, // 'T' // 10
  0x41, // 'A' // 11
  0x43, // 'C' // 12
  0x4f, // 'O' // 13
};

void setup()
{
  String stringy =  String(getDeviceIdLow(), HEX);
  advdata[10] = (uint8_t)stringy.charAt(0);
  advdata[11] = (uint8_t)stringy.charAt(1);
  advdata[12] = (uint8_t)stringy.charAt(2);
  advdata[13] = (uint8_t)stringy.charAt(3);
  SimbleeBLE_advdata = advdata;
  SimbleeBLE_advdata_len = sizeof(advdata);
  SimbleeBLE.advertisementData = "OpenHAK";
  // Device Information Service strings
  SimbleeBLE.manufacturerName = "openhak";
  SimbleeBLE.hardwareRevision = "0.3";
  SimbleeBLE.softwareRevision = "0.0.4";
  Wire.beginOnPins(SCL_PIN, SDA_PIN);
  // change the advertisement interval
  SimbleeBLE.advertisementInterval = bleInterval;
  SimbleeBLE.begin();
  pinMode(RED, OUTPUT);
  digitalWrite(RED, HIGH);
  pinMode(BLU, OUTPUT);
  digitalWrite(BLU, HIGH);
  pinMode(GRN, OUTPUT);
  digitalWrite(GRN, HIGH);

  //Setup all the devices
  BMI160.begin(0, -1); // use BMI_INT1 for internal interrupt, but we're handling the interrupt so using -1
  BMI160.attachInterrupt(NULL); // use bmi160_intr for internal interrupt callback, but we're handling the interrupt so NULL
  //BMI160.setIntTapEnabled(true);
  BMI160.setIntDoubleTapEnabled(true);
  BMI160.setStepDetectionMode(BMI160_STEP_MODE_NORMAL);
  BMI160.setStepCountEnabled(true);
  pinMode(BMI_INT1, INPUT); // set BMI interrupt pin
  Simblee_pinWake(BMI_INT1, LOW); // use this to wake the MCU if its sleeping


#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Initializing...");
#endif

const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
setTime(DEFAULT_TIME);

  // Initialize sensor
  if (!particleSensor.begin(Wire, 400)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  //  *Joel's notes  -> NEEDS MOVE TO DEFINES
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA *not a bad value
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32 *have not messed with this
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green *not a bad value?
  byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200  *higher = noisier but better resolution
  int pulseWidth = 118; //Options: 69, 118, 215, 411  *shorteded from 411
  int adcRange = 2048; //Options: 2048, 4096, 8192, 16384 *smaller range = larger sample number

  firstBeat = true;
  beat = lastBeat = beatCounter = 0;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

#ifdef OLED
  // Splash the OLED
  splashOLED();
#endif
  //Blink the startup pattern
  digitalWrite(RED, LOW);
  delay(400);
  digitalWrite(GRN, LOW);
  digitalWrite(RED, HIGH);
  delay(400);
  digitalWrite(GRN, HIGH);
  digitalWrite(BLU, LOW);
  delay(400);
  digitalWrite(BLU, HIGH);
#ifdef DEBUG
  digitalWrite(BLU, LOW);
  delay(400);
  digitalWrite(BLU, HIGH);
  delay(400);
  digitalWrite(BLU, LOW);
  delay(400);
  digitalWrite(BLU, HIGH);
#endif
  delay(2000);



  lastTime = millis();
}
void bmi160_intr(void)
{
  tapFlag = true;
  //Lazarus.ariseLazarus();
  //byte int_status = BMI160.getIntStatus0();
  // Serial.print("Steps ");
  // Serial.print(BMI160.getStepCount());
  // Serial.print(" Single ");
  // Serial.print(bitRead(int_status,5));
  // Serial.print(" Double ");
  // Serial.print(bitRead(int_status,4));
  // Serial.print(" REG ");
  // Serial.print(int_status,BIN);
  // Serial.println(" BMI160 interrupt: TAP! ");
}

void loop()
{
  if (Lazarus.lazarusArising()) {
    digitalWrite(BLU, LOW);
#ifdef DEBUG
    Serial.println("Lazarus has awakened!");
#endif
    // Serial.println("");
  }
  if (Simblee_pinWoke(BMI_INT1))
  {
    byte int_status = BMI160.getIntStatus0();
#ifdef DEBUG
    Serial.println("TAP has awakened!");
#endif
    Simblee_resetPinWake(BMI_INT1);
    delay(100);
    digitalWrite(RED, HIGH);
  }
  particleSensor.wakeUp();
  particleSensor.setup();
  long lastTime;
  int sleepTimeNow;
  uint32_t startTime;
  //SimbleeBLE.send(0);
  //lastBeatAvg = 0;
  //beatAvg = 0;
  //digitalWrite(RED,LOW);
  // if (timeStatus() != timeNotSet) {
  //         String printString = digitalClockDisplay();
  // }
  switch (mode) {
    case 0:
#ifdef DEBUG
      Serial.println("Enter mode 0");
#endif
      lastTime = millis();
      utc = now();  // This works to keep time incrementing that we send to the phone
      localTime = utc + (minutesOffset/60); // This does not work to keep track of time we pring on screen??
      samples[currentSample].epoch = utc;  // Send utc time to the phone. Phone will manage timezone, etc.
      samples[currentSample].steps = BMI160.getStepCount();
      memset(arrayBeats, 0, sizeof(arrayBeats));
      firstBeat = true;
      beat = lastBeat = beatCounter = 0;
#ifdef DEBUG
      Serial.println("Starting HR capture");
#endif
#ifdef OLED
      printOLED("Measuring Heart Rate",true);
#endif
      delay(1500);
      sumEMA_S = particleSensor.getIR() + particleSensor.getRed(); // + particleSensor.getGreen();     //set filter sumEMA S for t=1
      // irEMA_S = particleSensor.getIR();           //set filter irEMA S for t=1
      // redEMA_S = particleSensor.getRed();         //set filter redEMA S for t=1
      // greenEMA_S = particleSensor.getGreen();     //set filter greenEMA S for t=1
      startTime = millis();
      while (captureHR(startTime)) {
        ;
      }

//      BUILD THE REST OF THE BLE PACKET
      // watch out for this, make sure the arrayBeats is large enough for the sample size
      samples[currentSample].hr = stats.median(arrayBeats, beatCounter);
      samples[currentSample].hrDev = stats.stdev(arrayBeats, beatCounter);
      samples[currentSample].battery = getBatteryVoltage();
      //                samples[currentSample].aux1 = analogRead(PIN_2);
      //                samples[currentSample].aux2 = analogRead(PIN_3);
      //                samples[currentSample].aux3 = analogRead(PIN_4);
#ifdef OLED
      bpm = ""; // clear the bpm string
      bpm += String(samples[currentSample].hr);
      bpm += " BPM";
      printOLED(bpm,true);
      delay(5000);
      digitalWrite(OLED_RESET,LOW);
#endif
      if (bConnected) {
        sendSamples(samples[currentSample]);
      }
      if (currentSample < 511) {
        currentSample++;
      } else {
        //TODO: Check and see if this works!
        for (int k = currentSample; k > 0; k--) {
          samples[k] = samples[k - 1];
        }
        currentSample--;
        // for(uint16_t t=0;t<512;t++){
        //   memcpy(&samples[1], &samples[0], (512-1)*sizeof(*samples));
        // }
      }
#ifdef DEBUG
      Serial.print("Samples Captured: ");
      Serial.println(currentSample);
#endif

      awakeTime = millis() - lastTime;
      sleepTimeNow = sleepTime - (interval / 1000);
      sleepNow(sleepTimeNow);
      break;
    case 1:
#ifdef DEBUG
      Serial.println("Enter mode 1");
#endif
      startTime = millis();
      captureHR(startTime);
      break;
    case 2:
#ifdef DEBUG
      Serial.println("Enter mode 2");
#endif
      mode = 0;
      sleepTimeNow = sleepTime - (interval / 1000);
      sleepNow(sleepTimeNow);
      break;
    case 3:
#ifdef DEBUG
      Serial.println("Enter mode 3");
#endif
      transferSamples();
      break;
    case 10:
#ifdef DEBUG
      Serial.println("Enter mode 10");
#endif
      //mode = 0;
      digitalWrite(RED, LOW);
#ifdef OLED
      printOLED("Sync me :)",false);
      delay(500);
//      digitalWrite(OLED_RESET,LOW);
#endif
      sleepNow(10);
      break;
  }
  //Serial.print("IR=");
  //Serial.print(irValue);
  //Serial.print(", BPM=");
  //Serial.print(beatsPerMinute);
  //Serial.print(", Avg BPM=");
  //Serial.print(beatAvg);

  //if (irValue < 50000)
  // Serial.print(" No finger?");

  //Serial.println();
}



void sleepNow(long timeNow) {
  digitalWrite(RED, HIGH);
  digitalWrite(GRN, HIGH);
  digitalWrite(BLU, HIGH);
#ifdef OLED
  digitalWrite(OLED_RESET,LOW);
#endif
  particleSensor.setPulseAmplitudeIR(0); //Turn off IR LED
  particleSensor.setPulseAmplitudeRed(0); //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  particleSensor.shutDown();
  //int sleepTimeNow = timeNow - (interval/1000);
  Simblee_ULPDelay(SECONDS(timeNow));
}



String digitalClockDisplay() {
  // digital clock display of the time
  String dataString = "";
  dataString += year(localTime);
  dataString += "-";
  dataString += month(localTime);
  dataString += "-";
  dataString += day(localTime);
  dataString += " ";
  dataString += hour(localTime);
  dataString += ":";
  //Serial.print(hour());
  if (minute(localTime) < 10) {
    dataString += "0";
  }
  dataString += minute(localTime);
  dataString += ":";
  //Serial.print(hour());
  if (second(localTime) < 10) {
    dataString += "0";
  }
  dataString += second(localTime);
  return dataString;
}
uint8_t getBatteryVoltage() {
  int counts = analogRead(V_SENSE);
  // Serial.print(counts); Serial.print("\t");
  volts = float(counts) * (3.3 / 1023.0);
  // Serial.print(volts,3); Serial.print("\t");
  volts *= 2;
  // Serial.println(volts,3);

  return volts / BATT_VOLT_CONST;
}
