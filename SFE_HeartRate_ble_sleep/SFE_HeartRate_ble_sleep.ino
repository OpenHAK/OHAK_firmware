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
*/

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"
#include <BMI160Gen.h>
#include <ota_bootloader.h>

#include <SimbleeBLE.h>
#include "OHAK_Definitions.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
long lastTime;
long interval = 30000; //30000
int sleepTime = 600;

float beatsPerMinute;
int beatAvg;
int lastBeatAvg;

void setup()
{
  Wire.beginOnPins(SCL_PIN,SDA_PIN);
  SimbleeBLE.begin();
  pinMode(29,OUTPUT);
  digitalWrite(29,HIGH);
  BMI160.begin(0, BMI_INT1);
  BMI160.attachInterrupt(bmi160_intr);
  BMI160.setIntTapEnabled(true);
  BMI160.setIntDoubleTapEnabled(true);
  BMI160.setStepDetectionMode(BMI160_STEP_MODE_NORMAL);
  BMI160.setStepCountEnabled(true);
  //Serial.begin(115200);
  //Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  //particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  //particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  lastTime = millis();
}
void bmi160_intr(void)
{
 byte int_status = BMI160.getIntStatus0();
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
  particleSensor.wakeUp();
  particleSensor.setup();
  int steps = BMI160.getStepCount();
  long lastTime = millis();
  while(millis() - lastTime < interval){
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      digitalWrite(29,LOW);
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
    digitalWrite(29,HIGH);

    if(beatAvg != lastBeatAvg){
      SimbleeBLE.sendInt(beatAvg);
    }
    lastBeatAvg = beatAvg;
  }
  particleSensor.setPulseAmplitudeIR(0); //Turn off IR LED
  particleSensor.setPulseAmplitudeRed(0); //Turn off Red LED
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  particleSensor.shutDown();
  Simblee_ULPDelay(SECONDS(sleepTime));
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
