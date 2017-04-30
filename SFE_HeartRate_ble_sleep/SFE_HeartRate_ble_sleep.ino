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

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"
#include <BMI160Gen.h>

#include <TimeLib.h>
#include <Lazarus.h>
Lazarus Lazarus;

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
int sleepTime = 60;

float beatsPerMinute;
int beatAvg;
int lastBeatAvg;

byte mode = 0;
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
        byte flood : 1;
        byte accel : 1;
        byte digital_in1 : 1;
        byte digital_in2 : 1;
        byte digital_out1 : 1;
        byte digital_out2 : 1;
        byte extra0 : 1;
        byte extra1 : 1;
        byte conf1 : 1;
        byte conf2 : 1;
        byte conf3 : 1;
        byte conf4 : 1;
        byte conf5 : 1;
        byte conf6 : 1;
        byte conf7 : 1;
        byte conf8 : 1;
        short fwversion;
        int sleepTime;
        uint32_t nodeId; //store this nodeId
        uint32_t epoch; //uptime in ms
        float vbatt;
        float vbattl;
        float temp; //temperature maybe?
        float si_humidity;
        float si_temp;
        float ext_temp;
} Payload;


uint8_t advdata[15] =
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
        SimbleeBLE.softwareRevision = "0.0.1";
        Wire.beginOnPins(SCL_PIN,SDA_PIN);
        // change the advertisement interval
        SimbleeBLE.advertisementInterval = bleInterval;
        SimbleeBLE.begin();
        pinMode(RED,OUTPUT);
        digitalWrite(RED,HIGH);
        pinMode(BLU,OUTPUT);
        digitalWrite(BLU,HIGH);
        pinMode(GRN,OUTPUT);
        digitalWrite(GRN,HIGH);
        BMI160.begin(0, BMI_INT1);
        BMI160.attachInterrupt(bmi160_intr);
        BMI160.setIntTapEnabled(true);
        BMI160.setIntDoubleTapEnabled(true);
        BMI160.setStepDetectionMode(BMI160_STEP_MODE_NORMAL);
        BMI160.setStepCountEnabled(true);
        const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
        setTime(DEFAULT_TIME);
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
        if(Lazarus.lazarusArising()) {
                // Serial.println("Lazarus has awakened!");
                // Serial.println("");
        }
        particleSensor.wakeUp();
        particleSensor.setup();
        int steps = BMI160.getStepCount();
        long lastTime;
        lastBeatAvg = 0;
        beatAvg = 0;
        digitalWrite(GRN,LOW);
        // if (timeStatus() != timeNotSet) {
        //         String printString = digitalClockDisplay();
        // }
        switch (mode) {
        case 0:
                lastTime = millis();
                while(millis() - lastTime < interval) {
                        captureHR();
                }
                sleepNow();
                break;
        case 1:
                captureHR();
                break;
        case 2:
                mode = 0;
                sleepNow();
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
void SimbleeBLE_onReceive(char *data, int len) {
        // if the first byte is 0x01 / on / true
        //Serial.print("Received data over BLE ");
        //Serial.println(len);
        mode = data[0];
        if(mode==10) {
                if(len >= 5) {
                        unsigned long myNum = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
                        setTime(myNum);
                }
        }

}
void sleepNow(){
        digitalWrite(RED,HIGH);
        digitalWrite(GRN,HIGH);
        particleSensor.setPulseAmplitudeIR(0); //Turn off IR LED
        particleSensor.setPulseAmplitudeRed(0); //Turn off Red LED
        particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
        particleSensor.shutDown();
        Simblee_ULPDelay(SECONDS(sleepTime));
}
void captureHR(){
        long irValue = particleSensor.getGreen();
        digitalWrite(RED,LOW);
        if (checkForBeat(irValue) == true)
        {
                //We sensed a beat!
                digitalWrite(GRN,LOW);
                long delta = millis() - lastBeat;
                lastBeat = millis();

                beatsPerMinute = 60 / (delta / 1000.0);

                if (beatsPerMinute < 255 && beatsPerMinute > 20)
                {
                        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
                        rateSpot %= RATE_SIZE; //Wrap variable

                        //Take average of readings
                        beatAvg = 0;
                        for (byte x = 0; x < RATE_SIZE; x++)
                                beatAvg += rates[x];
                        beatAvg /= RATE_SIZE;
                }
        }
        digitalWrite(GRN,HIGH);

        if(beatAvg != lastBeatAvg) {
                if(bConnected) {
                        SimbleeBLE.sendInt(beatAvg);
                }

        }
        lastBeatAvg = beatAvg;
}
void SimbleeBLE_onConnect()
{
        bConnected = true;
        digitalWrite(BLU, LOW);
        mode = 1;
        Lazarus.ariseLazarus(); // Tell Lazarus to arise.
        //analogWrite(BLU,10);
}

void SimbleeBLE_onDisconnect()
{
        bConnected = false;
        mode = 2;
        digitalWrite(BLU, HIGH);
        //analogWrite(BLU,255);
        //pinMode(BLU,INPUT);
}

String digitalClockDisplay() {
        // digital clock display of the time
        String dataString = "";
        dataString += year();
        dataString += "-";
        dataString += month();
        dataString += "-";
        dataString += day();
        dataString += " ";
        dataString += hour();
        dataString += ":";
        //Serial.print(hour());
        if (minute() < 10) {
                dataString += "0";
        }
        dataString += minute();
        dataString += ":";
        //Serial.print(hour());
        if (second() < 10) {
                dataString += "0";
        }
        dataString += second();
        return dataString;
}
