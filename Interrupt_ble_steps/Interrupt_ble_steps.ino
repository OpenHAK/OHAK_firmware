#include <BMI160Gen.h>
#include <ota_bootloader.h>

#include <SimbleeBLE.h>
#include "OHAK_Definitions.h"

#include <Wire.h>


//int 1 = 20
//int 2 = 24

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

void setup() {
  Wire.beginOnPins(SCL_PIN,SDA_PIN);
  SimbleeBLE.advertisementData = "OHAK_021";
  pinMode(RED,OUTPUT);

  // start the BLE stack
  SimbleeBLE.begin();
  //Serial.begin(9600); // initialize Serial communication
  //while (!Serial);    // wait for the serial port to open

  // initialize device
  //Serial.println("Initializing IMU device...");
  BMI160.begin(0, BMI_INT1);
  BMI160.attachInterrupt(bmi160_intr);
  BMI160.setIntTapEnabled(true);
  BMI160.setIntDoubleTapEnabled(true);
  BMI160.setStepDetectionMode(BMI160_STEP_MODE_NORMAL);
  BMI160.setStepCountEnabled(true);
  //Serial.print("Tap Thresh");
  //BMI160.setTapDetectionThreshold();
  //uint8_t test = BMI160.getTapDetectionThreshold();
  //Serial.println(test);
}

void loop() {
  Simblee_ULPDelay(1000);
  int steps = BMI160.getStepCount();
  SimbleeBLE.sendInt(steps);
  digitalWrite(RED,LOW);
  Simblee_ULPDelay(100);
  digitalWrite(RED,HIGH);
}
