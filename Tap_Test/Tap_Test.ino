#include <BMI160Gen.h>
#include <ota_bootloader.h>

#include <SimbleeBLE.h>
#include "OHAK_Definitions.h"
#include <Lazarus.h>
Lazarus Lazarus;

#include <Wire.h>

bool interrupted = false;


//int 1 = 20
//int 2 = 24

void bmi160_intr(void)
{
        //digitalWrite(RED,LOW);
        byte int_status = BMI160.getIntStatus0();
        // Serial.print("Steps ");
        // Serial.print(BMI160.getStepCount());
        // Serial.print(" Single ");
        // Serial.print(bitRead(int_status,5));
        // Serial.print(" Double ");
        // Serial.print(bitRead(int_status,4));
        // Serial.print(" REG ");
        // Serial.print(int_status,BIN);
        //Serial.println(" BMI160 interrupt: TAP! ");
        //Lazarus.ariseLazarus();
        interrupted = true;
}
int myPinCallback(uint32_t ulPin)
{
        byte int_status = BMI160.getIntStatus0();
        digitalWrite(RED,LOW);
        // digitalWrite(4, HIGH);
        delay(250);
        digitalWrite(RED, HIGH);
        return 0; // don't exit Simblee_ULPDelay
}

void setup() {
        Wire.beginOnPins(SCL_PIN,SDA_PIN);
        SimbleeBLE.advertisementData = "OHAK_021";
        pinMode(RED,OUTPUT);
        digitalWrite(RED,HIGH);
        // start the BLE stack
        //SimbleeBLE.begin();
        Serial.begin(9600); // initialize Serial communication
        //while (!Serial);    // wait for the serial port to open

        // initialize device
        //Serial.println("Initializing IMU device...");
        BMI160.begin(0, -1);
        BMI160.attachInterrupt(NULL);

        //BMI160.setAccelerometerRange(4); 2,4,8,16

        //   CURIE_IMU_TAP:
        //       2G: 31.25 to 7968.75 (mg), in steps of 62.5 mg
        //       4G: 62.50 to 31937.50 (mg), in steps of 125.0 mg
        //       8G: 125.0 to 63875.00 (mg), in steps of 250.0 mg
        //       16G: 250.0 to 127750.00 (mg), in steps of 500 mg
        BMI160.setDetectionThreshold(CURIE_IMU_TAP, 1000);

        //   CURIE_IMU_DOUBLE_TAP: 50, 100, 150, 200, 250, 275, 500, 700 ms
        //   CURIE_IMU_TAP_SHOCK: 50, 75 ms
        //   CURIE_IMU_TAP_QUIET: 20, 30 ms
        //BMI160.setDetectionDuration(CURIE_IMU_DOUBLE_TAP,250);
        //BMI160.setDetectionDuration(CURIE_IMU_TAP_SHOCK,50);
        //BMI160.setDetectionDuration(CURIE_IMU_TAP_QUIET,30);

        float doubleTapTime = BMI160.getDetectionDuration(CURIE_IMU_DOUBLE_TAP);
        float shockTime = BMI160.getDetectionDuration(CURIE_IMU_TAP_SHOCK);
        float quietTime = BMI160.getDetectionDuration(CURIE_IMU_TAP_QUIET);
        float tapThresh = BMI160.getDetectionThreshold(CURIE_IMU_TAP);
        Serial.print("Tap Thresh: ");
        Serial.print(tapThresh);
        Serial.print("  shockTime: ");
        Serial.print(shockTime);
        Serial.print("  quietTime: ");
        Serial.print(quietTime);
        Serial.print("  doubleTapTime: ");
        Serial.println(doubleTapTime);

        //BMI160_STEP_MODE_NORMAL // recommended for most applications, well balanced between fals positives and false negatives
        //BMI160_STEP_MODE_SENSITIVE //recommended for light weighted persons, will give few false negatives, but eventually more false positives
        //BMI160_STEP_MODE_ROBUST //will give few false positives but eventually more false negatives

        BMI160.setStepDetectionMode(BMI160_STEP_MODE_NORMAL);
        int stepMode = BMI160.getStepDetectionMode();
        Serial.print("Step Mode ");
        Serial.println(stepMode);
        BMI160.setIntTapEnabled(true);
        BMI160.setIntDoubleTapEnabled(true);
        BMI160.setStepCountEnabled(true);
        //BMI160.setInterruptLatch(BMI160_LATCH_MODE_20_MS);
        //byte int_status = BMI160.getIntStatus0();
        pinMode(BMI_INT1, INPUT);
        Simblee_pinWake(BMI_INT1, LOW);
        //BMI160.resetInterrupt();
        // pinMode(BMI_INT1, INPUT);
        // Simblee_pinWakeCallback(BMI_INT1, LOW, myPinCallback);
        //Serial.print("Tap Thresh");
        //BMI160.setTapDetectionThreshold();
        //uint8_t test = BMI160.getTapDetectionThreshold();
        //Serial.println(test);
}

void loop() {
        // if(Lazarus.lazarusArising()) {
        //         digitalWrite(BLU,HIGH);
        //         //#ifdef DEBUG
        //                 Serial.println("Lazarus has awakened!");
        //         //#endif
        //         // Serial.println("");
        // }
        if (Simblee_pinWoke(BMI_INT1))
        {
                byte int_status = BMI160.getIntStatus0();
                Serial.println(int_status);
                if (BMI160.tapDetected(X_AXIS, NEGATIVE))
                        Serial.println("Tap detected on negative X-axis");
                if (BMI160.tapDetected(X_AXIS, POSITIVE))
                        Serial.println("Tap detected on positive X-axis");
                if (BMI160.tapDetected(Y_AXIS, NEGATIVE))
                        Serial.println("Tap detected on negative Y-axis");
                if (BMI160.tapDetected(Y_AXIS, POSITIVE))
                        Serial.println("Tap detected on positive Y-axis");
                if (BMI160.tapDetected(Z_AXIS, NEGATIVE))
                        Serial.println("Tap detected on negative Z-axis");
                if (BMI160.tapDetected(Z_AXIS, POSITIVE))
                        Serial.println("Tap detected on positive Z-axis");
                if (BMI160.getInterruptStatus(CURIE_IMU_TAP)) {

                }
                Simblee_resetPinWake(BMI_INT1);
                digitalWrite(RED,LOW);
                delay(100);
                digitalWrite(RED,HIGH);

                // BMI160.resetInterrupt();
        }
        //digitalWrite(RED,LOW);
        //delay(100);
        // byte int_status = BMI160.getIntStatus0();
        // Serial.print("Int Status: ");
        // Serial.print(int_status);
        // int inputPin = digitalRead(BMI_INT1);
        // //Serial.print(" pin Status: ");
        //
        // if(inputPin ==1){
        //   byte int_status = BMI160.getIntStatus0();
        //   Serial.println(int_status);
        //   interrupted = false;
        // }
        //delay(1);
        // if (!input) {
        //   byte int_status = BMI160.getIntStatus0();
        //   BMI160.resetInterrupt();
        //   interrupted = false;
        //   digitalWrite(RED,LOW);
        //   //delay(100);
        // }

        // byte int_status = BMI160.getIntStatus0();
        // delay(100);
        // digitalWrite(RED,HIGH);
        // delay(100);

        Simblee_ULPDelay(INFINITE);

        //int steps = BMI160.getStepCount();
        //SimbleeBLE.sendInt(steps);
        //digitalWrite(RED,LOW);
        //Simblee_ULPDelay(100);

}
