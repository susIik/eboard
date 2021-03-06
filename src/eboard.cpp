#include <Arduino.h>
#include <HX711_ADC.h>
#include <Servo.h>

#define ADDITION 500
#define NEUTRAL 1500
#define MIN_RANGE 7

float CalcSpeed(float a, float b);

//Pins setup
const int HX711_dout_1 = 4;
const int HX711_sck_1 = 5;
const int HX711_dout_2 = 6;
const int HX711_sck_2 = 7;

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2

//Servo setup
Servo ESC;

float pwm = NEUTRAL;
int ride = false;

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  ESC.attach(3);
  ESC.writeMicroseconds(NEUTRAL);

  float calibrationValue_1 = 18000.0;
  float calibrationValue_2 = 18000.0;
  
  LoadCell_1.begin();
  LoadCell_2.begin();
  
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }

  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
  Serial.println("Startup is complete");
}

void loop() {
  static boolean newDataReady = false;

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();

  //get smoothed value from data set
  if ((newDataReady)) {
      float a = LoadCell_1.getData();
      float b = LoadCell_2.getData();
      //Serial.print("Load_cell 1 output val: ");
      //Serial.print(a);
      //Serial.print("    Load_cell 2 output val: ");
      //Serial.println(b);
      Serial.println(int(NEUTRAL + 500 * CalcSpeed(a, b)));
      newDataReady = false;
      ESC.writeMicroseconds(int(NEUTRAL + ADDITION * CalcSpeed(a, b)));
  }
}


/**
 * @param a - first truck weight
 * @param b - back truck weight
 * @return PWM
 */

float CalcSpeed(float a, float b) {
    float diff = a - b;
    float range = MIN_RANGE + 5 * pwm;

    //Serial.println(range);

    if (a + b < 40 || a < 18 || b < 18) {
        pwm = 0;
        ride = false;
    } else if (abs(diff) < range && !ride) {
        ride = true;
    } else if (abs(diff) > range && ride) {
        if (pwm < 0.135 && diff > 0) {
            pwm += 0.0005;
            pwm = max(0.13, pwm);
        } else if (pwm < 0.15 && diff > 0) {
            pwm += 0.001;
        } else if (pwm < 0.19 && diff > 0) {
            pwm += 0.0025;
        } else if (diff < -25) {
            pwm = 0;
            return -0.5;
        } else {
            pwm += diff * 0.0002;
        }
    }
    pwm = min(1.0, max(pwm, 0.0));
    
    return pwm;
}
