#include <Arduino.h>
#include <HX711_ADC.h>
#include <Servo.h>
#include <VescUart.h>
#include <SoftwareSerial.h>

#define ADDITION 500
#define NEUTRAL 1500
#define MIN_RANGE 7
#define MIN_WEIGHT 40
#define MIN_WEIGHT_SINGLE 15

float CalcSpeed(float a, float b);
float GetAdjustedSpeed();
float SpeedValue(float diff);

//Pins setup
const int HX711_dout_1 = 4;
const int HX711_sck_1 = 3;
const int HX711_dout_2 = 11;
const int HX711_sck_2 = 10;

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2

//Servo setup
Servo ESC;

//UART setup
VescUart vesc;

//Global variables
float pwm = 0;
int ride = false;

void setup() {
  Serial.begin(4800);
  vesc.setSerialPort(&Serial);

  ESC.attach(9);
  ESC.writeMicroseconds(NEUTRAL);

  float calibrationValue_1 = -18000.0;
  float calibrationValue_2 = 18000.0;
  
  LoadCell_1.begin();
  LoadCell_2.begin();
  
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) {
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }

  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
}

void loop() {
  static boolean newDataReady = false;

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();

  if ((newDataReady)) {
      float a = LoadCell_1.getData();
      float b = LoadCell_2.getData();
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
    float range = MIN_RANGE + 3 * pwm;
    float stop;

    if (a + b < MIN_WEIGHT || a < MIN_WEIGHT_SINGLE || b < MIN_WEIGHT_SINGLE) {
        pwm = 0;
        ride = false;
    } else if (abs(diff) < range && !ride) {
        ride = true;
    } else if (abs(diff) > range && ride) {
        pwm = SpeedValue(diff);
        if (pwm < 0) {
            stop = pwm;
            pwm = 0;
            return stop;
        }
    }
    pwm = min(1.0, max(pwm, 0.0));
    
    return pwm;
}


/**
 * @return Motor rpm times constant
 */

float GetAdjustedSpeed() {
    if (vesc.getVescValues()) {
        return 0.0001 * vesc.data.rpm / 7 * 20 / 60;
    }
    return 0;
}


/**
 * @param diff - weight difference between trucks
 * @return PWM
 */

float SpeedValue(float diff) {
    if (pwm < 0.253 && diff > 0) {
            pwm += 0.0005 + GetAdjustedSpeed();
            pwm = max(0.25, pwm);
        } else if (pwm < 0.26 && diff > 0) {
            pwm += 0.001;
        } else if (pwm < 0.28 && diff > 0) {
            pwm += 0.0025;
        } else if (diff < -30) {
            pwm = 0;
            return -0.5;
        } else {
            pwm += diff * 0.0002;
        }

    return pwm;
}
