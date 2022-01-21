#include <HX711_ADC.h>
#include <Servo.h>

#define MIN_SPEED 1500
#define MAX_SPEED 2000
#define NEUTRAL 1500

//pins:
const int HX711_dout_1 = 4; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 5; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 6; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 7; //mcu > HX711 no 2 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2

//Servo setup
Servo ESC;

unsigned long t = 0;
int pwm = NEUTRAL;
int limitPwm = NEUTRAL;
int ride = 0;

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
  
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
  Serial.println("Startup is complete");
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      float a = LoadCell_1.getData();
      float b = LoadCell_2.getData();
      Serial.print("Load_cell 1 output val: ");
      Serial.print(a);
      Serial.print("    Load_cell 2 output val: ");
      Serial.println(b);
      Serial.println(pwm);
      newDataReady = 0;
      t = millis();

      if (a + b < 40 || a < 18 || b < 18) {
        pwm = NEUTRAL;
        ride = 0;
      } else if (abs(a - b) < 5 && !ride) {
        ride = 1;
      } else if (abs(a - b) > 5 && ride) {
        pwm += a - b;
        limitPwm = map(a - b, -40, 40, 1000, MAX_SPEED);
        if ((pwm > NEUTRAL && pwm > limitPwm) || (pwm < NEUTRAL && pwm < limitPwm)) {
          pwm = limitPwm;
        }
      } else {
        if (pwm > NEUTRAL) pwm = int((pwm - NEUTRAL) * 0.95) + NEUTRAL;
        if (pwm < NEUTRAL) pwm = NEUTRAL - int((NEUTRAL - pwm) * 0.95);
      }
      if (pwm < MIN_SPEED) pwm = MIN_SPEED;
      if (pwm > MAX_SPEED) pwm = MAX_SPEED;
      ESC.write(pwm);
      
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
    Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
    Serial.println("Tare load cell 2 complete");
  }

}
