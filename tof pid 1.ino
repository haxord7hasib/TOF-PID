#include <Wire.h>
#include <VL53L0X.h>
#include <PID_v1.h>

VL53L0X sensor;
double setpoint = 100.0;
double input, output;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
  Serial.begin(9600);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 100);
}

void loop() {
  input = sensor.readRangeContinuousMillimeters() - 20;
  pid.Compute();
  int delay_time = 100 - (int)output;
  if (delay_time < 0) {
    delay_time = 0;
  } else if (delay_time > 100) {
    delay_time = 100;
  }
  delay(delay_time);
  Serial.println(input-20);
}
