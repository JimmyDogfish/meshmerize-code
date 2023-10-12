#include <QTRSensors.h>
#include <L298N.h>

int motor1pin1 = 5;
int motor1pin2 = 3;
int ENA = 10;

int motor2pin1 = 4;
int motor2pin2 = 2;
int ENB = 9;

L298N motor1(ENA, motor1pin1, motor1pin2);
L298N motor2(ENB, motor2pin1, motor2pin2);

int button_calibration = A3;

int P;
int I;
int D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

void setup() {
  // put your setup code here, to run once:
  
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(button_calibration, INPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){6, 7, 8, 11, 12}, SensorCount);

  while(digitalRead(button_calibration) == LOW) {}
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 2000 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  int motorSpeedA = 200 + motorSpeedChange;
  int motorSpeedB = 200 - motorSpeedChange;

  if (motorSpeedA > 255) {
    motorSpeedA = 255;
  }
  if (motorSpeedB > 255) {
    motorSpeedB = 255;
  }
  if (motorSpeedA < -170) {
    motorSpeedA = -170;
  }
  if (motorSpeedB < -170) {
    motorSpeedB = -170;
  }
  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    motor1.forward();
  }
  else {
    motor1.backward();
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    motor2.backward();
  }
  else {
    motor2.forward();
  }
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);

}