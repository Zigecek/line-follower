#include <QTRSensors.h>
#include <math.h>

// motors
const int in1[] = {10, 11}; 
const int in2[] = {3, 9};

const int start = 15;

const int aIR[] = {17/*,18*/,19};
const int dIR[] = {4,5,6, /**/17, 19,/**/ 7,8,12};
const int baseSpeed = 200;
const int minSpeed = 15;

#define FW 0
#define BW 1

// middle sensors init
QTRSensors qtr;
const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
const int tolerance = 100;

int lastVals[8] = {};
int prevVals[8] = {};
unsigned long lastSeen[8] = {};
unsigned long lastMid = 0;
const int seenTolerance = 500;
const int stopDelay = 500;
int error;
int prevErr = 0;
int lastErr = 0;

int lastStart = 0;

#define FINDING 2
#define STOPPED 1
#define RUNNING 0
int mode = STOPPED;

boolean foundLine = false;

float p;
float i;
float d;
int pidVal;
float Kp = 50;
float Ki = 1;
float Kd = 100;

void setup()
{
  // pinModes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(start, INPUT_PULLUP);

  for (int i = 0; i < 8; i++){
    pinMode(dIR[i], INPUT);
  }
  
  // serial begin
  Serial.begin(9600);

  // middle sensors config
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){aIR[0]/* LINE */,aIR[1]}, SensorCount);

  // middle sensors calibration
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibration started!");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration completed!");

  // getting init values
  getSens();
  Serial.println("Completed getting init values");
}

void loop() {
  int curStart = digitalRead(start);
  if ((curStart == 1 && lastStart == 0)) {
    if (mode == STOPPED) {
      mode = RUNNING;
      delay(100);
    } else {
      Serial.println("Stopped 1!");
      foundLine = false;
      mode = STOPPED;
      stopAB();
      for (int i = 0; i < 125; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(20);
        digitalWrite(LED_BUILTIN, LOW);
        delay(20);
      }
    }
  }
  lastStart = curStart;
  switch (mode)
  {
    case STOPPED:
      stopAB();
      foundLine = false;
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case FINDING:
      digitalWrite(LED_BUILTIN, HIGH);
      getSens();
      error = getErr();
      Serial.print("err: ");
      Serial.println(error);
      motorB(FW, baseSpeed*1.5 > 255 ? 255 : baseSpeed*1.5);
      motorA(FW, baseSpeed*0.5);
      if (foundLine) {
        mode = RUNNING;
      }
      break;
    case RUNNING:
      digitalWrite(LED_BUILTIN, HIGH);
      getSens();
      error = getErr();
      for (int i = 0; i < 8; i++) {
        Serial.print(lastVals[i]);
        Serial.print('\t');
      }
      Serial.println();
      if (error == 69 || error == 36 || error == 99 || error == 34) {
        break;
      }
      PID();
      break;
  }
}

void PID(){
  if (error == 0 && (prevErr == 99 || millis() - lastMid < seenTolerance)) {
    motorA(FW, 255);
    motorB(FW, 255);
    delay(stopDelay);
    mode = STOPPED;
    Serial.println("Stopped 4!");
    return;
  }
  if (error == 0 && (prevErr == -7 || prevErr == -6 || prevErr == -5 || prevErr == -4 || prevErr == -3 || prevErr == 7 || prevErr == 6 || prevErr == 5 || prevErr == 4 || prevErr == 3)) {
    error = prevErr;
  }
  i = 0;
  p = error;
  i = i + error;
  d = error - prevErr;
  pidVal = (Kp*p) + (Ki*i) + (Kd*d);
  int pwmVal1 = getpwm(baseSpeed - pidVal);
  int pwmVal2 = getpwm(baseSpeed + pidVal);
  prevErr = error;

  motorA(FW,pwmVal1);
  motorB(FW,pwmVal2);
}

void getSens(){
  qtr.read(sensorValues);
  for (int i = 0; i < 8; i++) {
    prevVals[i] = lastVals[i];
  }
  for (int i = 0; i < 8; i++){
    if (i == 3) {
      lastVals[i] = sensorValues[0] > (qtr.calibrationOn.maximum[0]+tolerance) ? 1 : 0;
    } else if (i == 4) {
      lastVals[i] = sensorValues[1] > (qtr.calibrationOn.maximum[1]+tolerance) ? 1 : 0;
    } else {
      lastVals[i] = !digitalRead(dIR[i]);
    }

    if (lastVals[i] != 0){
      lastSeen[i] = millis();
    }
  }
}

int getErr(){
  if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    if (!foundLine) {
      mode = FINDING;
      return 36;
    } else {
      return 0;
    }
  } else if (lastVals[3] == 1 && lastVals[4] == 1){
    foundLine = true;
    lastMid = millis();
    return 34;
  } else if (lastVals[0] == 1 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -7;
  } else if (lastVals[0] == 1 && lastVals[1] == 1 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -6;
  } else if (lastVals[0] == 0 && lastVals[1] == 1 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -5;
  } else if (lastVals[0] == 0 && lastVals[1] == 1 && lastVals[2] == 1 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -4;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 1 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -3;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 1 && lastVals[3] == 1 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -2;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 1 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return -1;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 1 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return 1;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 1 && lastVals[5] == 1 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return 2;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 1 && lastVals[6] == 0 && lastVals[7] == 0){
    foundLine = true;
    return 3;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 1 && lastVals[6] == 1 && lastVals[7] == 0){
    foundLine = true;
    return 4;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 1 && lastVals[7] == 0){
    foundLine = true;
    return 5;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 1 && lastVals[7] == 1){
    foundLine = true;
    return 6;
  } else if (lastVals[0] == 0 && lastVals[1] == 0 && lastVals[2] == 0 && lastVals[3] == 0 && lastVals[4] == 0 && lastVals[5] == 0 && lastVals[6] == 0 && lastVals[7] == 1){
    foundLine = true;
    return 7;
  } else {
    foundLine = true;
    return 99;
  }
}

int getpwm(float f){
  int r = round(f);
  return r > 255 ? 255 : r < 0 ? 0 : r;
}

void motorA(int dir, int pwm){
  if (dir == FW) {
    analogWrite(in1[0], pwm);
    analogWrite(in1[1], 0);
  } else {
    analogWrite(in1[0], 0);
    analogWrite(in1[1], pwm);
  }
}
void motorB(int dir, int pwm){
  if (dir == FW) {
    analogWrite(in2[0], pwm);
    analogWrite(in2[1], 0);
  } else {
    analogWrite(in2[0], 0);
    analogWrite(in2[1], pwm);
  }
}
void stopAB(){
  analogWrite(in1[0], 0);
  analogWrite(in1[1], 0);
  analogWrite(in2[0], 0);
  analogWrite(in2[1], 0);
}

unsigned long absoluteMin(unsigned long ar[]){
  int abMin = 0;
  for (int i = 2; i<=5; i++){
    if (abMin == 0) {
      abMin = ar[i];
    } else {
      if (ar[i] > 0) {
        abMin = min(abMin, ar[i]);
      }
    }
  }
  return abMin;
}
unsigned long absoluteMax(unsigned long ar[]){
  int abMax = 0;
  for (int i = 2; i<=5; i++){
    if (abMax == 0) {
      abMax = ar[i];
    } else {
      abMax = max(abMax, ar[i]);
    }
  }
  return abMax;
}
