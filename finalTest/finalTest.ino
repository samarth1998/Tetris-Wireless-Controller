#include <CurieIMU.h>
#include "CurieTimerOne.h"
#include "QuickStats.h"

QuickStats stats;
int readVal1, readVal2, readVal3, GX, GY, GZ, AX, AY, AZ, sensorValMax, EMGData;
float sensorValTrial[1000];
#define GThre 27000
#define AThre 20000
#define BUTTON_DEBUG_DISPLAY_VALUE 40000

void setup() {

  CurieIMU.begin(); CurieIMU.setGyroRange(1000);
  CurieIMU.setAccelerometerRange(3);
  Serial.begin(9600);  //Setup serial at 115200 baud
  while (!Serial);  //Wait until Serial Monitor is opened
  int i;
  //Calibration of EMG
  Serial.println("Start Calibration Process");
  delay(1000);
  //Drops first 500 samples
  for (i = 0; i < 500; i++) {
    getEMGValue();
    delay(2);
  }

  for (i = 0; i < 1000; i++) {
    sensorValTrial[i] = getEMGValue();
    delay(5);
  }
  Serial.println("End Calibration");
  sensorValMax = stats.median(sensorValTrial, 900);
  Serial.print("sensorValMax = "); Serial.println(sensorValMax);
}
void loop() {
  //  sensorDebugLoop();
  updateData();
  while (GX <= GThre && GX >= -GThre && AX >= -AThre && AZ <= AThre && (EMGData <= sensorValMax)) {
    updateData();
    delay(1);
    //Serial.println(EMGData);
  }
  if (AX < -AThre)
    Serial.println("U");
  else if (AZ > AThre)
    Serial.println("B");
  else if (GX > GThre)
    Serial.println("L");
  else if (GX < -GThre)
    Serial.println("R");
  else if (EMGData > sensorValMax) {
    Serial.println("A");
    Serial.println("A");
    while (EMGData > 20)
      updateData();
    delay(50);
  }
  delay(100);
  updateData();
  if (abs(GX) > GThre || abs(AX) > AThre || EMGData > 40)
    delay(200);
}

double getEMGValue() {
  static double sensorValue;
  sensorValue = analogRead(A0); //* 0.0049 ;
  sensorValue = sensorValue * 3.3 / 1024 + 1.5;
  sensorValue *= 1000;
  sensorValue = highthirdOrderIIR_TEMPLATE(sensorValue);
  sensorValue = lowthirdOrderIIR_TEMPLATE(sensorValue);
  sensorValue = abs(sensorValue);
  sensorValue = boxcarFilterSample(sensorValue);
  return sensorValue;
}
void EMGIsr() {
  static int number = 0;
  //static int sensorValueAv = 0; //static int Av = 0;
  //static double treshold = 0;
  double sensorValue = getEMGValue();
  if (sensorValue > (sensorValMax - 15)) {
    if (number == 0) {
      Serial.println("B");
      number += 1;
    }
  }
  if (sensorValue < 60) {
    number = 0;
  }

  /* max value approach
    time2=micros();
    if ((time2-time1)>1000000){
      if(Av<100){
      sensorValueAv+=sensorValue; Av+=1;
      }
      }
       Serial.println(Av);
       Serial.println(sensorValueAv);
    if (Av==100) {
      treshold=sensorValueAv/Av;
      tresholdCalculation=1;
      Av=101;
     }
     Serial.println(treshold);
  */
}
int retrieveData(int sensor, int axis) {
  if (sensor == 1)
    CurieIMU.readGyro(readVal1, readVal2, readVal3);
  else if (sensor == 2)
    CurieIMU.readAccelerometer(readVal1, readVal2, readVal3);
  if (axis == 1)
    return readVal1;
  else if (axis == 2)
    return readVal2;
  else
    return readVal3;
}

void updateData() {
  GX = retrieveData(1, 1);
  GY = retrieveData(1, 2);
  GZ = retrieveData(1, 3);
  AX = retrieveData(2, 1);
  AY = retrieveData(2, 2);
  AZ = retrieveData(2, 3);
  EMGData = getEMGValue();
}

void sensorDebugLoop() {
  updateData();
  while (GX <= GThre && GX >= -GThre && AX >= -AThre) {
    updateData();
    debugPrint(GX, AX, 0, 0, 0);
  }
  if (AX < -AThre)
    debugPrint(GX, AX, BUTTON_DEBUG_DISPLAY_VALUE, 0, 0);
  else if (GX > GThre)
    debugPrint(GX, AX, 0, BUTTON_DEBUG_DISPLAY_VALUE, 0);
  else if (GX < -GThre)
    debugPrint(GX, AX, 0, 0, BUTTON_DEBUG_DISPLAY_VALUE);
  delay(100);
  if (abs(GX) > GThre || abs(AX) > AThre) {
    delay(200);
  }
}

void debugPrint(int gDataX, int aDataX, int uData, int lData, int rData) {
  Serial.print(gDataX); Serial.print(" "); Serial.print(aDataX); Serial.print(" "); Serial.print(uData); Serial.print(" "); Serial.print(lData); Serial.print(" "); Serial.println(rData);
}



//Filter Functions---------------------------------------------------------------------------------------------------------

float boxcarFilterSample(float sample)
{
  static const int boxcarWidth = 100; // Change this value to alter boxcar length
  static float recentSamples[boxcarWidth] = {0}; // hold onto recent samples
  static int readIndex = 0;              // the index of the current reading
  static float total = 0;                  // the running total
  static float average = 0;                // the average

  // subtract the last reading:
  total = total - recentSamples[readIndex];
  // add new sample to list (overwrite oldest sample)
  recentSamples[readIndex] = sample;
  // add the reading to the total:
  total = total + recentSamples[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= boxcarWidth) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / boxcarWidth;
  // send it to the computer as ASCII digits
  return average;
}

float highthirdOrderIIR_TEMPLATE(float sample)
{
  static const float a[4] = {1., -2.37409474,  1.92935567, -0.53207537}; // ADD A VALUES HERE
  static const float b[4] = {0.72944072, -2.18832217,  2.18832217, -0.72944072};// ADD B VALUES HERE

  // x array for holding recent inputs (newest input as index 0, delay
  //of 1 at index 1, etc.
  static float x[4] = {0};
  // x array for holding recent inputs (newest input as index 0, delay
  //of 1 at index 1, etc.
  static float y[4] = {0};

  x[0] = sample;

  // Calculate the output filtered signal based on a weighted sum of
  //previous inputs/outputs
  y[0] = (b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3]) - (a[1] * y[1] + a[2] * y[2] + a[3] * y[3]);
  y[0] /= a[0];

  // Shift the input signals by one timestep to prepare for the next
  //call to this function
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];

  // Shift the previously calculated output signals by one time step
  //to prepare for the next call to this function
  y[3] = y[2];
  y[2] = y[1];
  y[1] = y[0];

  return y[0];
}

float lowthirdOrderIIR_TEMPLATE(float sample)
{
  static const float a[4] = {1.00000000e+00, -2.77555756e-16, 3.33333333e-01, -1.85037171e-17}; // ADD A VALUES HERE
  static const float b[4] = {0.16666667,  0.5       ,  0.5      ,  0.16666667}; // ADD B VALUES HERE

  // x array for holding recent inputs (newest input as index 0, delay
  //of 1 at index 1, etc.
  static float x[4] = {0};
  // x array for holding recent inputs (newest input as index 0, delay
  //of 1 at index 1, etc.
  static float y[4] = {0};

  x[0] = sample;

  // Calculate the output filtered signal based on a weighted sum of
  //previous inputs/outputs
  y[0] = (b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3]) - (a[1] * y[1] + a[2] * y[2] + a[3] * y[3]);
  y[0] /= a[0];

  // Shift the input signals by one timestep to prepare for the next
  //call to this function
  x[3] = x[2];
  x[2] = x[1];
  x[1] = x[0];

  // Shift the previously calculated output signals by one time step
  //to prepare for the next call to this function
  y[3] = y[2];
  y[2] = y[1];
  y[1] = y[0];

  return y[0];
}
