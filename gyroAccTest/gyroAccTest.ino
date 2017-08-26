#include <CurieIMU.h>
int readVal1, readVal2, readVal3, GX, GY, GZ, AX, AY, AZ;
#define GThre 27000
#define AThre 20000
#define BUTTON_DEBUG_DISPLAY_VALUE 40000

void setup() {
  CurieIMU.begin(); CurieIMU.setGyroRange(1000);
  CurieIMU.setAccelerometerRange(3);
  Serial.begin(9600);  //Setup serial at 115200 baud
  while (!Serial);  //Wait until Serial Monitor is opened
}

void loop() {
  sensorDebugLoop();
  //  updateData();
  //  while (GX <= GThre && GX >= -GThre && AX >= -AThre) {
  //    updateData();
  //  }
  //  if (AX < -AThre)
  //    Serial.println("U");
  //  else if (GX > GThre)
  //    Serial.println("L");
  //  else if (GX < -GThre)
  //    Serial.println("R");
  //  delay(100);
  //  if (abs(GX) > GThre || abs(AX) > AThre)
  //    delay(200);
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
