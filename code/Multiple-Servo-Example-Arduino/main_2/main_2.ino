/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include "Wire.h"
#define KP_NUM 1
#define KP_DEN 1
#define KI_NUM 1
#define KI_DEN 1
#define KD_NUM 1
#define KD_DEN 1


unsigned long tPrev;
double e = 0; // error now
double ePrev = 0; // previous error
double eInt = 0;// integrated error 
double eDiff = 0;// differentiated error

double x = 0; //state 
double xPrev = 0; //previous state
double xDiff = 0;// differetiated state
double xRef = 0;//ref stae 

double y = 0; //state 
double yPrev = 0; //previous state
double yDiff = 0;// differetiated state
double yRef = 0;//ref stae

double z = 0; //state 
double zPrev = 0; //previous state
double zDiff = 0;// differetiated state
double zRef = 0;//ref stae

double u = 0;// input 
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);//0x68
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  //Wire.begin();
  while(!Serial) {}
  
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print("accel X:");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print("accel Y:");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print("accel Z:");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print("Gyro X:");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print("Gyro Y:");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print("Gyro Z:");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);

  z += IMU.getGyroZ_rads()*0.004;
  Serial.print("Z angle:");
  Serial.print(z);
  unsigned int tProc = millis() - tPrev;
  Serial.print("tProc:");
  Serial.println(tProc);
  if(tProc < 20){
    delay(20 - tProc);
  } 
  tPrev = millis();
  
}
