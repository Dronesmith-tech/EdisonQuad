/*  AeroQuad v3.0.1 - June 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.


  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.


  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.


  Implemented by Lars Karlslund
*/


#ifndef _AEROQUAD_GYROSCOPE_L3G3200D_H_
#define _AEROQUAD_GYROSCOPE_L3G3200D_H_


#include <Gyroscope.h>
#include <SensorsStatus.h>
#include <GlobalDefined.h>

#define GYRO_CALIBRATION_TRESHOLD 4

#define GYRO_ADDRESS 0x6B


#define GYRO_RATE 2000
//#define GYRO_RATE 1000
//#define GYRO_RATE 500


#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24

// Axis inversion: -1 = invert, 1 = don't invert
int gyroAxisInversionFactor[3] = {1,-1,-1};


void initializeGyro() {

  sendByteI2C(GYRO_ADDRESS, 0x0f);
  if (readByteI2C(GYRO_ADDRESS) == 0b11010100) {
    vehicleState |= GYRO_DETECTED;
  }

  // 3.) Setup the gyroscope
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  // lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
   lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
  gyroScaleFactor = radians(0.061);

  gyroSampleCount = 1;

}

// Read raw values from sensor (inverted if required)
void readGyroRaw(int *gyroRaw) {

    lsm.readGyro();

    gyroRaw[XAXIS] = gyroAxisInversionFactor[XAXIS] * lsm.gyroData.x;
    gyroRaw[YAXIS] = gyroAxisInversionFactor[YAXIS] * lsm.gyroData.y;
    gyroRaw[ZAXIS] = gyroAxisInversionFactor[ZAXIS] * lsm.gyroData.z;
}

void measureGyro() {
    int gyroRaw[3];
    readGyroRaw(gyroRaw);

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      gyroRate[axis] = (gyroRaw[axis] - gyroZero[axis]) * gyroScaleFactor;
  }

  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}


void measureGyroSum() {
    int gyroRaw[3];
    readGyroRaw(gyroRaw);

    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
        gyroSample[axis] += gyroRaw[axis];
    }

    gyroSampleCount++;
}


void evaluateGyroRate() {
    if (!gyroSampleCount) {
      return;
    }

    for (byte axis = 0; axis <= ZAXIS; axis++) {
        gyroRate[axis] = ((gyroSample[axis] / (float)gyroSampleCount) - gyroZero[axis]) * gyroScaleFactor;
        gyroSample[axis] = 0;
    }

    gyroSampleCount = 0;

    // Measure gyro heading
    long int currentTime = micros();
    if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
        gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
    }
    gyroLastMesuredTime = currentTime;
}


boolean calibrateGyro() {

  int findZero[FINDZERO];
  int diff = 0;
  int tmp;
  // for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      findZero[i] = lsm.gyroData.x;
    }
    tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
	  gyroZero[XAXIS] = tmp;
	}
	else {
		return false; //Calibration failed.
	}

  for (int i=0; i<FINDZERO; i++) {
    findZero[i] = lsm.gyroData.y;
  }
  tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
  gyroZero[YAXIS] = tmp;
}
else {
  return false; //Calibration failed.
}

for (int i=0; i<FINDZERO; i++) {
  findZero[i] = lsm.gyroData.z;
}
tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
gyroZero[ZAXIS] = tmp;
}
else {
return false; //Calibration failed.
}


  return true;
}
#endif
