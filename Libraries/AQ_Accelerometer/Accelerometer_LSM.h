/*
  AeroQuad v3.0.1 - February 2012
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
*/

#ifndef _AEROQUAD_ACCELEROMETER_LSM_H_
#define _AEROQUAD_ACCELEROMETER_LSM_H_

#include <Accelerometer.h>
#include <Device_I2C.h>
#include <SensorsStatus.h>
#include <GlobalDefined.h>

#define LSM_ACC_ADDRESS     (0x1D)

#define LSM_ACC_IDENTITY    (0x49)

#define LSM_ACC_CTRLREG0    (0x1F)
#define LSM_ACC_CTRLREG1    (0x20)
#define LSM_ACC_CTRLREG2    (0x21)
#define LSM_ACC_CTRLREG3    (0x22)
#define LSM_ACC_CTRLREG4    (0x23)
#define LSM_ACC_CTRLREG5    (0x24)
#define LSM_ACC_CTRLREG6    (0x25)
#define LSM_ACC_CTRLREG7    (0x26)

#define LSM_ACC_STATUS      (0x27)

#define LSM_ACC_OUTXL       (0x28)
#define LSM_ACC_OUTXH       (0x29)

#define LSM_ACC_OUTYL       (0x2A)
#define LSM_ACC_OUTYH       (0x2B)

#define LSM_ACC_OUTZL       (0x2C)
#define LSM_ACC_OUTZH       (0x2D)


void initializeAccel() {
  // probe
  // if (readWhoI2C(LSM_ACC_ADDRESS) == LSM_ACC_IDENTITY) {
    vehicleState |= ACCEL_DETECTED;
  // }

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
}

void measureAccel() {
  byte axis = XAXIS;

  lsm.readAccel();

  meterPerSecSec[XAXIS] = lsm.accelData.x * (19.613/65536.0)/*accelScaleFactor[XAXIS]*/ + runTimeAccelBias[XAXIS];
  meterPerSecSec[YAXIS] = lsm.accelData.y * (19.613/65536.0)/**accelScaleFactor[YAXIS]*/ + runTimeAccelBias[YAXIS];
  meterPerSecSec[ZAXIS] = lsm.accelData.z * (19.613/65536.0)/*elScaleFactor[ZAXIS]*/ + runTimeAccelBias[ZAXIS];

  // for (byte i = LSM_ACC_OUTXL; i <= LSM_ACC_OUTZH; i +=  2) {
  //   sendByteI2C(LSM_ACC_ADDRESS, i);
  //   sendByteI2C(LSM_ACC_ADDRESS, i+1);
  //   Wire.requestFrom(LSM_ACC_ADDRESS, 2);
  //   meterPerSecSec[axis++] =
  //     (readByteI2C() | ((readByteI2C() << 8) & 0xFF00))
  //     * accelScaleFactor[axis] + runTimeAccelBias[axis];
  // }
}

void measureAccelSum() {
  byte axis = XAXIS;

  accelSample[XAXIS] = lsm.accelData.x * (19.613/655536.0);/*accelScaleFactor[XAXIS]  + runTimeAccelBias[XAXIS]; */
  accelSample[YAXIS] = lsm.accelData.y * (19.613/655536.0);/*accelScaleFactor[XAXIS]  + runTimeAccelBias[YAXIS]; */
  accelSample[ZAXIS] = lsm.accelData.z * (19.613/655536.0);/*accelScaleFactor[XAXIS] + runTimeAccelBias[ZAXIS]; */

  // for (byte i = LSM_ACC_OUTXL; i <= LSM_ACC_OUTZH; i +=  2) {
  //   sendByteI2C(LSM_ACC_ADDRESS, i);
  //   sendByteI2C(LSM_ACC_ADDRESS, i+1);
  //   Wire.requestFrom(LSM_ACC_ADDRESS, 2);
  //   accelSample[axis++] +=
  //     readByteI2C() | ((readByteI2C() << 8) & 0xFF00);
  // }

}

void evaluateMetersPerSec() {
  if (!accelSampleCount) {
    return;
  }
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    /*accelSample[axis] = (accelSample[axis]) * (19.613/65536.0); /*(G/2byte number range)*/
    meterPerSecSec[axis] = (accelSample[axis]) /*/ (float)accelSampleCount*/ * 1.0/*accelScaleFactor[axis]*/ + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  /*accelSampleCount = 0;*/
}

void computeAccelBias() {

  /*for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }
*/
  //for (byte axis = 0; axis < 3; axis++) {
    //meterPerSecSec[axis] = (float(accelSample[axis])/*/SAMPLECOUNT*/)  * 1.0/*accelScaleFactor[axis]*/;
  //  accelSample[axis] = 0;
//  }
  /*accelSampleCount = 0;*/

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = fabs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

#endif
