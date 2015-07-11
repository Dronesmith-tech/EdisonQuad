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


#ifndef _AEROQUAD_MOTORS_EDISON_H_
#define _AEROQUAD_MOTORS_EDISON_H_

#include "Arduino.h"
#include "Motors.h"
#include <servo.h>

#define MOTORPIN0    3
#define MOTORPIN1    5
#define MOTORPIN2    6
#define MOTORPIN3    9

Servo myServo[4];

void initializeMotors(NB_Motors numbers) {
  numberOfMotors = numbers;


  // TODO init servos

  myServo[MOTOR1].attach(MOTORPIN0);
  myServo[MOTOR2].attach(MOTORPIN1);
  myServo[MOTOR3].attach(MOTORPIN2);
  myServo[MOTOR4].attach(MOTORPIN3);

  commandAllMotors(0);
}

void writeMotors() {

  int i;
  for (i = 0; i < 4; ++i) {
    if (motorCommand[i] > 255 || motorCommand[i] < 0) {
      return;
    }
    //  motorCommand[i] = constrain(motorCommand[i], 0, 255);
  }

  // TODO update motor value
  // Serial.println(motorCommand[MOTOR1]);
	myServo[MOTOR1].write(motorCommand[MOTOR1]);
	myServo[MOTOR2].write(motorCommand[MOTOR2]);
	myServo[MOTOR3].write(motorCommand[MOTOR3]);
	myServo[MOTOR4].write(motorCommand[MOTOR4]);

}

void commandAllMotors(int command) {
  // TODO drive motors
  int i;
  for (i = 0; i < 4; ++i) {
    // motorCommand[i] = constrain(motorCommand[i], 0, 255);
    if (motorCommand[i] > 255 || motorCommand[i] < 0) {
      return;
    }
    // motorCommand[i] = constrain(motorCommand[i], 0, 255);
  }

	myServo[MOTOR1].write(command);
	myServo[MOTOR2].write(command);
	myServo[MOTOR3].write(command);
	myServo[MOTOR4].write(command);


}


#endif _AEROQUAD_MOTORS_EDISON_H_
