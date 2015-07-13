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

#ifndef _AEROQUAD_RECEIVER_BT_H_
#define _AEROQUAD_RECEIVER_BT_H_

#include "GlobalDefined.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/input.h>

#define DEVICE_NAME     "/dev/input/event2"

#define TYPE_RELATIVE   3
#define TYPE_BUTTON     1

#define REL_LEFT_THUMB_X      0x00
#define REL_LEFT_THUMB_Y      0x01
#define REL_RIGHT_THUMB_X     0x02
#define REL_LEFT_TRIG2        0x03
#define REL_RIGHT_TRIG2       0x04
#define REL_RIGHT_THUMB_Y     0x05

#define BTN_SQUARE            0x130
#define BTN_CROSS             0x131
#define BTN_CIRCLE            0x132
#define BTN_TRIANGLE          0x133
#define BTN_LEFT_TRIG1        0x134
#define BTN_RIGHT_TRIG1       0x135
#define BTN_LEFT_TRIG2        0x136
#define BTN_RIGHT_TRIG2       0x137
#define BTN_LEFT_THUMB        0x13A
#define BTN_RIGHT_THUMB       0x13B


int BTfd = NULL;

struct input_event BTev[64];

bool Bluetooth_Open() {
  if (BTfd) {
    close (BTfd);
  }

  if ((BTfd = open(DEVICE_NAME, O_RDONLY)) < 0) {
    // Serial.println("[BT Receiver] Could not open controller.");
    BTfd = NULL;
    return false;
  }

  // Serial.println("[BT Receiver] Ready.");

  return true;
}

void Bluetooth_Close() {
  if (!BTfd || BTfd < 0) {
    Serial.println("[BT Receiver] No Device open!");
  } else {
    close(BTfd);
    BTfd = NULL;
    Serial.println("[BT Receiver] Controller closed.");
  }
}

/*
receiverCommand[XAXIS] = 1500;
receiverCommand[YAXIS] = 1500;
receiverCommand[ZAXIS] = 1500;
receiverCommand[THROTTLE] = 1000;
receiverCommand[MODE] = 1000;
receiverCommand[AUX1] = 1000;
receiverCommand[AUX2] = 1000;
receiverCommand[AUX3] = 1000;
receiverCommand[AUX4] = 1000;
receiverCommand[AUX5] = 1000;
*/

void Bluetooth_Read() {
  if (BTfd) {
    int rd = 0;
    int i = 0;
    rd = read(BTfd, BTev, sizeof(struct input_event) * 64);
    if (rd < (int) sizeof(struct input_event)) {
      // try again?
    } else {
      for (i = 0; i < rd / sizeof(struct input_event); i++) {

        if (BTev[i].type == TYPE_RELATIVE) {
          switch (BTev[i].code) {
            case REL_LEFT_THUMB_X:
              receiverCommand[XAXIS]
                = map(BTev[i].value, 0, 255, 1000, 2000);
              break;

            case REL_LEFT_TRIG2:
              receiverCommand[THROTTLE]
                = map(BTev[i].value, 0, 255, 1000, 2000);
              break;

            case REL_RIGHT_THUMB_X:
              receiverCommand[ZAXIS]
                = map(BTev[i].value, 0, 255, 1000, 2000);
              break;

            case REL_RIGHT_THUMB_Y:
              receiverCommand[YAXIS]
                = map(BTev[i].value, 0, 255, 1000, 2000);
              break;
          }
        } else if (BTev[i].type = TYPE_BUTTON) {
          switch (BTev[i].code) {
            case BTN_CROSS:
              // BTReceiver.arm = (bool)ev[i].value;
              break;

            case BTN_TRIANGLE:
              // BTReceiver.disarm = (bool)ev[i].value;
              break;

            case BTN_CIRCLE:
              receiverCommand[AUX1]
                = map(BTev[i].value, 0, 255, 1000, 2000);
              break;

            case BTN_SQUARE:
              receiverCommand[MODE]
                = map(BTev[i].value, 0, 255, 1000, 2000);
              break;
          }
        }
      }
    }
  }
}

void initializeReceiver(int nbChannel = 6) {

  if (Bluetooth_Open()) {
    Serial.println("[BT Receiver] Ready.");
  } else {
    Serial.println("[BT Receiver] Error during init.");
  }

}

int getRawChannelValue(byte channel) {

  return receiverCommand[channel];
}

#endif
