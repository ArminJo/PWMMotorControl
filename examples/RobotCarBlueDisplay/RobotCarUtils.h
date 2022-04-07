/*
 * RobotCarUtils.h
 *
 *  Contains miscellaneous convenience utility functions for the robot cars.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _ROBOT_CAR_UTILS_H
#define _ROBOT_CAR_UTILS_H

#include <Arduino.h>

void printConfigInfo();
void initRobotCarPWMMotorControl();
unsigned int getDistanceAndPlayTone();
void checkVinPeriodicallyAndPrintIfChanged();

#endif // _ROBOT_CAR_UTILS_H
#pragma once
