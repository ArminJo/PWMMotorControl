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

bool isVINVoltageDividerAttached(uint8_t aPin);
void printConfigInfo(Print *aSerial);
void printProgramOptions(Print *aSerial);
void initRobotCarPWMMotorControl();
unsigned int getDistanceAndPlayTone();

// for MONITOR_VIN_VOLTAGE
extern uint16_t sLastVINRawSum;   // Sum of NUMBER_OF_VIN_SAMPLES raw readings of ADC
extern float sVINVoltage;
bool readVINVoltage();
void readVINVoltageAndAdjustDriveSpeedAndPrint();
void calibrateDriveSpeedPWMAndPrint();
#if (defined(USE_IR_REMOTE) || defined(ROBOT_CAR_BLUE_DISPLAY)) && !defined(USE_MPU6050_IMU) \
    && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))
bool calibrateRotation(turn_direction_t aTurnDirection);
#endif
void checkVinPeriodicallyAndPrintIfChanged();

#endif // _ROBOT_CAR_UTILS_H
