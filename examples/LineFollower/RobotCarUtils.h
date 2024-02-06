/*
 * RobotCarUtils.h
 *
 *  Contains miscellaneous convenience utility functions for the robot cars.
 *
 *  Copyright (C) 2022-2024  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _ROBOT_CAR_UTILS_H
#define _ROBOT_CAR_UTILS_H

#include <Arduino.h>

#include "RobotCarPinDefinitionsAndMore.h" // This is not necessary here, but helps the Eclipse indexer :-(

void printConfigInfo(Print *aSerial);
void printProgramOptions(Print *aSerial);
void printConfigPinInfo(Print *aSerial, uint8_t aConfigPinNumber, const __FlashStringHelper *aConfigPinDescription);

void initRobotCarPWMMotorControl();

#if defined(VIN_ATTENUATED_INPUT_PIN)
bool isVINProvided();
extern uint16_t sLastVINRawSum;   // Sum of NUMBER_OF_VIN_SAMPLES raw readings of ADC
extern float sVINVoltage;
bool readVINVoltage();
void readVINVoltageAndAdjustDriveSpeedAndPrint();
void calibrateDriveSpeedPWMAndPrint();
void checkVinPeriodicallyAndPrintIfChanged();
#endif

/*
 * Test movements compatible wit IR dispatcher
 */
void testDriveTwoTurnsBothDirections();
void testDriveTwoTurnsIn5PartsBothDirections();
void testRotation();

#if !defined(USE_MPU6050_IMU) && (defined(_IR_COMMAND_DISPATCHER_HPP) || defined(VERSION_BLUE_DISPLAY)) \
    && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))
bool calibrateRotation(turn_direction_t aTurnDirection);
#endif

#endif // _ROBOT_CAR_UTILS_H
