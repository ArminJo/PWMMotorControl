/*
 * AutonomousDrive.h
 *
 *  Copyright (C) 2016-2022  Armin Joachimsmeyer
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef SRC_AUTONOMOUSDRIVE_H_
#define SRC_AUTONOMOUSDRIVE_H_

#include <stdint.h>

/*
 * Different autonomous driving modes
 */
#define MODE_MANUAL_DRIVE               0
#define MODE_AUTONOMOUS_DRIVE_BUILTIN   1
#define MODE_AUTONOMOUS_DRIVE_USER      2
#define MODE_FOLLOWER                   3
extern uint8_t sDriveMode;

/*
 * Step modes for MODE_AUTONOMOUS_DRIVE
 */
#define MODE_CONTINUOUS         0
#define MODE_STEP_TO_NEXT_TURN  1 // stop before a turn
#define MODE_SINGLE_STEP        2 // stop after CENTIMETER_PER_RIDE_2
extern uint8_t sStepMode;
extern bool sDoStep;

#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER      22
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER      30
#define FOLLOWER_DISTANCE_DELTA_CENTIMETER   (FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER - FOLLOWER_DISTANCE_MINIMUM_CENTIMETER)
#define FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER  70 // search if target moved to side

/*
 * Used for adaptive collision detection
 */
extern uint8_t sCentimeterPerScanTimesTwo; // Statistics
extern uint8_t sCentimeterPerScan; // = sCentimeterPerScanTimesTwo / 2

int postProcessAndCollisionDetection();

void startStopAutomomousDrive(bool aDoStart, uint8_t aDriveMode = MODE_MANUAL_DRIVE);
void driveAutonomousOneStep();
void driveFollowerModeOneStep();

#endif /* SRC_AUTONOMOUSDRIVE_H_ */
#pragma once
