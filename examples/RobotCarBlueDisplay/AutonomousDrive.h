/*
 * AutonomousDrive.h
 *
 *  Created on: 08.11.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
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

#define FOLLOWER_MIN_DISTANCE_CENTIMETER     22
#define FOLLOWER_MAX_DISTANCE_CENTIMETER     30
#define FOLLOWER_DELTA_DISTANCE_CENTIMETER   (FOLLOWER_MAX_DISTANCE_CENTIMETER - FOLLOWER_MIN_DISTANCE_CENTIMETER)
#define FOLLOWER_RESCAN_DISTANCE_CENTIMETER  50 // search if target moved to side

/*
 * Different result types acquired at one scan
 */
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#define SCAN_MODE_MINIMUM   0
#define SCAN_MODE_MAXIMUM   1
#define SCAN_MODE_US        2
#define SCAN_MODE_IR        3
extern uint8_t sScanMode;
#endif

#define GO_BACK_AND_SCAN_AGAIN 360 // possible result of doBuiltInCollisionDetection()
#define SCAN_AGAIN 360 // possible result of scanForTarget()

/*
 * Used for adaptive collision detection
 */
extern uint8_t sCentimeterPerScanTimesTwo; // Statistics
extern uint8_t sCentimeterPerScan; // = sCentimeterPerScanTimesTwo / 2

int postProcessAndCollisionDetection();

unsigned int getDistanceAndPlayTone();

void startStopAutomomousDrive(bool aDoStart, uint8_t aDriveMode = MODE_MANUAL_DRIVE);
void driveAutonomousOneStep();
void driveFollowerModeOneStep();

#endif /* SRC_AUTONOMOUSDRIVE_H_ */

#pragma once
