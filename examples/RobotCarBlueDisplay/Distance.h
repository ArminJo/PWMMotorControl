/*
 * RobotCar.h
 *
 *  Created on: 29.09.2016
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

#ifndef DISTANCE_H_
#define DISTANCE_H_

#include <Arduino.h>
#include <Servo.h>

/*
 * Comment this out / enable this if the distance servo is mounted head down to detect small obstacles.
 */
//#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN
extern Servo DistanceServo;

/*
 * Constants for fillAndShowForwardDistancesInfo(), doWallDetection etc.
 */
#define NUMBER_OF_DISTANCES 10
#define DEGREES_PER_STEP    18
#define STEPS_PER_SCAN      (NUMBER_OF_DISTANCES - 1) // -> 162 degrees for 18 DEGREES_PER_STEP, 153 for 17 degrees
#define START_DEGREES       ((180 - (DEGREES_PER_STEP * STEPS_PER_SCAN)) / 2) // 9 for 18, 13,5 for 17 - we need it symmetrical in the 180 degrees range

// for future use maybe
//#define SERVO_CURRENT_LOW_THRESHOLD 100
//#define SERVO_INITIAL_DELAY 5
//#define SERVO_CURRENT_LOW_MILLIS_FOR_SERVO_STOPPED 12

/*
 * Index definitions for ForwardDistancesInfoStruct
 */
#define INDEX_RIGHT 0
#define INDEX_LEFT STEPS_PER_SCAN
#if (STEPS_PER_SCAN == 9)
// Works only for STEPS_PER_SCAN = 9
#define INDEX_FORWARD_1 4
#define INDEX_FORWARD_2 5
#endif

struct ForwardDistancesInfoStruct {
    uint8_t RawDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degrees
    uint8_t ProcessedDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degrees
    uint8_t IndexOfMaxDistance;
    uint8_t IndexOfMinDistance; // do not take first (0) and last index for minimum (we may measure the distance to our wheel there)
    uint8_t IndexOfDistanceGreaterThanThreshold;
    uint8_t MaxDistance;
    uint8_t MinDistance;
    // 0 degree => wall parallel to side of car. 90 degrees => wall in front of car. degrees of wall -> degrees to turn.
    int8_t WallRightAngleDegrees;
    int8_t WallLeftAngleDegrees;
//    uint8_t WallRightDistance;
//    uint8_t WallLeftDistance;
};
extern ForwardDistancesInfoStruct sForwardDistancesInfo;

extern bool sDoSlowScan;
extern uint8_t sLastServoAngleInDegrees; // needed for optimized delay for servo repositioning

extern int sLastDecisionDegreesToTurnForDisplay;
extern int sNextDegreesToTurn;
extern int sLastDegreesTurned;

void initDistance();
void DistanceServoWriteAndDelay(uint8_t aValue, bool doDelay = false);
unsigned int getDistanceAsCentiMeter(bool doShow = false);
void scanForTarget();
bool fillAndShowForwardDistancesInfo(bool aDoFirstValue, bool aForceScan = false);
void doWallDetection();
int doBuiltInCollisionDetection();
void postProcessDistances(uint8_t aDistanceThreshold);

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
#include "vl53l1x_class.h"
extern VL53L1X sToFDistanceSensor;
uint8_t getToFDistanceAsCentimeter();
uint8_t readToFDistanceAsCentimeter(); // no start of measurement, just read result.
#define OFFSET_MILLIMETER 10 // The offset measured manually or by calibrateOffset(). Offset = RealDistance - MeasuredDistance
#endif

#ifdef CAR_HAS_IR_DISTANCE_SENSOR
uint8_t getIRDistanceAsCentimeter();
#endif

#endif //  DISTANCE_H_

#pragma once
