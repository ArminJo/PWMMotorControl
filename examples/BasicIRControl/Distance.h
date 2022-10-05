/*
 * Distance.h
 *
 *  Contains all distance measurement functions.
 *
 *  Copyright (C) 2016-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  PWMMotorControl and Arduino-RobotCar are free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _ROBOT_CAR_DISTANCE_H
#define _ROBOT_CAR_DISTANCE_H

#include <Arduino.h>

#include "HCSR04.h" // for DISTANCE_TIMEOUT_RESULT if Distance.h is included

#if defined(CAR_HAS_SERVO)
#  if defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
#include "LightweightServo.h" // We do not have a Servo object here
#  else
#    if defined(ESP32)
#include <ESP32Servo.h>
#    else
#include <Servo.h>
#    endif
#    if defined(CAR_HAS_DISTANCE_SERVO)
extern Servo DistanceServo;     // The pan servo instance for distance sensor
#    endif
#  endif
#endif // defined(CAR_HAS_SERVO)


/*
 * Default values for the behavior of the follower
 */
#if !defined(FOLLOWER_DISTANCE_MINIMUM_CENTIMETER)
#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER        22 // If measured distance is less than this value, go backwards
#endif
#if !defined(FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER)
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER        30 // If measured distance is greater than this value, go forward
#endif
#if !defined(FOLLOWER_DISTANCE_TIMEOUT_CENTIMETER)
#define FOLLOWER_DISTANCE_TIMEOUT_CENTIMETER        70 // Do not accept target with distance greater than this value
#endif
#define FOLLOWER_DISTANCE_DELTA_CENTIMETER          (FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER - FOLLOWER_DISTANCE_MINIMUM_CENTIMETER)

#include "PWMDcMotor.h" // for DIRECTION_STOP etc.
typedef enum distance_range {
    DISTANCE_OK = DIRECTION_STOP, // == TURN_IN_PLACE | 0
    DISTANCE_TO_SMALL = DIRECTION_FORWARD, // == TURN_FORWARD | 1
    DISTANCE_TO_GREAT = DIRECTION_BACKWARD, // == TURN_BACKWARD | 2
    DISTANCE_TIMEOUT = 3
} distance_range_t;
distance_range_t getDistanceRange(uint8_t aCentimeter);
extern const char RangeCharacterArray[];
void printDistanceRangeCharacter(distance_range_t aRange, Print *aSerial);

/*
 * Constants for uint8_t sDistanceFeedbackMode
 * using enum costs 100 bytes of program memory :-(
 */
//typedef enum distance_feedback_mode {
//    DISTANCE_FEEDBACK_NO_TONE,
//    DISTANCE_FEEDBACK_PENTATONIC,
//    DISTANCE_FEEDBACK_CONTINUOUSLY
//} distance_feedback_mode_t;
extern uint8_t sDistanceFeedbackMode;

#define DISTANCE_FEEDBACK_NO_TONE       0
#define DISTANCE_FEEDBACK_PENTATONIC    1
#define DISTANCE_FEEDBACK_CONTINUOUSLY  2
#define DISTANCE_FEEDBACK_MAX           DISTANCE_FEEDBACK_CONTINUOUSLY

/*
 * Different result types acquired at one scan
 */
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#define DISTANCE_SOURCE_MODE_MINIMUM    0 // Take the minimum of the US and IR or TOF values
#define DISTANCE_SOURCE_MODE_MAXIMUM    1
#define DISTANCE_SOURCE_MODE_US         2 // Take just US value
#define DISTANCE_SOURCE_MODE_IR_OR_TOF  3 // Take just IR or TOF value
#define DISTANCE_LAST_SOURCE_MODE       DISTANCE_SOURCE_MODE_IR_OR_TOF
#if !defined(DISTANCE_SOURCE_MODE_DEFAULT)
//#define DISTANCE_SOURCE_MODE_DEFAULT    DISTANCE_SOURCE_MODE_US
#define DISTANCE_SOURCE_MODE_DEFAULT    DISTANCE_SOURCE_MODE_IR_OR_TOF
#endif
extern uint8_t sDistanceSourceMode;
#endif

/*
 * Constants for fillAndShowForwardDistancesInfo(), doWallDetection etc.
 */
#define NUMBER_OF_DISTANCES             10
#define DEGREES_PER_STEP                18
#define DEGREES_PER_TARGET_SCAN_STEP    20  // 20 -> sinus is 0.34. At a distance of 60 cm this is 20 cm; 10 degree -> 10 cm
#define STEPS_PER_SCAN      (NUMBER_OF_DISTANCES - 1) // -> 162 degrees for 18 DEGREES_PER_STEP, 153 for 17 degrees
#define START_DEGREES       ((180 - (DEGREES_PER_STEP * STEPS_PER_SCAN)) / 2) // 9 for 18, 13,5 for 17 - we need it symmetrical in the 180 degrees range

#define DISTANCE_TIMEOUT_CM                     200 // do not measure distances greater than 200 cm
#define DISTANCE_TIMEOUT_CM_FOLLOWER            150 // do not measure and process distances greater than 150 cm
#define DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE    100 // do not measure and process distances greater than 100 cm

#define DISTANCE_MAX_FOR_WALL_DETECTION_CM      40

#define MINIMUM_DISTANCE_TOO_SMALL 360 // possible result of doBuiltInCollisionAvoiding()

#define DISTANCE_TIMEOUT_COLOR COLOR16_CYAN

/*
 * Index definitions for ForwardDistancesInfoStruct
 */
#define INDEX_TARGET_RIGHT      0
#define INDEX_TARGET_FORWARD    1
#define INDEX_TARGET_LEFT       2
#define INDEX_RIGHT             0
#define INDEX_LEFT              STEPS_PER_SCAN
#if (STEPS_PER_SCAN == 9)
// Works only for STEPS_PER_SCAN = 9
#define INDEX_FORWARD_1         4
#define INDEX_FORWARD_2         5
#endif
#define INVALID_DEGREE   127 // To mark non valid DegreeOfDistanceGreaterThanThreshold or DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold in ForwardDistancesInfoStruct

struct ForwardDistancesInfoStruct {
    uint8_t RawDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degrees
    uint8_t ProcessedDistancesArray[NUMBER_OF_DISTANCES]; // From 0 (right) to 180 degrees (left) with steps of 20 degrees, invalid if ProcessedDistancesArray[0] == 0
    int8_t DegreeOfDistanceGreaterThanThreshold;
    int8_t DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold;
    uint8_t MaxDistance;    // Maximum of all distances (index 0 to 9)
    int8_t DegreeOfMaxDistance;
    uint8_t MinDistance;    // Minimum of distances of index 1 to 8, not using values at 0 and 9
    int8_t DegreeOfMinDistance; // do not take first (0) and last index for minimum (we may measure the distance to our wheel there)
    // 0 degree => wall parallel to side of car. 90 degrees => wall in front of car. degrees of wall -> degrees to turn.
    int8_t WallRightAngleDegrees;
    int8_t WallLeftAngleDegrees;
//    uint8_t WallRightDistance;
//    uint8_t WallLeftDistance;
};
extern ForwardDistancesInfoStruct sForwardDistancesInfo;
extern uint8_t sRawForwardDistancesArray[3];

extern uint8_t sUSDistanceTimeoutCentimeter;
extern uint8_t sIROrTofDistanceCentimeter;

#if defined(CAR_HAS_DISTANCE_SERVO)
extern bool sDoSlowScan;
extern uint8_t sLastDistanceServoAngleInDegrees; // needed for optimized delay for servo repositioning
#endif

extern int sLastDecisionDegreesToTurnForDisplay;
extern int sNextRotationDegree;
extern int sLastDegreesTurned;

void initDistance();
void printDistanceIfChanged(Print *aSerial);
void playDistanceFeedbackTone(uint8_t aCentimeter);
unsigned int getDistanceAsCentimeter(uint8_t aDistanceTimeoutCentimeter = DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE,
        bool aWaitForCurrentMeasurementToEnd = false, uint8_t aMinimumUSDistanceForMinimumMode = 0);

#if defined(CAR_HAS_DISTANCE_SERVO)
#define NO_TARGET_FOUND     360     // return value of scanTarget()
int8_t scanTarget(uint8_t aMaximumTargetDistance);
void printForwardDistanceInfo(Print *aSerial);
void DistanceServoWriteAndWaitForStop(uint8_t aValue, bool doDelay = false);
bool fillAndShowForwardDistancesInfo(bool aDoFirstValue, bool aForceScan = false);
void doWallDetection();
void postProcessDistances(uint8_t aDistanceThreshold);
#define IndexToDegree(aIndex) (((aIndex * DEGREES_PER_STEP) + START_DEGREES) - 90) // generates smaller code than a function
#endif

int doBuiltInCollisionAvoiding();

#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
uint8_t getIRDistanceAsCentimeter(bool aWaitForCurrentMeasurementToEnd = false);
#endif

#if defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#include "vl53l1x_class.h"
extern VL53L1X sToFDistanceSensor;
uint8_t getToFDistanceAsCentimeter();
uint8_t readToFDistanceAsCentimeter(); // no start of measurement, just read result.
#endif

#endif // _ROBOT_CAR_DISTANCE_H
