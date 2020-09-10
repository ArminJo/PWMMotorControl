/*
 * AutonomousDrive.cpp
 *
 * Contains:
 * fillForwardDistancesInfoPro(): Acquisition of 180 degrees distances by ultrasonic sensor and servo
 * doWallDetection(): Enhancement of acquired data because of lack of detecting flat surfaces by US at angels out of 70 to 110 degree.
 * doBuiltInCollisionDetection(): decision where to turn in dependency of the acquired distances.
 * driveAutonomousOneStep(): The loop which handles the start/stop, single step and path output functionality.
 *
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
 *
 */

#include "AutonomousDrive.h"

#include "RobotCar.h"
#include "RobotCarGui.h"

#include "HCSR04.h"
#include "Distance.h"

uint8_t sDriveMode = MODE_MANUAL_DRIVE; // one of MODE_MANUAL_DRIVE, MODE_AUTONOMOUS_DRIVE_BUILTIN, MODE_AUTONOMOUS_DRIVE_USER or MODE_FOLLOWER

uint8_t sStepMode = MODE_CONTINUOUS; // one ofMODE_CONTINUOUS,  MODE_STEP_TO_NEXT_TURN or MODE_SINGLE_STEP
bool sDoStep = false; // if true => do one step
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
uint8_t sScanMode = SCAN_MODE_MINIMUM; // one of SCAN_MODE_MINIMUM, SCAN_MODE_MAXIMUM, SCAN_MODE_US or SCAN_MODE_IR
#endif

bool sDoSlowScan = false;
bool sRuningAutonomousDrive = false; // = (sDriveMode == MODE_AUTONOMOUS_DRIVE_BUILTIN || sDriveMode == MODE_AUTONOMOUS_DRIVE_USER || sDriveMode == MODE_FOLLOWER) is modified by buttons on this page

bool sSearchFollowerTarget = true;

uint8_t sTurnMode = TURN_IN_PLACE;

// Storage for turning decision especially for single step mode
int sNextDegreesToTurn = 0;
// Storage of last turning for insertToPath()
int sLastDegreesTurned = 0;

#define CENTIMETER_PER_RIDE 20
uint8_t sCentimeterPerScanTimesTwo = CENTIMETER_PER_RIDE * 2; // = encoder counts per US scan in autonomous mode
uint8_t sCentimeterPerScan = CENTIMETER_PER_RIDE;

/*
 * aDriveMode required if aDoStart is true otherwise sDriveMode = MODE_MANUAL_DRIVE;
 */
void startStopAutomomousDrive(bool aDoStart, uint8_t aDriveMode) {
    sRuningAutonomousDrive = aDoStart;
    noTone(PIN_SPEAKER); // for follower mode

    if (aDoStart) {
        /*
         * Start autonomous driving.
         */
        resetPathData();
        clearPrintedForwardDistancesInfos();
        sDoStep = true; // enable next step
        sDriveMode = aDriveMode;
        // decide which button called us
        if (aDriveMode == MODE_AUTONOMOUS_DRIVE_USER) {
            /*
             * Own test always starts in mode SINGLE_STEP
             */
            setStepMode(MODE_SINGLE_STEP);
        } else if (aDriveMode == MODE_FOLLOWER) {
            DistanceServoWriteAndDelay(90); // reset Servo
            SliderUSDistance.drawSlider();
            sSearchFollowerTarget = true;
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
            SliderIRDistance.drawSlider();
#  endif
        }

    } else {
        /*
         * Stop autonomous driving.
         */
#ifdef USE_ENCODER_MOTOR_CONTROL
        if (sStepMode != MODE_SINGLE_STEP) {
            // add last driven distance to path
            insertToPath(rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
        }
#endif
        DistanceServoWriteAndDelay(90);
        sDriveMode = MODE_MANUAL_DRIVE;
    }
    handleAutomomousDriveRadioButtons();
    startStopRobotCar(aDoStart);
}

int postProcessAndCollisionDetection() {
    doWallDetection();
    postProcessDistances();
    int tNextDegreesToTurn;
    if (sDriveMode == MODE_AUTONOMOUS_DRIVE_BUILTIN) {
        tNextDegreesToTurn = doBuiltInCollisionDetection();
    } else {
        tNextDegreesToTurn = doUserCollisionDetection();
    }
    drawCollisionDecision(tNextDegreesToTurn, sCentimeterPerScan, false);
    return tNextDegreesToTurn;
}

/*
 * Do one step of autonomous driving, called by main loop.
 * Compute sNextDegreesToTurn AFTER the movement to be able to stop before next turn
 */
void driveAutonomousOneStep() {

    bool tCarIsStopped = RobotCarMotorControl.isStopped();
    /*
     * Check step conditions if step should happen
     */
    if (sDoStep || sStepMode == MODE_CONTINUOUS || (sStepMode == MODE_STEP_TO_NEXT_TURN && (!tCarIsStopped))) {
        /*
         * Do one step
         */
        bool tMovementJustStarted = sDoStep || tCarIsStopped; // tMovementJustStarted is needed for speeding up US scanning by skipping first scan angle if not just started.
        sDoStep = false; // Now it can be set again by GUI

        if (sNextDegreesToTurn != 0) {
            /*
             * rotate car / go backward accordingly to sNextDegreesToTurn
             */
            if (sNextDegreesToTurn == GO_BACK_AND_SCAN_AGAIN) {
                // go backwards and do a new scan
                RobotCarMotorControl.goDistanceCentimeter(10, DIRECTION_BACKWARD, &loopGUI);
            } else {
                // rotate and go
                RobotCarMotorControl.rotateCar(sNextDegreesToTurn, sTurnMode);
                // wait to really stop after turning
                delay(100);
                sLastDegreesTurned = sNextDegreesToTurn;
            }
        }
        if (sNextDegreesToTurn != GO_BACK_AND_SCAN_AGAIN) {
            /*
             * No rotation or standard rotation here. Go fixed distance or keep moving
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                // Go fixed distance
                RobotCarMotorControl.goDistanceCentimeter(sCentimeterPerScan, DIRECTION_FORWARD, &loopGUI);
            } else
            /*
             * Continuous mode, start car or let it run
             */
            if (tCarIsStopped) {
                RobotCarMotorControl.startCarAndWaitForDriveSpeed();
            }
        }

        /*
         * Here car is moving
         */
#ifdef USE_ENCODER_MOTOR_CONTROL
        uint16_t tStepStartDistanceCount = rightCarMotor.EncoderCount; // get count before distance scanning
#endif
        bool tCurrentPageIsAutomaticControl = (sCurrentPage == PAGE_AUTOMATIC_CONTROL);

        /*
         * The magic happens HERE
         * This runs as fast as possible and mainly determine the duration of one step
         */
        if (fillAndShowForwardDistancesInfo(tMovementJustStarted)) {
            // User canceled autonomous drive, ForwardDistancesInfo may be incomplete then
            return;
        }

        // Clear old decision marker by redrawing it with a white line
        drawCollisionDecision(sNextDegreesToTurn, sCentimeterPerScan, true);
        sNextDegreesToTurn = postProcessAndCollisionDetection();

        /*
         * compute distance driven for one US scan
         * First check if car is stopped
         */
        if (!RobotCarMotorControl.isStopped() && sStepMode != MODE_SINGLE_STEP) {
            /*
             * No stop here => distance is valid
             */
#ifdef USE_ENCODER_MOTOR_CONTROL
            sCentimeterPerScanTimesTwo = rightCarMotor.EncoderCount - tStepStartDistanceCount;
            sCentimeterPerScan = sCentimeterPerScanTimesTwo / 2;
#endif
            if (tCurrentPageIsAutomaticControl) {
                char tStringBuffer[6];
                sprintf_P(tStringBuffer, PSTR("%2d%s"), sCentimeterPerScan, "cm");
                BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tStringBuffer, TEXT_SIZE_11,
                COLOR_BLACK, COLOR_WHITE);
            }
        }

        /*
         * Handle stop of car and path data
         */
        if ((sNextDegreesToTurn != 0 && sStepMode == MODE_STEP_TO_NEXT_TURN) || sStepMode == MODE_SINGLE_STEP) {
            /*
             * Stop if rotation requested or single step
             */
            RobotCarMotorControl.stopCarAndWaitForIt();
#ifdef USE_ENCODER_MOTOR_CONTROL
            /*
             * Insert / update last ride in path
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                insertToPath(CENTIMETER_PER_RIDE * 2, sLastDegreesTurned, true);
            } else {
                // add last driven distance to path
                insertToPath(rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
            }
        } else {
            /*
             * No stop, just continue => overwrite last path element with current riding distance and try to synchronize motors
             */
            insertToPath(rightCarMotor.EncoderCount, sLastDegreesTurned, false);
            rightCarMotor.synchronizeMotor(&leftCarMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
#endif
        }

        if (sCurrentPage == PAGE_SHOW_PATH) {
            drawPathInfoPage();
        }
    }
}

/*
 * This documentation assumes 20 degrees stepping.
 * By changing DEGREES_PER_STEP it can easily adopted to other degrees values.
 *
 * Assume the value of 20 and 40 degrees are distances to a wall.
 * @return the clipped (aClipValue) distance to the wall of the vector at 0 degree.
 *
 * @param [out] aDegreeOfEndpointConnectingLine: The angle of the line from endpoint 0 degrees to given endpoints
 * 0 means x values of given endpoints are the same >= wall is parallel / rectangular to the vector at 0 degree
 * Positive means wall is more ore less in front, to avoid we must turn positive angle
 * 90 means y values are the same =>  wall is in front
 * Negative means we are heading away from wall
 */
uint8_t computeNeigbourValue(uint8_t aDegreesPerStepValue, uint8_t a2DegreesPerStepValue, uint8_t aClipValue,
        int8_t * aDegreeOfEndpointConnectingLine) {

    /*
     * Name of the variables are for DEGREES_PER_STEP = 20 only for better understanding.
     * The computation of course works for other values of DEGREES_PER_STEP!
     */
    float tYat20degrees = sin((PI / 180) * DEGREES_PER_STEP) * aDegreesPerStepValue; // e.g. 20 Degree
// assume current = 40 Degree
    float tYat40degrees = sin((PI / 180) * (DEGREES_PER_STEP * 2)) * a2DegreesPerStepValue; // e.g. 40 Degree

//    char tStringBuffer[] = "A=_______ L=_______";
//    dtostrf(tYat40degrees, 7, 2, &tStringBuffer[2]);
//    tStringBuffer[9] = ' ';
//    dtostrf(tYat20degrees, 7, 2, &tStringBuffer[12]);
//    BlueDisplay1.debugMessage(tStringBuffer);

    uint8_t tDistanceAtZeroDegree = aClipValue;

    /*
     * if tY40degrees == tY20degrees the tInvGradient is infinite (distance at 0 is infinite)
     */
    if (tYat40degrees > tYat20degrees) {
        float tXat20degrees = cos((PI / 180) * DEGREES_PER_STEP) * aDegreesPerStepValue; // 20 Degree
        float tXat40degrees = cos((PI / 180) * (DEGREES_PER_STEP * 2)) * a2DegreesPerStepValue; // 40 Degree

//        dtostrf(tXat40degrees, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXat20degrees, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        /*  Here the graphic for 90 and 60 degrees (since we have no ASCII graphic symbols for 20 and 40 degree)
         *  In this function we have e.g. 40 and 20 degrees and compute 0 degrees!
         *      90 degrees value
         *      |\ \==wall
         *      | \  60 degrees value
         *      | /\
 *      |/__\ 0 degrees value to be computed
         */

        /*
         * InvGradient line represents the wall
         * if tXat20degrees > tXat40degrees InvGradient is negative => X0 value is bigger than X20 one / right wall is in front if we look in 90 degrees direction
         * if tXat20degrees == tXat40degrees InvGradient is 0 / wall is parallel right / 0 degree
         * if tXat20degrees < tXat40degrees InvGradient is positive / right wall is behind / degrees is negative (from direction front which is 90 degrees)
         */
        float tInvGradient = (tXat40degrees - tXat20degrees) / (tYat40degrees - tYat20degrees);
        float tXatZeroDegree = tXat20degrees - (tInvGradient * tYat20degrees);
        *aDegreeOfEndpointConnectingLine = -(atan(tInvGradient) * RAD_TO_DEG);
//        tStringBuffer[0] = 'G';
//        tStringBuffer[10] = 'B';
//        dtostrf(tInvGradient, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXZeroDegree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        if (tXatZeroDegree < 255) {
            tDistanceAtZeroDegree = tXatZeroDegree + 0.5;
            if (tDistanceAtZeroDegree > aClipValue) {
                tDistanceAtZeroDegree = aClipValue;
            }
        }
    }
    return tDistanceAtZeroDegree;
}

/*
 * The problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
 * For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.
 *
 * Therefore I take samples every 18 degrees and if I get 2 adjacent short (< sCentimeterPerScanTimesTwo) distances, I assume a wall determined by these 2 samples.
 * The (invalid) values 18 degrees right and left of these samples are then extrapolated by computeNeigbourValue().
 */
//#define TRACE // only used for this function
void doWallDetection() {
    uint8_t tTempDistancesArray[NUMBER_OF_DISTANCES];
    /*
     * First copy all raw values
     */
    memcpy(tTempDistancesArray, sForwardDistancesInfo.RawDistancesArray, NUMBER_OF_DISTANCES);

    uint8_t tCurrentAngleToCheck = START_DEGREES + (2 * DEGREES_PER_STEP); // first angle to adjust at index 2
    int8_t tDegreeOfConnectingLine;
    sForwardDistancesInfo.WallRightAngleDegrees = 0;
    sForwardDistancesInfo.WallLeftAngleDegrees = 0;

    /*
     * Parse the array from 0 to STEPS_PER_SCAN
     * Check values at i and i-1 and adjust value at i+1
     * i is index of CurrentValue
     */
    uint8_t tLastDistance = tTempDistancesArray[0];
    uint8_t tCurrentDistance = tTempDistancesArray[1];
    for (uint8_t i = 1; i < STEPS_PER_SCAN; ++i) {
        uint8_t tNextDistanceOriginal = tTempDistancesArray[i + 1];
        if (tLastDistance < sCentimeterPerScanTimesTwo && tCurrentDistance < sCentimeterPerScanTimesTwo) {
            /*
             * 2 adjacent short distances -> assume a wall -> adjust adjacent values
             */

            /*
             * Use computeNeigbourValue the other way round
             * i.e. put 20 degrees to 40 degrees parameter and vice versa in order to use the 0 degree value as the 60 degrees one
             */
            uint8_t tNextDistanceComputed = computeNeigbourValue(tCurrentDistance, tLastDistance, DISTANCE_TIMEOUT_CM,
                    &tDegreeOfConnectingLine);
#ifdef TRACE
            BlueDisplay1.debug("i=", i);
            BlueDisplay1.debug("AngleToCheck @i+1=", tCurrentAngleToCheck);
            BlueDisplay1.debug("Original distance @i+1=", tNextDistanceOriginal);
            BlueDisplay1.debug("New distance @i+1=", tNextDistanceComputed);
            BlueDisplay1.debug("Connecting degrees @i+1=", tDegreeOfConnectingLine);
#endif

            if (tNextDistanceOriginal > tNextDistanceComputed + 5) {
                /*
                 * Adjust and draw next value if computed value is less than original value - 5
                 *
                 * Start with i=1 and adjust for 2 at (2 * DEGREES_PER_STEP) + START_DEGREES.
                 * Since we use computeNeigbourValue the other way round, we must change sign of tDegreeOfConnectingLine!
                 * The formula is 90 - (180->sum of degrees in triangle - tCurrentAngleToCheck - (90 - tDegreeOfConnectingLine)->since we use it the other way round)
                 * Which leads to -90 + tCurrentAngleToCheck + 90 - tDegreeOfConnectingLine.
                 * If we then get a tDegreeOfConnectingLine value of 0 we have a wall at right rectangular to the vector at 2,
                 * Negative raw values means the wall is more in front / the the wall angle is greater,
                 */
                int tDegreeOfWallAngle = tCurrentAngleToCheck - tDegreeOfConnectingLine;
#ifdef TRACE
                BlueDisplay1.debug("tDegreeOfWallAngle=", tDegreeOfWallAngle);
#endif
                if (tDegreeOfWallAngle <= 90) {
                    // wall at right
                    sForwardDistancesInfo.WallRightAngleDegrees = tDegreeOfWallAngle;
                } else {
                    // wall at left
                    sForwardDistancesInfo.WallLeftAngleDegrees = 180 - tDegreeOfWallAngle;
                }

                // store and draw adjusted value
                tTempDistancesArray[i + 1] = tNextDistanceComputed;
                tNextDistanceOriginal = tNextDistanceComputed;
                if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
                    BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextDistanceComputed,
                            tCurrentAngleToCheck, COLOR_WHITE, 1);
                }
            }
        }
        tLastDistance = tCurrentDistance;
        tCurrentDistance = tNextDistanceOriginal;
        tCurrentAngleToCheck += DEGREES_PER_STEP;
    }

    /*
     * Go backwards through the array
     */
    memcpy(sForwardDistancesInfo.ProcessedDistancesArray, tTempDistancesArray, NUMBER_OF_DISTANCES);

    tLastDistance = tTempDistancesArray[STEPS_PER_SCAN];
    tCurrentDistance = tTempDistancesArray[STEPS_PER_SCAN - 1];
    tCurrentAngleToCheck = 180 - (START_DEGREES + (2 * DEGREES_PER_STEP));

    /*
     * check values at i and i+1 and adjust value at i-1
     */
    for (uint8_t i = STEPS_PER_SCAN - 1; i > 0; --i) {
        uint8_t tNextValue = tTempDistancesArray[i - 1];

// Do it only if none of the 3 values are processed before
        if (tTempDistancesArray[i + 1] == sForwardDistancesInfo.RawDistancesArray[i + 1]
                && tTempDistancesArray[i] == sForwardDistancesInfo.RawDistancesArray[i]
                && tNextValue == sForwardDistancesInfo.RawDistancesArray[i - 1]) {

            /*
             * check values at i+1 and i and adjust value at i-1
             */
            if (tLastDistance < sCentimeterPerScanTimesTwo && tCurrentDistance < sCentimeterPerScanTimesTwo) {
                /*
                 * Wall detected -> adjust adjacent values
                 * Use computeNeigbourValue in the intended way, so do not change sign of tDegreeOfConnectingLine!
                 */
                uint8_t tNextValueComputed = computeNeigbourValue(tCurrentDistance, tLastDistance, DISTANCE_TIMEOUT_CM,
                        &tDegreeOfConnectingLine);
#ifdef TRACE
                BlueDisplay1.debug("i=", i);
                BlueDisplay1.debug("AngleToCheck @i+1=", tCurrentAngleToCheck);
                BlueDisplay1.debug("Original distance @i-1=", tNextValue);
                BlueDisplay1.debug("New distance @i-1=", tNextValueComputed);
                BlueDisplay1.debug("Connecting degrees @i-1=", tDegreeOfConnectingLine);
#endif
                if (tNextValue > tNextValueComputed + 5) {
                    // start with i = 8 and adjust for 7
                    // degrees at index i-1 are ((i - 1) * DEGREES_PER_STEP) + START_DEGREES
                    int tWallBackwardDegrees = tCurrentAngleToCheck + tDegreeOfConnectingLine;
#ifdef TRACE
                    BlueDisplay1.debug("tWallBackwardDegrees=", tWallBackwardDegrees);
#endif
                    if (tWallBackwardDegrees <= 90) {
                        // wall at right - overwrite only if greater
                        if (sForwardDistancesInfo.WallRightAngleDegrees < tWallBackwardDegrees) {
                            sForwardDistancesInfo.WallRightAngleDegrees = tWallBackwardDegrees;
#ifdef TRACE
                            BlueDisplay1.debug("WallRightAngleDegrees=", sForwardDistancesInfo.WallRightAngleDegrees);
#endif
                        }
                    } else if (sForwardDistancesInfo.WallLeftAngleDegrees < (180 - tWallBackwardDegrees)) {
                        // wall at right - overwrite only if greater
                        sForwardDistancesInfo.WallLeftAngleDegrees = 180 - tWallBackwardDegrees;
#ifdef TRACE
                        BlueDisplay1.debug("WallLeftAngleDegrees=", sForwardDistancesInfo.WallLeftAngleDegrees);
#endif

                    }
                    //Adjust and draw next value if original value is greater
                    sForwardDistancesInfo.ProcessedDistancesArray[i - 1] = tNextValueComputed;
                    tNextValue = tNextValueComputed;
                    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
                        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
                                tCurrentAngleToCheck, COLOR_WHITE, 1);
                    }
                }
            }
        }
        tLastDistance = tCurrentDistance;
        tCurrentDistance = tNextValue;
        tCurrentAngleToCheck -= DEGREES_PER_STEP;

    }
}

/*
 * Checks distances and returns degrees to turn
 * 0 -> no turn, > 0 -> turn left, < 0 -> turn right, > 360 go back, since too close to wall
 */
int doBuiltInCollisionDetection() {
    int tDegreeToTurn = 0;
// 5 is too low
    if (sForwardDistancesInfo.MinDistance < 7) {
        /*
         * Min Distance too small => go back and scan again
         */
        return GO_BACK_AND_SCAN_AGAIN;
    }
    /*
     * First check if free ahead
     */
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] > sCentimeterPerScanTimesTwo
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] > sCentimeterPerScanTimesTwo) {
        /*
         * Free ahead, check if our side is near to the wall and make corrections
         */
        if (sForwardDistancesInfo.WallRightAngleDegrees != 0 || sForwardDistancesInfo.WallLeftAngleDegrees != 0) {
            /*
             * Wall detected
             */
            if (sForwardDistancesInfo.WallRightAngleDegrees > sForwardDistancesInfo.WallLeftAngleDegrees) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sForwardDistancesInfo.WallRightAngleDegrees;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sForwardDistancesInfo.WallLeftAngleDegrees;
            }

        }
    } else {
        if (sForwardDistancesInfo.WallRightAngleDegrees != 0 || sForwardDistancesInfo.WallLeftAngleDegrees != 0) {
            /*
             * Wall detected
             */
            if (sForwardDistancesInfo.WallRightAngleDegrees > sForwardDistancesInfo.WallLeftAngleDegrees) {
                /*
                 * Wall at right => turn left
                 */
                tDegreeToTurn = sForwardDistancesInfo.WallRightAngleDegrees;
            } else {
                /*
                 * Wall at left => turn right
                 */
                tDegreeToTurn = -sForwardDistancesInfo.WallLeftAngleDegrees;
            }
        } else {
            /*
             * Not free ahead, must turn, check if another forward direction is suitable
             */
            if (sForwardDistancesInfo.MaxDistance > sCentimeterPerScanTimesTwo) {
                /*
                 * Go to max distance
                 */
                tDegreeToTurn = ((sForwardDistancesInfo.IndexOfMaxDistance * DEGREES_PER_STEP) + START_DEGREES) - 90;
            } else {
                /*
                 * Max distances are all too short => must go back / turn by 180 degree
                 */
                tDegreeToTurn = 180;
            }
        }
    }
    return tDegreeToTurn;
}

/***************************************************
 * Code for follower mode
 ***************************************************/
bool sLastFollowerTargetFoundRight;

void driveFollowerModeOneStep() {

    unsigned int tCentimeter = getDistanceAndPlayTone();

    /*
     * check if distance too high, then search target in other direction
     */
    if (!sSearchFollowerTarget && tCentimeter > FOLLOWER_RESCAN_DISTANCE) {
        // Stop and get new distance info
        RobotCarMotorControl.stopMotors();

        clearPrintedForwardDistancesInfos();
        // show current distance
        showUSDistance(tCentimeter);

        /*
         * Check 70, 90 and 110 degree for moved target
         */
        uint8_t tDegreeForSearch;
        int tDeltaDegree;
        if (sLastFollowerTargetFoundRight) {
            // Start searching at right
            tDegreeForSearch = 70;
            tDeltaDegree = 20;
        } else {
            // Start searching at left
            tDegreeForSearch = 110;
            tDeltaDegree = -20;
        }
        for (uint8_t i = 0; i < 3; ++i) {
            DistanceServoWriteAndDelay(tDegreeForSearch, true);
            tCentimeter = getDistanceAsCentiMeter();
            if (sCurrentPage == PAGE_AUTOMATIC_CONTROL && BlueDisplay1.isConnectionEstablished()) {
                /*
                 * Determine color
                 */
                color16_t tColor;
                tColor = COLOR_RED; // tCentimeter <= sCentimeterPerScan
                if (tCentimeter <= FOLLOWER_MAX_DISTANCE) {
                    tColor = COLOR_GREEN;
                } else if (tCentimeter < FOLLOWER_MIN_DISTANCE) {
                    tColor = COLOR_YELLOW;
                }
                /*
                 * Draw distance line
                 */
                BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tCentimeter, tDegreeForSearch,
                        tColor, 3);
            }
            if (tCentimeter <= FOLLOWER_RESCAN_DISTANCE) {
                break;
            }
            // prepare for next scan
            loopGUI();
            tDegreeForSearch += tDeltaDegree;
        }
        if (tDegreeForSearch <= 90) {
            sLastFollowerTargetFoundRight = true;
        }
        int8_t tDegreeToTurn = tDegreeForSearch - 90;

//        if (fillAndShowForwardDistancesInfo(true)) {
//            // User canceled autonomous drive, ForwardDistancesInfo may be incomplete then
//            return;
//        }
//        postProcessDistances();
//        int tDegreeToTurn = sForwardDistancesInfo.IndexOfMinDistance * DEGREES_PER_STEP + START_DEGREES - 90;
//        tCentimeter = sForwardDistancesInfo.MinDistance;

        // reset distance servo to 90 degree
        DistanceServoWriteAndDelay(90, false);

        if (tCentimeter <= FOLLOWER_RESCAN_DISTANCE) {
            sSearchFollowerTarget = true; // Moved target found -> turn and start new search again
            /*
             *  Draw turn vector
             */
//        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tCentimeter, tDegreeToTurn + 90,
//        COLOR_BLACK);
            // Print turn info
            sprintf_P(sStringBuffer, PSTR("rotation:%3d\xB0 min:%2dcm"), tDegreeToTurn, tCentimeter); // \xB0 is degree character
            BlueDisplay1.drawText(BUTTON_WIDTH_3_5_POS_2, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11, sStringBuffer, TEXT_SIZE_11,
            COLOR_BLACK, COLOR_WHITE);
            /*
             * Rotate car
             */
            RobotCarMotorControl.rotateCar(tDegreeToTurn, TURN_FORWARD, true);
//        RobotCarMotorControl.rotateCar(tDegreeToTurn, TURN_IN_PLACE);
        }
        return;
    }

    if (tCentimeter > FOLLOWER_MAX_DISTANCE) {
//        Serial.println(F("Go forward"));
        RobotCarMotorControl.startCarAndWaitForDriveSpeed(DIRECTION_FORWARD);

    } else if (tCentimeter < FOLLOWER_MIN_DISTANCE) {
//        Serial.println(F("Go backward"));
        RobotCarMotorControl.startCarAndWaitForDriveSpeed(DIRECTION_BACKWARD);

    } else {
        // Target found here :-)
        sSearchFollowerTarget = false;
//        Serial.println(F("Stop"));
        RobotCarMotorControl.stopMotors();
    }

    // show distance bars
    showUSDistance(tCentimeter);
    delayAndLoopGUI(40); // the IR sensor takes 39 ms for one measurement
}

unsigned int __attribute__((weak)) getDistanceAndPlayTone() {
    /*
     * Get distance; timeout is 1 meter
     */
    unsigned int tCentimeter = getDistanceAsCentiMeter();
    /*
     * play tone
     */
    int tFrequency = map(tCentimeter, 0, 100, 100, 2000);
    tone(PIN_SPEAKER, tFrequency);
    return tCentimeter;
}
