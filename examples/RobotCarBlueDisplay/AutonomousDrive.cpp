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

bool sFollowerTargetFound = false; // One time flag to start with scanning for target.

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
            sFollowerTargetFound = false;
            // Show distance sliders
            SliderUSDistance.drawSlider();
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
            insertToPath(RobotCarMotorControl.rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
        }
#endif
        DistanceServoWriteAndDelay(90);
        sDriveMode = MODE_MANUAL_DRIVE;
        RobotCarMotorControl.stopMotors(MOTOR_RELEASE);
    }

    // manage on off buttons
    handleAutomomousDriveRadioButtons();
    TouchButtonRobotCarStartStop.setValue(aDoStart, false);
}

int postProcessAndCollisionDetection() {
    doWallDetection();
    postProcessDistances(sCentimeterPerScanTimesTwo);
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
                RobotCarMotorControl.rotateCar(sNextDegreesToTurn, sTurnMode, true, &loopGUI);
//                RobotCarMotorControl.rotateCar(sNextDegreesToTurn, TURN_FORWARD, true, &loopGUI);
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
                RobotCarMotorControl.startRampUpAndWaitForDriveSpeed(DIRECTION_FORWARD, &loopGUI);
            }
        }

        /*
         * Here car is moving
         */
#ifdef USE_ENCODER_MOTOR_CONTROL
        uint16_t tStepStartDistanceCount = RobotCarMotorControl.rightCarMotor.EncoderCount; // get count before distance scanning
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
            sCentimeterPerScanTimesTwo = RobotCarMotorControl.rightCarMotor.EncoderCount - tStepStartDistanceCount;
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
                insertToPath(RobotCarMotorControl.rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
            }
        } else {
            /*
             * No stop, just continue => overwrite last path element with current riding distance and try to synchronize motors
             */
            insertToPath(RobotCarMotorControl.rightCarMotor.EncoderCount, sLastDegreesTurned, false);
            RobotCarMotorControl.rightCarMotor.synchronizeMotor(&RobotCarMotorControl.leftCarMotor,
            MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
#endif
        }

        if (sCurrentPage == PAGE_SHOW_PATH) {
            drawPathInfoPage();
        }
    }
}

/*
 * Checks distances and returns degrees to turn
 * Wall degree 0 -> wall parallel to our driving direction, 90 -> wall in front.
 * 0 -> no turn, > 0 -> turn left, < 0 -> turn right, > 360 go back, since not free ahead
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
         * Free ahead, currently do nothing
         */
    } else {
        /*
         * Not free ahead, must turn, check if another forward direction is suitable
         */
        if (sForwardDistancesInfo.IndexOfDistanceGreaterThanThreshold < NUMBER_OF_DISTANCES) {
            /*
             * We have at least index with distance greater than threshold of sCentimeterPerScanTimesTwo
             */
            tDegreeToTurn = ((sForwardDistancesInfo.IndexOfDistanceGreaterThanThreshold * DEGREES_PER_STEP) + START_DEGREES) - 90;
        } else {
            if (sForwardDistancesInfo.MaxDistance > sCentimeterPerScanTimesTwo) {
                /*
                 * Go to max distance if greater than threshold of sCentimeterPerScanTimesTwo, currently the same as above
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

void checkSpeedAndGo(unsigned int aSpeed, uint8_t aRequestedDirection) {
    if (aSpeed > RobotCarMotorControl.rightCarMotor.DriveSpeed * 2) {
        aSpeed = RobotCarMotorControl.rightCarMotor.DriveSpeed * 2;
    }
    if (aSpeed > MAX_SPEED) {
        aSpeed = MAX_SPEED;
    }
    RobotCarMotorControl.startRampUpAndWait(aSpeed, aRequestedDirection, &loopGUI);

}

/*
 * Start with scanning for target, since sFollowerTargetFound is false initially.
 *
 * If mode == MODE_STEP_TO_NEXT_TURN, stop if the scan has found a target (sNextDegreesToTurn != SCAN_AGAIN).
 * Start at next step with the turn until the next target has been searched for and found.
 */
void driveFollowerModeOneStep() {

    unsigned int tCentimeter = getDistanceAndPlayTone();

    if (sStepMode == MODE_STEP_TO_NEXT_TURN && sNextDegreesToTurn != SCAN_AGAIN) {
        // we had fount a target before -> wait for step signal
        if (sDoStep) {
            sDoStep = false;
            RobotCarMotorControl.rotateCar(sNextDegreesToTurn, TURN_FORWARD, true);
            //RobotCarMotorControl.rotateCar(sNextDegreesToTurn, TURN_IN_PLACE, true);
            // reset flag, that we have to stop
            sNextDegreesToTurn = SCAN_AGAIN;
        }
        return; // wait for step
    }

    if (!sFollowerTargetFound || tCentimeter > FOLLOWER_RESCAN_DISTANCE_CENTIMETER) {
        /*
         * Distance too high, stop car and search target in front directions
         */
        RobotCarMotorControl.stopMotors();
        clearPrintedForwardDistancesInfos();
        // show current distance (as US distance), which triggers the scan
        showUSDistance(tCentimeter);

        scanForTarget();
        if (sNextDegreesToTurn != SCAN_AGAIN) {
            sFollowerTargetFound = true; // (Moved) target found
        }
        delayAndLoopGUI(50); // to display the values
        return;
    }

    unsigned int tSpeed;
    if (tCentimeter > FOLLOWER_MAX_DISTANCE_CENTIMETER) {
//        if (RobotCarMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_FORWARD) {
//            Serial.println(F("Go forward"));
//        }
        tSpeed = RobotCarMotorControl.rightCarMotor.StartSpeed + (tCentimeter - FOLLOWER_MAX_DISTANCE_CENTIMETER) * 2;
        checkSpeedAndGo(tSpeed, DIRECTION_FORWARD);

    } else if (tCentimeter < FOLLOWER_MIN_DISTANCE_CENTIMETER) {
//        if (RobotCarMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_BACKWARD) {
//            Serial.println(F("Go backward"));
//        }
        tSpeed = RobotCarMotorControl.rightCarMotor.StartSpeed + (FOLLOWER_MIN_DISTANCE_CENTIMETER - tCentimeter) * 4;
        checkSpeedAndGo(tSpeed, DIRECTION_BACKWARD);

    } else {
        // Target found here :-)
        sFollowerTargetFound = true;
        if (RobotCarMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
//        Serial.println(F("Stop"));
            RobotCarMotorControl.stopMotors(MOTOR_RELEASE);
        }
    }
    delayAndLoopGUI(100); // the IR sensor takes 39 ms for one measurement
}

unsigned int __attribute__((weak)) getDistanceAndPlayTone() {
    /*
     * Get distance; timeout is 1 meter
     */
    unsigned int tCentimeter = getDistanceAsCentiMeter(true);
    /*
     * play tone
     */
    int tFrequency = map(tCentimeter, 0, 100, 100, 2000);
    tone(PIN_SPEAKER, tFrequency);
    return tCentimeter;
}
