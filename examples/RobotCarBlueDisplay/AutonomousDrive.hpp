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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */
#ifndef _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
#define _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
#include <Arduino.h>

#include "AutonomousDrive.h"

#include "RobotCarPinDefinitionsAndMore.h"
#include "RobotCarBlueDisplay.h"
#include "RobotCarGui.h"
#include "Distance.h"

uint8_t sDriveMode = MODE_MANUAL_DRIVE; // one of MODE_MANUAL_DRIVE, MODE_COLLISION_AVOIDING_BUILTIN, MODE_COLLISION_AVOIDING_USER or MODE_FOLLOWER

uint8_t sStepMode = MODE_CONTINUOUS; // one ofMODE_CONTINUOUS,  MODE_STEP_TO_NEXT_TURN or MODE_SINGLE_STEP
bool sDoStep = false; // if true => do one step

turn_direction_t sTurnMode = TURN_IN_PLACE;

// Storage for turning decision especially for single step mode
int sNextDegreesToTurn = 0;
// Storage of last turning for insertToPath()
int sLastDegreesTurned = 0;

/*
 * Used for adaptive collision detection
 */
#define CENTIMETER_PER_RIDE 20
#if defined(USE_ENCODER_MOTOR_CONTROL)
uint8_t sCentimetersDrivenPerScan = CENTIMETER_PER_RIDE; // Encoder counts per US scan in autonomous mode
#else
uint8_t const sCentimetersDrivenPerScan = CENTIMETER_PER_RIDE; // Constant
#endif

void driveAutonomousOneStep() {

    if (sDriveMode != MODE_MANUAL_DRIVE) {
        if (sDriveMode == MODE_FOLLOWER) {
            driveFollowerModeOneStep();
        } else {
            driveCollisonAvoidingOneStep();
        }
    }
}

/*
 * aDriveMode required if aDoStart is true otherwise sDriveMode = MODE_MANUAL_DRIVE;
 */
void startStopAutomomousDrive(bool aDoStart, uint8_t aDriveMode) {
    noTone(PIN_BUZZER); // for follower mode

    if (aDoStart) {
        /*
         * Start autonomous driving.
         */
#if defined(ENABLE_PATH_INFO_PAGE)
        resetPathData();
#endif
#if defined(CAR_HAS_DISTANCE_SERVO)
        clearPrintedForwardDistancesInfos();
#endif
        sDoStep = true; // enable next step
        sDriveMode = aDriveMode;

        // decide which button called us
        if (aDriveMode == MODE_COLLISION_AVOIDING_USER) {
            // User mode always starts in mode SINGLE_STEP
            setStepMode(MODE_SINGLE_STEP);

        } else if (aDriveMode == MODE_FOLLOWER) {
#if defined(CAR_HAS_DISTANCE_SERVO)
            DistanceServoWriteAndDelay(90); // reset Servo
#endif
            // Show distance sliders
            SliderUSDistance.drawSlider();
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
            SliderIROrTofDistance.drawSlider();
#  endif
        }

    } else {
        /*
         * Stop autonomous driving.
         */
#if defined(USE_ENCODER_MOTOR_CONTROL) && defined(ENABLE_PATH_INFO_PAGE)
        if (sStepMode != MODE_SINGLE_STEP) {
            // add last driven distance to path
            insertToPath(RobotCarPWMMotorControl.rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
        }
#endif
#if defined(CAR_HAS_DISTANCE_SERVO)
        DistanceServoWriteAndDelay(90);
#endif
        sDriveMode = MODE_MANUAL_DRIVE;
        RobotCarPWMMotorControl.stop(STOP_MODE_RELEASE);
    }

    // manage on off buttons
    handleAutomomousDriveRadioButtons();
    TouchButtonRobotCarStartStop.setValue(aDoStart, false);
}

int postProcessAndCollisionDetection() {
#if defined(CAR_HAS_DISTANCE_SERVO)
    doWallDetection();
    postProcessDistances(sCentimetersDrivenPerScan);
    int tNextDegreesToTurn;
    if (sDriveMode == MODE_COLLISION_AVOIDING_BUILTIN) {
        tNextDegreesToTurn = doBuiltInCollisionDetection();
    } else {
        tNextDegreesToTurn = doUserCollisionDetection();
    }
    drawCollisionDecision(tNextDegreesToTurn, sCentimetersDrivenPerScan, false);
    return tNextDegreesToTurn;
#else
    return 0;
#endif
}

/*
 * Do one step of collision avoiding driving, called by main loop.
 * Compute sNextDegreesToTurn AFTER the movement to be able to stop before next turn
 */
void driveCollisonAvoidingOneStep() {

    bool tCarIsStopped = RobotCarPWMMotorControl.isStopped();
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
            if (sNextDegreesToTurn == MINIMUM_DISTANCE_TOO_SMALL) {
                // go backwards and do a new scan
                RobotCarPWMMotorControl.goDistanceMillimeter(100, DIRECTION_BACKWARD, &loopGUI);
            } else {
                // rotate and go
                RobotCarPWMMotorControl.rotate(sNextDegreesToTurn, sTurnMode, false, &loopGUI); // do not use slow speed
                // wait to really stop after turning
                delay(100);
                sLastDegreesTurned = sNextDegreesToTurn;
            }
        }
        if (sNextDegreesToTurn != MINIMUM_DISTANCE_TOO_SMALL) {
            /*
             * No rotation or standard rotation here. Go fixed distance or keep moving
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                // Go fixed distance
                RobotCarPWMMotorControl.goDistanceMillimeter(sCentimetersDrivenPerScan * 10, DIRECTION_FORWARD, &loopGUI);
            } else
            /*
             * Continuous mode, start car or let it run
             */
            if (tCarIsStopped) {
                RobotCarPWMMotorControl.startRampUpAndWaitForDriveSpeedPWM(DIRECTION_FORWARD, &loopGUI);
            }
        }

        /*
         * Here car is moving
         */
#if defined(USE_ENCODER_MOTOR_CONTROL)
        uint16_t tStepStartDistanceCount = RobotCarPWMMotorControl.rightCarMotor.EncoderCount; // get count before distance scanning
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
        drawCollisionDecision(sNextDegreesToTurn, sCentimetersDrivenPerScan, true);
        sNextDegreesToTurn = postProcessAndCollisionDetection();

        /*
         * compute distance driven for one US scan
         * First check if car is stopped
         */
        if (!RobotCarPWMMotorControl.isStopped() && sStepMode != MODE_SINGLE_STEP) {
            /*
             * No stop here => distance is valid
             * One encoder count is 11 mm so just take the count as centimeter here :-)
             */
#if defined(USE_ENCODER_MOTOR_CONTROL)
            sCentimetersDrivenPerScan = RobotCarPWMMotorControl.rightCarMotor.EncoderCount - tStepStartDistanceCount;
#endif
            if (tCurrentPageIsAutomaticControl) {
                char tStringBuffer[6];
                sprintf_P(tStringBuffer, PSTR("%2d%s"), sCentimetersDrivenPerScan, "cm");
                BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tStringBuffer, TEXT_SIZE_11,
                COLOR16_BLACK, COLOR16_WHITE);
            }
        }

        /*
         * Handle stop of car and path data
         */
        if ((sNextDegreesToTurn != 0 && sStepMode == MODE_STEP_TO_NEXT_TURN) || sStepMode == MODE_SINGLE_STEP) {
            /*
             * Stop if rotation requested or single step
             */
            RobotCarPWMMotorControl.stopAndWaitForIt();
#if defined(USE_ENCODER_MOTOR_CONTROL)
#if defined(ENABLE_PATH_INFO_PAGE)

            /*
             * Insert / update last ride in path
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                insertToPath(CENTIMETER_PER_RIDE * 2, sLastDegreesTurned, true);
            } else {
                // add last driven distance to path
                insertToPath(RobotCarPWMMotorControl.rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
            }
#endif
        } else {
            /*
             * No stop, just continue => overwrite last path element with current riding distance and try to synchronize motors
             */
#if defined(ENABLE_PATH_INFO_PAGE)
            insertToPath(RobotCarPWMMotorControl.rightCarMotor.EncoderCount, sLastDegreesTurned, false);
#endif
//            RobotCarPWMMotorControl.rightCarMotor.synchronizeMotor(&RobotCarPWMMotorControl.leftCarMotor, 100);
#endif
        }

#if defined(ENABLE_PATH_INFO_PAGE)
        if (sCurrentPage == PAGE_SHOW_PATH) {
            drawPathInfoPage();
        }
#endif

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
        return MINIMUM_DISTANCE_TOO_SMALL;
    }
    /*
     * First check if free ahead
     */
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] > sCentimetersDrivenPerScan
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] > sCentimetersDrivenPerScan) {
        /*
         * Free ahead, currently do nothing
         */
    } else {
        /*
         * Not free ahead, must turn, check if another forward direction is suitable
         */
        if (sForwardDistancesInfo.IndexOfDistanceGreaterThanThreshold < NUMBER_OF_DISTANCES) {
            /*
             * We have at least index with distance greater than threshold of sCentimetersDrivenPerScan
             */
            tDegreeToTurn = ((sForwardDistancesInfo.IndexOfDistanceGreaterThanThreshold * DEGREES_PER_STEP) + START_DEGREES) - 90;
        } else {
            if (sForwardDistancesInfo.MaxDistance > sCentimetersDrivenPerScan) {
                /*
                 * Go to max distance if greater than threshold of sCentimetersDrivenPerScan, currently the same as above
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
    if (aSpeed > RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM * 2) {
        aSpeed = RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM * 2;
    }
    if (aSpeed > MAX_SPEED_PWM) {
        aSpeed = MAX_SPEED_PWM;
    }
    RobotCarPWMMotorControl.startRampUpAndWait(aSpeed, aRequestedDirection, &loopGUI);

}

/*
 * Start with scanning for target, since sFollowerTargetFound is false initially.
 *
 * If mode == MODE_STEP_TO_NEXT_TURN, stop if the scan has found a target (sNextDegreesToTurn != SCAN_AGAIN).
 * Start at next step with the turn until the next target has been searched for and found.
 */
void driveFollowerModeOneStep() {

    if (sNextDegreesToTurn == NO_TARGET_FOUND) {
        noTone(PIN_BUZZER);

        sNextDegreesToTurn = scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
        if (sNextDegreesToTurn == NO_TARGET_FOUND) {
            delayAndLoopGUI(50); // to display the values
            return;
        }
        /*
         * Found target here
         */
        if (sStepMode == MODE_STEP_TO_NEXT_TURN) {
            // wait for GUI to do enable the next step
            sDoStep = false;
        }
    }

    if (sDoStep) {
        if (sNextDegreesToTurn != 0) {
            /*
             * we had a pending turn
             */
            RobotCarPWMMotorControl.rotate(sNextDegreesToTurn, TURN_FORWARD, false, &loopGUI); // do not use slow speed
            sNextDegreesToTurn = 0;

        } else {
            /*
             * No scanning, no waiting for step, no pending turn
             * Measure distance, display it and drive or start scanning
             */
            unsigned int tCentimeter = getDistanceAsCentimeterAndPlayTone();

            if (tCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {
                // trigger scanning in the next loop
                // Stop car, clear display area and show distance
                RobotCarPWMMotorControl.stop();
                clearPrintedForwardDistancesInfos();
                // show current distance (as US distance), which triggers the scan
//                showUSDistance(); // TODO we do this in getDistanceAsCentimeterAndPlayTone()????
                sNextDegreesToTurn = NO_TARGET_FOUND;
                return;
            }

            /*
             * Simple follower without any turn
             */
            // show current distance (as US distance)
//            showUSDistance(); // TODO we do this in getDistanceAsCentimeterAndPlayTone()????
            unsigned int tSpeed;
            if (tCentimeter > FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
//        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_FORWARD) {
//            Serial.println(F("Go forward"));
//        }
                tSpeed = RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM / 2
                        + (tCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) * 2;
                checkSpeedAndGo(tSpeed, DIRECTION_FORWARD);

            } else if (tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
//        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_BACKWARD) {
//            Serial.println(F("Go backward"));
//        }
                tSpeed = RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM / 2
                        + (FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - tCentimeter) * 4;
                checkSpeedAndGo(tSpeed, DIRECTION_BACKWARD);

            } else {
                if (RobotCarPWMMotorControl.getCarDirection() != DIRECTION_STOP) {
//        Serial.println(F("Stop"));
                    RobotCarPWMMotorControl.stop(STOP_MODE_RELEASE);
                }
            }
        }
    }
    delayAndLoopGUI(100); // the IR sensor takes 39 ms for one measurement
}
#endif // _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
#pragma once
