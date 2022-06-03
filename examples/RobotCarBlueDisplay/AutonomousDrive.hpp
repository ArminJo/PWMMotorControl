/*
 * AutonomousDrive.hpp
 *
 * Contains:
 * fillForwardDistancesInfoPro(): Acquisition of 180 degrees distances by ultrasonic sensor and servo
 * doWallDetection(): Enhancement of acquired data because of lack of detecting flat surfaces by US at angels out of 70 to 110 degree.
 * doBuiltInCollisionDetection(): decision where to turn in dependency of the acquired distances.
 * driveAutonomousOneStep(): The loop which handles the start/stop, single step and path output functionality.
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#if defined(ENABLE_AUTONOMOUS_DRIVE)

#ifndef _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
#define _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP

#include "AutonomousDrive.h"

uint8_t sDriveMode = MODE_MANUAL_DRIVE; // one of MODE_MANUAL_DRIVE, MODE_COLLISION_AVOIDING_BUILTIN, MODE_COLLISION_AVOIDING_USER or MODE_FOLLOWER

uint8_t sStepMode = MODE_CONTINUOUS; // one of MODE_CONTINUOUS,  MODE_STEP_TO_NEXT_TURN or MODE_SINGLE_STEP
bool sDoStep = false; // if true => do one step

turn_direction_t sTurnMode = TURN_IN_PLACE;
int sNextRotationDegree = 0; // Storage for turning decision especially for single step mode
#if defined(ENABLE_PATH_INFO_PAGE)
int sLastDegreesTurned = 0; // Storage of last turning for insertToPath()
#endif

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
 * Stop is called from startStopRobotCar()!
 * @param aDriveMode required if aDoStart is true otherwise sDriveMode = MODE_MANUAL_DRIVE;
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
        clearPrintedForwardDistancesInfos(true);
        sDoStep = true; // enable next step
        sDriveMode = aDriveMode;

        // decide which button called us
        if (aDriveMode == MODE_COLLISION_AVOIDING_USER) {
            // User mode always starts in mode SINGLE_STEP
            setStepMode(MODE_SINGLE_STEP);

        } else if (aDriveMode == MODE_FOLLOWER) {
            DistanceServoWriteAndDelay(90); // reset Servo
            // Show distance sliders
            SliderUSDistance.drawSlider();
            TouchButtonDistanceFeedbackMode.drawButton();
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
            SliderIROrTofDistance.drawSlider();
#  endif
        }

    } else {
        /*
         * Stop autonomous driving.
         */
#if defined(ENABLE_PATH_INFO_PAGE)
        if (sStepMode != MODE_SINGLE_STEP) {
            // add last driven distance to path
            insertToPath(RobotCar.rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
        }
#endif
        DistanceServoWriteAndDelay(90);
        sDriveMode = MODE_MANUAL_DRIVE;
        RobotCar.stop(STOP_MODE_RELEASE);
        TouchButtonDistanceFeedbackMode.removeButton(COLOR16_WHITE);
    }

    // manage 3 on off buttons
    handleAutomomousDriveRadioButtons();
}

/*
 * Also draws collision decision, but does not clear it before, since for manual scan the whole area was cleared before
 */
int postProcessAndCollisionDetection() {
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
}

/*
 * Do one step of collision avoiding driving, called by main loop.
 * Compute sNextRotationDegree AFTER the movement to be able to stop before next turn
 */
void driveCollisonAvoidingOneStep() {

    bool tCarIsStopped = RobotCar.isStopped();
    /*
     * Check step conditions if step should happen
     */
    if (sDoStep || sStepMode == MODE_CONTINUOUS || (sStepMode == MODE_STEP_TO_NEXT_TURN && (!tCarIsStopped))) {
        /*
         * Do one step
         */
        bool tMovementJustStarted = sDoStep || tCarIsStopped; // tMovementJustStarted is needed for speeding up US scanning by skipping first scan angle if not just started.
        sDoStep = false; // Now it can be set again by GUI

        /*
         * Check sNextRotationDegree
         */
        if (sNextRotationDegree == MINIMUM_DISTANCE_TOO_SMALL) {
            /*
             * Go backwards and do a new scan
             */
            RobotCar.goDistanceMillimeter(100, DIRECTION_BACKWARD, &loopGUI);
        } else if (sNextRotationDegree != 0) {
            /*
             * Rotate and stop
             */
            RobotCar.rotate(sNextRotationDegree, sTurnMode, false, &loopGUI); // do not use slow speed
            // wait to really stop after turning
            delay(100);
#if defined(ENABLE_PATH_INFO_PAGE)
            sLastDegreesTurned = sNextRotationDegree;
#endif
        } else {
            /*
             * No rotation or go back here. Go fixed distance or keep moving
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                // Go fixed distance
                RobotCar.goDistanceMillimeter(sCentimetersDrivenPerScan * 10, DIRECTION_FORWARD, &loopGUI);
            } else if (tCarIsStopped) {
                /*
                 * Continuous mode, start car or let it run (do nothing)
                 */
                RobotCar.startRampUpAndWaitForDriveSpeedPWM(DIRECTION_FORWARD, &loopGUI);
            }
#if defined(ENABLE_PATH_INFO_PAGE)
            sLastDegreesTurned = 0; // rotation was 0 here
#endif
        }

        /*
         * Here car is moving or did a rotation or go back
         */
#if defined(USE_ENCODER_MOTOR_CONTROL)
        uint16_t tStepStartDistanceCount = RobotCar.rightCarMotor.EncoderCount; // get count before distance scanning
#endif
        /*
         * The magic happens HERE
         * This runs as fast as possible and mainly determine the duration of one step
         */
        if (fillAndShowForwardDistancesInfo(tMovementJustStarted)) {
            return; // User canceled autonomous drive, ForwardDistancesInfo may be incomplete then
        }

        // Clear old decision marker by redrawing it with a white line
        drawCollisionDecision(sNextRotationDegree, sCentimetersDrivenPerScan, true);
        sNextRotationDegree = postProcessAndCollisionDetection();

        /*
         * compute distance driven for one US scan
         * First check if car is stopped
         */
        if (!RobotCar.isStopped() && sStepMode != MODE_SINGLE_STEP) {
            /*
             * No stop here => distance is valid
             * One encoder count is 11 mm so just take the count as centimeter here :-)
             */
#if defined(USE_ENCODER_MOTOR_CONTROL)
            sCentimetersDrivenPerScan = RobotCar.rightCarMotor.EncoderCount - tStepStartDistanceCount;
#endif
            if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
                char tStringBuffer[6];
                sprintf_P(tStringBuffer, PSTR("%2d%s"), sCentimetersDrivenPerScan, "cm");
                BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tStringBuffer, TEXT_SIZE_11,
                COLOR16_BLACK, COLOR16_WHITE);
            }
        }

        /*
         * Handle stop of car and path data
         */
        if ((sNextRotationDegree != 0 && sStepMode == MODE_STEP_TO_NEXT_TURN) || sStepMode == MODE_SINGLE_STEP) {
            /*
             * Stop if rotation requested or single step
             */
            RobotCar.stopAndWaitForIt();
#if defined(ENABLE_PATH_INFO_PAGE)
            /*
             * Insert / update last ride in path
             */
            if (sStepMode == MODE_SINGLE_STEP) {
                insertToPath(CENTIMETER_PER_RIDE * 2, sLastDegreesTurned, true);
            } else {
                // add last driven distance to path
                insertToPath(RobotCar.rightCarMotor.LastRideEncoderCount, sLastDegreesTurned, true);
            }
        } else {
            /*
             * No stop, just continue => overwrite last path element with current riding distance and try to synchronize motors
             */
            insertToPath(RobotCar.rightCarMotor.EncoderCount, sLastDegreesTurned, false);
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
/*
 * Start with scanning for target, since sFollowerTargetFound is false initially.
 *
 * If mode == MODE_STEP_TO_NEXT_TURN, stop if the scan has found a target (sNextRotationDegree != SCAN_AGAIN).
 * Start at next step with the turn until the next target has been searched for and found.
 */
void driveFollowerModeOneStep() {

    if (sNextRotationDegree == NO_TARGET_FOUND) {
        /*
         * No target was found in last step, just try again
         */
        sNextRotationDegree = scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
        if (sNextRotationDegree == NO_TARGET_FOUND) {
            delayAndLoopGUI(50); // to display the values
            return; // try again next time
        }
        /*
         * Found target here, manage step mode
         */
        if (sStepMode == MODE_STEP_TO_NEXT_TURN) {
            // wait for GUI to do enable the next step
            sDoStep = false;
        }
    }

    if (sNextRotationDegree != 0) {
        /*
         * We have a pending turn
         */
        if (sDoStep) { // Check if we shall do this step
            DistanceServoWriteAndDelay(90, false); // reset distance servo direction
            RobotCar.rotate(sNextRotationDegree, TURN_FORWARD, false, &loopGUI); // do not use slow speed
            sNextRotationDegree = 0;
        }

    } else {
        /*
         * Target in range, no pending turn
         * Measure distance, display it and drive or start scanning
         */
        unsigned int tCentimeter = getDistanceAsCentimeterAndPlayTone();
        if (sDoStep) { // Check if we shall do this step

            if (tCentimeter == DISTANCE_TIMEOUT_RESULT || tCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {
                /*
                 * No target found -> stop car, clear display and start scanning in the next step
                 */
                RobotCar.stop(STOP_MODE_RELEASE);
                clearPrintedForwardDistancesInfos(false); // clear area for next scan results
                sNextRotationDegree = NO_TARGET_FOUND; // scan at next step
            } else {

                /*
                 * Target found -> keep distance
                 */
                int tSpeedPWM = 0;
                uint8_t tDirection = DIRECTION_STOP;
                if (tCentimeter > FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
                    /*
                     * Target too far, but below scan threshold -> drive FORWARD with speed proportional to the gap
                     * We start with DEFAULT_START_SPEED_PWM, which is adjusted to avoid undervoltage which prevents moving
                     */
                    tSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_START_SPEED_PWM, sVINVoltage)
                            + (tCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) * 2;
                    tDirection = DIRECTION_FORWARD;

                } else if (tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
                    /*
                     * Target too close -> drive BACKWARD with speed proportional to the gap
                     * We start with DEFAULT_START_SPEED_PWM, which is adjusted to avoid undervoltage which prevents moving
                     */
                    tSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_START_SPEED_PWM, sVINVoltage)
                            + (FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - tCentimeter) * 4;
                    tDirection = DIRECTION_BACKWARD;
                }

                if (tSpeedPWM != 0) {
                    if (tSpeedPWM > MAX_SPEED_PWM) {
                        tSpeedPWM = MAX_SPEED_PWM;
                    }
                    RobotCar.setSpeedPWMAndDirection(tSpeedPWM, tDirection);
                } else {
                    /*
                     * Target is in the right distance -> stop
                     */
                    if (!RobotCar.isStopped()) {
                        RobotCar.stop(STOP_MODE_RELEASE);
                    }
                }
            }
        } // if (sDoStep)
    } // if (sNextRotationDegree != 0)
    if(sStepMode == MODE_SINGLE_STEP){
        /*
         * Single step here -> wait 1/2 second and then stop
         */
        sDoStep = false;
        delayAndLoopGUI(500);
        if (!RobotCar.isStopped()) {
            RobotCar.stop(STOP_MODE_RELEASE);
        }
    }
    delayAndLoopGUI(100); // the IR sensor takes 39 ms for one measurement
}
#endif // _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
#endif // defined(ENABLE_AUTONOMOUS_DRIVE)
