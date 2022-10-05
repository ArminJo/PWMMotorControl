/*
 * AutonomousDrive.hpp
 *
 * Contains:
 * fillForwardDistancesInfoPro(): Acquisition of 180 degrees distances by ultrasonic sensor and servo
 * doWallDetection(): Enhancement of acquired data because of lack of detecting flat surfaces by US at angels out of 70 to 110 degree.
 * doBuiltInCollisionAvoiding(): decision where to turn in dependency of the acquired distances.
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

#ifndef _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
#define _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP

#include "AutonomousDrive.h"
#include "Distance.h"

uint8_t sDriveMode = MODE_MANUAL_DRIVE; // one of MODE_MANUAL_DRIVE, MODE_COLLISION_AVOIDING_BUILTIN, MODE_COLLISION_AVOIDING_USER or MODE_FOLLOWER

uint8_t sStepMode = MODE_CONTINUOUS; // one of MODE_CONTINUOUS,  MODE_STEP_TO_NEXT_TURN or MODE_SINGLE_STEP
bool sDoStep = false; // if true => do one step

turn_direction_t sTurnMode = TURN_IN_PLACE;
int sNextRotationDegree = 0; // Storage for turning decision especially for single step and step to turn mode
distance_range_t sDistanceRange; // Storage for current distance range, is used for single step and step to turn mode
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
    noTone (PIN_BUZZER); // for follower mode

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
            setStepMode (MODE_SINGLE_STEP);

        } else if (aDriveMode == MODE_FOLLOWER) {
            DistanceServoWriteAndWaitForStop(90); // reset Servo
            // Show distance sliders
//            SliderUSDistance.drawSlider();
            TouchButtonDistanceFeedbackMode.drawButton();
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
//            SliderIROrTofDistance.drawSlider();
#  endif
        }

    } else {
        /*
         * Stop autonomous driving.
         */
#if defined(ENABLE_PATH_INFO_PAGE)
        if (sStepMode != MODE_SINGLE_STEP) {
            // add last driven distance to path
            insertToPath(RobotCar.rightCarMotor.EncoderCount, sLastDegreesTurned, true);
        }
#endif
        DistanceServoWriteAndWaitForStop(90);
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
int postProcessAndCollisionAvoidingAndDraw() {

    memcpy(sForwardDistancesInfo.ProcessedDistancesArray, sForwardDistancesInfo.RawDistancesArray, NUMBER_OF_DISTANCES);
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    if (sDistanceSourceMode == DISTANCE_SOURCE_MODE_MAXIMUM || sDistanceSourceMode == DISTANCE_SOURCE_MODE_US) {
        // wall detection handles long distances of US measurements and modifies ProcessedDistancesArray
        doWallDetection();
    }
#else
    doWallDetection(); // It is required for US distance measurements
#endif

    postProcessDistances(sCentimetersDrivenPerScan);

    int tNextDegreesToTurn;
    if (sDriveMode == MODE_COLLISION_AVOIDING_USER) {
        // User provided result
        tNextDegreesToTurn = doUserCollisionAvoiding();
    } else {
        tNextDegreesToTurn = doBuiltInCollisionAvoiding();
    }
    drawCollisionDecision(tNextDegreesToTurn, sCentimetersDrivenPerScan, false);
    return tNextDegreesToTurn;
}

/*
 * Checks distances and returns degrees to turn
 * Wall degree 0 -> wall parallel to our driving direction, 90 -> wall in front.
 * 0 -> no turn, > 0 -> turn left, < 0 -> turn right, > 360 go back, since not free ahead
 */
int doBuiltInCollisionAvoiding() {
    int tDegreeToTurn = 0;
// 5 is too low
    if (sForwardDistancesInfo.MinDistance < 7) {
        /*
         * Min Distance too small => go back and scan again
         */
        return MINIMUM_DISTANCE_TOO_SMALL;
    }
    /*
     * First check if free ahead (green bars)
     */
    if (sForwardDistancesInfo.DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold != INVALID_DEGREE) {
        /*
         * Free found two consecutive long distances (maybe ahead) so turn (by maybe 0 degree)
         */
        tDegreeToTurn = sForwardDistancesInfo.DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold;
    } else {
        /*
         * Not free ahead, must turn, check if another forward direction is suitable
         */
        if (sForwardDistancesInfo.DegreeOfDistanceGreaterThanThreshold != INVALID_DEGREE) {
            /*
             * We have at least one index with distance greater than threshold of sCentimetersDrivenPerScan (yellow or green bar)
             */
            tDegreeToTurn = sForwardDistancesInfo.DegreeOfDistanceGreaterThanThreshold;
        } else {
            if (sForwardDistancesInfo.MaxDistance > sCentimetersDrivenPerScan) {
//                /*
//                 * Go to max distance if greater than threshold of sCentimetersDrivenPerScan
//                 * !!! Currently the same as above
//                 */
//                tDegreeToTurn = sForwardDistancesInfo.DegreeOfMaxDistance;
//            } else {
                /*
                 * Distances are all shorter than sCentimetersDrivenPerScan => must turn and go back
                 */
                tDegreeToTurn = 180;
            }
        }
    }
    return tDegreeToTurn;
}

/*
 * Do one step of collision avoiding driving, called by main loop.
 * First, do the movement computed in last step
 * Second, read distances
 * Third, compute sNextRotationDegree
 * This order enables it to stop before the next turn
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
             * Go backwards
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
         * Here car is (still) moving or just did a rotation or back move and has stooped now
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

        drawCollisionDecision(sNextRotationDegree, sCentimetersDrivenPerScan, true); // Clear old decision marker by redrawing it with a white line
        sNextRotationDegree = postProcessAndCollisionAvoidingAndDraw();

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
                BlueDisplay1.drawText(0, BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND, tStringBuffer, TEXT_SIZE_11, COLOR16_BLACK,
                        COLOR16_WHITE);
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
                insertToPath(RobotCar.rightCarMotor.EncoderCount, sLastDegreesTurned, true);
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

/***************************************************
 * Code for follower mode
 ***************************************************/
/*
 * If mode == MODE_STEP_TO_NEXT_TURN, stop if the scan has requested a turn.
 * Start at next step with the turn until the next target has been searched for and found.
 */
void driveFollowerModeOneStep() {

    if (sDoStep) { // Check if we shall do this step. Do not check for changed distance, since we might have other measurements in between.
        if (sNextRotationDegree != 0) {
            /*
             * We have a pending turn
             */
            DistanceServoWriteAndWaitForStop(90, false); // reset distance servo direction
            // Use old distance range. Do a cast, since the values of tRange and rotation match!
            RobotCar.rotate(sNextRotationDegree, static_cast<turn_direction_t>(sDistanceRange), false, &loopGUI); // do not use slow speed
            sNextRotationDegree = 0; // reset pending turn
        }

        /*
         * We have NO pending turn no more, scan target at 70, 90 and 110 degree
         */
//        clearPrintedForwardDistancesInfos(false); // clear area for next scan results
        int8_t tNextRotationDegree = scanTarget(FOLLOWER_DISTANCE_TIMEOUT_CENTIMETER);
        unsigned int tForwardCentimeter = sRawForwardDistancesArray[INDEX_TARGET_FORWARD]; // Values between 1 and FOLLOWER_DISTANCE_TIMEOUT_CENTIMETER
        sDistanceRange = getDistanceRange(tForwardCentimeter);

        /*
         * Rotate or stop for step if rotation requested. Do not move and scan again.
         */
        if (tNextRotationDegree != 0) {
            /*
             * Manage step mode
             */
            if (sStepMode == MODE_STEP_TO_NEXT_TURN) {
                // wait for GUI to do enable the next step
                sDoStep = false;
                delayAndLoopGUI(50); // to display the values
                sNextRotationDegree = tNextRotationDegree; // store pending turn
            } else {
                // Do a cast, since the values of tRange and rotation match!
                RobotCar.rotate(tNextRotationDegree, static_cast<turn_direction_t>(sDistanceRange), false, &loopGUI);
            }
        } else {

            /*
             * No turn, compute new speed depending on the forward distance
             */
            unsigned int tNewSpeedPWM = 0;
            uint8_t tDirection = DIRECTION_STOP;

            if (sDistanceRange == DISTANCE_TO_GREAT) {
                /*
                 * FORWARD  - Distance (31 to 60) too far
                 * Drive FORWARD with speed proportional to the gap. We start with DEFAULT_START_SPEED_PWM.
                 * Maximum difference between current and target distance (tCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) is 30.
                 */
                uint8_t tDifferenceCentimeter = tForwardCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER;
                tNewSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_START_SPEED_PWM, sVINVoltage)
                        + tDifferenceCentimeter * 4; // maximum is + 120 here
                tDirection = DIRECTION_FORWARD;

            } else if (sDistanceRange == DISTANCE_TO_SMALL) {
                /*
                 * BACKWARD  - Distance (1 to 19) too close
                 * Drive with speed proportional to the gap. We start with DEFAULT_DRIVE_SPEED_PWM, to be a bit more agile here.
                 * Maximum difference between current and target distance is FOLLOWER_DISTANCE_MINIMUM_CENTIMETER / 20.
                 */
                uint16_t tDifferenceCentimeter = FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - tForwardCentimeter;
                tNewSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_DRIVE_SPEED_PWM, sVINVoltage)
                //                        + tDifferenceCentimeter * 8; // maximum is + 320 here
                        + tDifferenceCentimeter * 4; // maximum is + 80 here
                tDirection = DIRECTION_BACKWARD;
            }

            /*
             * Process new speed
             */
            if (tNewSpeedPWM != 0) {
                /*
                 * Clip speed, since we have a delay introduced by scanning,
                 * and we do not want that the car is moving from minimum to maximum or back during this delay.
                 * And additionally, it seems that to much speed generates high frequency noise, which disturbs the US sensor.
                 */
                uint8_t tMaxSpeed = DEFAULT_DRIVE_SPEED_PWM + (DEFAULT_DRIVE_SPEED_PWM / 2);
                if (sDoSlowScan) {
                    tMaxSpeed = DEFAULT_DRIVE_SPEED_PWM;
                }
                if (tNewSpeedPWM > tMaxSpeed) {
                    tNewSpeedPWM = tMaxSpeed;
                }
                /*
                 * Set speed and direction
                 */
                RobotCar.setSpeedPWMAndDirection(tNewSpeedPWM, tDirection);

            } else {
                /*
                 * STOP - Target is in the right distance, or we have a timeout
                 */
                if (!RobotCar.isStopped()) {
                    RobotCar.stop(STOP_MODE_RELEASE); // stop only once
                }
            }
        }
    } // if (sDoStep)
    if (sStepMode == MODE_SINGLE_STEP) {
        /*
         * Single step here -> wait 1/2 second and then stop
         */
        sDoStep = false;
        delayAndLoopGUI(500);
        if (!RobotCar.isStopped()) {
            RobotCar.stop(STOP_MODE_RELEASE);
        }
    }
}
#endif // _ROBOT_CAR_AUTOMOMOUS_DRIVE_HPP
