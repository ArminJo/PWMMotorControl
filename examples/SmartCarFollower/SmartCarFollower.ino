/*
 *  SmartCarFollower.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel smart car with an Arduino.
 *  If the target vanishes, the distance sensor scans to get the vanished (or a new) target.
 *
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
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
 *
 */

#include <Arduino.h>

/*
 * Values to configure the behavior of the follower
 */
#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER        20 // If measured distance is less than this value, go backwards
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER        30 // If measured distance is greater than this value, go forward
#define FOLLOWER_DISTANCE_DELTA_CENTIMETER          (FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER - FOLLOWER_DISTANCE_MINIMUM_CENTIMETER)
#define FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER    60 // assume that target moved to side, and search

/*
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 4AA.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION   // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION    // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define L298_2WD_4AA_BASIC_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_4WD_4AA_BASIC_CONFIGURATION          // China set with L298 + 4AA.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_VIN_IR_DISTANCE_CONFIGURATION    // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_VIN_IR_IMU_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
#define DO_NOT_SUPPORT_RAMP         // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
#define DO_NOT_SUPPORT_AVERAGE_SPEED // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.
#define USE_SOFT_I2C_MASTER         // saves up to 2400 bytes program memory and 220 bytes RAM compared with Arduino Wire
//#define DEBUG
//#define INFO
#include "RobotCarConfigurations.h" // sets e.g. USE_ENCODER_MOTOR_CONTROL, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"

/*
 * Enabling program features dependent on car configuration
 */
#if !defined(CAR_HAS_DISTANCE_SERVO)
#error This program requires a distance servo mounted indicated by a #define CAR_HAS_DISTANCE_SERVO
#endif

#if defined(CAR_HAS_ENCODERS)
#define USE_ENCODER_MOTOR_CONTROL   // Enable if by default, if available
#endif
#if defined(CAR_HAS_MPU6050_IMU)
#define USE_MPU6050_IMU
#endif

#include "CarPWMMotorControl.hpp"
#include "Distance.hpp"  // provides DistanceServo definition and uses FOLLOWER_DISTANCE_MINIMUM_CENTIMETER definition

/*
 * Enable functionality of this program
 */
// if CAR_HAS_VIN_VOLTAGE_DIVIDER is NOT specified, existence of voltage divider will be determined dynamically if MONITOR_VIN_VOLTAGE is enabled.
#define MONITOR_VIN_VOLTAGE         // Enable monitoring of VIN voltage for exact movements, if available
//#define ADC_INTERNAL_REFERENCE_MILLIVOLT 1100L    // Value measured at the AREF pin
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000

//#define USE_IR_REMOTE
#if defined(USE_IR_REMOTE)
/*
 * Choose remote
 * For available IR commands see RobotCarIRCommands.hpp and for the mapping to remote buttons see IRCommandMapping.h
 * https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/RobotCarIRCommands.hpp
 * https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
 */
//#define USE_KEYES_REMOTE_CLONE // With number pad above direction control, will be taken as default
//#define USE_KEYES_REMOTE
//#define USE_DVBT_STICK_REMOTE
#define USE_TINY_IR_RECEIVER // Supports only NEC protocol. Must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#include "RobotCarIRCommands.hpp" // requires #include "Distance.hpp"
#include "IRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#define LOCAL_INFO // Enable info just for IRCommandDispatcher
#include "IRCommandDispatcher.hpp"
#else
#define DISTANCE_FEEDBACK_MODE   DISTANCE_FEEDBACK_PENTATONIC // one of DISTANCE_FEEDBACK_CONTINUOUSLY or DISTANCE_FEEDBACK_PENTATONIC
#endif

#include "RobotCarUtils.hpp" // must be after #define MONITOR_VIN_VOLTAGE

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT                0

void doFollowerOneStep();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
    printConfigInfo(&Serial);
    printProgramOptions(&Serial);
    PWMDcMotor::printCompileOptions(&Serial);

    initRobotCarPWMMotorControl();
    RobotCar.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set left/right speed compensation

    /*
     * Initialize US servo and set to forward position
     */
    initDistance();
    DistanceServoWriteAndDelay(90);

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);

#if defined(US_DISTANCE_SENSOR_ENABLE_PIN) // If this pin is connected to ground, use the US distance sensor instead of the IR distance sensor
    pinMode(US_DISTANCE_SENSOR_ENABLE_PIN, INPUT_PULLUP);
#endif

#if defined(USE_MPU6050_IMU)
    /*
     * Wait after pressing the reset button, or attaching the power
     * and then take offset values for 1/2 second
     */
    delay(1000);
    tone(PIN_BUZZER, 2200, 50);
    delay(100);
    RobotCar.calculateAndPrintIMUOffsets(&Serial);
    tone(PIN_BUZZER, 2200, 50);
#endif
#if defined(USE_IR_REMOTE)
    // For available IR commands see IRCommandMapping.h https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
    IRDispatcher.init();
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_INPUT_PIN)));
#else
#  if defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) // If this pin is connected to ground, enable distance feedback
    pinMode(DISTANCE_TONE_FEEDBACK_ENABLE_PIN, INPUT_PULLUP);
#  endif
#endif

    /*
     * Do not start immediately with driving
     */
    delay(2000);

    /*
     * Servo feedback for start of loop
     */
    DistanceServoWriteAndDelay(135);
    delay(500);
    DistanceServoWriteAndDelay(45);
    delay(500);
    DistanceServoWriteAndDelay(90);
    delay(500);

    /*
     * Move car forward and measure voltage with load to enable exact turns
     */
    calibrateDriveSpeedPWM();

    Serial.println(F("Start loop"));
}

void loop() {
#if defined(USE_IR_REMOTE)
    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested action
     */
    IRDispatcher.checkAndRunSuspendedBlockingCommands();
#endif

#if defined(USE_IR_REMOTE)
    // we can enable / disable follower / distance (no turn) mode by IR
    if (sEnableKeepDistance || sEnableFollower) {
        doFollowerOneStep();
    } else {
        getDistanceAndPlayTone();
        printDistanceIfChanged(&Serial); // prints "Distance=XXcm" with or "Distance timeout" without a newline{
        if (sDistanceJustChanged) {
            Serial.println(); // Terminate line "Distance=57cm ->  SpeedPWM=255" etc.
        }
    }
#else
    doFollowerOneStep();
#endif

    checkVinPeriodicallyAndPrintIfChanged();

#if defined(USE_ENCODER_MOTOR_CONTROL)
    RobotCar.delayAndUpdateMotors(100);
#else
    delay(100);
#endif
}

void doFollowerOneStep() {
    static uint8_t sTargetNotFoundCount; // Allow 4 "timeouts" before stop and scanning

    unsigned int tCentimeter;
    if (sTargetNotFoundCount <= 4) {
        /*
         * No scanning here, get new distance and check
         */
        tCentimeter = getDistanceAndPlayTone();
        printDistanceIfChanged(&Serial); // prints "Distance=XXcm" or "Distance timeout" without a newline

        if (tCentimeter == DISTANCE_TIMEOUT_RESULT || tCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {
            /*
             * Distance too high or timeout / target not found. Allow 4 "timeouts" before stop and scanning.
             */
#if defined(USE_IR_REMOTE)
            if (sEnableFollower) { // Scan for target only if FollowerMode enabled, and not if KeepDistance enabled
                sTargetNotFoundCount++;
                Serial.print(F("TargetNotFoundCount="));
                Serial.println(sTargetNotFoundCount);
            } else {
                if (!RobotCar.isStopped()) {
                    Serial.println(F("Stop car"));
                    RobotCar.stop(STOP_MODE_RELEASE);
                }
            }
#else
            sTargetNotFoundCount++;
            Serial.print(F("TargetNotFoundCount="));
            Serial.println(sTargetNotFoundCount);
#endif
        } else {
            /*
             * Target in range here. No action required, if distance did not change
             */
            sTargetNotFoundCount = 0;
            if (sDistanceJustChanged) {
                unsigned int tSpeedPWM = 0;
                uint8_t tDirection = DIRECTION_STOP;
                // we are after print distance here
                Serial.print(F(" -> "));

                if (tCentimeter > FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
                    /*
                     * Target too far, but below scan threshold -> drive FORWARD with speed proportional to the gap.
                     * We start with DEFAULT_START_SPEED_PWM, which is adjusted to avoid undervoltage which prevents moving
                     * Maximum difference between current and target distance (tCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) is 30.
                     */
                    tSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_START_SPEED_PWM, sVINVoltage)
                            + (tCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) * 8; // maximum is 240 here
                    tDirection = DIRECTION_FORWARD;

                } else if (tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
                    /*
                     * Target too close -> drive BACKWARD with speed proportional to the gap.
                     * We start with DEFAULT_START_SPEED_PWM, which is adjusted to avoid undervoltage which prevents moving
                     */
                    tSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_START_SPEED_PWM, sVINVoltage)
                            + (FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - tCentimeter) * 16; // maximum is 320 here
                    tDirection = DIRECTION_BACKWARD;
                }

                if (RobotCar.getCarDirection() != tDirection) {
                    // print only once at direction change
                    Serial.print(F("go "));
                    PWMDcMotor::printDirectionString(&Serial, tDirection);
                }

                if (tSpeedPWM != 0) {
                    if (tSpeedPWM > MAX_SPEED_PWM) {
                        tSpeedPWM = MAX_SPEED_PWM;
                    }
                    Serial.print(F("SpeedPWM="));
                    Serial.print(tSpeedPWM);
                    RobotCar.setSpeedPWMAndDirection(tSpeedPWM, tDirection);
                } else {
                    /*
                     * Target is in the right distance -> stop
                     */
                    if (!RobotCar.isStopped()) {
                        RobotCar.stop(STOP_MODE_RELEASE); // stop only once
                    } else {
                        Serial.print(F("ok"));
                    }
                }
            }
            if (sDistanceJustChanged) {
                Serial.println(); // Terminate line "Distance=57cm ->  SpeedPWM=255" etc.
            }
        }
    } else {

        /*
         * We do not get here in  mode KeepDistance.
         * 4 times no target was found straight ahead, so scan left and right
         * for target at different directions and turn if found.
         */
        if (!RobotCar.isStopped()) {
            Serial.println(F("Stop car and start scanning"));
            RobotCar.stop(STOP_MODE_RELEASE);
        }

        /*
         * Scan for target at 70, 90 and 110 degree and rotate if found.
         */
        int tRotationDegree = scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
        if (tRotationDegree != NO_TARGET_FOUND) {
            // Rotate after target found
            sTargetNotFoundCount = 0;
            DistanceServoWriteAndDelay(90, false); // reset distance servo direction
            RobotCar.rotate(tRotationDegree, TURN_IN_PLACE, false, readVINVoltageAndAdjustDriveSpeed);
        }
    }
}
