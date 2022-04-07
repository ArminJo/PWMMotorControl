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
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
//#define L298_BASIC_2WD_4AA_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_BASIC_2WD_2LI_ION_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion's.
//#define L298_VIN_IR_DISTANCE_CONFIGURATION        // L298_Basic_2WD + VIN voltage divider + IR distance
//#define L298_VIN_IR_IMU_CONFIGURATION             // L298_Basic_2WD + VIN voltage divider + IR distance + MPU6050
#define DO_NOT_SUPPORT_RAMP         // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
#define DO_NOT_SUPPORT_AVERAGE_SPEED // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.

#include "RobotCarConfigurations.h" // sets e.g. USE_ENCODER_MOTOR_CONTROL, USE_ADAFRUIT_MOTOR_SHIELD

#undef USE_MPU6050_IMU

#include "RobotCarPinDefinitionsAndMore.h"

/*
 * Enable VCC monitoring if possible
 */
#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
#define MONITOR_VIN_VOLTAGE // Enable if by default, if available
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000
#endif

/*
 * Choose remote
 * For available IR commands see IRCommandMapping.h https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
 */
//#define USE_IR_REMOTE               // Use an IR remote receiver attached for car control
//#define USE_KEYES_REMOTE
//#define USE_DVBT_STICK_REMOTE
//#define DEBUG
//#define INFO
#include "CarPWMMotorControl.hpp"
#include "HCSR04.h"
#include "ADCUtils.h"
#include "pitches.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT                0

#if (DEFAULT_DRIVE_SPEED_PWM * 2) > 255
#define MAX_SPEED_PWM_FOLLOWER                    255
#else
#define MAX_SPEED_PWM_FOLLOWER                      (DEFAULT_DRIVE_SPEED_PWM * 2) // Max speed PWM value used for follower.
#endif

/*
 * Values to configure the behavior of the follower
 */
#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER        30 // If measured distance is less than this value, go backwards
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER        40 // If measured distance is greater than this value, go forward
#define FOLLOWER_DISTANCE_DELTA_CENTIMETER          (FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER - FOLLOWER_DISTANCE_MINIMUM_CENTIMETER)
#define FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER    70 // assume that target moved to side, and search

#define USE_STANDARD_SERVO_LIBRARY // we have enough memory, so make it simple and do not use LightweightServo library here
#include "Distance.hpp" // provides DistanceServo definition and uses FOLLOWER_DISTANCE_MINIMUM_CENTIMETER definition

#if defined(USE_IR_REMOTE)
#define USE_TINY_IR_RECEIVER // Supports only NEC protocol. Must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#include "IRCommandDispatcher.h" // for RETURN_IF_STOP
#include "RobotCarIRCommands.hpp" // requires #include "Distance.hpp"
#define INFO
#include "IRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"
#endif

#include "RobotCarUtils.hpp"

bool sDistanceChanged;
void doFollowerOneStep(unsigned int aCentimeter);

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
    printConfigInfo();

    initRobotCarPWMMotorControl();
    RobotCarPWMMotorControl.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set compensation

    /*
     * Initialize US servo and set to forward position
     */
    initDistance();
    DistanceServoWriteAndDelay(90);

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);

#if defined(US_DISTANCE_SENSOR_ENABLE_PIN)
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
    RobotCarPWMMotorControl.calculateAndPrintIMUOffsets(&Serial);
    tone(PIN_BUZZER, 2200, 50);
#endif
#if defined(USE_IR_REMOTE)
    // For available IR commands see IRCommandMapping.h https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
    IRDispatcher.init();
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_INPUT_PIN)));
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

    unsigned int tCentimeter = getDistanceAndPlayTone();
    sDistanceChanged = printDistanceIfChanged(&Serial);
#if defined(USE_IR_REMOTE)
    // we can enable / disable follower / distance (no turn) mode by IR
    if (sEnableKeepDistance || sEnableFollower) {
        doFollowerOneStep(tCentimeter);
    } else if(sDistanceChanged){
        Serial.println(); // Terminate the printed distance line
    }
#else
    doFollowerOneStep(tCentimeter);
#endif

#if defined(MONITOR_VIN_VOLTAGE)
    checkVinPeriodicallyAndPrintIfChanged();
#endif

#if defined(USE_ENCODER_MOTOR_CONTROL)
    RobotCarPWMMotorControl.delayAndUpdateMotors(100);
#else
    delay(100);
#endif
}

void doFollowerOneStep(unsigned int aCentimeter) {
    static uint8_t sTargetNotFoundCount;

    // aCentimeter == 0 is timeout
#if defined(USE_IR_REMOTE)
    if (sEnableFollower && (aCentimeter == 0 || aCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER)) {
#else
    if (aCentimeter == 0 || aCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {
#endif
        /*
         * Distance too high or timeout / target not found -> search for target at different directions and turn if found
         */
        if (!RobotCarPWMMotorControl.isStopped()) {
            Serial.print(F(" -> stop"));
            RobotCarPWMMotorControl.stop(STOP_MODE_RELEASE);
        }

        if (sDistanceChanged) {
            Serial.println(); // Terminate line "Distance timeout" or "Distance timeout -> stop"
        }

        if (sTargetNotFoundCount > 4) {
            int tRotationDegree = scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
            if (tRotationDegree != 0 && tRotationDegree != NO_TARGET_FOUND) {
                // reset servo position after target found
                DistanceServoWriteAndDelay(90);
                Serial.print(F("rotate "));
                Serial.print(tRotationDegree);
                RobotCarPWMMotorControl.rotate(tRotationDegree, TURN_IN_PLACE);
            }
            Serial.println(); // terminate scan result line with optional rotate info

            delay(100); // Additional delay to let the servo reach its 90 degree position
        } else {
            sTargetNotFoundCount++;
        }

    } else {
        sTargetNotFoundCount = 0;

        /*
         * No action required, if distance did not change
         */
        if (sDistanceChanged) {
            unsigned int tSpeedPWM;
            // we are after print distance here
            Serial.print(F(" -> "));
            if (aCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
                /*
                 * Target too close -> drive BACKWARD with speed proportional to the gap
                 */
                tSpeedPWM = DEFAULT_DRIVE_SPEED_PWM + (FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - aCentimeter) * 4;
                if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
                    tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
                }
                Serial.print(F("SpeedPWM="));
                Serial.print(tSpeedPWM);

                if (RobotCarPWMMotorControl.getCarDirection() != DIRECTION_BACKWARD) {
                    Serial.print(F(" -> go backward")); // print only once at direction change
                }

#if defined(USE_ENCODER_MOTOR_CONTROL)
                RobotCarPWMMotorControl.startGoDistanceMillimeter(tSpeedPWM,
                        ((aCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) + FOLLOWER_DISTANCE_DELTA_CENTIMETER / 2) * 10,
                        DIRECTION_BACKWARD);
#else
                RobotCarPWMMotorControl.setSpeedPWMAndDirection(tSpeedPWM, DIRECTION_BACKWARD);
#endif

            } else if (aCentimeter <= FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
                /*
                 * Target is in the right distance -> stop
                 */
                if (!RobotCarPWMMotorControl.isStopped()) {
                    Serial.print(F("stop"));
                    RobotCarPWMMotorControl.stop(STOP_MODE_RELEASE); // stop only once
                } else {
                    Serial.print(F("ok"));
                }

            } else if (aCentimeter < FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {

                /*
                 * Target too far, but below scan threshold -> drive FORWARD with speed proportional to the gap
                 */
                tSpeedPWM = DEFAULT_DRIVE_SPEED_PWM + (aCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) * 2;
                if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
                    tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
                }
                Serial.print(F("SpeedPWM="));
                Serial.print(tSpeedPWM);

                if (RobotCarPWMMotorControl.getCarDirection() != DIRECTION_FORWARD) {
                    Serial.print(F(" -> go forward")); // print only once at direction change
                }

#if defined(USE_ENCODER_MOTOR_CONTROL)
                RobotCarPWMMotorControl.startGoDistanceMillimeter(tSpeedPWM,
                        ((aCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) + FOLLOWER_DISTANCE_DELTA_CENTIMETER / 2) * 10,
                        DIRECTION_FORWARD);
#else
                RobotCarPWMMotorControl.setSpeedPWMAndDirection(tSpeedPWM, DIRECTION_FORWARD);
#endif
            }
            Serial.println(); // Terminate the printed line
        }
    }
}

