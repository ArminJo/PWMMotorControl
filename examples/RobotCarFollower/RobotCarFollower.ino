/*
 *  RobotCarFollower.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino.
 *  To find the target to follow, a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo scans the area on demand. (not yet implemented)
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#define MONITOR_VIN_VOLTAGE
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000
//#define USE_IR_REMOTE               // Use an IR remote receiver attached for car control

/*
 * Car configuration
 */
#define ENCODER_MOTOR_SHIELD_2WD_IR_TOF
//#define LAVWIN_IR_CONFIGURATION
#include "RobotCarConfigurations.h"
#include "RobotCarPinDefinitionsAndMore.h"

//#define DEBUG
#include "CarPWMMotorControl.h"
#include "CarPWMMotorControl.hpp"
CarPWMMotorControl RobotCarPWMMotorControl;

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
Servo DistanceServo;

#include "HCSR04.h"
#include "pitches.h"
#include "Distance.hpp" // requires definitions from RobotCarGui.h

#if defined(USE_IR_REMOTE)
#define USE_TINY_IR_RECEIVER // Supports only NEC protocol. Must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#define USE_LAFVIN_REMOTE
#include "IRCommandDispatcher.h" // for RETURN_IF_STOP
#include "RobotCarIRCommands.hpp"
#define INFO
#include "IRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"
#endif

#if defined(MONITOR_VIN_VOLTAGE)
#include "ADCUtils.h"
float sVINVoltage;
#endif

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT                 0

#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER        30 // If measured distance is less than this value, go backwards
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER        40 // If measured distance is greater than this value, go forward
#define FOLLOWER_DISTANCE_DELTA_CENTIMETER          (FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER - FOLLOWER_DISTANCE_MINIMUM_CENTIMETER)
#define FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER    70 // search if target moved to side

#if (DEFAULT_DRIVE_SPEED_PWM * 2) > 255
#define MAX_SPEED_PWM_FOLLOWER                     255
#else
#define MAX_SPEED_PWM_FOLLOWER                      (DEFAULT_DRIVE_SPEED_PWM * 2) // Max speed PWM value used for follower.
#endif

//#define INFO
void doFollowerOneStep(unsigned int aCentimeter);
void readCheckAndPrintVinPeriodically();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
        RobotCarPWMMotorControl.init();
#else
#  ifdef USE_ENCODER_MOTOR_CONTROL
        RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT, LEFT_MOTOR_FORWARD_PIN,
                LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_INTERRUPT);
#  else
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#  endif
#endif

    RobotCarPWMMotorControl.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set compensation

    /*
     * Set US servo to forward position
     */
    DistanceServo.attach(PIN_DISTANCE_SERVO);
    DistanceServo.write(90);

    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);

    /*
     * Do not start immediately with driving
     */
#ifdef USE_MPU6050_IMU
        /*
         * Wait after pressing the reset button, or attaching the power
         * and then take offset values for 1/2 second
         */
        delay(1000);
        tone(PIN_BUZZER, 2200, 50);
        delay(100);
        RobotCarPWMMotorControl.initIMU();
        RobotCarPWMMotorControl.printIMUOffsets(&Serial);
        tone(PIN_BUZZER, 2200, 50);
#endif
#if defined(USE_IR_REMOTE)
    IRDispatcher.init();
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_RECEIVE_PIN)));
#endif

    delay(2000);

    /*
     * Servo feedback for start of loop
     */
    DistanceServo.write(135);
    delay(500);
    DistanceServo.write(45);
    delay(500);
    DistanceServo.write(90);
    delay(500);

    Serial.println(F("Start loop"));
}

// Todo call update for ramps

void loop() {
    static uint8_t sLoopCount = 0;

#if defined(USE_IR_REMOTE)
    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested movement
     */
    IRDispatcher.checkAndRunSuspendedBlockingCommands();
#endif

    unsigned int tCentimeter = getDistanceAsCentimeterAndPlayTone();

    Serial.print(tCentimeter);
    Serial.print("cm, ");
    if ((sLoopCount & 0x07) == 0) {
        Serial.println();
        Serial.print("Distance=");
    }

#if defined(USE_IR_REMOTE)
    if (sEnableKeepDistance || sEnableFollower) {
        doFollowerOneStep(tCentimeter);
    }
#else
    doFollowerOneStep(tCentimeter);
#endif

#if defined(MONITOR_VIN_VOLTAGE)
    readCheckAndPrintVinPeriodically();
#endif

#ifdef USE_ENCODER_MOTOR_CONTROL
    RobotCarPWMMotorControl.delayAndUpdateMotors(100);
#else
    delay(100);
#endif
    sLoopCount++;
}

void doFollowerOneStep(unsigned int aCentimeter) {
    unsigned int tSpeedPWM;
    Serial.print(F(" -> "));

    if (aCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
        /*
         * Target too close -> drive BACKWARD with speed proportional to the gap
         */
        tSpeedPWM = DEFAULT_DRIVE_SPEED_PWM + (FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - aCentimeter) * 4;
        if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
            tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
        }

        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_BACKWARD) {
            Serial.println(F("go backward")); // print only once at direction change
        }
        Serial.print(F("SpeedPWM="));
        Serial.print(tSpeedPWM);

#ifdef USE_ENCODER_MOTOR_CONTROL
            RobotCarPWMMotorControl.startGoDistanceMillimeter(tSpeedPWM, ((aCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) + FOLLOWER_DISTANCE_DELTA_CENTIMETER / 2) * 10,
                    DIRECTION_BACKWARD);
#else
        RobotCarPWMMotorControl.setSpeedPWM(tSpeedPWM, DIRECTION_BACKWARD);
#endif

    } else if (aCentimeter < FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
        /*
         * Target is in the right distance -> stop
         */
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
            Serial.print(F("stop"));
            RobotCarPWMMotorControl.stop(MOTOR_RELEASE); // stop only once
        }
    } else {

#if defined(USE_IR_REMOTE)
        if (sEnableFollower && (aCentimeter == 0 || aCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER)) {
#else
        if (aCentimeter == 0 || aCentimeter > FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {
#endif
            /*
             * Distance too high or timeout / target not found -> search for target at different directions and turn if found
             */
            int_fast8_t tRotationDegree = scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
            // reset servo position
            DistanceServo.write(90);

            if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
                Serial.print(F(" stop and"));
                RobotCarPWMMotorControl.stop(MOTOR_RELEASE);
            }

            if (tRotationDegree != 0) {
                Serial.print(F(" rotate "));
                Serial.print(tRotationDegree);
                RobotCarPWMMotorControl.rotate(tRotationDegree, TURN_IN_PLACE, false); // do not use slow speed for turn
//            RobotCarPWMMotorControl.rotate(tRotationDegree);
            }

            delay(500); // Additional delay to let the servo reach its position
        } else {
            /*
             * Target too far -> drive FORWARD with speed proportional to the gap
             */
            tSpeedPWM = DEFAULT_DRIVE_SPEED_PWM + (aCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) * 2;
            if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
                tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
            }

            if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_FORWARD) {
                Serial.println(F("go forward")); // print only once at direction change
            }
            Serial.print(F("SpeedPWM="));
            Serial.print(tSpeedPWM);
#ifdef USE_ENCODER_MOTOR_CONTROL
                RobotCarPWMMotorControl.startGoDistanceMillimeter(tSpeedPWM, ((aCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) + FOLLOWER_DISTANCE_DELTA_CENTIMETER / 2) * 10,
                        DIRECTION_FORWARD);
#else
            RobotCarPWMMotorControl.setSpeedPWM(tSpeedPWM, DIRECTION_FORWARD);
#endif
        }
    }
    Serial.println();
}

#if defined(MONITOR_VIN_VOLTAGE)

void readVINVoltage() {
    uint16_t tVIN = waitAndReadADCChannelWithReferenceAndRestoreADMUX(VIN_11TH_IN_CHANNEL, INTERNAL);

//    BlueDisplay1.debug("VIN Raw=", tVIN);

// assume resistor network of 1MOhm / 100kOhm (divider by 11)
// tVIN * 0,01182795
#  if defined(VIN_VOLTAGE_CORRECTION)
    // we have a diode (requires 0.8 volt) between LIPO and VIN
    sVINVoltage = (tVIN * ((11.0 * 1.1) / 1023)) + VIN_VOLTAGE_CORRECTION;
#  else
    sVINVoltage = tVIN * ((11.0 * 1.1) / 1023);
#  endif
}

void readCheckAndPrintVinPeriodically() {
    static uint32_t sMillisOfLastVCCInfo;
    uint32_t tMillis = millis();

    if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
        sMillisOfLastVCCInfo = tMillis;
        readVINVoltage();
        Serial.print(F("VIN="));
        Serial.print(sVINVoltage);
        Serial.print(F("V "));
    }
}

#endif // MONITOR_VIN_VOLTAGE
