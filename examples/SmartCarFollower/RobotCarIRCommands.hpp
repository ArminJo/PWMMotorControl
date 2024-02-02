/*
 *  RobotCarIRCommands.hpp
 *
 *  Implementation of all commands required for IRCommandMapping.h / accessible by IR remote.
 *
 *  Copyright (C) 2022-2024  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_CAR_IR_COMMANDS_HPP
#define _ROBOT_CAR_IR_COMMANDS_HPP

#include "RobotCarIRCommands.h"

#include "IRCommandDispatcher.h" // for RETURN_IF_STOP
#include "RobotCarUtils.h"

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

/*
 * IR functions, which require code for distance measurement from Distance.hpp
 */
#if defined(_ROBOT_CAR_DISTANCE_HPP)
bool sEnableFollower;    // Follower mode activated
bool sEnableScanAndTurn; // Follower mode with scan and turn
#endif

void doStop() {
    RobotCar.stop();
#if defined(_ROBOT_CAR_DISTANCE_HPP)
    sEnableFollower = false;
    sEnableScanAndTurn = false;
    DistanceServoWriteAndWaitForStop(90);
#endif
}

/*
 * Reset i.e. stop car and set all values to default.
 */
void doReset() {
    doStop();
    RobotCar.setDefaultsForFixedDistanceDriving();

#if defined(_ROBOT_CAR_DISTANCE_HPP)
    sDoSlowScan = false;
    sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    sDistanceSourceMode = DISTANCE_SOURCE_MODE_DEFAULT;
#  endif
#endif
#if defined(MILLIS_OF_INACTIVITY_BEFORE_ATTENTION)
    sMillisOfLastMovement = 0; // to start follower without initial scanning
#endif
    noTone(BUZZER_PIN);
}

/*
 * Drive forward / backward for a complete wheel turn. Each repeat will extend the distance by another 1/4 of a wheel turn.
 */
void goForward() {
    if (IRDispatcher.IRReceivedData.isRepeat) {
        // if repeat was pressed, we enable a "fast" stop
        RobotCar.startGoDistanceMillimeter(RobotCar.rightCarMotor.DriveSpeedPWM, DEFAULT_CIRCUMFERENCE_MILLIMETER / 4,
                DIRECTION_FORWARD);
    } else {
        RobotCar.startGoDistanceMillimeter(RobotCar.rightCarMotor.DriveSpeedPWM, DEFAULT_CIRCUMFERENCE_MILLIMETER,
                DIRECTION_FORWARD);
    }
}
void goBackward() {
    if (IRDispatcher.IRReceivedData.isRepeat) {
        RobotCar.startGoDistanceMillimeter(RobotCar.rightCarMotor.DriveSpeedPWM, DEFAULT_CIRCUMFERENCE_MILLIMETER / 4,
                DIRECTION_BACKWARD);
    } else {
        RobotCar.startGoDistanceMillimeter(RobotCar.rightCarMotor.DriveSpeedPWM, DEFAULT_CIRCUMFERENCE_MILLIMETER,
                DIRECTION_BACKWARD);
    }
}

/*
 * Turn in place left / right by around 15 degree.
 */
void turnLeft() {
    RobotCar.startRotate(15, TURN_IN_PLACE);
}
void turnRight() {
    RobotCar.startRotate(-15, TURN_IN_PLACE);
}

/*
 * Set driving speed PWM to default, which depends on motor supply voltage.
 */
void doDefaultSpeed() {
    RobotCar.setDriveSpeedPWM(RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
    RobotCar.rightCarMotor.printValues(&Serial);
}

/*
 * Increase / decrease driving speed PWM by 1/16 of max PWM, but clip at start speed PWM, which depends on motor supply voltage.
 */
void doIncreaseSpeed() {
    uint8_t tNewSpeed = RobotCar.rightCarMotor.DriveSpeedPWM + SPEED_PWM_CHANGE_VALUE;
    if (tNewSpeed < SPEED_PWM_CHANGE_VALUE) {
        // unsigned overflow happened here
        tNewSpeed = MAX_SPEED_PWM;
    }
    RobotCar.setDriveSpeedPWM(tNewSpeed);
    RobotCar.rightCarMotor.printValues(&Serial);
}

void doDecreaseSpeed() {
    uint8_t tNewSpeed = RobotCar.rightCarMotor.DriveSpeedPWM - SPEED_PWM_CHANGE_VALUE;
    if (tNewSpeed < DEFAULT_START_SPEED_PWM) {
        tNewSpeed = DEFAULT_START_SPEED_PWM;
    }
    RobotCar.setDriveSpeedPWM(tNewSpeed);
    RobotCar.rightCarMotor.printValues(&Serial);
}

/*
 * First measure the motor supply voltage under normal load, i.e the fixed DEFAULT_DRIVE_SPEED_PWM, while turning in place.
 */
void doCalibrate() {
    RobotCar.readCarValuesFromEeprom();

#if defined(VIN_ATTENUATED_INPUT_PIN)
    calibrateDriveSpeedPWMAndPrint();
#endif

#if !defined(USE_MPU6050_IMU) && (defined(_IR_COMMAND_DISPATCHER_HPP) || defined(VERSION_BLUE_DISPLAY)) \
    && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))
    // Not for 4WD cars with IMU or 2WD car with encoder motor.
    delay(500);
    /*
     * Start in place rotation calibration
     */
    if (calibrateRotation(TURN_IN_PLACE)) {
        return;
    }
    DELAY_AND_RETURN_IF_STOP(2000);
    // Now show 90 degree
    RobotCar.rotate(90, TURN_IN_PLACE);
    DELAY_AND_RETURN_IF_STOP(500);
    RobotCar.rotate(-90, TURN_IN_PLACE);
    DELAY_AND_RETURN_IF_STOP(4000);
    /*
     * Start forward rotation calibration
     */
    if (calibrateRotation(TURN_FORWARD)) {
        return;
    }
    DELAY_AND_RETURN_IF_STOP(2000);
    // Now show 90 degree
    RobotCar.rotate(90, TURN_FORWARD);
    DELAY_AND_RETURN_IF_STOP(500);
    RobotCar.rotate(-90, TURN_FORWARD);
#endif
    Serial.println(F("Store values to EEPROM"));
    RobotCar.printCalibrationValues(&Serial);
    RobotCar.writeCarValuesToEeprom();
}

/*
 * Drive first testDriveTwoTurnsBothDirections then testDriveTwoTurnsIn5PartsBothDirections.
 */
void doTestDrive() {
    testDriveTwoTurnsBothDirections();
    delay(2000);
    testDriveTwoTurnsIn5PartsBothDirections();
}

/*
 * Drive the car for 2 times 1/8, wheel turn, then 1/ and 1/2 wheel turn. First forward, then backward.
 * If distance driving formula and values are correct, this results in 2 full wheel turns ending at the start position.
 */
void doTestCommand() {
    testDriveTwoTurnsIn5PartsBothDirections();
}

void doTestRotation() {
    testRotation();
}

/*
 * Beep once after a delay if false, resulting in
 */
void doAdditionalBeepFeedback(bool aDoBeep) {
    if (aDoBeep) {
        delay(100);
        tone(BUZZER_PIN, 2200, 50);
        delay(50);
    }
}

#if defined(_ROBOT_CAR_DISTANCE_HPP)
/*
 * Functions which require the include of Distance.hpp
 */
/*
 * Toggle keep distance mode.
 */
void doKeepDistance() {
    if(sEnableScanAndTurn) {
        // the last command was doFollower
        sEnableFollower = true;
    } else {
        sEnableFollower = !sEnableFollower;
    }
    sEnableScanAndTurn = false;
    doAdditionalBeepFeedback(!sEnableFollower);
}

/*
 * Toggle follower mode (the one that scans)
 */
void doFollower() {
    if(!sEnableScanAndTurn) {
        // the last command was doKeepDistance
        sEnableFollower = true;
    } else {
        sEnableFollower = !sEnableFollower;
    }
    sEnableScanAndTurn = true;
    doAdditionalBeepFeedback(!sEnableFollower);
}

/*
 * This steps the distance feedback modes:
 * - No feedback tone (default).
 * - Pentatonic frequency feedback tone.
 * - Continuous frequency feedback tone.
 */
void stepDistanceFeedbackMode() {
    sDistanceFeedbackMode++;
    if (sDistanceFeedbackMode > DISTANCE_FEEDBACK_MAX) {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
        noTone(BUZZER_PIN);
    }
    Serial.print(F("DistanceFeedbackMode="));
    Serial.println(sDistanceFeedbackMode);
}

#  if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
/*
 * Step distance source, if a IR distance (Sharp GP2Y0A21YK / 1080) as well as an ultrasonic distance (HC-SR04) sensor are connected.
 *  - Use IR as distance sensor (default).
 *  - Use US as distance sensor.
 *  - Use minimum of both sensors as distance.
 *  - Use maximum of both sensors as distance.
 */
void stepDistanceSourceMode() {
    sDistanceSourceMode++;
    Serial.print(F("DistanceSourceMode="));
    Serial.print(sDistanceSourceMode);
    Serial.print(F(" | "));
    switch (sDistanceSourceMode) {
    case DISTANCE_SOURCE_MODE_MAXIMUM:
        Serial.println(F("Max"));
        break;
    case DISTANCE_SOURCE_MODE_US:
        Serial.println(F("US"));
        break;
    case DISTANCE_SOURCE_MODE_IR_OR_TOF:
        Serial.println(F("IR"));
        break;
    default:
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_MINIMUM;
        Serial.println(F("Min"));
        break;
    }
}
#  endif

/*
 * Toggle scan speed of distance servo
 */
void toggleDistanceScanSpeed() {
    doAdditionalBeepFeedback(sDoSlowScan);
    sDoSlowScan = !sDoSlowScan;
}
#endif // defined(_ROBOT_CAR_DISTANCE_HPP)

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _ROBOT_CAR_IR_COMMANDS_HPP
