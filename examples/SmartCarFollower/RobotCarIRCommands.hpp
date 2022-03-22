/*
 *  RobotCarIRCommands.cpp
 *
 *  Implementation of all commands required for IRCommandMapping.h / accessible by IR remote.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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

#ifndef ROBOT_CAR_IR_COMMANDS_HPP
#define ROBOT_CAR_IR_COMMANDS_HPP

#include <Arduino.h>

void doBeepFeedback(bool aEnableFlag);
void doStop();
void doReset();
void goForward();
void goBackward();
void doDefaultSpeed();
void doIncreaseSpeed();
void doDecreaseSpeed();
void turnLeft();
void turnRight();
void testRotation();
void testDrive();

bool sEnableKeepDistance; // Follower mode without a turn
void doKeepDistance() {
    sEnableKeepDistance = !sEnableKeepDistance;
    doBeepFeedback(sEnableKeepDistance);
}

bool sEnableFollower; // Follower mode with turn
void doFollower() {
    sEnableFollower = !sEnableFollower;
    doBeepFeedback(sEnableFollower);
}

void stepDistanceFeedbackMode() {
    sDistanceFeedbackMode++;
    if (sDistanceFeedbackMode > DISTANCE_FEEDBACK_MAX) {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
        noTone(PIN_BUZZER);
    }
}

#if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
void stepDistanceSourceMode() {
    sDistanceSourceMode++;
    Serial.print(F("DistanceSourceMode="));
    Serial.print(sDistanceSourceMode);
    Serial.print(F(" / "));
    switch (sDistanceSourceMode) {
    case DISTANCE_SOURCE_MODE_MINIMUM:
        Serial.println(F("Min"));
        break;
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
        break;
    }
}
#endif

void toggleDistanceScanSpeed() {
    sDoSlowScan = !sDoSlowScan;
    doBeepFeedback(sDoSlowScan);
}

void doBeepFeedback(bool aEnableFlag) {
    tone(PIN_BUZZER, 2200, 50);
    delay(100);
    if (aEnableFlag) {
        tone(PIN_BUZZER, 2200, 50);
        delay(50);
    }
}

void doStop() {
    RobotCarPWMMotorControl.stop();
    DistanceServo.write(90);
    sEnableFollower = false;
    sEnableKeepDistance = false;
}

void doReset() {
    doStop();
    RobotCarPWMMotorControl.setDefaultsForFixedDistanceDriving();
    sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    sDistanceSourceMode = DISTANCE_SOURCE_MODE_DEFAULT;
#endif
    sDoSlowScan = false;
    sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
    noTone(PIN_BUZZER);
}

void doDefaultSpeed() {
    RobotCarPWMMotorControl.setDefaultsForFixedDistanceDriving();
    RobotCarPWMMotorControl.rightCarMotor.printValues(&Serial);
}

#define SPEED_PWM_CHANGE_VALUE  16
void doIncreaseSpeed() {
    uint8_t tNewSpeed = RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM + SPEED_PWM_CHANGE_VALUE;
    if (tNewSpeed < SPEED_PWM_CHANGE_VALUE) {
        // overflow happened here
        tNewSpeed = MAX_SPEED_PWM;
    }
    RobotCarPWMMotorControl.setDriveSpeedPWM(tNewSpeed);
    RobotCarPWMMotorControl.rightCarMotor.printValues(&Serial);
}

void doDecreaseSpeed() {
    uint8_t tNewSpeed = RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM - SPEED_PWM_CHANGE_VALUE;
    if (tNewSpeed < DEFAULT_START_SPEED_PWM) {
        tNewSpeed = DEFAULT_START_SPEED_PWM;
    }
    RobotCarPWMMotorControl.setDriveSpeedPWM(tNewSpeed);
    RobotCarPWMMotorControl.rightCarMotor.printValues(&Serial);
}

void goForward() {
    RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER);
}
void goBackward() {
    RobotCarPWMMotorControl.goDistanceMillimeter(-DEFAULT_CIRCUMFERENCE_MILLIMETER);
}

void turnLeft() {
    RobotCarPWMMotorControl.rotate(45, TURN_IN_PLACE, false);
}
void turnRight() {
    RobotCarPWMMotorControl.rotate(-45, TURN_IN_PLACE, false);
}

/*
 * Rotate by 9 times 10 degree in place with normal and with slow motion
 */
void testRotation() {
#define DEGREE_OF_TEST_ROTATION    10
#define NUMBER_OF_TEST_ROTATIONS    9 // to have 90 degree at 9 times 10 degree rotation
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCarPWMMotorControl.rotate(DEGREE_OF_TEST_ROTATION);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCarPWMMotorControl.rotate(-DEGREE_OF_TEST_ROTATION);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCarPWMMotorControl.rotate(DEGREE_OF_TEST_ROTATION, TURN_IN_PLACE, true);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCarPWMMotorControl.rotate(-DEGREE_OF_TEST_ROTATION, TURN_IN_PLACE, true);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);
}

void testDrive() {
#define NUMBER_OF_TEST_DRIVES       2
    for (int i = 0; i < NUMBER_OF_TEST_DRIVES; ++i) {
        RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_DRIVES; ++i) {
        RobotCarPWMMotorControl.goDistanceMillimeter(-DEFAULT_CIRCUMFERENCE_MILLIMETER);
        DELAY_AND_RETURN_IF_STOP(500);
    }
}

void testCommand() {
    uint8_t tDirection = DIRECTION_FORWARD;
    for (int i = 0; i < 2; ++i) {
        RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 8, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 8, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 4, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 2, tDirection);
        DELAY_AND_RETURN_IF_STOP(1000);
        RobotCarPWMMotorControl.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, tDirection);

        DELAY_AND_RETURN_IF_STOP(2000);
        tDirection = DIRECTION_BACKWARD;
    }
}

#endif // #ifndef ROBOT_CAR_IR_COMMANDS_HPP
#pragma once
