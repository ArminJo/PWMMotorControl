/*
 *  RobotCarIRCommands.hpp
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_CAR_IR_COMMANDS_HPP
#define _ROBOT_CAR_IR_COMMANDS_HPP

#include "IRCommandDispatcher.h" // for RETURN_IF_STOP

/*
 * Basic IR functions
 */
void doStop();
void doReset();
void goForward();
void goBackward();
void turnLeft();
void turnRight();
void doDefaultSpeed();
#define SPEED_PWM_CHANGE_VALUE  16
void doIncreaseSpeed();
void doDecreaseSpeed();

void testRotation();
void testDrive();

/*
 * IR functions, which require code for distance measurement from Distance.hpp
 */
#if defined(_ROBOT_CAR_DISTANCE_HPP)
void doKeepDistance();
void doFollower();
void stepDistanceFeedbackMode();
void stepDistanceSourceMode();
void toggleDistanceScanSpeed();
bool sEnableFollower; // Follower mode with turn
bool sEnableKeepDistance; // Follower mode without a turn
#endif

void doStop() {
    RobotCar.stop();
#if defined(_ROBOT_CAR_DISTANCE_HPP)
    DistanceServoWriteAndDelay(90);
    sEnableFollower = false;
    sEnableKeepDistance = false;
#endif
}

void doReset() {
    doStop();
    RobotCar.setDefaultsForFixedDistanceDriving();
#if defined(_ROBOT_CAR_DISTANCE_HPP)
    sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    sDistanceSourceMode = DISTANCE_SOURCE_MODE_DEFAULT;
#  endif
    sDoSlowScan = false;
    sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
#endif
    noTone(PIN_BUZZER);
}

void goForward() {
    if (IRDispatcher.IRReceivedData.isRepeat) {
        // if repeat was pressed, we enable a "fast" stop
        RobotCar.startGoDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 4);
    } else {
        RobotCar.startGoDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER);
    }
}
void goBackward() {
    if (IRDispatcher.IRReceivedData.isRepeat) {
        RobotCar.startGoDistanceMillimeter(-DEFAULT_CIRCUMFERENCE_MILLIMETER / 4);
    } else {
        RobotCar.startGoDistanceMillimeter(-DEFAULT_CIRCUMFERENCE_MILLIMETER);
    }
}

void turnLeft() {
    RobotCar.startRotate(45, TURN_IN_PLACE, false);
}
void turnRight() {
    RobotCar.startRotate(-45, TURN_IN_PLACE, false);
}

void doDefaultSpeed() {
    RobotCar.setDefaultsForFixedDistanceDriving();
    RobotCar.rightCarMotor.printValues(&Serial);
}

void doIncreaseSpeed() {
    uint8_t tNewSpeed = RobotCar.rightCarMotor.DriveSpeedPWM + SPEED_PWM_CHANGE_VALUE;
    if (tNewSpeed < SPEED_PWM_CHANGE_VALUE) {
        // overflow happened here
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
 * Rotate by 9 times 10 degree in place with normal and with slow motion
 */
void testRotation() {
#define DEGREE_OF_TEST_ROTATION    10
#define NUMBER_OF_TEST_ROTATIONS    9 // to have 90 degree at 9 times 10 degree rotation
    Serial.println(F("Rotate 9 times for 10 degree"));
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(DEGREE_OF_TEST_ROTATION, TURN_FORWARD);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    Serial.println(F("Rotate back for 90 degree"));
    RobotCar.rotate(-(DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_FORWARD);
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(-DEGREE_OF_TEST_ROTATION, TURN_FORWARD);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    RobotCar.rotate((DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_FORWARD);
    DELAY_AND_RETURN_IF_STOP(2000);

    Serial.println(F("Now rotate in place and use slow speed"));
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(DEGREE_OF_TEST_ROTATION, TURN_IN_PLACE, true);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    RobotCar.rotate(-(DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_IN_PLACE, true);
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(-DEGREE_OF_TEST_ROTATION, TURN_IN_PLACE, true);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    RobotCar.rotate((DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_IN_PLACE, true);
    DELAY_AND_RETURN_IF_STOP(2000);
}

void testDrive() {
#define NUMBER_OF_TEST_DRIVES       2
    Serial.print(F("Move the wheels a full turn i.e. "));
    Serial.print(DEFAULT_CIRCUMFERENCE_MILLIMETER);
    Serial.println(F(" mm"));

    for (int i = 0; i < NUMBER_OF_TEST_DRIVES; ++i) {
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_DRIVES; ++i) {
        RobotCar.goDistanceMillimeter(-DEFAULT_CIRCUMFERENCE_MILLIMETER);
        DELAY_AND_RETURN_IF_STOP(500);
    }
}

void testCommand() {
    uint8_t tDirection = DIRECTION_FORWARD;
    for (int i = 0; i < 2; ++i) {
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 8, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 8, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 4, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 2, tDirection);
        DELAY_AND_RETURN_IF_STOP(1000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, tDirection);

        DELAY_AND_RETURN_IF_STOP(2000);
        tDirection = DIRECTION_BACKWARD;
    }
}

#if defined(_ROBOT_CAR_DISTANCE_HPP)
void doBeepFeedback(bool aEnableFlag) {
    tone(PIN_BUZZER, 2200, 50);
    delay(100);
    if (aEnableFlag) {
        tone(PIN_BUZZER, 2200, 50);
        delay(50);
    }
}

/*
 * Functions which require the include of Distance.hpp
 */
void doKeepDistance() {
    sEnableKeepDistance = !sEnableKeepDistance;
    doBeepFeedback(sEnableKeepDistance);
}

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
    Serial.print(F("DistanceFeedbackMode="));
    Serial.println(sDistanceFeedbackMode);
}

#if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
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
#endif

void toggleDistanceScanSpeed() {
    sDoSlowScan = !sDoSlowScan;
    doBeepFeedback(sDoSlowScan);
}

#endif
#endif // _ROBOT_CAR_IR_COMMANDS_HPP
