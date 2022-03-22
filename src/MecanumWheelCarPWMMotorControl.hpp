/*
 * MecanumWheelCarPWMMotorControl.hpp
 *
 *  Contains functions for control of the 4 motors of a car equipped with 4 mecanum wheels.
 * *
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
 */

#ifndef MECANUM_WHEEL_CAR_MOTOR_CONTROL_HPP
#define MECANUM_WHEEL_CAR_MOTOR_CONTROL_HPP

#include <Arduino.h>

#include "PWMDcMotor.hpp"
#include "MecanumWheelCarPWMMotorControl.h"

/*
 * The Car Control instance to be used by the main program
 */
MecanumWheelCarPWMMotorControl MecanumWheelCarMotorControl;

//#define DEBUG // Only for development

MecanumWheelCarPWMMotorControl::MecanumWheelCarPWMMotorControl() { // @suppress("Class members should be properly initialized")
}

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
void CarPWMMotorControl::init() {
#  if defined(USE_ENCODER_MOTOR_CONTROL)
    frontLeftMotor.init(1, INT1);
    frontRightMotor.init(2, INT0);
#  else
    frontLeftMotor.init(1);
    frontRightMotor.init(2);
#  endif

    backLeftMotor.init(3);
    backRightMotor.init(4);

//    FactorDegreeToMillimeter = FACTOR_DEGREE_TO_MILLIMETER_DEFAULT;
}
#else

void MecanumWheelCarPWMMotorControl::init(uint8_t aFrontRightMotorForwardPin, uint8_t aFrontRightMotorBackwardPin, uint8_t aPWMPin,
        uint8_t aFrontLeftMotorForwardPin, uint8_t aFrontLeftMotorBackwardPin, uint8_t aBackRightMotorForwardPin,
        uint8_t aBackRightMotorBackwardPin, uint8_t aBackLeftMotorForwardPin, uint8_t aBackLeftMotorBackwardPin) {
    frontRightMotor.init(aFrontRightMotorForwardPin, aFrontRightMotorBackwardPin, aPWMPin);
    frontLeftMotor.init(aFrontLeftMotorForwardPin, aFrontLeftMotorBackwardPin, aPWMPin);
    backRightMotor.init(aBackRightMotorForwardPin, aBackRightMotorBackwardPin, aPWMPin);
    backLeftMotor.init(aBackLeftMotorForwardPin, aBackLeftMotorBackwardPin, aPWMPin);
}
#endif

void MecanumWheelCarPWMMotorControl::setSpeedPWM(uint8_t aRequestedSpeedPWM) {
    frontRightMotor.setSpeed(aRequestedSpeedPWM); // we assume, that all motors share the same PWM pin
}

/*
 * @param aRequestedSpeedPWM    any value between 0 and MAX_SPEED_PWM (255)
 * @param aRequestedDirection   a combination of:
 *          DIRECTION_FORWARD, DIRECTION_BACKWARD, DIRECTION_STOP
 *          DIRECTION_LEFT, DIRECTION_RIGHT, DIRECTION_STRAIGHT and DIRECTION_TURN
 *          e.g. DIRECTION_BACKWARD | DIRECTION_RIGHT | DIRECTION_TURN
 */
void MecanumWheelCarPWMMotorControl::setSpeedPWMAndDirectionAndDelay(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection,
        unsigned long aDelay) {
    setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
    delay(aDelay);
    setSpeedPWM(0);

}
void MecanumWheelCarPWMMotorControl::setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
    uint8_t tRequestedForwardBackwardDirection = aRequestedDirection & DIRECTION_AND_STOP_MODE_MASK;
#ifdef DEBUG
    Serial.print(F("Speed="));
    Serial.print(aRequestedSpeedPWM);
    Serial.print(F(" forward/backward direction="));
    Serial.print(tRequestedForwardBackwardDirection);
    Serial.print(F(" left/right="));
    Serial.print(aRequestedDirection & DIRECTION_LEFT_RIGHT_MASK);
    Serial.print(F(" turn="));
    Serial.print(aRequestedDirection & DIRECTION_TURN);
    Serial.println();
#endif

    uint8_t tFrontLeftMotorDirection;
    uint8_t tBackLeftMotorDirection;
    uint8_t tFrontRightMotorDirection;
    uint8_t tBackRightMotorDirection;

    /*
     * We set all for straight or movements to the left,  movements to the right are implemented by just swapping left and right motors
     */
    if (aRequestedDirection & DIRECTION_TURN) {
        /*
         * TURN requested
         * We have TURN and FOWARD -> turn center is front axis, TURN and BACKWARD  -> turn center is back axis
         * and TURN and STOP -> turn center is car center
         */
        // default for turn is brake
        tFrontLeftMotorDirection = MOTOR_BRAKE;
        tBackLeftMotorDirection = MOTOR_BRAKE;
        tFrontRightMotorDirection = MOTOR_BRAKE;
        tBackRightMotorDirection = MOTOR_BRAKE;
        // All 4 wheels for CENTER TURN LEFT
        if (tRequestedForwardBackwardDirection == DIRECTION_STOP || tRequestedForwardBackwardDirection == DIRECTION_FORWARD) {
            // FRONT TURN LEFT
            tFrontLeftMotorDirection = DIRECTION_BACKWARD;
            tFrontRightMotorDirection = DIRECTION_FORWARD;
        }
        if (tRequestedForwardBackwardDirection == DIRECTION_STOP || tRequestedForwardBackwardDirection == DIRECTION_BACKWARD) {
            // BACK TURN LEFT
            tBackLeftMotorDirection = DIRECTION_BACKWARD;
            tBackRightMotorDirection = DIRECTION_FORWARD;
        }
    } else if (aRequestedDirection & DIRECTION_LEFT_RIGHT_MASK) {
        /*
         * LEFT or RIGHT requested
         */
        if (aRequestedDirection & STOP_MODE_MASK) {
            // STRAIGHT LEFT
            tFrontLeftMotorDirection = DIRECTION_BACKWARD;
            tBackLeftMotorDirection = DIRECTION_FORWARD;
            tFrontRightMotorDirection = DIRECTION_FORWARD;
            tBackRightMotorDirection = DIRECTION_BACKWARD;
        } else {
            if (tRequestedForwardBackwardDirection == DIRECTION_FORWARD) {
                // FORWARD DIAGONAL LEFT
                tFrontLeftMotorDirection = MOTOR_BRAKE;
                tBackLeftMotorDirection = DIRECTION_FORWARD;
                tFrontRightMotorDirection = DIRECTION_FORWARD;
                tBackRightMotorDirection = MOTOR_BRAKE;
            } else {
                // BACKWARD DIAGONAL LEFT
                tFrontLeftMotorDirection = DIRECTION_BACKWARD;
                tBackLeftMotorDirection = MOTOR_BRAKE;
                tFrontRightMotorDirection = MOTOR_BRAKE;
                tBackRightMotorDirection = DIRECTION_BACKWARD;
            }
        }
    } else {
        /*
         * FORWARD or BACKWARD or STOP
         */
        tFrontLeftMotorDirection = tRequestedForwardBackwardDirection;
        tBackLeftMotorDirection = tRequestedForwardBackwardDirection;
        tFrontRightMotorDirection = tRequestedForwardBackwardDirection;
        tBackRightMotorDirection = tRequestedForwardBackwardDirection;
    }

    if (aRequestedDirection & DIRECTION_RIGHT) {
        // Swap left and right motors
        frontLeftMotor.setDirection(tFrontRightMotorDirection);
        backLeftMotor.setDirection(tBackRightMotorDirection);
        frontRightMotor.setDirection(tFrontLeftMotorDirection);
        backRightMotor.setDirection(tBackLeftMotorDirection);
    } else {
        frontLeftMotor.setDirection(tFrontLeftMotorDirection);
        backLeftMotor.setDirection(tBackLeftMotorDirection);
        frontRightMotor.setDirection(tFrontRightMotorDirection);
        backRightMotor.setDirection(tBackRightMotorDirection);
    }

    frontRightMotor.setSpeed(aRequestedSpeedPWM);
}

#endif // MECANUM_WHEEL_CAR_MOTOR_CONTROL_HPP
#pragma once
