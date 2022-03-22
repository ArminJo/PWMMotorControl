/*
 * MecanumWheelCarPWMMotorControl.h
 *
 *  Motor control for a car with 4 mecanum wheels driven by brushed DC motors
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
 */

#ifndef MECANUM_WHEEL_CAR_MOTOR_CONTROL_H
#define MECANUM_WHEEL_CAR_MOTOR_CONTROL_H

#include "PWMDcMotor.h"

#include <stdint.h>

#define FACTOR_DEGREE_TO_MILLIMETER_MECANUM_4WD_CAR_DEFAULT      5.0 // estimated, with slip

#if ! defined(FACTOR_DEGREE_TO_MILLIMETER_DEFAULT)
#define FACTOR_DEGREE_TO_MILLIMETER_DEFAULT  FACTOR_DEGREE_TO_MILLIMETER_MECANUM_4WD_CAR_DEFAULT

#endif

class MecanumWheelCarPWMMotorControl {
public:

    MecanumWheelCarPWMMotorControl();
//    virtual ~MecanumWheelCarPWMMotorControl();

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    void init();
#else
    void init(uint8_t aFrontRightMotorForwardPin, uint8_t aFrontRightMotorBackwardPin, uint8_t aFrontRightPWMPin,
            uint8_t aFrontLeftMotorForwardPin, uint8_t aFrontLeftMotorBackwardPin, uint8_t aBackRightMotorForwardPin,
            uint8_t aBackRightMotorBackwardPin, uint8_t aBackLeftMotorForwardPin, uint8_t aBackLeftMotorBackwardPin);
#endif

    /*
     * Functions for car with 4 mecanum wheels
     */
    void setSpeedPWM(uint8_t aRequestedSpeedPWM); // we assume, that all wheels use the same PWM
    void setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void setSpeedPWMAndDirectionAndDelay(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection, unsigned long aDelay);

    PWMDcMotor frontRightMotor;
    PWMDcMotor frontLeftMotor;
    PWMDcMotor backRightMotor;
    PWMDcMotor backLeftMotor;

};

extern MecanumWheelCarPWMMotorControl MecanumWheelCarMotorControl;

#endif /* MECANUM_WHEEL_CAR_MOTOR_CONTROL_H */

#pragma once

