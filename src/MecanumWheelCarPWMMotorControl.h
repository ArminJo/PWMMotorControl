/*
 * MecanumWheelCarPWMMotorControl.h
 *
 *  Contains functions for control of the 4 motors of a mecanum wheel car.
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
 */

#ifndef _MECANUM_WHEEL_CAR_PWM_MOTOR_CONTROL_H
#define _MECANUM_WHEEL_CAR_PWM_MOTOR_CONTROL_H

#if defined(CAR_HAS_4_MECANUM_WHEELS)

#include "CarPWMMotorControl.h"

class MecanumWheelCarPWMMotorControl : public CarPWMMotorControl {
public:

    MecanumWheelCarPWMMotorControl();
//    virtual ~MecanumWheelCarPWMMotorControl();

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    void init();
#else
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aPWMPin, uint8_t aLeftMotorForwardPin,
            uint8_t aLeftMotorBackwardPin, uint8_t aBackRightCarMotorForwardPin, uint8_t aBackRightCarMotorBackwardPin,
            uint8_t aBackLeftCarMotorForwardPin, uint8_t aBackLeftCarMotorBackwardPin);
#endif // USE_ADAFRUIT_MOTOR_SHIELD

    void setDefaultsForFixedDistanceDriving();
    void setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM, int8_t aSpeedPWMCompensationRight);
    void setSpeedPWMCompensation(int8_t aSpeedPWMCompensationRight);
    void changeSpeedPWMCompensation(int8_t aSpeedPWMCompensationRightDelta);
    void setDriveSpeedPWM(uint8_t aDriveSpeedPWM);
    void setDriveSpeedPWMFor2Volt(uint16_t aFullBridgeInputVoltageMillivolt);
    void setDriveSpeedPWMFor2Volt(float aFullBridgeInputVoltageMillivolt);

    void writeMotorValuesToEeprom();
    void readMotorValuesFromEeprom();

#if !defined(USE_ENCODER_MOTOR_CONTROL)
    // makes no sense for encoder motor
    void setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond);
#endif

    // If ramp up is not supported, these functions just sets the speed and return immediately
    void startRampUp(uint8_t aRequestedDirection = DIRECTION_FORWARD);
    void setSpeedPWMWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void waitForDriveSpeedPWM(void (*aLoopCallback)(void) = NULL);
    void startRampUpAndWait(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection = DIRECTION_FORWARD,
            void (*aLoopCallback)(void) = NULL);
    void startRampUpAndWaitForDriveSpeedPWM(uint8_t aRequestedDirection = DIRECTION_FORWARD, void (*aLoopCallback)(void) = NULL);

    /*
     * Functions for moving a fixed distance
     */
    // With signed distance
    void startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
            uint8_t aRequestedDirection); // only setup values
    void startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection); // only setup values
    void startGoDistanceMillimeter(int aRequestedDistanceMillimeter); // only setup values, no movement -> use updateMotors()

    void goDistanceMillimeter(int aRequestedDistanceMillimeter, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilStopped
    void goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection,
            void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilStopped

    bool checkAndHandleDirectionChange(uint8_t aRequestedDirection); // used internally

    /*
     * Functions for rotation
     */
    void startRotate(int aRotationDegrees, turn_direction_t aTurnDirection, bool aUseSlowSpeed = false);
    void rotate(int aRotationDegrees, turn_direction_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = false,
            void (*aLoopCallback)(void) = NULL);

    bool updateMotors();    bool updateMotors(void (*aLoopCallback)(void));

    void delayAndUpdateMotors(unsigned int aDelayMillis);

    /*
     * Start/Stop functions
     */
    void startRampDown();
    void stopAndWaitForIt(void (*aLoopCallback)(void) = NULL); // uses waitUntilStopped()
    void waitUntilStopped(void (*aLoopCallback)(void) = NULL);

    /*
     * Check motor state functions
     */
    bool isStopped();
    bool isState(uint8_t aState);
    bool isStateRamp(); // MOTOR_STATE_RAMP_UP OR MOTOR_STATE_RAMP_DOWN

    /*
     * Functions, which directly call motor functions for both motors
     */
    void setSpeedPWM(uint8_t aRequestedSpeedPWM);
    void setDirection(uint8_t aRequestedDirection);

    void setSpeedPWMAndDirection(int aRequestedSpeedPWM);
    void setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void setSpeedPWMWithDeltaAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection, int8_t aSpeedPWMCompensationRightDelta);
    void changeSpeedPWM(uint8_t aRequestedSpeedPWM); // Keeps direction

    void stop(uint8_t aStopMode = STOP_MODE_KEEP); // STOP_MODE_KEEP (take previously defined DefaultStopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
    void setStopMode(uint8_t aStopMode);

    /*
     * Additions for mecanum wheel car
     */
    void setSpeedPWMAndDirectionAndDelay(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection, unsigned long aDelay);

    void moveStar(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveFullStar(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveSqare(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveHexagon(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveTriangle0(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveTriangle45(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveBigPlus(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveRhombus(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveTrapezium(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);
    void moveTestDistances(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove, unsigned int aDelayBetweenMoves = 0);

    PWMDcMotor backRightCarMotor;
    PWMDcMotor backLeftCarMotor;
};

extern MecanumWheelCarPWMMotorControl RobotCar;

#endif // defined(CAR_HAS_4_MECANUM_WHEELS)
#endif // _MECANUM_WHEEL_CAR_PWM_MOTOR_CONTROL_H
