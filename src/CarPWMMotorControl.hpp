/*
 * CarPWMMotorControl.hpp
 *
 *  Contains functions for control of the 2 motors of a car like setDirection, goDistanceMillimeter() and rotate().
 *  Checks input of PIN aPinFor2WDDetection since we need different factors for rotating a 4 wheel and a 2 wheel car.
 *
 *  Requires EncoderMotor.hpp
 *
 *  Created on: 12.05.2019
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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

#ifndef _CAR_PWM_MOTOR_CONTROL_HPP
#define _CAR_PWM_MOTOR_CONTROL_HPP

#include <Arduino.h>

#if defined(USE_MPU6050_IMU)
#include "IMUCarData.hpp"
#endif

#if defined(USE_ENCODER_MOTOR_CONTROL)
#include "EncoderMotor.hpp"
#endif

#include "PWMDcMotor.hpp"
#include "CarPWMMotorControl.h"

/*
 * The Car Control instance to be used by the main program
 */
CarPWMMotorControl RobotCarPWMMotorControl;

//#define DEBUG // Only for development

CarPWMMotorControl::CarPWMMotorControl() { // @suppress("Class members should be properly initialized")
}

#if defined(USE_MPU6050_IMU)
/*
 * This must be done when the car is not moving, best after at least 100 ms after boot up.
 */
void CarPWMMotorControl::calculateAndPrintIMUOffsets(Print *aSerial) {
    IMUData.calculateSpeedAndTurnOffsets();
    IMUData.printSpeedAndTurnOffsets(aSerial);
}
#endif

/*
 * If no parameter and we have encoder motors, we use a fixed assignment of rightCarMotor interrupts to INT0 / Pin2 and leftCarMotor to INT1 / Pin3
 */
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
void CarPWMMotorControl::init() {
#  if defined(USE_ENCODER_MOTOR_CONTROL)
    leftCarMotor.init(1, INT1);
    rightCarMotor.init(2, INT0);

#  else
    leftCarMotor.init(1);
    rightCarMotor.init(2);
#  endif
#  if defined(CAR_HAS_4_MECANUM_WHEELS)
    // we have two wheels without encoders
    backLeftCarMotor.init(3);
    backRightCarMotor.init(4);
#  endif
#  if defined(USE_MPU6050_IMU)
    CarRequestedRotationDegrees = 0;
    CarRequestedDistanceMillimeter = 0;
    IMUData.initMPU6050FifoForCarData();
#  else
    FactorDegreeToMillimeter = FACTOR_DEGREE_TO_MILLIMETER_DEFAULT;
#  endif
}

#else // USE_ADAFRUIT_MOTOR_SHIELD
#  if defined(CAR_HAS_4_MECANUM_WHEELS)
void CarPWMMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aPWMPin,
        uint8_t aLeftMotorForwardPin, uint8_t aLeftMotorBackwardPin, uint8_t aBackRightMotorForwardPin,
        uint8_t aBackRightMotorBackwardPin, uint8_t aBackLeftMotorForwardPin, uint8_t aBackLeftMotorBackwardPin) {
    leftCarMotor.init(aLeftMotorForwardPin, aLeftMotorBackwardPin, aPWMPin);
    rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aPWMPin);
    backRightCarMotor.init(aBackRightMotorForwardPin, aBackRightMotorBackwardPin, aPWMPin);
    backLeftCarMotor.init(aBackLeftMotorForwardPin, aBackLeftMotorBackwardPin, aPWMPin);

#  else
void CarPWMMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin,
        uint8_t aLeftMotorForwardPin, uint8_t aLeftMotorBackwardPin, uint8_t aLeftMotorPWMPin) {
    leftCarMotor.init(aLeftMotorForwardPin, aLeftMotorBackwardPin, aLeftMotorPWMPin);
    rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin);
#  endif // defined(CAR_HAS_4_MECANUM_WHEELS)
    CarDirection = DIRECTION_STOP;

#  if defined(USE_MPU6050_IMU)
    CarRequestedRotationDegrees = 0;
    CarRequestedDistanceMillimeter = 0;
    IMUData.initMPU6050FifoForCarData();

#  else
    FactorDegreeToMillimeter = FACTOR_DEGREE_TO_MILLIMETER_DEFAULT;
#  endif // defined(USE_MPU6050_IMU)

#  if defined(USE_ENCODER_MOTOR_CONTROL)
    /*
     * Take default interrupt configuration
     * Slot type optocoupler interrupts on pin D2 + D3
     */
#    if defined (INT0)
    rightCarMotor.attachEncoderInterrupt(INT0);
    leftCarMotor.attachEncoderInterrupt(INT1);
#    else
    rightCarMotor.attachEncoderInterrupt(0); // We use interrupt numbers here, not pin numbers
    rightCarMotor.attachEncoderInterrupt(1);
#    endif
#  endif // defined(USE_ENCODER_MOTOR_CONTROL)
}
#    if defined(USE_ENCODER_MOTOR_CONTROL)
/*
 * With parameters aRightInterruptNumber + aLeftInterruptNumber
 */
void CarPWMMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin,
        uint8_t aRightInterruptNumber, uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin,
        uint8_t aLeftInterruptNumber) {
    leftCarMotor.init(aLeftMotorForwardPin, LeftMotorBackwardPin, aLeftMotorPWMPin, aLeftInterruptNumber);
    rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin, aRightInterruptNumber);

#    if defined(USE_MPU6050_IMU)
    CarRequestedRotationDegrees = 0;
    CarRequestedDistanceMillimeter = 0;
    IMUData.initMPU6050FifoForCarData();
#    else
    FactorDegreeToMillimeter = FACTOR_DEGREE_TO_MILLIMETER_DEFAULT;
#    endif
}
#  endif // USE_ENCODER_MOTOR_CONTROL
#endif // USE_ADAFRUIT_MOTOR_SHIELD

/*
 * Sets default values for min and max speed, factor for distance to time conversion for non encoder motors and reset speed compensation
 * Is called automatically at init if parameter aReadFromEeprom is set to false
 */
void CarPWMMotorControl::setDefaultsForFixedDistanceDriving() {
    rightCarMotor.setDefaultsForFixedDistanceDriving();
    leftCarMotor.setDefaultsForFixedDistanceDriving();
}

/**
 * @param aSpeedPWMCompensationRight if positive, this value is added to the compensation value of the right motor, or subtracted from the left motor value.
 *  If negative, -value is added to the compensation value the left motor, or subtracted from the right motor value.
 */
void CarPWMMotorControl::setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM, int8_t aSpeedPWMCompensationRight) {
    if (aSpeedPWMCompensationRight >= 0) {
        rightCarMotor.setDriveSpeedAndSpeedCompensationPWM(aDriveSpeedPWM, aSpeedPWMCompensationRight);
        leftCarMotor.setDriveSpeedAndSpeedCompensationPWM(aDriveSpeedPWM, 0);
    } else {
        rightCarMotor.setDriveSpeedAndSpeedCompensationPWM(aDriveSpeedPWM, 0);
        leftCarMotor.setDriveSpeedAndSpeedCompensationPWM(aDriveSpeedPWM, -aSpeedPWMCompensationRight);
    }
}

/**
 * @param aSpeedPWMCompensationRight if positive, this value is added to the compensation value of the right motor, or subtracted from the left motor value.
 *  If negative, -value is added to the compensation value the left motor, or subtracted from the right motor value.
 */
void CarPWMMotorControl::setSpeedPWMCompensation(int8_t aSpeedPWMCompensationRight) {

    if (aSpeedPWMCompensationRight >= 0) {
        rightCarMotor.setSpeedPWMCompensation(aSpeedPWMCompensationRight);
        leftCarMotor.setSpeedPWMCompensation(0);
    } else {
        rightCarMotor.setSpeedPWMCompensation(0);
        leftCarMotor.setSpeedPWMCompensation(-aSpeedPWMCompensationRight);
    }
}

/**
 * The motor compensation value is subtracted from the requested speed
 * @param aSpeedPWMCompensationRightDelta If positive, this value is added to the compensation value of the right motor,
 *   or subtracted from the left motor value.
 *  If negative, -value is added to the compensation value of the left motor, or subtracted from the right motor value.
 */
void CarPWMMotorControl::changeSpeedPWMCompensation(int8_t aSpeedPWMCompensationRightDelta) {
    uint8_t tLeftCarMotorSpeedPWMCompensation = leftCarMotor.SpeedPWMCompensation;
    uint8_t tRightCarMotorSpeedPWMCompensation = rightCarMotor.SpeedPWMCompensation;
    if (aSpeedPWMCompensationRightDelta > 0) {
        if (tLeftCarMotorSpeedPWMCompensation >= aSpeedPWMCompensationRightDelta) {
            tLeftCarMotorSpeedPWMCompensation -= aSpeedPWMCompensationRightDelta;
        } else {
            tRightCarMotorSpeedPWMCompensation += aSpeedPWMCompensationRightDelta;
        }
    } else {
        aSpeedPWMCompensationRightDelta = -aSpeedPWMCompensationRightDelta;
        if (tRightCarMotorSpeedPWMCompensation >= aSpeedPWMCompensationRightDelta) {
            tRightCarMotorSpeedPWMCompensation -= aSpeedPWMCompensationRightDelta;
        } else {
            tLeftCarMotorSpeedPWMCompensation += aSpeedPWMCompensationRightDelta;
        }
    }
    leftCarMotor.setSpeedPWMCompensation(tLeftCarMotorSpeedPWMCompensation);
    rightCarMotor.setSpeedPWMCompensation(tRightCarMotorSpeedPWMCompensation);

    PWMDcMotor::MotorControlValuesHaveChanged = true;
}

void CarPWMMotorControl::setDriveSpeedPWM(uint8_t aDriveSpeedPWM) {
    rightCarMotor.setDriveSpeedPWM(aDriveSpeedPWM);
    leftCarMotor.setDriveSpeedPWM(aDriveSpeedPWM);
}

/*
 * @return true if direction has changed and motor has stopped
 */
bool CarPWMMotorControl::checkAndHandleDirectionChange(uint8_t aRequestedDirection) {
    bool tReturnValue = false;
    if (CarDirection != aRequestedDirection) {
        uint8_t tMaxRequestedSpeedPWM = max(rightCarMotor.RequestedSpeedPWM, leftCarMotor.RequestedSpeedPWM);
        if (tMaxRequestedSpeedPWM > 0) {
            /*
             * Direction change requested but motor(s) still running-> first stop motor(s)
             */
#if defined(DEBUG)
            Serial.println(F("First stop motor(s) and wait"));
#endif
            stop(STOP_MODE_BRAKE);
//            delay(((tMaxCompensatedSpeedPWM * tMaxCompensatedSpeedPWM) >> 8) * 2); // to let motors stop
            delay(tMaxRequestedSpeedPWM); // to let motors stop
            tReturnValue = true;
        }
#if defined(DEBUG)
        Serial.print(F("Change car mode from "));
        Serial.print(sDirectionCharArray[CarDirection]);
        Serial.print(F(" to "));
        Serial.println(sDirectionCharArray[aRequestedDirection]);
#endif
        CarDirection = aRequestedDirection; // The only statement which changes CarDirection to DIRECTION_FORWARD or DIRECTION_BACKWARD
    }
    return tReturnValue;
}

/*
 *  Direct motor control, no state or flag handling
 */
void CarPWMMotorControl::setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
#if defined(CAR_HAS_4_MECANUM_WHEELS)
    setDirection(aRequestedDirection);
    rightCarMotor.setSpeedPWM(aRequestedSpeedPWM);
#else
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
    leftCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
#endif
}
/*
 * Sets speed adjusted by current compensation value and keeps direction
 */
void CarPWMMotorControl::changeSpeedPWM(uint8_t aRequestedSpeedPWM) {
    rightCarMotor.changeSpeedPWM(aRequestedSpeedPWM);
    leftCarMotor.changeSpeedPWM(aRequestedSpeedPWM);
}

/*
 * Sets speed adjusted by current compensation value and handle motor state and flags
 * @param aLeftRightSpeedPWM if positive, this value is subtracted from the left motor value, if negative subtracted from the right motor value
 */
void CarPWMMotorControl::setSpeedPWMWithDeltaAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection,
        int8_t aLeftRightSpeedPWMDelta) {
    checkAndHandleDirectionChange(aRequestedDirection);
#if defined(USE_ENCODER_MOTOR_CONTROL)
    EncoderMotor *tMotorWithModifiedSpeedPWM;
#else
    PWMDcMotor *tMotorWithModifiedSpeedPWM;
#endif
    if (aLeftRightSpeedPWMDelta >= 0) {
        rightCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
        tMotorWithModifiedSpeedPWM = &leftCarMotor;
    } else {
        aLeftRightSpeedPWMDelta = -aLeftRightSpeedPWMDelta;
        leftCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
        tMotorWithModifiedSpeedPWM = &rightCarMotor;
    }

    if (aRequestedSpeedPWM >= aLeftRightSpeedPWMDelta) {
        tMotorWithModifiedSpeedPWM->setSpeedPWMAndDirection(aRequestedSpeedPWM - aLeftRightSpeedPWMDelta, aRequestedDirection);
    } else {
        tMotorWithModifiedSpeedPWM->setSpeedPWMAndDirection(0, aRequestedDirection);
    }
}

/*
 *  Direct motor control, no state or flag handling
 */
#if defined(CAR_HAS_4_MECANUM_WHEELS)
void CarPWMMotorControl::setDirection(uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);

    uint8_t tRequestedDirection = aRequestedDirection & DIRECTION_MASK;
#  if defined(DEBUG)
    Serial.print(F("Speed="));
    Serial.print(aRequestedSpeedPWM);
    Serial.print(F(" forward/backward direction="));
    Serial.print(tRequestedDirection);
    Serial.print(F(" left/right="));
    Serial.print(aRequestedDirection & DIRECTION_LEFT_RIGHT_MASK);
    Serial.print(F(" turn="));
    Serial.print(aRequestedDirection & DIRECTION_TURN);
    Serial.println();
#  endif

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
        tFrontLeftMotorDirection = STOP_MODE_BRAKE;
        tBackLeftMotorDirection = STOP_MODE_BRAKE;
        tFrontRightMotorDirection = STOP_MODE_BRAKE;
        tBackRightMotorDirection = STOP_MODE_BRAKE;
        // All 4 wheels for CENTER TURN LEFT
        if (tRequestedDirection == DIRECTION_STOP || tRequestedDirection == DIRECTION_FORWARD) {
            // FRONT TURN LEFT
            tFrontLeftMotorDirection = DIRECTION_BACKWARD;
            tFrontRightMotorDirection = DIRECTION_FORWARD;
        }
        if (tRequestedDirection == DIRECTION_STOP || tRequestedDirection == DIRECTION_BACKWARD) {
            // BACK TURN LEFT
            tBackLeftMotorDirection = DIRECTION_BACKWARD;
            tBackRightMotorDirection = DIRECTION_FORWARD;
        }
    } else if (aRequestedDirection & DIRECTION_LEFT_RIGHT_MASK) {
        /*
         * LEFT or RIGHT requested
         */
        if (tRequestedDirection == DIRECTION_STOP) {
            // STRAIGHT LEFT
            tFrontLeftMotorDirection = DIRECTION_BACKWARD;
            tBackLeftMotorDirection = DIRECTION_FORWARD;
            tFrontRightMotorDirection = DIRECTION_FORWARD;
            tBackRightMotorDirection = DIRECTION_BACKWARD;
        } else {
            if (tRequestedDirection == DIRECTION_FORWARD) {
                // FORWARD DIAGONAL LEFT
                tFrontLeftMotorDirection = STOP_MODE_BRAKE;
                tBackLeftMotorDirection = DIRECTION_FORWARD;
                tFrontRightMotorDirection = DIRECTION_FORWARD;
                tBackRightMotorDirection = STOP_MODE_BRAKE;
            } else {
                // BACKWARD DIAGONAL LEFT
                tFrontLeftMotorDirection = DIRECTION_BACKWARD;
                tBackLeftMotorDirection = STOP_MODE_BRAKE;
                tFrontRightMotorDirection = STOP_MODE_BRAKE;
                tBackRightMotorDirection = DIRECTION_BACKWARD;
            }
        }
    } else {
        /*
         * FORWARD or BACKWARD or STOP
         */
        tFrontLeftMotorDirection = tRequestedDirection;
        tBackLeftMotorDirection = tRequestedDirection;
        tFrontRightMotorDirection = tRequestedDirection;
        tBackRightMotorDirection = tRequestedDirection;
    }

    if (aRequestedDirection & DIRECTION_RIGHT) {
        // Swap left and right motors
        leftCarMotor.setDirection(tFrontRightMotorDirection);
        backLeftCarMotor.setDirection(tBackRightMotorDirection);
        rightCarMotor.setDirection(tFrontLeftMotorDirection);
        backRightCarMotor.setDirection(tBackLeftMotorDirection);
    } else {
        leftCarMotor.setDirection(tFrontLeftMotorDirection);
        backLeftCarMotor.setDirection(tBackLeftMotorDirection);
        rightCarMotor.setDirection(tFrontRightMotorDirection);
        backRightCarMotor.setDirection(tBackRightMotorDirection);
    }
}
#else
void CarPWMMotorControl::setDirection(uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.setDirection(aRequestedDirection);
    leftCarMotor.setDirection(aRequestedDirection);
}
#endif

void CarPWMMotorControl::setSpeedPWM(uint8_t aRequestedSpeedPWM) {
    rightCarMotor.setSpeedPWM(aRequestedSpeedPWM);
#if !defined(CAR_HAS_4_MECANUM_WHEELS) // we assume, that all motors share the same PWM pin
    leftCarMotor.setSpeedPWM(aRequestedSpeedPWM);
#endif
}

void CarPWMMotorControl::setSpeedPWMAndDirection(int aRequestedSpeedPWM) {
    rightCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM);
    leftCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM);
}

uint8_t CarPWMMotorControl::getCarDirection() {
    return CarDirection;;
}

void CarPWMMotorControl::readMotorValuesFromEeprom() {
    leftCarMotor.readMotorValuesFromEeprom(0);
    rightCarMotor.readMotorValuesFromEeprom(1);
}

void CarPWMMotorControl::writeMotorValuesToEeprom() {
    leftCarMotor.writeMotorValuesToEeprom(0);
    rightCarMotor.writeMotorValuesToEeprom(1);
}

/*
 * Stop car
 * @param aStopMode STOP_MODE_KEEP (take previously defined StopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void CarPWMMotorControl::stop(uint8_t aStopMode) {
    rightCarMotor.stop(aStopMode);
    leftCarMotor.stop(aStopMode);
    CarDirection = DIRECTION_STOP;
}

/*
 * @param aStopMode used for speed == 0 or STOP_MODE_KEEP: STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void CarPWMMotorControl::setStopMode(uint8_t aStopMode) {
    rightCarMotor.setStopMode(aStopMode);
    leftCarMotor.setStopMode(aStopMode);
}

void CarPWMMotorControl::resetEncoderControlValues() {
#if defined(USE_ENCODER_MOTOR_CONTROL)
    rightCarMotor.resetEncoderControlValues();
    leftCarMotor.resetEncoderControlValues();
#endif
}

#if defined(USE_MPU6050_IMU)
void CarPWMMotorControl::updateIMUData() {
    if (IMUData.readCarDataFromMPU6050Fifo()) {
        if (IMUData.AcceleratorForwardOffset != 0) {
            if (CarTurn2DegreesPerSecondFromIMU != IMUData.getGyroscopePan2DegreePerSecond()) {
                CarTurn2DegreesPerSecondFromIMU = IMUData.getGyroscopePan2DegreePerSecond();
//                PWMDcMotor::SensorValuesHaveChanged = true; is not displayed
            }
            if (CarTurnAngleHalfDegreesFromIMU != IMUData.getTurnAngleHalfDegree()) {
                CarTurnAngleHalfDegreesFromIMU = IMUData.getTurnAngleHalfDegree();
                PWMDcMotor::SensorValuesHaveChanged = true;
            }
            if (CarSpeedCmPerSecondFromIMU != (unsigned int) abs(IMUData.getSpeedCmPerSecond())) {
                CarSpeedCmPerSecondFromIMU = abs(IMUData.getSpeedCmPerSecond());
                PWMDcMotor::SensorValuesHaveChanged = true;
            }
            if (CarDistanceMillimeterFromIMU != (unsigned int) abs(IMUData.getDistanceMillimeter())) {
                CarDistanceMillimeterFromIMU = abs(IMUData.getDistanceMillimeter());
                PWMDcMotor::SensorValuesHaveChanged = true;
            }
        }
    }
}
#endif

/*
 * Convenience funtion to be used as aLoopCallback
 */
void stopMotorAfter1Second() {

}

/*
 * @return true if not stopped (motor expects another update)
 */
#define TURN_OVERRUN_HALF_ANGLE     1 // 1/2 degree overrun after stop(STOP_MODE_BRAKE)
#define RAMP_DOWN_MILLIMETER       50
#define STOP_OVERRUN_MILLIMETER    10 // 1 cm overrun after stop(STOP_MODE_BRAKE)
/*
 * If IMU data are available, rotation is always handled here.
 * For non encoder motors also distance driving is handled here.
 * @return true if not stopped (motor expects another update)
 */
bool CarPWMMotorControl::updateMotors() {
#if defined(USE_MPU6050_IMU)
    bool tReturnValue = !isStopped();
    updateIMUData();
    if (CarRequestedRotationDegrees != 0) {
        /*
         * Rotation here. Using ramps for the rotation SpeedPWMs used makes no sense
         */
#  if defined(TRACE)
        Serial.println(CarTurnAngleHalfDegreesFromIMU);
        delay(10);
#  endif
        // putting abs(CarTurnAngleHalfDegreesFromIMU) also into a variable increases code size by 8
        int tRequestedRotationDegreesForCompare = abs(CarRequestedRotationDegrees * 2);
        int tCarTurnAngleHalfDegreesFromIMUForCompare = abs(CarTurnAngleHalfDegreesFromIMU);
        if ((tCarTurnAngleHalfDegreesFromIMUForCompare + TURN_OVERRUN_HALF_ANGLE) >= tRequestedRotationDegreesForCompare) {
            /*
             * End of rotation detected
             */
            stop(STOP_MODE_BRAKE);
            CarRequestedRotationDegrees = 0;
            tReturnValue = false;
        } else if ((tCarTurnAngleHalfDegreesFromIMUForCompare + getTurnDistanceHalfDegree())
                >= tRequestedRotationDegreesForCompare) {
//            Serial.print(getTurnDistanceHalfDegree());
            /*
             * Reduce SpeedPWM just before target angle is reached. If motors are not stopped, we run for extra 2 to 4 degree
             */
            changeSpeedPWM(rightCarMotor.DriveSpeedPWM / 2);
        }
    } else {
        if (CarRequestedDistanceMillimeter != 0) {
#  if !defined(USE_ENCODER_MOTOR_CONTROL)
            if (rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_UP || rightCarMotor.MotorRampState == MOTOR_STATE_DRIVE
                    || rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_DOWN) {
                unsigned int tBrakingDistanceMillimeter = getBrakingDistanceMillimeter();
#if defined(DEBUG)
                Serial.print(F("Dist="));
                Serial.print(CarDistanceMillimeterFromIMU);
                Serial.print(F(" Breakdist="));
                Serial.print(tBrakingDistanceMillimeter);
                Serial.print(F(" St="));
                Serial.print(rightCarMotor.MotorRampState);
                Serial.print(F(" Ns="));
                Serial.println(rightCarMotor.CompensatedSpeedPWM);
#endif
                if (CarDistanceMillimeterFromIMU >= CarRequestedDistanceMillimeter) {
                    CarRequestedDistanceMillimeter = 0;
                    stop(STOP_MODE_BRAKE);
                }
                // Transition criteria to brake/ramp down is: Target distance - braking distance reached
                if (rightCarMotor.MotorRampState != MOTOR_STATE_RAMP_DOWN
                        && (CarDistanceMillimeterFromIMU + tBrakingDistanceMillimeter) >= CarRequestedDistanceMillimeter) {
                    // Start braking
                    startRampDown();
                }
            }
#  endif // USE_ENCODER_MOTOR_CONTROL
        }
        /*
         * In case of IMU distance driving only ramp up and down are managed by these calls
         */
        tReturnValue = rightCarMotor.updateMotor();
        tReturnValue |= leftCarMotor.updateMotor();
    }

#else // USE_MPU6050_IMU
    bool tReturnValue = rightCarMotor.updateMotor();
    tReturnValue |= leftCarMotor.updateMotor();
#endif // USE_MPU6050_IMU

    return tReturnValue;;
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool CarPWMMotorControl::updateMotors(void (*aLoopCallback)(void)) {
    if (aLoopCallback != NULL) {
        aLoopCallback();
    }
    return updateMotors();
}

void CarPWMMotorControl::delayAndUpdateMotors(unsigned int aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        updateMotors();
    } while (millis() - tStartMillis <= aDelayMillis);
}

void CarPWMMotorControl::startRampUp(uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startRampUp(aRequestedDirection);
    leftCarMotor.startRampUp(aRequestedDirection);
}

void CarPWMMotorControl::setSpeedPWMWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.setSpeedPWMAndDirectionWithRamp(aRequestedSpeedPWM, aRequestedDirection);
    leftCarMotor.setSpeedPWMAndDirectionWithRamp(aRequestedSpeedPWM, aRequestedDirection);
}

/*
 * Blocking wait until both motors are at drive SpeedPWM. 256 milliseconds for ramp up.
 * @param aLoopCallback The callback called while waiting for motor to reach MOTOR_STATE_DRIVE.
 */
void CarPWMMotorControl::waitForDriveSpeedPWM(void (*aLoopCallback)(void)) {
#if !defined(DO_NOT_SUPPORT_RAMP)
    while (updateMotors(aLoopCallback)
            && (rightCarMotor.MotorRampState != MOTOR_STATE_DRIVE || leftCarMotor.MotorRampState != MOTOR_STATE_DRIVE)) {
        ;
    }
#else
    (void) aLoopCallback;
#endif
}

/*
 * If ramp up is not supported, this functions just sets the SpeedPWM and returns immediately.
 * 256 milliseconds for ramp up.
 * @param aLoopCallback The callback called while waiting for motor to reach MOTOR_STATE_DRIVE.
 */
void CarPWMMotorControl::startRampUpAndWait(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
    setSpeedPWMWithRamp(aRequestedSpeedPWM, aRequestedDirection);
    waitForDriveSpeedPWM(aLoopCallback);
}

void CarPWMMotorControl::startRampUpAndWaitForDriveSpeedPWM(uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
    startRampUp(aRequestedDirection);
    waitForDriveSpeedPWM(aLoopCallback);
}

void CarPWMMotorControl::startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(rightCarMotor.DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * initialize motorInfo fields LastDirection and CompensatedSpeedPWM
 */
void CarPWMMotorControl::startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {

#if defined(USE_MPU6050_IMU)
    IMUData.resetCarData();
    CarRequestedDistanceMillimeter = aRequestedDistanceMillimeter;
#endif

#if defined(USE_MPU6050_IMU) && !defined(USE_ENCODER_MOTOR_CONTROL)
    // for non encoder motor we use the IMU distance, and require only the ramp up
    setSpeedPWMWithRamp(aRequestedSpeedPWM, aRequestedDirection);
#else
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startGoDistanceMillimeter(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
#  if defined(CAR_HAS_4_MECANUM_WHEELS)
    setDirection(aRequestedDirection); // this sets the direction for all the other motors
#  else
    leftCarMotor.startGoDistanceMillimeter(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
#endif
#endif
}

void CarPWMMotorControl::goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection,
        void (*aLoopCallback)(void)) {
    startGoDistanceMillimeter(rightCarMotor.DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
    waitUntilStopped(aLoopCallback);
}

void CarPWMMotorControl::startGoDistanceMillimeter(int aRequestedDistanceMillimeter) {
    if (aRequestedDistanceMillimeter < 0) {
        aRequestedDistanceMillimeter = -aRequestedDistanceMillimeter;
        startGoDistanceMillimeter(rightCarMotor.DriveSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_BACKWARD);
    } else {
        startGoDistanceMillimeter(rightCarMotor.DriveSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_FORWARD);
    }
}

/**
 * Wait until distance is reached
 * @param  aLoopCallback called until car has stopped to avoid blocking
 */
void CarPWMMotorControl::goDistanceMillimeter(int aRequestedDistanceMillimeter, void (*aLoopCallback)(void)) {
    startGoDistanceMillimeter(aRequestedDistanceMillimeter);
    waitUntilStopped(aLoopCallback);
}

/*
 * Stop car (with ramp)
 */
void CarPWMMotorControl::stopAndWaitForIt(void (*aLoopCallback)(void)) {
    if (isStopped()) {
        return;
    }

    rightCarMotor.startRampDown();
    leftCarMotor.startRampDown();
    /*
     * blocking wait for stop
     */
    waitUntilStopped(aLoopCallback);
}

void CarPWMMotorControl::startRampDown() {
    if (isStopped()) {
        return;
    }
    /*
     * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_DRIVE to MOTOR_STATE_RAMP_DOWN
     */
    rightCarMotor.startRampDown();
    leftCarMotor.startRampDown();
}

/*
 * Wait with optional wait loop callback
 */
void CarPWMMotorControl::waitUntilStopped(void (*aLoopCallback)(void)) {
    while (updateMotors(aLoopCallback)) {
        ;
    }
}

bool CarPWMMotorControl::isState(uint8_t aState) {
#if defined(DO_NOT_SUPPORT_RAMP)
    (void) aState;
    return true;
#else
    return (rightCarMotor.MotorRampState == aState && leftCarMotor.MotorRampState == aState);
#endif
}

bool CarPWMMotorControl::isStopped() {
    return (rightCarMotor.RequestedSpeedPWM == 0 && leftCarMotor.RequestedSpeedPWM == 0);
}

/*
 * If motor is accelerating or decelerating then updateMotor needs to be called at a fast rate otherwise it will not work correctly
 * Used to suppress time consuming display of motor values
 */
bool CarPWMMotorControl::isStateRamp() {
#if defined(DO_NOT_SUPPORT_RAMP)
    return false;
#else
    return (rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_DOWN || rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_UP
            || leftCarMotor.MotorRampState == MOTOR_STATE_RAMP_DOWN || leftCarMotor.MotorRampState == MOTOR_STATE_RAMP_UP);
#endif
}

void CarPWMMotorControl::setFactorDegreeToMillimeter(float aFactorDegreeToMillimeter) {
#if defined(USE_MPU6050_IMU)
    (void) aFactorDegreeToMillimeter;
#else
    FactorDegreeToMillimeter = aFactorDegreeToMillimeter;
#endif
}

/**
 * Set distances and SpeedPWM for 2 motors to turn the requested angle
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE
 * @param  aUseSlowSpeed true -> use slower SpeedPWM (0.5 times DriveSpeedPWM) instead of DriveSpeedPWM for rotation to be more exact
 */
char sTurnDirectionCharArray[3] = { 'F', 'B', 'P' };
void CarPWMMotorControl::startRotate(int aRotationDegrees, turn_direction_t aTurnDirection, bool aUseSlowSpeed) {
#if defined(CAR_HAS_4_MECANUM_WHEELS)
    uint8_t tDirection;
    if(aRotationDegrees > 0){
        tDirection = DIRECTION_LEFT;
    } else {
        aRotationDegrees = -aRotationDegrees;
        tDirection = DIRECTION_RIGHT;
    }

    if(aTurnDirection == TURN_FORWARD){
        tDirection |= DIRECTION_FORWARD;
        aRotationDegrees *=2; // we need twice the time of an in place turn
    } else if(aTurnDirection == TURN_BACKWARD){
        tDirection |= DIRECTION_BACKWARD;
        aRotationDegrees *=2;
    }
    setDirection(tDirection | DIRECTION_TURN);

    uint8_t tTurnSpeed = rightCarMotor.DriveSpeedPWM;
    if (aUseSlowSpeed) {
        tTurnSpeed = DEFAULT_START_SPEED_PWM;
    }
    unsigned int tDistanceMillimeter = (aRotationDegrees * FactorDegreeToMillimeter) + 0.5;
    rightCarMotor.startGoDistanceMillimeter(tTurnSpeed, tDistanceMillimeter, rightCarMotor.getDirection());
#else // CAR_HAS_4_MECANUM_WHEELS

    /*
     * We have 6 cases
     * - aTurnDirection = TURN_FORWARD      + -> left, right motor F, left 0    - -> right, right motor 0, left F
     * - aTurnDirection = TURN_BACKWARD     + -> left, right motor 0, left B    - -> right, right motor B, left 0
     * - aTurnDirection = TURN_IN_PLACE     + -> left, right motor F, left B    - -> right, right motor B, left F
     * Turn direction TURN_IN_PLACE is masked to TURN_FORWARD
     */

#if defined(DEBUG)
    Serial.print(F("RotationDegrees="));
    Serial.print(aRotationDegrees);
    Serial.print(F(" TurnDirection="));
    Serial.println(sTurnDirectionCharArray[aTurnDirection]);
    Serial.flush();
#endif

#if defined(USE_MPU6050_IMU)
    IMUData.resetCarData();
    CarRequestedRotationDegrees = aRotationDegrees;
#endif

    /*
     * Handle positive and negative rotation degrees
     */
#if defined(USE_ENCODER_MOTOR_CONTROL)
    EncoderMotor *tRightMotorIfPositiveTurn;
    EncoderMotor *tLeftMotorIfPositiveTurn;
#else
    PWMDcMotor *tRightMotorIfPositiveTurn;
    PWMDcMotor *tLeftMotorIfPositiveTurn;
#endif
    if (aRotationDegrees >= 0) {
        tRightMotorIfPositiveTurn = &rightCarMotor;
        tLeftMotorIfPositiveTurn = &leftCarMotor;
    } else {
// swap turn sign and left / right motors
        aRotationDegrees = -aRotationDegrees;
        tRightMotorIfPositiveTurn = &leftCarMotor;
        tLeftMotorIfPositiveTurn = &rightCarMotor;
    }

    /*
     * Here aRotationDegrees is positive
     * Now handle different turn directions
     */
#if defined(USE_MPU6050_IMU)
    unsigned int tDistanceMillimeter = 2000; // Dummy value for distance - equivalent to #define tDistanceCount 200 give a timeout of around 10 wheel rotations
#else
    unsigned int tDistanceMillimeter = (aRotationDegrees * FactorDegreeToMillimeter) + 0.5;
#endif

    unsigned int tDistanceMillimeterRight;
    unsigned int tDistanceMillimeterLeft;

    if (aTurnDirection == TURN_FORWARD) {
        tDistanceMillimeterRight = tDistanceMillimeter;
        tDistanceMillimeterLeft = 0;
    } else if (aTurnDirection == TURN_BACKWARD) {
        tDistanceMillimeterRight = 0;
        tDistanceMillimeterLeft = tDistanceMillimeter;
    } else {
        tDistanceMillimeterRight = tDistanceMillimeter / 2;
        tDistanceMillimeterLeft = tDistanceMillimeter / 2;
    }

    /*
     * Handle slow speed flag and reduce turn SpeedPWMs
     */
    uint8_t tTurnSpeedPWMRight = tRightMotorIfPositiveTurn->DriveSpeedPWM;
    uint8_t tTurnSpeedPWMLeft = tLeftMotorIfPositiveTurn->DriveSpeedPWM;
    if (aUseSlowSpeed) {
#if defined(CAR_HAS_4_WHEELS)
        // DEFAULT_START_SPEED_PWM does not really work for 4 WD cars
        tTurnSpeedPWMRight = DEFAULT_START_SPEED_PWM + (DEFAULT_START_SPEED_PWM / 2);
        tTurnSpeedPWMLeft = DEFAULT_START_SPEED_PWM + (DEFAULT_START_SPEED_PWM / 2);
#else
        tTurnSpeedPWMRight = DEFAULT_START_SPEED_PWM;
        tTurnSpeedPWMLeft = DEFAULT_START_SPEED_PWM;
#endif
    }

#if defined(DEBUG)
    Serial.print(F("TurnSpeedPWMRight="));
    Serial.print(tTurnSpeedPWMRight);
#  if !defined(USE_MPU6050_IMU)
    Serial.print(F(" DistanceMillimeterRight="));
    Serial.print(tDistanceMillimeterRight);
#  endif
    Serial.println();
#endif

#if defined(USE_MPU6050_IMU)
    // We do not really have ramps for turn speed
    if (tDistanceMillimeterRight > 0) {
        tRightMotorIfPositiveTurn->setSpeedPWMAndDirection(tTurnSpeedPWMRight, DIRECTION_FORWARD);
    }
    if (tDistanceMillimeterLeft > 0) {
        tLeftMotorIfPositiveTurn->setSpeedPWMAndDirection(tTurnSpeedPWMLeft, DIRECTION_BACKWARD);
    }
#else
    tRightMotorIfPositiveTurn->startGoDistanceMillimeter(tTurnSpeedPWMRight, tDistanceMillimeterRight, DIRECTION_FORWARD);
    tLeftMotorIfPositiveTurn->startGoDistanceMillimeter(tTurnSpeedPWMLeft, tDistanceMillimeterLeft, DIRECTION_BACKWARD);
#endif
#endif // CAR_HAS_4_MECANUM_WHEELS
}

/**
 * @param  aRotationDegrees positive -> turn left (counterclockwise), negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE (default)
 * @param  aUseSlowSpeed true (not default) -> use slower SpeedPWM (for 4WD cars 3/4 times, for 2WD 0.5 times DriveSpeedPWM)
 *                                             instead of DriveSpeedPWM for rotation to be more exact.
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void CarPWMMotorControl::rotate(int aRotationDegrees, turn_direction_t aTurnDirection, bool aUseSlowSpeed,
        void (*aLoopCallback)(void)) {
    if (aRotationDegrees != 0) {
        startRotate(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilStopped(aLoopCallback);
    }
}

#if defined(USE_ENCODER_MOTOR_CONTROL)
/*
 * Get count / distance value from right motor
 */
unsigned int CarPWMMotorControl::getDistanceCount() {
    return (rightCarMotor.EncoderCount);
}

unsigned int CarPWMMotorControl::getDistanceMillimeter() {
    return (rightCarMotor.getDistanceMillimeter());
}

#else
void CarPWMMotorControl::setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond) {
    rightCarMotor.setMillimeterPerSecondForFixedDistanceDriving(aMillimeterPerSecond);
    leftCarMotor.setMillimeterPerSecondForFixedDistanceDriving(aMillimeterPerSecond);
}

#endif // USE_ENCODER_MOTOR_CONTROL

#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
unsigned int CarPWMMotorControl::getBrakingDistanceMillimeter() {
#  if defined(USE_ENCODER_MOTOR_CONTROL)
    return rightCarMotor.getBrakingDistanceMillimeter();
#  else
    unsigned int tCarSpeedCmPerSecond = CarSpeedCmPerSecondFromIMU;
//    return (tCarSpeedCmPerSecond * tCarSpeedCmPerSecond * 100) / RAMP_DECELERATION_TIMES_2; // overflow!
    return (tCarSpeedCmPerSecond * tCarSpeedCmPerSecond) / (RAMP_DECELERATION_TIMES_2 / 100);
#  endif
}

#  if defined(USE_MPU6050_IMU)
uint8_t CarPWMMotorControl::getTurnDistanceHalfDegree() {
    uint8_t tCarTurn2DegreesPerSecondFromIMU = CarTurn2DegreesPerSecondFromIMU;
    return ((tCarTurn2DegreesPerSecondFromIMU * tCarTurn2DegreesPerSecondFromIMU) / 20);
}
#  endif

///*
// * Generates a rising ramp and detects the first movement -> this sets dead band / minimum SpeedPWM
// * aLoopCallback is responsible for calling readCarDataFromMPU6050Fifo();
// */
//void CarPWMMotorControl::getStartSpeedPWM(void (*aLoopCallback)(void)) {
//    stop();
//#  if defined(USE_ENCODER_MOTOR_CONTROL)
//    resetEncoderControlValues();
//#  endif
//
//    rightCarMotor.StartSpeedPWM = 0;
//    leftCarMotor.StartSpeedPWM = 0;
//
//#  if defined(USE_ENCODER_MOTOR_CONTROL)
//    uint8_t tMotorMovingCount = 0;
//#  else
//    IMUData.resetOffsetDataAndWait();
//#  endif
//
//    /*
//     * increase motor SpeedPWM by 1 every 200 ms until motor moves
//     */
//    for (uint_fast8_t tSpeedPWM = 20; tSpeedPWM != MAX_SPEED_PWM; ++tSpeedPWM) {
//// as long as no start speed is computed increase speed
//        if (rightCarMotor.StartSpeedPWM == 0) {
//            // as long as no start speed is computed, increase motor speed
//            rightCarMotor.setSpeedPWMAndDirection(tSpeedPWM, DIRECTION_FORWARD);
//        }
//        if (leftCarMotor.StartSpeedPWM == 0) {
//            leftCarMotor.setSpeedPWMAndDirection(tSpeedPWM, DIRECTION_FORWARD);
//        }
//
//        /*
//         * Active delay of 200 ms
//         */
//        uint32_t tStartMillis = millis();
//        do {
//            if (aLoopCallback != NULL) {
//                aLoopCallback();
//            }
//            if (isStopped()) {
//                // we were stopped by aLoopCallback()
//                return;
//            }
//#  if defined(USE_ENCODER_MOTOR_CONTROL)
//            delay(10);
//#  else
//            delay(DELAY_TO_NEXT_IMU_DATA_MILLIS);
//            updateIMUData();
//#  endif
//        } while (millis() - tStartMillis <= 200);
//
//        /*
//         * Check if wheel moved
//         */
//#  if defined(USE_ENCODER_MOTOR_CONTROL)
//        /*
//         * Store speed after 6 counts (3cm)
//         */
//        if (rightCarMotor.StartSpeedPWM == 0 && rightCarMotor.EncoderCount > 6) {
//            rightCarMotor.setStartSpeedPWM(tSpeedPWM);
//            tMotorMovingCount++;
//        }
//        if (leftCarMotor.StartSpeedPWM == 0 && leftCarMotor.EncoderCount > 6) {
//            leftCarMotor.setStartSpeedPWM(tSpeedPWM);
//            tMotorMovingCount++;
//        }
//        if (tMotorMovingCount >= 2) {
//            // Do not end loop if one motor is still not moving
//            break;
//        }
//
//#  else
//        if (abs(IMUData.getSpeedCmPerSecond()) >= 10) {
//            setStartSpeedPWM(tSpeedPWM);
//            break;
//        }
//#  endif
//    }
//    stop();
//}

#endif // defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)

#endif // _CAR_PWM_MOTOR_CONTROL_HPP
#pragma once
