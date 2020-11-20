/*
 * CarMotorControl.cpp
 *
 *  Contains functions for control of the 2 motors of a car like setDirection, goDistanceCentimeter() and rotate().
 *  Checks input of PIN aPinFor2WDDetection since we need different factors for rotating a 4 wheel and a 2 wheel car.
 *
 *  Requires EncoderMotor.cpp
 *
 *  Created on: 12.05.2019
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
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

#include <Arduino.h>
#include "CarMotorControl.h"
#ifdef USE_MPU6050_IMU
#include "CarIMUData.h"
#endif

//#define DEBUG // Only for development

CarMotorControl::CarMotorControl() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_MPU6050_IMU
/*
 * This must be done when the car is not moving, best after at least 100 ms after boot up.
 */
void CarMotorControl::initIMU() {
    calculateSpeedAndTurnOffsets();
}
void CarMotorControl::printIMUOffsets(Print *aSerial) {
    printSpeedAndTurnOffsets(aSerial);
}
#endif
/*
 * If no parameter and we have encoder motors, we use a fixed assignment of rightCarMotor interrupts to INT0 / Pin2 and leftCarMotor to INT1 / Pin3
 */
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
void CarMotorControl::init() {
#  ifdef USE_ENCODER_MOTOR_CONTROL
    leftCarMotor.init(1, INT1);
    rightCarMotor.init(2, INT0);
#  else
    leftCarMotor.init(1);
    rightCarMotor.init(2);
#  endif

#  ifdef USE_MPU6050_IMU
    CarRotationDegrees = 0;
#  else
#    if defined(CAR_HAS_4_WHEELS)
    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_4WD_CAR_DEFAULT;
#    else
    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT;
#    endif
#  endif
}

#else // USE_ADAFRUIT_MOTOR_SHIELD
void CarMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin,
        uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin) {
    leftCarMotor.init(aLeftMotorForwardPin, LeftMotorBackwardPin, aLeftMotorPWMPin);
    rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin);

#    ifdef USE_MPU6050_IMU
    CarRotationDegrees = 0;
#    else
    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_DEFAULT;
#    endif
#  ifdef USE_ENCODER_MOTOR_CONTROL
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    rightCarMotor.attachInterrupt(INT0);
    leftCarMotor.attachInterrupt(INT1);
#  endif
}

#  ifdef USE_ENCODER_MOTOR_CONTROL
/*
 * With parameters aRightInterruptNumber + aLeftInterruptNumber
 */
void CarMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin,
        uint8_t aRightInterruptNumber, uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin,
        uint8_t aLeftInterruptNumber) {
    leftCarMotor.init(aLeftMotorForwardPin, LeftMotorBackwardPin, aLeftMotorPWMPin, aLeftInterruptNumber);
    rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin, aRightInterruptNumber);

#    ifdef USE_MPU6050_IMU
    CarRotationDegrees = 0;
#    else
    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_DEFAULT;
#    endif
}
#  endif
#endif

/*
 * Sets default values for min and max speed, factor for distance to time conversion for non encoder motors and reset speed compensation
 * Is called automatically at init if parameter aReadFromEeprom is set to false
 */
void CarMotorControl::setDefaultsForFixedDistanceDriving() {
    rightCarMotor.setDefaultsForFixedDistanceDriving();
    leftCarMotor.setDefaultsForFixedDistanceDriving();
}

/**
 * @param aSpeedCompensationRight if positive, this value is added to the compensation value of the right motor, or subtracted from the left motor value.
 *  If negative, -value is added to the compensation value the left motor, or subtracted from the right motor value.
 */
void CarMotorControl::setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, int8_t aSpeedCompensationRight) {
    if (aSpeedCompensationRight >= 0) {
        rightCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, aSpeedCompensationRight);
        leftCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, 0);
    } else {
        rightCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, 0);
        leftCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, -aSpeedCompensationRight);
    }
}

/**
 * @param aSpeedCompensationRight if positive, this value is added to the compensation value of the right motor, or subtracted from the left motor value.
 *  If negative, -value is added to the compensation value the left motor, or subtracted from the right motor value.
 */
void CarMotorControl::changeSpeedCompensation(int8_t aSpeedCompensationRight) {
    if (aSpeedCompensationRight > 0) {
        if (leftCarMotor.SpeedCompensation >= aSpeedCompensationRight) {
            leftCarMotor.SpeedCompensation -= aSpeedCompensationRight;
        } else {
            rightCarMotor.SpeedCompensation += aSpeedCompensationRight;
        }
    } else {
        aSpeedCompensationRight = -aSpeedCompensationRight;
        if (rightCarMotor.SpeedCompensation >= aSpeedCompensationRight) {
            rightCarMotor.SpeedCompensation -= aSpeedCompensationRight;
        } else {
            leftCarMotor.SpeedCompensation += aSpeedCompensationRight;
        }
    }
    PWMDcMotor::MotorValuesHaveChanged = true;
}

void CarMotorControl::setDriveSpeed(uint8_t aDriveSpeed) {
    rightCarMotor.setDriveSpeed(aDriveSpeed);
    leftCarMotor.setDriveSpeed(aDriveSpeed);
}

/*
 * @return true if direction has changed and motor has stopped
 */
bool CarMotorControl::checkAndHandleDirectionChange(uint8_t aRequestedDirection) {
    bool tReturnValue = false;
    if (CarDirectionOrBrakeMode != aRequestedDirection) {
        uint8_t tMaxSpeed = max(rightCarMotor.CurrentSpeed, leftCarMotor.CurrentSpeed);
        if (tMaxSpeed > 0) {
            /*
             * Direction change requested but motor still running-> first stop motor
             */
#ifdef DEBUG
            Serial.println(F("First stop motor and wait"));
#endif
            stop(MOTOR_BRAKE);
            delay(tMaxSpeed / 2); // to let motors stop
            tReturnValue = true;
        }
#ifdef DEBUG
        Serial.print(F("Change car mode from "));
        Serial.print(CarDirectionOrBrakeMode);
        Serial.print(F(" to "));
        Serial.println(aRequestedDirection);
#endif
        CarDirectionOrBrakeMode = aRequestedDirection; // The only statement which changes CarDirectionOrBrakeMode to DIRECTION_FORWARD or DIRECTION_BACKWARD
    }
    return tReturnValue;
}

/*
 *  Direct motor control, no state or flag handling
 */
void CarMotorControl::setSpeed(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.setSpeed(aRequestedSpeed, aRequestedDirection);
    leftCarMotor.setSpeed(aRequestedSpeed, aRequestedDirection);
}

/*
 * Sets speed adjusted by current compensation value and handle motor state and flags
 */
void CarMotorControl::setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
    leftCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
}

/*
 * Sets speed adjusted by current compensation value and handle motor state and flags
 * @param aLeftRightSpeed if positive, this value is subtracted from the left motor value, if negative subtracted from the right motor value
 *
 */
void CarMotorControl::setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection, int8_t aLeftRightSpeed) {
    checkAndHandleDirectionChange(aRequestedDirection);
#ifdef USE_ENCODER_MOTOR_CONTROL
    EncoderMotor *tMotorWithModifiedSpeed;
#else
    PWMDcMotor *tMotorWithModifiedSpeed;
#endif
    if (aLeftRightSpeed >= 0) {
        rightCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
        tMotorWithModifiedSpeed = &leftCarMotor;
    } else {
        aLeftRightSpeed = -aLeftRightSpeed;
        leftCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
        tMotorWithModifiedSpeed = &rightCarMotor;
    }

    if (aRequestedSpeed >= aLeftRightSpeed) {
        tMotorWithModifiedSpeed->setSpeedCompensated(aRequestedSpeed - aLeftRightSpeed, aRequestedDirection);
    } else {
        tMotorWithModifiedSpeed->setSpeedCompensated(0, aRequestedDirection);
    }
}

/*
 *  Direct motor control, no state or flag handling
 */
void CarMotorControl::setSpeed(int aRequestedSpeed) {
    rightCarMotor.setSpeed(aRequestedSpeed);
    leftCarMotor.setSpeed(aRequestedSpeed);
}

/*
 * Sets signed speed adjusted by current compensation value and handle motor state and flags
 */
void CarMotorControl::setSpeedCompensated(int aRequestedSpeed) {
    rightCarMotor.setSpeedCompensated(aRequestedSpeed);
    leftCarMotor.setSpeedCompensated(aRequestedSpeed);
}

uint8_t CarMotorControl::getCarDirectionOrBrakeMode() {
    return CarDirectionOrBrakeMode;;
}

void CarMotorControl::readMotorValuesFromEeprom() {
    leftCarMotor.readMotorValuesFromEeprom(0);
    rightCarMotor.readMotorValuesFromEeprom(1);
}

void CarMotorControl::writeMotorValuesToEeprom() {
    leftCarMotor.writeMotorValuesToEeprom(0);
    rightCarMotor.writeMotorValuesToEeprom(1);
}

/*
 * Stop car
 * @param aStopMode STOP_MODE_KEEP (take previously defined StopMode) or MOTOR_BRAKE or MOTOR_RELEASE
 */
void CarMotorControl::stop(uint8_t aStopMode) {
    rightCarMotor.stop(aStopMode);
    leftCarMotor.stop(aStopMode);
    CarDirectionOrBrakeMode = rightCarMotor.CurrentDirectionOrBrakeMode; // get right stopMode, STOP_MODE_KEEP is evaluated here
}

/*
 * @param aStopMode MOTOR_BRAKE or MOTOR_RELEASE
 */
void CarMotorControl::setStopMode(uint8_t aStopMode) {
    rightCarMotor.setStopMode(aStopMode);
    leftCarMotor.setStopMode(aStopMode);
}

/*
 * Stop car and reset all control values as speed, distances, debug values etc. to 0x00
 */
void CarMotorControl::resetControlValues() {
#ifdef USE_ENCODER_MOTOR_CONTROL
    rightCarMotor.resetEncoderControlValues();
    leftCarMotor.resetEncoderControlValues();
#endif
}

/*
 * If motor is accelerating or decelerating then updateMotor needs to be called at a fast rate otherwise it will not work correctly
 * Used to suppress time consuming display of motor values
 */
bool CarMotorControl::isStateRamp() {
#ifdef SUPPORT_RAMP_UP
    return (rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_DOWN || rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_UP
            || leftCarMotor.MotorRampState == MOTOR_STATE_RAMP_DOWN || leftCarMotor.MotorRampState == MOTOR_STATE_RAMP_UP);
#else
    return false;
#endif
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool CarMotorControl::updateMotors() {
#ifdef USE_MPU6050_IMU
    if (CarRotationDegrees != 0) {
        readGyroZFromMPU6050Fifo();
        Serial.println(getTurnAngleWithHalfDegreeResolution());
        delay(10);
        if (abs(getTurnAngleWithHalfDegreeResolution()) >= (abs(CarRotationDegrees * 2))) {
            stop(MOTOR_BRAKE);
            CarRotationDegrees = 0;
            return false;
        }
    }
#endif
    bool tMotorsNotStopped = rightCarMotor.updateMotor();
    tMotorsNotStopped |= leftCarMotor.updateMotor();
    return tMotorsNotStopped;
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool CarMotorControl::updateMotors(void (*aLoopCallback)(void)) {
    if (aLoopCallback != NULL) {
        aLoopCallback();
    }
    return updateMotors();
}

void CarMotorControl::delayAndUpdateMotors(unsigned int aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        updateMotors();
    } while (millis() - tStartMillis <= aDelayMillis);
}

#ifdef SUPPORT_RAMP_UP
void CarMotorControl::startRampUp(uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startRampUp(aRequestedDirection);
    leftCarMotor.startRampUp(aRequestedDirection);
}

void CarMotorControl::startRampUp(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startRampUp(aRequestedSpeed, aRequestedDirection);
    leftCarMotor.startRampUp(aRequestedSpeed, aRequestedDirection);
}

/*
 * Blocking wait until both motors are at drive speed. 256 milliseconds for ramp up.
 */
void CarMotorControl::waitForDriveSpeed(void (*aLoopCallback)(void)) {
    while (updateMotors(aLoopCallback)
            && (rightCarMotor.MotorRampState != MOTOR_STATE_DRIVE_SPEED || leftCarMotor.MotorRampState != MOTOR_STATE_DRIVE_SPEED)) {
        ;
    }
}
#endif

/*
 * If ramp up is not supported, this functions just sets the speed and returns immediately.
 * 256 milliseconds for ramp up.
 */
void CarMotorControl::startRampUpAndWait(uint8_t aRequestedSpeed, uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
#ifdef SUPPORT_RAMP_UP
    startRampUp(aRequestedSpeed, aRequestedDirection);
    waitForDriveSpeed(aLoopCallback);
#else
    checkAndHandleDirectionChange(aRequestedDirection);
    (void) aLoopCallback;
    rightCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
    leftCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
#endif
}

void CarMotorControl::startRampUpAndWaitForDriveSpeed(uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
#ifdef SUPPORT_RAMP_UP
    startRampUp(aRequestedDirection);
    waitForDriveSpeed(aLoopCallback);
#else
    (void) aLoopCallback;
    rightCarMotor.setSpeedCompensated(rightCarMotor.DriveSpeed, aRequestedDirection);
    leftCarMotor.setSpeedCompensated(leftCarMotor.DriveSpeed, aRequestedDirection);
#endif
}

/*
 * Initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::startGoDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT, aRequestedDirection);
    leftCarMotor.startGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT, aRequestedDirection);
}

/*
 * initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::startGoDistanceCentimeter(uint8_t aRequestedSpeed, unsigned int aDistanceCentimeter,
        uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startGoDistanceCount(aRequestedSpeed, aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT,
            aRequestedDirection);
    leftCarMotor.startGoDistanceCount(aRequestedSpeed, aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT,
            aRequestedDirection);
}

void CarMotorControl::goDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection,
        void (*aLoopCallback)(void)) {
    startGoDistanceCentimeter(aDistanceCentimeter, aRequestedDirection);
    waitUntilStopped(aLoopCallback);
}

/*
 * initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::startGoDistanceCentimeter(int aDistanceCentimeter) {
    rightCarMotor.startGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT);
    leftCarMotor.startGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT);
}
/**
 * Wait until distance is reached
 * @param  aLoopCallback called until car has stopped to avoid blocking
 */
void CarMotorControl::goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void)) {
    startGoDistanceCentimeter(aDistanceCentimeter);
    waitUntilStopped(aLoopCallback);
}

/*
 * Stop car with ramp and give DistanceCountAfterRampUp counts for braking.
 */
void CarMotorControl::stopAndWaitForIt(void (*aLoopCallback)(void)) {
    if (isStopped()) {
        return;
    }
#if defined(USE_ENCODER_MOTOR_CONTROL) && defined(SUPPORT_RAMP_UP)
    /*
     * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_DRIVE_SPEED to MOTOR_STATE_RAMP_DOWN
     * Use DistanceCountAfterRampUp as ramp down count
     */
    rightCarMotor.NextChangeMaxTargetCount = rightCarMotor.EncoderCount;
    rightCarMotor.TargetDistanceCount = rightCarMotor.EncoderCount + rightCarMotor.DistanceCountAfterRampUp;
    leftCarMotor.NextChangeMaxTargetCount = leftCarMotor.EncoderCount;
    leftCarMotor.TargetDistanceCount = leftCarMotor.EncoderCount + leftCarMotor.DistanceCountAfterRampUp;
    /*
     * blocking wait for stop
     */
    waitUntilStopped(aLoopCallback);
#else
    (void) aLoopCallback;
    stop();
#endif
}

/*
 * Wait with optional wait loop callback
 */
void CarMotorControl::waitUntilStopped(void (*aLoopCallback)(void)) {
    while (updateMotors(aLoopCallback)) {
        ;
    }
    CarDirectionOrBrakeMode = rightCarMotor.CurrentDirectionOrBrakeMode; // get right stopMode
}

bool CarMotorControl::isState(uint8_t aState) {
#if defined(SUPPORT_RAMP_UP)
    return (rightCarMotor.MotorRampState == aState && leftCarMotor.MotorRampState == aState);
#else
    (void) aState;
    return false;
#endif
}

bool CarMotorControl::isStopped() {
    return (rightCarMotor.CurrentSpeed == 0 && leftCarMotor.CurrentSpeed == 0);
}

void CarMotorControl::setFactorDegreeToCount(float aFactorDegreeToCount) {
#ifndef USE_MPU6050_IMU
    FactorDegreeToCount = aFactorDegreeToCount;
#else
    (void) aFactorDegreeToCount;
#endif
}

/**
 * Set distances and speed for 2 motors to turn the requested angle
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE
 * @param  aUseSlowSpeed true -> use slower speed (1.5 times StartSpeed) instead of DriveSpeed for rotation to be more exact
 */
void CarMotorControl::startRotate(int aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    /*
     * We have 6 cases
     * - aTurnDirection = TURN_FORWARD      + -> left, right motor F, left 0    - -> right, right motor 0, left F
     * - aTurnDirection = TURN_BACKWARD     + -> left, right motor 0, left B    - -> right, right motor B, left 0
     * - aTurnDirection = TURN_IN_PLACE     + -> left, right motor F, left B    - -> right, right motor B, left F
     * Turn direction TURN_IN_PLACE is masked to TURN_FORWARD
     */

#ifdef DEBUG
    Serial.print(F("RotationDegrees="));
    Serial.print(aRotationDegrees);
    Serial.print(F(" TurnDirection="));
    Serial.println(aTurnDirection);
    Serial.flush();
#endif

#ifdef USE_MPU6050_IMU
    initMPU6050ForGyroZFifo();
    CarRotationDegrees = aRotationDegrees;
#endif

    /*
     * Handle positive and negative rotation degrees
     */
#ifdef USE_ENCODER_MOTOR_CONTROL
    EncoderMotor * tRightMotorIfPositiveTurn;
    EncoderMotor * tLeftMotorIfPositiveTurn;
#else
    PWMDcMotor * tRightMotorIfPositiveTurn;
    PWMDcMotor * tLeftMotorIfPositiveTurn;
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
#ifdef USE_MPU6050_IMU
    unsigned int tDistanceCount = 200; // Dummy value for distance - equivalent to #define tDistanceCount 0xFF
#else
    unsigned int tDistanceCount = (aRotationDegrees * FactorDegreeToCount) + 0.5;
#endif
    unsigned int tDistanceCountRight;
    unsigned int tDistanceCountLeft;

    if (aTurnDirection == TURN_FORWARD) {
        tDistanceCountRight = tDistanceCount;
        tDistanceCountLeft = 0;
    } else if (aTurnDirection == TURN_BACKWARD) {
        tDistanceCountRight = 0;
        tDistanceCountLeft = tDistanceCount;
    } else {
        tDistanceCountRight = tDistanceCount / 2;
        tDistanceCountLeft = tDistanceCount / 2;
    }

    /*
     * Handle slow speed flag
     */
    uint8_t tTurnSpeedRight = tRightMotorIfPositiveTurn->DriveSpeed;
    uint8_t tTurnSpeedLeft = tLeftMotorIfPositiveTurn->DriveSpeed;
    if (aUseSlowSpeed) {
        // avoid overflow, the reduced speed is almost max speed then.
        if (tRightMotorIfPositiveTurn->StartSpeed < 160) {
            tTurnSpeedRight = tRightMotorIfPositiveTurn->StartSpeed + (tRightMotorIfPositiveTurn->StartSpeed / 2);
        }
        if (tLeftMotorIfPositiveTurn->StartSpeed < 160) {
            tTurnSpeedLeft = tLeftMotorIfPositiveTurn->StartSpeed + (tLeftMotorIfPositiveTurn->StartSpeed / 2);
        }
    }

#ifdef DEBUG
    Serial.print(F("TurnSpeedRight="));
    Serial.print(tTurnSpeedRight);
    Serial.print(F(" DistanceCountRight="));
    Serial.println(tDistanceCountRight);
#endif
    tRightMotorIfPositiveTurn->startGoDistanceCount(tTurnSpeedRight, tDistanceCountRight, DIRECTION_FORWARD);
    tLeftMotorIfPositiveTurn->startGoDistanceCount(tTurnSpeedLeft, tDistanceCountLeft, DIRECTION_BACKWARD);
}

/**
 * @param  aRotationDegrees positive -> turn left (counterclockwise), negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE (default)
 * @param  aUseSlowSpeed true (default) -> use slower speed (1.5 times StartSpeed) instead of DriveSpeed for rotation to be more exact
 *         only sensible for encoder motors
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void CarMotorControl::rotate(int aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed, void (*aLoopCallback)(void)) {
    if (aRotationDegrees != 0) {
        startRotate(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilStopped(aLoopCallback);
    }
}

#ifndef USE_ENCODER_MOTOR_CONTROL
void CarMotorControl::setMillisPerDistanceCountForFixedDistanceDriving(uint8_t aMillisPerDistanceCount) {
    rightCarMotor.setMillisPerDistanceCountForFixedDistanceDriving(aMillisPerDistanceCount);
    leftCarMotor.setMillisPerDistanceCountForFixedDistanceDriving(aMillisPerDistanceCount);
}

#else

/*
 * Get count / distance value from right motor
 */
unsigned int CarMotorControl::getDistanceCount() {
    return (rightCarMotor.EncoderCount);
}

int CarMotorControl::getDistanceCentimeter() {
    return (rightCarMotor.EncoderCount / FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT);
}

/*
 * generates a rising ramp and detects the first movement -> this sets dead band / minimum Speed
 */
void CarMotorControl::calibrate() {
    stop();
    resetControlValues();

    rightCarMotor.StartSpeed = 0;
    leftCarMotor.StartSpeed = 0;

    uint8_t tMotorMovingCount = 0;

    /*
     * increase motor speed by 1 until motor moves
     */
    for (uint8_t tSpeed = 20; tSpeed != 0xFF; ++tSpeed) {
        if (rightCarMotor.StartSpeed == 0) {
            rightCarMotor.setSpeed(tSpeed, DIRECTION_FORWARD);
        }
        if (leftCarMotor.StartSpeed == 0) {
            leftCarMotor.setSpeed(tSpeed, DIRECTION_FORWARD);
        }

        delay(100);
        /*
         * Check if wheel moved
         */

        /*
         * Store speed after 6 counts (3cm)
         */
        if (rightCarMotor.StartSpeed == 0 && rightCarMotor.EncoderCount > 6) {
            rightCarMotor.StartSpeed = tSpeed;
            tMotorMovingCount++;
        }
        if (leftCarMotor.StartSpeed == 0 && leftCarMotor.EncoderCount > 6) {
            leftCarMotor.StartSpeed = tSpeed;
            tMotorMovingCount++;
        }
        if (tMotorMovingCount >= 2) {
            // Do not end loop if one motor still not moving
            break;
        }
    }

    /*
     * TODO calibrate StopSpeed separately
     */
    stop();
}
#endif // USE_ENCODER_MOTOR_CONTROL
