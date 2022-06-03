/*
 * MecanumWheelCarPWMMotorControl.hpp
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

#ifndef _MECANUM_WHEEL_CAR_PWM_MOTOR_CONTROL_HPP
#define _MECANUM_WHEEL_CAR_PWM_MOTOR_CONTROL_HPP

#if !defined(CAR_HAS_4_MECANUM_WHEELS)
#define CAR_HAS_4_MECANUM_WHEELS
#endif

#include "MecanumWheelCarPWMMotorControl.h"
#include "CarPWMMotorControl.hpp"

/*
 * The Car Control instance to be used by the main program
 */
//MecanumWheelCarPWMMotorControl RobotCar;

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

MecanumWheelCarPWMMotorControl::MecanumWheelCarPWMMotorControl() { // @suppress("Class members should be properly initialized")
}

#if defined(USE_MPU6050_IMU)
/*
 * This must be done when the car is not moving, best after at least 100 ms after boot up.
 */
void MecanumWheelCarPWMMotorControl::calculateAndPrintIMUOffsets(Print *aSerial) {
    IMUData.initMPU6050CarDataAndCalculateAllOffsetsAndWait();
    IMUData.printSpeedAndTurnOffsets(aSerial);
}
#endif

/*
 * If no parameter and we have encoder motors, we use a fixed assignment of rightCarMotor interrupts to INT0 / Pin2 and leftCarMotor to INT1 / Pin3
 */
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
void MecanumWheelCarPWMMotorControl::init() {
    CarPWMMotorControl::init();

    // we have two wheels without encoders
    backLeftCarMotor.init(3);
    backRightCarMotor.init(4);
}

#else // USE_ADAFRUIT_MOTOR_SHIELD
void MecanumWheelCarPWMMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aPWMPin,
        uint8_t aLeftMotorForwardPin, uint8_t aLeftMotorBackwardPin, uint8_t aBackRightMotorForwardPin,
        uint8_t aBackRightMotorBackwardPin, uint8_t aBackLeftMotorForwardPin, uint8_t aBackLeftMotorBackwardPin) {

    CarPWMMotorControl::init(aRightMotorForwardPin, aRightMotorBackwardPin, aPWMPin, aLeftMotorForwardPin, aLeftMotorBackwardPin,
            aPWMPin);

    backRightCarMotor.init(aBackRightMotorForwardPin, aBackRightMotorBackwardPin, aPWMPin);
    backLeftCarMotor.init(aBackLeftMotorForwardPin, aBackLeftMotorBackwardPin, aPWMPin);
}
#endif // USE_ADAFRUIT_MOTOR_SHIELD

/*
 * Stop car
 * @param aStopMode STOP_MODE_KEEP (take previously defined StopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void MecanumWheelCarPWMMotorControl::stop(uint8_t aStopMode) {
    rightCarMotor.stop(aStopMode);
    leftCarMotor.stop(aStopMode);
    backRightCarMotor.stop(aStopMode);
    backLeftCarMotor.stop(aStopMode);
    CarDirection = DIRECTION_STOP;
}

/*
 * @param aStopMode used for speed == 0 or STOP_MODE_KEEP: STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void MecanumWheelCarPWMMotorControl::setStopMode(uint8_t aStopMode) {
    rightCarMotor.setStopMode(aStopMode);
    leftCarMotor.setStopMode(aStopMode);
    backRightCarMotor.setStopMode(aStopMode);
    backLeftCarMotor.setStopMode(aStopMode);
}

/*
 * Sets default values for min and max speed, factor for distance to time conversion for non encoder motors and reset speed compensation
 * Is called automatically at init if parameter aReadFromEeprom is set to false
 */
void MecanumWheelCarPWMMotorControl::setDefaultsForFixedDistanceDriving() {
    rightCarMotor.setDefaultsForFixedDistanceDriving();
}

/**
 * @param aSpeedPWMCompensationRight if positive, this value is added to the compensation value of the right motor, or subtracted from the left motor value.
 *  If negative, -value is added to the compensation value the left motor, or subtracted from the right motor value.
 */
void MecanumWheelCarPWMMotorControl::setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM,
        int8_t aSpeedPWMCompensationRight) {
    rightCarMotor.setDriveSpeedPWM(aDriveSpeedPWM);
    (void) aSpeedPWMCompensationRight;

}

void MecanumWheelCarPWMMotorControl::setSpeedPWMCompensation(int8_t aSpeedPWMCompensationRight) {
    (void) aSpeedPWMCompensationRight;
}

void MecanumWheelCarPWMMotorControl::changeSpeedPWMCompensation(int8_t aSpeedPWMCompensationRightDelta) {
    (void) aSpeedPWMCompensationRightDelta;
}

void MecanumWheelCarPWMMotorControl::setDriveSpeedPWM(uint8_t aDriveSpeedPWM) {
    rightCarMotor.setDriveSpeedPWM(aDriveSpeedPWM);
}

void MecanumWheelCarPWMMotorControl::setDriveSpeedPWMFor2Volt(uint16_t aFullBridgeInputVoltageMillivolt) {
    rightCarMotor.setDriveSpeedPWMFor2Volt(aFullBridgeInputVoltageMillivolt);
}

void MecanumWheelCarPWMMotorControl::setDriveSpeedPWMFor2Volt(float aFullBridgeInputVoltageMillivolt) {
    rightCarMotor.setDriveSpeedPWMFor2Volt(aFullBridgeInputVoltageMillivolt);
}

/*
 * Checks speed and direction and stops car if required
 * @return true if direction has changed and motor has stopped
 */
bool MecanumWheelCarPWMMotorControl::checkAndHandleDirectionChange(uint8_t aRequestedDirection) {
    bool tReturnValue = false;
    if (CarDirection != aRequestedDirection) {
        uint8_t tMaxRequestedSpeedPWM = rightCarMotor.RequestedSpeedPWM;
        if (tMaxRequestedSpeedPWM > 0) {
            /*
             * Direction change requested but motor(s) still running-> first stop motor(s)
             */
#if defined(LOCAL_DEBUG)
            Serial.println(F("First stop motor(s) and wait"));
#endif
            stop(STOP_MODE_BRAKE);
//            delay(((tMaxCompensatedSpeedPWM * tMaxCompensatedSpeedPWM) >> 8) * 2); // to let motors stop
            delay(tMaxRequestedSpeedPWM); // to let motors stop
            tReturnValue = true;
        }
#if defined(LOCAL_DEBUG)
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
void MecanumWheelCarPWMMotorControl::setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    setDirection(aRequestedDirection); // sets direction for all 4 motors
    rightCarMotor.setSpeedPWM(aRequestedSpeedPWM);
}

/**
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
    setSpeedPWMAndDirection(0);
}

/*
 * Sets speed adjusted by current compensation value and keeps direction
 */
void MecanumWheelCarPWMMotorControl::changeSpeedPWM(uint8_t aRequestedSpeedPWM) {
    rightCarMotor.changeSpeedPWM(aRequestedSpeedPWM);
}

/*
 * Sets speed adjusted by current compensation value and handle motor state and flags
 * @param aLeftRightSpeedPWM if positive, this value is subtracted from the left motor value, if negative subtracted from the right motor value
 */
void MecanumWheelCarPWMMotorControl::setSpeedPWMWithDeltaAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection,
        int8_t aSpeedPWMCompensationRightDelta) {
    checkAndHandleDirectionChange(aRequestedDirection);
    setDirection(aRequestedDirection); // sets direction for all 4 motors
    rightCarMotor.setSpeedPWM(aRequestedSpeedPWM);
    (void) aSpeedPWMCompensationRightDelta;
}

/*
 *  Direct motor control, no state or flag handling
 */
void MecanumWheelCarPWMMotorControl::setDirection(uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);

    uint8_t tRequestedDirection = aRequestedDirection & DIRECTION_MASK;
#  if defined(LOCAL_DEBUG)
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

void MecanumWheelCarPWMMotorControl::setSpeedPWM(uint8_t aRequestedSpeedPWM) {
    rightCarMotor.setSpeedPWM(aRequestedSpeedPWM);
}

void MecanumWheelCarPWMMotorControl::setSpeedPWMAndDirection(int aRequestedSpeedPWM) {
    rightCarMotor.setSpeedPWMAndDirection(aRequestedSpeedPWM);
    if (aRequestedSpeedPWM < 0) {
        aRequestedSpeedPWM = -aRequestedSpeedPWM;
        setSpeedPWMAndDirection(aRequestedSpeedPWM, DIRECTION_BACKWARD);
    } else {
        setSpeedPWMAndDirection(aRequestedSpeedPWM, DIRECTION_FORWARD);
    }
}

void MecanumWheelCarPWMMotorControl::readMotorValuesFromEeprom() {
    rightCarMotor.readMotorValuesFromEeprom(0);
}

void MecanumWheelCarPWMMotorControl::writeMotorValuesToEeprom() {
    rightCarMotor.writeMotorValuesToEeprom(0);
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
bool MecanumWheelCarPWMMotorControl::updateMotors() {
#if defined(USE_MPU6050_IMU)
    CarPWMMotorControl::updateMotors();
#else // USE_MPU6050_IMU
    bool tReturnValue = rightCarMotor.updateMotor();
    if (!tReturnValue && rightCarMotor.MotorPWMHasChanged) {
        stop(); // stop all other motors too, if right car motor was just stopped
    }
#endif // USE_MPU6050_IMU

    return tReturnValue;;
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool MecanumWheelCarPWMMotorControl::updateMotors(void (*aLoopCallback)(void)) {
    if (aLoopCallback != NULL) {
        aLoopCallback();
    }
    return updateMotors();
}

void MecanumWheelCarPWMMotorControl::delayAndUpdateMotors(unsigned int aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        updateMotors();
    } while (millis() - tStartMillis <= aDelayMillis);
}

void MecanumWheelCarPWMMotorControl::startRampUp(uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startRampUp(aRequestedDirection);
    setDirection(aRequestedDirection); // set direction for all other motors too
}

void MecanumWheelCarPWMMotorControl::setSpeedPWMWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.setSpeedPWMAndDirectionWithRamp(aRequestedSpeedPWM, aRequestedDirection);
    setDirection(aRequestedDirection); // set direction for all other motors too
}

/*
 * Blocking wait until both motors are at drive SpeedPWM. 256 milliseconds for ramp up.
 * @param aLoopCallback The callback called while waiting for motor to reach MOTOR_STATE_DRIVE.
 */
void MecanumWheelCarPWMMotorControl::waitForDriveSpeedPWM(void (*aLoopCallback)(void)) {
#if !defined(DO_NOT_SUPPORT_RAMP)
    while (updateMotors(aLoopCallback) && (rightCarMotor.MotorRampState != MOTOR_STATE_DRIVE)) {
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
void MecanumWheelCarPWMMotorControl::startRampUpAndWait(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection,
        void (*aLoopCallback)(void)) {
    setSpeedPWMWithRamp(aRequestedSpeedPWM, aRequestedDirection);
    waitForDriveSpeedPWM(aLoopCallback);
}

void MecanumWheelCarPWMMotorControl::startRampUpAndWaitForDriveSpeedPWM(uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
    startRampUp(aRequestedDirection);
    waitForDriveSpeedPWM(aLoopCallback);
}

void MecanumWheelCarPWMMotorControl::startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(rightCarMotor.DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * initialize motorInfo fields LastDirection and CompensatedSpeedPWM
 */
void MecanumWheelCarPWMMotorControl::startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM,
        unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {

#if defined(USE_MPU6050_IMU)
    IMUData.resetAllIMUCarOffsetAdjustedValues();
    CarRequestedDistanceMillimeter = aRequestedDistanceMillimeter;
#endif

#if defined(USE_MPU6050_IMU) && !defined(USE_ENCODER_MOTOR_CONTROL)
    // for non encoder motor we use the IMU distance, and require only the ramp up
    setSpeedPWMWithRamp(aRequestedSpeedPWM, aRequestedDirection);
#else
    checkAndHandleDirectionChange(aRequestedDirection);
    rightCarMotor.startGoDistanceMillimeter(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
    setDirection(aRequestedDirection); // this sets the direction for all the other motors
#endif
}

void MecanumWheelCarPWMMotorControl::goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection,
        void (*aLoopCallback)(void)) {
    startGoDistanceMillimeter(rightCarMotor.DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
    waitUntilStopped(aLoopCallback);
}

void MecanumWheelCarPWMMotorControl::startGoDistanceMillimeter(int aRequestedDistanceMillimeter) {
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
void MecanumWheelCarPWMMotorControl::goDistanceMillimeter(int aRequestedDistanceMillimeter, void (*aLoopCallback)(void)) {
    startGoDistanceMillimeter(aRequestedDistanceMillimeter);
    waitUntilStopped(aLoopCallback);
}

/*
 * Stop car (with ramp)
 */
void MecanumWheelCarPWMMotorControl::stopAndWaitForIt(void (*aLoopCallback)(void)) {
    if (isStopped()) {
        return;
    }

    startRampDown();
    /*
     * blocking wait for stop
     */
    waitUntilStopped(aLoopCallback);
}

void MecanumWheelCarPWMMotorControl::startRampDown() {
    if (isStopped()) {
        return;
    }
#if defined(DO_NOT_SUPPORT_RAMP)
    stop(STOP_MODE_KEEP);
#else
    /*
     * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_DRIVE to MOTOR_STATE_RAMP_DOWN
     */
    rightCarMotor.startRampDown();
#endif
}

/*
 * Wait with optional wait loop callback
 */
void MecanumWheelCarPWMMotorControl::waitUntilStopped(void (*aLoopCallback)(void)) {
    while (updateMotors(aLoopCallback)) {
        ;
    }
}

bool MecanumWheelCarPWMMotorControl::isState(uint8_t aState) {
#if defined(DO_NOT_SUPPORT_RAMP)
    (void) aState;
    return true;
#else
    return (rightCarMotor.MotorRampState == aState);
#endif
}

bool MecanumWheelCarPWMMotorControl::isStopped() {
    return (rightCarMotor.RequestedSpeedPWM == 0);
}

/*
 * If motor is accelerating or decelerating then updateMotor needs to be called at a fast rate otherwise it will not work correctly
 * Used to suppress time consuming display of motor values
 */
bool MecanumWheelCarPWMMotorControl::isStateRamp() {
#if defined(DO_NOT_SUPPORT_RAMP)
    return false;
#else
    return (rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_DOWN || rightCarMotor.MotorRampState == MOTOR_STATE_RAMP_UP);
#endif
}

/**
 * Set distances and SpeedPWM (DriveSpeedPWM or DEFAULT_START_SPEED_PWM) for 2 motors to turn the requested angle
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE
 * @param  aUseSlowSpeed true -> use slower DEFAULT_START_SPEED_PWM instead of DriveSpeedPWM for rotation to be more exact
 */
void MecanumWheelCarPWMMotorControl::startRotate(int aRotationDegrees, turn_direction_t aTurnDirection, bool aUseSlowSpeed) {
    uint8_t tDirection;
    if (aRotationDegrees > 0) {
        tDirection = DIRECTION_LEFT;
    } else {
        aRotationDegrees = -aRotationDegrees;
        tDirection = DIRECTION_RIGHT;
    }

    unsigned int tDistanceMillimeter;
    if (aTurnDirection == TURN_FORWARD) {
        tDirection |= DIRECTION_FORWARD;
        tDistanceMillimeter = aRotationDegrees * FACTOR_DEGREE_TO_MILLIMETER;   // We need around twice the time of an in place turn
    } else if (aTurnDirection == TURN_BACKWARD) {
        tDirection |= DIRECTION_BACKWARD;
        tDistanceMillimeter = aRotationDegrees * FACTOR_DEGREE_TO_MILLIMETER;   // We need around twice the time of an in place turn
    } else {
        tDistanceMillimeter = aRotationDegrees * FACTOR_DEGREE_TO_MILLIMETER_IN_PLACE;
    }
    setDirection(tDirection | DIRECTION_TURN);

    uint8_t tTurnSpeed = rightCarMotor.DriveSpeedPWM;
    if (aUseSlowSpeed) {
        tTurnSpeed = DEFAULT_START_SPEED_PWM;
    }
    // Use direction set by setDirection() above
    rightCarMotor.startGoDistanceMillimeter(tTurnSpeed, tDistanceMillimeter, rightCarMotor.getDirection());
#if defined(LOCAL_DEBUG)
    Serial.print(F("RotationDegrees="));
    Serial.print(aRotationDegrees);
    Serial.print(F(" TurnDirection="));
    Serial.println(sTurnDirectionCharArray[aTurnDirection]);
    Serial.flush();
#endif
}

/**
 * @param  aRotationDegrees positive -> turn left (counterclockwise), negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE (default)
 * @param  aUseSlowSpeed true (not default) -> use slower SpeedPWM (for 4WD cars 3/4 times, for 2WD 0.5 times DriveSpeedPWM)
 *                                             instead of DriveSpeedPWM for rotation to be more exact.
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void MecanumWheelCarPWMMotorControl::rotate(int aRotationDegrees, turn_direction_t aTurnDirection, bool aUseSlowSpeed,
        void (*aLoopCallback)(void)) {
    if (aRotationDegrees != 0) {
        startRotate(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilStopped(aLoopCallback);
    }
}

#if !defined(USE_ENCODER_MOTOR_CONTROL)
void MecanumWheelCarPWMMotorControl::setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond) {
    rightCarMotor.setMillimeterPerSecondForFixedDistanceDriving(aMillimeterPerSecond);
}
#endif // USE_ENCODER_MOTOR_CONTROL

#define MECANUM_FORWARD_TO_LATERAL_FACTOR   (82.0 / 62.0)
#define MECANUM_FORWARD_TO_DIAGONAL_FACTOR   (82.0 / 50.0)
#define MECANUM_FORWARD_TO_DIAGONAL_FACTOR_ORTHOGONAL   (82.0 * M_SQRT2 / 50.0)

/*
 *  ->-
 *  | /
 *  |/
 *  o
 * Forward  82 cm
 * Right    62 cm
 * Diagonal 50 cm
 */
void MecanumWheelCarPWMMotorControl::moveTestDistances(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
}

/*
 * The compensated moveTestDistances in order to go back to the origin
 * The triangle base is at 45 degree
 *  ->-
 *  | /
 *  |/
 *  o
 */
void MecanumWheelCarPWMMotorControl::moveTriangle45(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove * MECANUM_FORWARD_TO_LATERAL_FACTOR);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR_ORTHOGONAL);
}
/**
 * o is origin of movement. The star starts with left forward and goes clockwise.
 *
 * \ /
 *  o
 * / \
 * @param aRequestedSpeedPWM    Any value between 0 and MAX_SPEED_PWM (255)
 * @param aMillisforOneMove     The time used for a sub-move / a move in one direction
 * @param aDelayBetweenMoves    The delay between sub-moves of the move
 */
void MecanumWheelCarPWMMotorControl::moveStar(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_FORWARD, aMillisforOneMove);
}

/*
 * o is origin of movement. The full star starts with forward and goes clockwise.
 *
 * \|/
 * -o-
 * /|\
 */
void MecanumWheelCarPWMMotorControl::moveFullStar(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);

    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_BACKWARD, aMillisforOneMove);
}

/*
 *  ->-
 *  | |
 *  o<-
 */
void MecanumWheelCarPWMMotorControl::moveSqare(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove * MECANUM_FORWARD_TO_LATERAL_FACTOR);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove * MECANUM_FORWARD_TO_LATERAL_FACTOR);
}

/*
 *   /\
 *  |  |
 *  o\/
 */
void MecanumWheelCarPWMMotorControl::moveHexagon(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_FORWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_BACKWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR);
}

/*
 * The triangle base is at 0 degree
 *   / \
 *  o->-
 */
void MecanumWheelCarPWMMotorControl::moveTriangle0(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
}

/*
 *      -<-
 *     |   |
 *  -<-    -<-
 * |          |
 *  ->-    ->-
 *     |   |
 *     o->-
 */
void MecanumWheelCarPWMMotorControl::moveBigPlus(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_BACKWARD, aMillisforOneMove);
}

/*
 *  /\
 *  \/
 *  o
 */
void MecanumWheelCarPWMMotorControl::moveRhombus(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD, aMillisforOneMove);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_RIGHT_BACKWARD, aMillisforOneMove);
}

/*
 *    -
 *   / \
 *  o->-
 */
void MecanumWheelCarPWMMotorControl::moveTrapezium(uint8_t aRequestedSpeedPWM, unsigned int aMillisforOneMove,
        unsigned int aDelayBetweenMoves) {
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_RIGHT, aMillisforOneMove + (aMillisforOneMove / 2));
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_FORWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR_ORTHOGONAL);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_LEFT, aMillisforOneMove / 2);
    delay(aDelayBetweenMoves);
    setSpeedPWMAndDirectionAndDelay(aRequestedSpeedPWM, DIRECTION_DIAGONAL_LEFT_BACKWARD,
            aMillisforOneMove * MECANUM_FORWARD_TO_DIAGONAL_FACTOR_ORTHOGONAL);
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _MECANUM_WHEEL_CAR_PWM_MOTOR_CONTROL_HPP
