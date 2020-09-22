/*
 * CarMotorControl.cpp
 *
 *  Contains functions for control of the 2 motors of a car like setDirection, goDistanceCentimeter() and rotateCar().
 *  Checks input of PIN aPinFor2WDDetection since we need different factors for rotating a 4 wheel and a 2 wheel car.
 *
 *  Requires EncoderMotor.cpp
 *
 *  Created on: 16.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
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

CarMotorControl * sCarMotorControlPointerForISR;

CarMotorControl::CarMotorControl() { // @suppress("Class members should be properly initialized")
    sCarMotorControlPointerForISR = this;
}

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
void CarMotorControl::init(bool aReadFromEeprom) {
    leftCarMotor.init(1, aReadFromEeprom);
    rightCarMotor.init(2, aReadFromEeprom);

#if defined(CAR_HAS_4_WHEELS)
    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_4WD_CAR_DEFAULT;
#else
    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT;
#endif

#  ifdef USE_ENCODER_MOTOR_CONTROL
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();
#  endif
}

#else
void CarMotorControl::init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin,
        uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin, bool aReadFromEeprom) {
    if (aReadFromEeprom) {
        leftCarMotor.init(aLeftMotorForwardPin, LeftMotorBackwardPin, aLeftMotorPWMPin, 1);
        rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin, 2);
    } else {
        leftCarMotor.init(aLeftMotorForwardPin, LeftMotorBackwardPin, aLeftMotorPWMPin);
        rightCarMotor.init(aRightMotorForwardPin, aRightMotorBackwardPin, aRightPWMPin);
    }

    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_DEFAULT;

#  ifdef USE_ENCODER_MOTOR_CONTROL
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();
#  endif
}
#endif

/*
 * Sets default values for min and max speed, factor for distance to time conversion for non encoder motors and reset compensation
 */
void CarMotorControl::setDefaultsForFixedDistanceDriving() {
    rightCarMotor.setDefaultsForFixedDistanceDriving();
    leftCarMotor.setDefaultsForFixedDistanceDriving();
}

/**
 * @param aSpeedCompensationRight if positive, this value is subtracted from the speed of the right motor, if negative, -value is subtracted from the left speed.
 */
void CarMotorControl::setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, int8_t aSpeedCompensationRight) {
    if (aSpeedCompensationRight > 0) {
        rightCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, aSpeedCompensationRight);
        leftCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, 0);
    } else {
        rightCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, 0);
        leftCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, -aSpeedCompensationRight);
    }
}

/*
 *  Direct motor control, no state or flag handling
 */
void CarMotorControl::setSpeed(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    rightCarMotor.setSpeed(aRequestedSpeed, aRequestedDirection);
    leftCarMotor.setSpeed(aRequestedSpeed, aRequestedDirection);
}

/*
 * Sets speed adjusted by current compensation value and handle motor state and flags
 */
void CarMotorControl::setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    rightCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
    leftCarMotor.setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
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

/*
 * Stop car
 * @param aStopMode STOP_MODE_KEEP (take previously defined StopMode) or MOTOR_BRAKE or MOTOR_RELEASE
 */
void CarMotorControl::stopMotors(uint8_t aStopMode) {
    rightCarMotor.stop(aStopMode);
    leftCarMotor.stop(aStopMode);
}

/*
 * @param aStopMode MOTOR_BRAKE or MOTOR_RELEASE
 */
void CarMotorControl::setStopMode(uint8_t aStopMode) {
    rightCarMotor.setStopMode(aStopMode);
    leftCarMotor.setStopMode(aStopMode);
}

/*
 * Stop car and reset all control values as speed, distances, debug values to 0x00
 * Leave calibration and compensation (EEPROM) values unaffected.
 */
void CarMotorControl::resetControlValues() {
#ifdef USE_ENCODER_MOTOR_CONTROL
    rightCarMotor.resetControlValues();
    leftCarMotor.resetControlValues();
#endif
}

/*
 * If motor is accelerating or decelerating then updateMotor needs to be called at a fast rate otherwise it will not work correctly
 * Used to suppress time consuming display of motor values
 */
bool CarMotorControl::needsFastUpdates() {
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
    bool tMotorsNotStopped = rightCarMotor.updateMotor();
    tMotorsNotStopped |= leftCarMotor.updateMotor();
    return tMotorsNotStopped;
}

#ifdef SUPPORT_RAMP_UP
void CarMotorControl::initRampUp(uint8_t aRequestedDirection) {
    rightCarMotor.initRampUp(aRequestedDirection);
    leftCarMotor.initRampUp(aRequestedDirection);
}

/*
 * Blocking wait until both motors are at drive speed.
 */
void CarMotorControl::waitForDriveSpeed() {
    /*
     * blocking wait for MOTOR_STATE_DRIVE_SPEED
     */
    bool tMotorsNotStopped;
    do {
        tMotorsNotStopped = rightCarMotor.updateMotor();
        tMotorsNotStopped |= leftCarMotor.updateMotor();
    } while (tMotorsNotStopped
            && (rightCarMotor.MotorRampState != MOTOR_STATE_DRIVE_SPEED || leftCarMotor.MotorRampState != MOTOR_STATE_DRIVE_SPEED));
}
#endif

void CarMotorControl::initRampUpAndWaitForDriveSpeed(uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
#ifdef SUPPORT_RAMP_UP
    rightCarMotor.initRampUp(aRequestedDirection);
    leftCarMotor.initRampUp(aRequestedDirection);
    /*
     * blocking wait for MOTOR_STATE_DRIVE_SPEED, 256 milliseconds
     */
    bool tMotorsNotStopped; // to check if motors are not stopped by aLoopCallback
    do {
        tMotorsNotStopped = rightCarMotor.updateMotor();
        tMotorsNotStopped |= leftCarMotor.updateMotor();
        if (aLoopCallback != NULL) {
            aLoopCallback(); // this may stop motors
        }
    } while (tMotorsNotStopped
            && (rightCarMotor.MotorRampState != MOTOR_STATE_DRIVE_SPEED || leftCarMotor.MotorRampState != MOTOR_STATE_DRIVE_SPEED));
#else
    (void) aLoopCallback;
    rightCarMotor.setSpeedCompensated(rightCarMotor.DriveSpeed, aRequestedDirection);
    leftCarMotor.setSpeedCompensated(leftCarMotor.DriveSpeed, aRequestedDirection);
#endif
}
/*
 * initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::initGoDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection) {
    rightCarMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT, aRequestedDirection);
    leftCarMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT, aRequestedDirection);
}

void CarMotorControl::goDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection,
        void (*aLoopCallback)(void)) {
    initGoDistanceCentimeter(aDistanceCentimeter, aRequestedDirection);
    waitUntilCarStopped(aLoopCallback);
}

/*
 * initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void CarMotorControl::initGoDistanceCentimeter(int aDistanceCentimeter) {
    rightCarMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT);
    leftCarMotor.initGoDistanceCount(aDistanceCentimeter * FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT);
}
/**
 * Wait until distance is reached
 * @param  aLoopCallback called until car has stopped to avoid blocking
 */
void CarMotorControl::goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void)) {
    initGoDistanceCentimeter(aDistanceCentimeter);
    waitUntilCarStopped(aLoopCallback);
}

/*
 * Stop car with ramp and give DistanceCountAfterRampUp counts for braking.
 *
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_DRIVE_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Blocking wait for stop
 */
void CarMotorControl::stopCarAndWaitForIt() {
    if (isStopped()) {
        return;
    }
#if defined(USE_ENCODER_MOTOR_CONTROL) && defined(SUPPORT_RAMP_UP)
    rightCarMotor.NextChangeMaxTargetCount = rightCarMotor.EncoderCount;
    rightCarMotor.TargetDistanceCount = rightCarMotor.EncoderCount + rightCarMotor.DistanceCountAfterRampUp;
    leftCarMotor.NextChangeMaxTargetCount = leftCarMotor.EncoderCount;
    leftCarMotor.TargetDistanceCount = leftCarMotor.EncoderCount + leftCarMotor.DistanceCountAfterRampUp;
    /*
     * blocking wait for stop
     */
    waitUntilCarStopped();
#else
    rightCarMotor.stop();
    leftCarMotor.stop();
#endif
}

/*
 * Wait with optional wait loop callback
 */
void CarMotorControl::waitUntilCarStopped(void (*aLoopCallback)(void)) {
    do {
        rightCarMotor.updateMotor();
        leftCarMotor.updateMotor();
        if (aLoopCallback != NULL) {
            aLoopCallback();
        }
    } while (!isStopped());
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
    FactorDegreeToCount = aFactorDegreeToCount;
}

/**
 * Set distances and speed for 2 motors to turn the requested angle
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE
 * @param  aUseSlowSpeed true -> use slower speed (1.5 times StartSpeed) instead of DriveSpeed for rotation to be more exact
 */
void CarMotorControl::initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    int tDistanceCountRight;
    int tDistanceCountLeft;
    uint16_t tDistanceCount;

    uint8_t tTurnDirectionRightMotor = aTurnDirection; // set turn direction as start direction
    uint8_t tTurnDirectionLeftMotor = aTurnDirection;
    if (aTurnDirection == TURN_BACKWARD) {
        // swap turn sign / (left / right) to move other motor
        aRotationDegrees = -aRotationDegrees;
    }

    /*
     * Here aRotationDegrees is modified for TURN_FORWARD
     */
    if (aRotationDegrees > 0) {
        tDistanceCount = (aRotationDegrees * FactorDegreeToCount) + 0.5;
        /*
         * Move right motor (except for TURN_IN_PLACE)
         * Here we have a left turn and a direction TURN_FORWARD or TURN_IN_PLACE
         * OR a right turn and a direction TURN_BACKWARD
         */
        tDistanceCountRight = tDistanceCount;
        tDistanceCountLeft = 0;

        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountRight /= 2;
            tDistanceCountLeft = tDistanceCountRight;
            // The right motor has turn direction TURN_IN_PLACE which is masked to TURN_FORWARD by initGoDistanceCount()
            tTurnDirectionLeftMotor = TURN_BACKWARD;
        }
    } else {
        tDistanceCount = (-aRotationDegrees * FactorDegreeToCount) + 0.5;
        /*
         * Move left motor (except for TURN_IN_PLACE)
         * Here we have a right turn and a direction TURN_FORWARD or TURN_IN_PLACE
         * OR a left turn and a direction TURN_BACKWARD
         */
        tDistanceCountLeft = tDistanceCount;
        tDistanceCountRight = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountLeft /= 2;
            tDistanceCountRight = tDistanceCountLeft;
            tTurnDirectionRightMotor = TURN_BACKWARD;
        }
    }

    // This in turn sets CurrentDriveSpeed to DriveSpeed.
    rightCarMotor.initGoDistanceCount(tDistanceCountRight, tTurnDirectionRightMotor);
    leftCarMotor.initGoDistanceCount(tDistanceCountLeft, tTurnDirectionLeftMotor);
    if (aUseSlowSpeed) {
        // adjust CurrentDriveSpeed
#if defined(SUPPORT_RAMP_UP)
        // avoid overflow, the reduced speed is almost max speed then.
        if (rightCarMotor.StartSpeed < 160) {
            rightCarMotor.CurrentDriveSpeed = rightCarMotor.StartSpeed + rightCarMotor.StartSpeed / 2;
        }
        if (leftCarMotor.StartSpeed < 160) {
            leftCarMotor.CurrentDriveSpeed = leftCarMotor.StartSpeed + leftCarMotor.StartSpeed / 2;
        }
#else
        if (tDistanceCountRight > 0) {
            rightCarMotor.setSpeedCompensated(rightCarMotor.StartSpeed + rightCarMotor.StartSpeed / 2,
                    rightCarMotor.CurrentDirectionOrBrakeMode);
        }
        if (tDistanceCountLeft > 0) {
            leftCarMotor.setSpeedCompensated(leftCarMotor.StartSpeed + leftCarMotor.StartSpeed / 2,
                    leftCarMotor.CurrentDirectionOrBrakeMode);
        }
#endif
    }
}

/**
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aTurnDirection direction of turn TURN_FORWARD, TURN_BACKWARD or TURN_IN_PLACE (default)
 * @param  aUseSlowSpeed true (default) -> use slower speed (1.5 times StartSpeed) instead of DriveSpeed for rotation to be more exact
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void CarMotorControl::rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed, void (*aLoopCallback)(void)) {
    if (aRotationDegrees != 0) {
        initRotateCar(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilCarStopped(aLoopCallback);
    }
}

#ifndef USE_ENCODER_MOTOR_CONTROL
void CarMotorControl::setDistanceToTimeFactorForFixedDistanceDriving(uint16_t aDistanceToTimeFactor) {
    rightCarMotor.setDistanceToTimeFactorForFixedDistanceDriving(aDistanceToTimeFactor);
    leftCarMotor.setDistanceToTimeFactorForFixedDistanceDriving(aDistanceToTimeFactor);
}
#else
/*
 * generates a rising ramp and detects the first movement -> this sets dead band / minimum Speed
 */
void CarMotorControl::calibrate() {
    stopMotors();
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

    rightCarMotor.writeMotorvaluesToEeprom();
    leftCarMotor.writeMotorvaluesToEeprom();

    stopMotors();
}

// ISR for PIN PD2 / RIGHT
ISR(INT0_vect) {
    sCarMotorControlPointerForISR->rightCarMotor.handleEncoderInterrupt();
}

// ISR for PIN PD3 / LEFT
ISR(INT1_vect) {
    sCarMotorControlPointerForISR->leftCarMotor.handleEncoderInterrupt();
}
#endif // USE_ENCODER_MOTOR_CONTROL
