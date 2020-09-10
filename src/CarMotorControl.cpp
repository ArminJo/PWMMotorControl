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

#ifdef USE_ENCODER_MOTOR_CONTROL
EncoderMotor rightCarMotor;
EncoderMotor leftCarMotor;
#else
PWMDcMotor rightCarMotor;
PWMDcMotor leftCarMotor;
#endif

CarMotorControl::CarMotorControl() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
void CarMotorControl::init(bool aReadFromEeprom) {
    leftCarMotor.init(1, aReadFromEeprom);
    rightCarMotor.init(2, aReadFromEeprom);

    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT;

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

    FactorDegreeToCount = FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT;

#ifdef USE_ENCODER_MOTOR_CONTROL
    /*
     * For slot type optocoupler interrupts on pin PD2 + PD3
     */
    EncoderMotor::enableINT0AndINT1Interrupts();
#endif
}
#endif

/*
 * Sets default values for min and max speed, factor for distance to time conversion for non encoder motors and reset compensation
 */
void CarMotorControl::setDefaultsForFixedDistanceDriving() {
    rightCarMotor.setDefaultsForFixedDistanceDriving();
    leftCarMotor.setDefaultsForFixedDistanceDriving();
}

void CarMotorControl::setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, uint8_t aSpeedCompensation) {
    rightCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, aSpeedCompensation);
    leftCarMotor.setValuesForFixedDistanceDriving(aStartSpeed, aDriveSpeed, aSpeedCompensation);
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
#ifdef USE_ENCODER_MOTOR_CONTROL
    return (rightCarMotor.State == MOTOR_STATE_RAMP_DOWN || rightCarMotor.State == MOTOR_STATE_RAMP_UP
            || leftCarMotor.State == MOTOR_STATE_RAMP_DOWN || leftCarMotor.State == MOTOR_STATE_RAMP_UP);
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
 * Start motor for "infinite" distance and then blocking wait until both motors are at drive speed
 * If motor is still running, just update motor.
 */
void CarMotorControl::startCarAndWaitForDriveSpeed(uint8_t aDriveDirection) {
#ifdef USE_ENCODER_MOTOR_CONTROL
    if (rightCarMotor.State == MOTOR_STATE_STOPPED || leftCarMotor.State == MOTOR_STATE_STOPPED
            || rightCarMotor.CurrentDirection != aDriveDirection) {
        /*
         * Start only if not already started or direction changed
         */
        initGoDistanceCentimeter(INFINITE_DISTANCE_CM, aDriveDirection);

#  ifdef DEBUG
        Serial.print(F("Start car dir="));
        Serial.println(aDriveDirection);
#  endif
    }
    /*
     * blocking wait for start
     */
    bool tMotorsNotStopped;
    do {
        tMotorsNotStopped = rightCarMotor.updateMotor();
        tMotorsNotStopped |= leftCarMotor.updateMotor();
    } while (tMotorsNotStopped && (rightCarMotor.State != MOTOR_STATE_FULL_SPEED || leftCarMotor.State != MOTOR_STATE_FULL_SPEED));
#else
    rightCarMotor.setSpeedCompensated(rightCarMotor.DriveSpeed, aDriveDirection);
    leftCarMotor.setSpeedCompensated(leftCarMotor.DriveSpeed, aDriveDirection);
#endif
}

/*
 * Stop car with ramp and give DistanceCountAfterRampUp counts for braking.
 *
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_FULL_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Blocking wait for stop
 */
void CarMotorControl::stopCarAndWaitForIt() {
    if (isStopped()) {
        return;
    }
#ifdef USE_ENCODER_MOTOR_CONTROL
    rightCarMotor.NextChangeMaxTargetCount = rightCarMotor.EncoderCount;
    rightCarMotor.TargetDistanceCount = rightCarMotor.EncoderCount + rightCarMotor.DistanceCountAfterRampUp;
    leftCarMotor.NextChangeMaxTargetCount = leftCarMotor.EncoderCount;
    leftCarMotor.TargetDistanceCount = leftCarMotor.EncoderCount + leftCarMotor.DistanceCountAfterRampUp;
#endif
    /*
     * blocking wait for stop
     */
    waitUntilCarStopped();
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
#ifdef USE_ENCODER_MOTOR_CONTROL
    return (rightCarMotor.State == aState && leftCarMotor.State == aState);
#else
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
 * @param  if aUseSlowSpeed is true then use slower speed (1.5 times StartSpeed) for rotation to be more exact
 */
void CarMotorControl::initRotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed) {
    int tDistanceCountRight;
    int tDistanceCountLeft;
    int tDistanceCount;

    uint8_t tTurnDirectionRightMotor = aTurnDirection; // set turn direction as start direction
    uint8_t tTurnDirectionLeftMotor = aTurnDirection;
    if (aTurnDirection == TURN_BACKWARD) {
        // swap turn sign / (left / right) to enable other motor
        aRotationDegrees = -aRotationDegrees;
    }

    tDistanceCount = (aRotationDegrees * FactorDegreeToCount) + 0.5;

    /*
     * Here aRotationDegrees is modified for TURN_FORWARD
     */
    if (aRotationDegrees > 0) {
        // turn left, compute values for TURN_FORWARD
        tDistanceCountRight = tDistanceCount;
        tDistanceCountLeft = 0;
        if (aTurnDirection == TURN_IN_PLACE) {
            tDistanceCountRight /= 2;
            tDistanceCountLeft = tDistanceCountRight;
            // The other motor has turn direction TURN_IN_PLACE which is masked to TURN_FORWARD by initGoDistanceCount()
            tTurnDirectionLeftMotor = TURN_BACKWARD;
        }
    } else {
        // turn right, compute values for TURN_FORWARD
        tDistanceCountLeft = tDistanceCount;
        tDistanceCountLeft = -tDistanceCountLeft;
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
#ifdef USE_ENCODER_MOTOR_CONTROL
        rightCarMotor.CurrentDriveSpeed = rightCarMotor.StartSpeed + rightCarMotor.StartSpeed / 2;
        leftCarMotor.CurrentDriveSpeed = leftCarMotor.StartSpeed + leftCarMotor.StartSpeed / 2;
#else
        rightCarMotor.setSpeedCompensated(rightCarMotor.StartSpeed + rightCarMotor.StartSpeed / 2, rightCarMotor.CurrentDirection);
        leftCarMotor.setSpeedCompensated(leftCarMotor.StartSpeed + leftCarMotor.StartSpeed / 2, leftCarMotor.CurrentDirection);
#endif
    }
}

/*
 * @param  aRotationDegrees positive -> turn left, negative -> turn right
 * @param  aLoopCallback avoid blocking and call aLoopCallback on waiting for stop
 */
void CarMotorControl::rotateCar(int16_t aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed, void (*aLoopCallback)(void)) {
    if (aRotationDegrees != 0) {
        initRotateCar(aRotationDegrees, aTurnDirection, aUseSlowSpeed);
        waitUntilCarStopped(aLoopCallback);
    }
}

/*
 * generates a rising ramp and detects the first movement -> this sets dead band / minimum Speed
 */
void CarMotorControl::calibrate() {
    stopMotors();
    resetControlValues();
#ifdef USE_ENCODER_MOTOR_CONTROL

    rightCarMotor.StartSpeed = 0;
    leftCarMotor.StartSpeed = 0;

    /*
     * increase motor speed by 1 until motor moves
     */
    for (uint8_t tSpeed = 20; tSpeed != 0xFF; ++tSpeed) {
        if (rightCarMotor.StartSpeed == 0) {
            rightCarMotor.setSpeed(tSpeed, DIRECTION_FORWARD);
            rightCarMotor.CurrentSpeed = tSpeed;
        }
        if (leftCarMotor.StartSpeed == 0) {
            leftCarMotor.setSpeed(tSpeed, DIRECTION_FORWARD);
            leftCarMotor.CurrentSpeed = tSpeed;
        }

        delay(100);
        /*
         * Check if wheel moved
         */
        uint8_t tMotorMovingCount = 0;
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
#endif
}

#ifdef USE_ENCODER_MOTOR_CONTROL
// ISR for PIN PD2 / RIGHT
ISR(INT0_vect) {
    rightCarMotor.handleEncoderInterrupt();
}

// ISR for PIN PD3 / LEFT
ISR(INT1_vect) {
    leftCarMotor.handleEncoderInterrupt();
}
#endif
