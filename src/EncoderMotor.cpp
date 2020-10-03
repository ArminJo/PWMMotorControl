/*
 * EncoderMotor.cpp
 *
 *  Functions for controlling a DC-motor which rotary encoder implemented by slot-type photo interrupter and an attached encoder disc (with 20 slots).
 *  Works with positive (unsigned) speed and direction or signed speed.
 *
 *  Contains functions to go a specified distance.
 *  These functions generates ramps for acceleration and deceleration and tries to stop at target distance.
 *  This enables deterministic turns for 2-Wheel Cars.  For 4-Wheel cars it is impossible
 *  to get deterministic turns, therefore I use approximated thumb values.
 *
 *  Tested for Adafruit Motor Shield and plain TB6612 breakout board.
 *
 *  Created on: 16.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
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
#include "EncoderMotor.h"

volatile bool EncoderMotor::EncoderTickCounterHasChanged;

EncoderMotor::EncoderMotor() : // @suppress("Class members should be properly initialized")
        PWMDcMotor() {
#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
    /*
     * The list version saves 100 bytes and is more flexible, compared with the array version
     */
    MotorValuesEepromStorageNumber = EncoderMotor::sNumberOfMotorControls;
    EncoderMotor::sNumberOfMotorControls++;
    NextMotorControl = NULL;
    if (sMotorControlListStart == NULL) {
        // first constructor
        sMotorControlListStart = this;
    } else {
        // put object in control list
        EncoderMotor * tObjectPointer = sMotorControlListStart;
        // search last list element
        while (tObjectPointer->NextMotorControl != NULL) {
            tObjectPointer = tObjectPointer->NextMotorControl;
        }
        //insert current control in last element
        tObjectPointer->NextMotorControl = this;
    }
#endif
}

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
void EncoderMotor::init(uint8_t aMotorNumber, bool aReadFromEeprom) {
    PWMDcMotor::init(aMotorNumber, aReadFromEeprom);  // create with the default frequency 1.6KHz
    resetControlValues();
}
#else
void EncoderMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aMotorNumber) {
    PWMDcMotor::init(aForwardPin, aBackwardPin, aPWMPin, aMotorNumber);
    resetControlValues();
}
#endif

/*
 * If motor is already running, adjust TargetDistanceCount to go to aRequestedDistanceCount
 */
void EncoderMotor::startGoDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection) {
//    if (aRequestedDistanceCount > DEFAULT_COUNTS_PER_FULL_ROTATION * 10) {
//        PanicWithLed(400, 22);
//    }
    if (aRequestedDistanceCount == 0) {
        return;
    }
    if (CurrentSpeed == 0) {
        TargetDistanceCount = aRequestedDistanceCount;
#ifdef SUPPORT_RAMP_UP
        MotorRampState = MOTOR_STATE_START;  // This in turn resets EncoderCount etc. at first call of updateMotor()
        CurrentDriveSpeed = aRequestedSpeed;
        setMotorDriverMode(aRequestedDirection); // this in turn sets CurrentDirectionOrBrakeMode
#else
        setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
#endif
    } else {
        TargetDistanceCount = EncoderCount + aRequestedDistanceCount;
        /*
         * prolong NextChangeMaxTargetCount for the new distance
         */
#ifdef SUPPORT_RAMP_UP
        MotorRampState = MOTOR_STATE_DRIVE_SPEED;
        uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
        // guarantee minimal ramp down length
        if (tDistanceCountForRampDown < 3 && TargetDistanceCount > 6) {
            tDistanceCountForRampDown = 3;
        }
        NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;
#endif
        PWMDcMotor::setSpeed(aRequestedSpeed, aRequestedDirection);
    }
    LastTargetDistanceCount = TargetDistanceCount;
    MotorMovesFixedDistance = true;
}

void EncoderMotor::startGoDistanceCount(unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection) {
    startGoDistanceCount(DriveSpeed, aRequestedDistanceCount, aRequestedDirection);
}

/*
 * if aRequestedDistanceCount < 0 then use DIRECTION_BACKWARD
 * initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void EncoderMotor::startGoDistanceCount(int aRequestedDistanceCount) {
    if (aRequestedDistanceCount < 0) {
        aRequestedDistanceCount = -aRequestedDistanceCount;
        startGoDistanceCount(aRequestedDistanceCount, DIRECTION_BACKWARD);
    } else {
        startGoDistanceCount(aRequestedDistanceCount, DIRECTION_FORWARD);
    }
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool EncoderMotor::updateMotor() {
    unsigned long tMillis = millis();
#ifdef SUPPORT_RAMP_UP
    uint8_t tNewSpeed = CurrentSpeed;

    if (MotorRampState == MOTOR_STATE_START) {
        //  --> RAMP_UP
        MotorRampState = MOTOR_STATE_RAMP_UP;
        LastRideEncoderCount = 0;
        EncoderCount = 0;
        /*
         * Start motor
         */
        tNewSpeed = StartSpeed;
        NextRampChangeMillis = tMillis + RAMP_UP_UPDATE_INTERVAL_MILLIS;
        NextChangeMaxTargetCount = TargetDistanceCount / 2;
        // initialize for timeout detection
        EncoderTickLastMillis = tMillis - ENCODER_SENSOR_MASK_MILLIS - 1;

        RampDelta = RAMP_UP_VALUE_DELTA; // 16 steps a 16 millis for ramp up => 256 milliseconds
        if (RampDelta < 2) {
            RampDelta = 2;
        }
        DebugCount = 0;
        Debug = 0;
    }

    // do not use else if since state can be changed in code before
    if (MotorRampState == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed RAMP_UP_UPDATE_INTERVAL_STEPS (16) times every RAMP_UP_UPDATE_INTERVAL_MILLIS (16) milliseconds
         * or until more than half of distance is done
         * Distance required for ramp is 0 to 10 or more, increasing with increasing CurrentDriveSpeed
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_UP_UPDATE_INTERVAL_MILLIS;
            tNewSpeed = tNewSpeed + RampDelta;
            // Clip value and check for 8 bit overflow
            if (tNewSpeed > CurrentDriveSpeed || tNewSpeed <= RampDelta) {
                tNewSpeed = CurrentDriveSpeed;
            }

            /*
             * Transition criteria is:
             * Max Speed reached or more than half of distance is done
             */
            if (tNewSpeed == CurrentDriveSpeed || (MotorMovesFixedDistance && EncoderCount >= NextChangeMaxTargetCount)) {
                //  --> DRIVE_SPEED
                MotorRampState = MOTOR_STATE_DRIVE_SPEED;

                DistanceCountAfterRampUp = EncoderCount;
                uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
                // guarantee minimal ramp down length
                if (tDistanceCountForRampDown < 3 && TargetDistanceCount > 6) {
                    tDistanceCountForRampDown = 3;
                }
                NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;
            }
        }
    }

    // do not use else if since state can be changed in code before
    if (MotorRampState == MOTOR_STATE_DRIVE_SPEED) {
        /*
         * Wait until ramp down count is reached
         */
        if (MotorMovesFixedDistance && EncoderCount >= NextChangeMaxTargetCount) {
            //  --> RAMP_DOWN
            MotorRampState = MOTOR_STATE_RAMP_DOWN;
            /*
             * Ramp to reach StartSpeed after 1/2 of remaining distance
             * TODO reduce speed just here? e.g. to 1.5 of start speed?
             */
            RampDeltaPerDistanceCount = ((CurrentSpeed - StartSpeed) * 2) / ((TargetDistanceCount - EncoderCount)) + 1;
        }
    }

    // do not use else if since state can be changed in code before
    if (MotorRampState == MOTOR_STATE_RAMP_DOWN) {
        DebugCount++;

        /*
         * Decrease motor speed depending on distance to target count
         */
        if (EncoderCount >= NextChangeMaxTargetCount) {
            Debug++;
            NextChangeMaxTargetCount++;
            if (tNewSpeed > RampDeltaPerDistanceCount) {
                tNewSpeed -= RampDeltaPerDistanceCount;
            } else {
                tNewSpeed = StartSpeed;
            }
            // safety net for slow speed
            if (tNewSpeed < StartSpeed) {
                tNewSpeed = StartSpeed;
            }
            // not really needed here since output is disabled during ramps
            MotorValuesHaveChanged = true;
        }
    }
    // End of motor state machine

    if (tNewSpeed != CurrentSpeed) {
        PWMDcMotor::setSpeed(tNewSpeed, CurrentDirectionOrBrakeMode);
    }
#endif

    /*
     * Check if target count is reached or encoder tick timeout
     */
    if (CurrentSpeed > 0) {
        if (MotorMovesFixedDistance
                && (EncoderCount >= TargetDistanceCount || tMillis > (EncoderTickLastMillis + ENCODER_TICKS_TIMEOUT_MILLIS))) {
#ifdef SUPPORT_RAMP_UP
            DebugSpeedAtTargetCountReached = CurrentSpeed;
            MotorRampState = MOTOR_STATE_STOPPED;
#endif
            stop(MOTOR_BRAKE);
            return false;
        }
    }

    return true;
}

/*
 * Computes motor speed compensation value in order to go exactly straight ahead
 * Compensate only at forward direction
 */
void EncoderMotor::synchronizeMotor(EncoderMotor * aOtherMotorControl, unsigned int aCheckInterval) {
    if (CurrentDirectionOrBrakeMode != DIRECTION_FORWARD) {
        return;
    }
    static long sNextMotorSyncMillis;
    long tMillis = millis();
    if (tMillis >= sNextMotorSyncMillis) {
        sNextMotorSyncMillis += aCheckInterval;
#ifdef SUPPORT_RAMP_UP
// only synchronize if manually operated or at full speed
        if ((MotorRampState == MOTOR_STATE_STOPPED && aOtherMotorControl->MotorRampState == MOTOR_STATE_STOPPED && CurrentSpeed > 0)
                || (MotorRampState == MOTOR_STATE_DRIVE_SPEED && aOtherMotorControl->MotorRampState == MOTOR_STATE_DRIVE_SPEED)) {
#endif
            MotorValuesHaveChanged = false;
            if (EncoderCount >= (aOtherMotorControl->EncoderCount + 2)) {
                EncoderCount = aOtherMotorControl->EncoderCount;
                /*
                 * This motor is too fast, first try to reduce other motors compensation
                 */
                if (aOtherMotorControl->SpeedCompensation >= 2) {
                    aOtherMotorControl->SpeedCompensation -= 2;
                    aOtherMotorControl->CurrentSpeed += 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->CurrentSpeed, DIRECTION_FORWARD);
                    MotorValuesHaveChanged = true;
                    EncoderCount = aOtherMotorControl->EncoderCount;
                } else if (CurrentSpeed > StartSpeed) {
                    /*
                     * else increase this motors compensation
                     */
                    SpeedCompensation += 2;
                    CurrentSpeed -= 2;
                    PWMDcMotor::setSpeed(CurrentSpeed, DIRECTION_FORWARD);
                    MotorValuesHaveChanged = true;
                }

            } else if (aOtherMotorControl->EncoderCount >= (EncoderCount + 2)) {
                aOtherMotorControl->EncoderCount = EncoderCount;
                /*
                 * Other motor is too fast, first try to reduce this motors compensation
                 */
                if (SpeedCompensation >= 2) {
                    SpeedCompensation -= 2;
                    CurrentSpeed += 2;
                    PWMDcMotor::setSpeed(CurrentSpeed, DIRECTION_FORWARD);
                    MotorValuesHaveChanged = true;
                } else if (aOtherMotorControl->CurrentSpeed > aOtherMotorControl->StartSpeed) {
                    /*
                     * else increase other motors compensation
                     */
                    aOtherMotorControl->SpeedCompensation += 2;
                    aOtherMotorControl->CurrentSpeed -= 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->CurrentSpeed, DIRECTION_FORWARD);
                    MotorValuesHaveChanged = true;
                }
            }

            if (MotorValuesHaveChanged) {
                writeMotorvaluesToEeprom();
            }
#ifdef SUPPORT_RAMP_UP
        }
#endif
    }
}

/*************************
 * Direct motor control
 *************************/
/*
 * Reset all control values as distances, debug values to 0x00
 * Leave calibration and compensation values unaffected.
 */
void EncoderMotor::resetControlValues() {
    memset(reinterpret_cast<uint8_t*>(&CurrentVelocity), 0,
            (((uint8_t *) &Debug) + sizeof(Debug)) - reinterpret_cast<uint8_t*>(&CurrentVelocity));
// to force display of initial values
    EncoderTickCounterHasChanged = true;
}

/***************************************************
 * Encoder functions
 ***************************************************/
void EncoderMotor::handleEncoderInterrupt() {
    long tMillis = millis();
    unsigned int tDeltaMillis = tMillis - EncoderTickLastMillis;
    if (tDeltaMillis <= ENCODER_SENSOR_MASK_MILLIS) {
// signal is ringing
        CurrentVelocity = 99;
    } else {
        EncoderTickLastMillis = tMillis;
        EncoderCount++;
        LastRideEncoderCount++;
        CurrentVelocity = VELOCITY_SCALE_VALUE / tDeltaMillis;
        EncoderTickCounterHasChanged = true;
    }
}

// The code for the interrupt is placed at the calling class since we need a fixed relation between ISR and EncoderMotor
// //ISR for PIN PD2 / RIGHT
// ISR(INT0_vect) {
//    myCar.rightMotorControl.handleEncoderInterrupt();
// }

/******************************************************************************************
 * Static methods
 *****************************************************************************************/
/*
 * Enable both interrupts INT0/D2 or INT1/D3
 */
void EncoderMotor::enableINT0AndINT1Interrupts() {

// interrupt on any logical change
    EICRA |= (_BV(ISC00) | _BV(ISC10));
// clear interrupt bit
    EIFR |= (_BV(INTF0) | _BV(INTF1));
// enable interrupt on next change
    EIMSK |= (_BV(INT0) | _BV(INT1));
}

/*
 * Enable only one interrupt
 * aIntPinNumber can be one of INT0/D2 or INT1/D3 for Atmega328
 */
void EncoderMotor::enableInterruptOnBothEdges(uint8_t aIntPinNumber) {
    if (aIntPinNumber > 1) {
        return;
    }

    if (aIntPinNumber == 0) {
// interrupt on any logical change
        EICRA |= (_BV(ISC00));
// clear interrupt bit
        EIFR |= _BV(INTF0);
// enable interrupt on next change
        EIMSK |= _BV(INT0);
    } else {
        EICRA |= (_BV(ISC10));
        EIFR |= _BV(INTF1);
        EIMSK |= _BV(INT1);
    }
}

#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
/*
 * The list version saves 100 bytes and is more flexible, compared with the array version
 */
uint8_t EncoderMotor::sNumberOfMotorControls = 0;
EncoderMotor * EncoderMotor::sMotorControlListStart = NULL;

/*****************************************************
 * Static convenience functions affecting all motors.
 * If you have 2 motors, better use CarControl
 *****************************************************/

bool EncoderMotor::updateAllMotors() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    bool tMotorsNotStopped = false; // to check if motors are not stopped by aLoopCallback
    while (tEncoderMotorControlPointer != NULL) {
        tMotorsNotStopped |= tEncoderMotorControlPointer->updateMotor();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tMotorsNotStopped;
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::startRampUpAndWaitForDriveSpeedForAll(uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startRampUp(aRequestedDirection);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    bool tMotorsNotStopped; // to check if motors are not stopped by aLoopCallback
    do {
        tMotorsNotStopped = EncoderMotor::updateAllMotors();
        if (aLoopCallback != NULL) {
            aLoopCallback(); // this may stop motors
        }
    } while (tMotorsNotStopped && !EncoderMotor::allMotorsStarted());
}

void EncoderMotor::startGoDistanceCountForAll(int aRequestedDistanceCount) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startGoDistanceCount(aRequestedDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::goDistanceCountForAll(int aRequestedDistanceCount, void (*aLoopCallback)(void)) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startGoDistanceCount(aRequestedDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    waitUntilAllMotorsStopped(aLoopCallback);
}

bool EncoderMotor::allMotorsStarted() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStarted = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->MotorRampState != MOTOR_STATE_DRIVE_SPEED) {
            tAllAreStarted = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStarted;
}

bool EncoderMotor::allMotorsStopped() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStopped = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->MotorRampState != MOTOR_STATE_STOPPED) {
            tAllAreStopped = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStopped;
}

/*
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_DRIVE_SPEED to MOTOR_STATE_RAMP_DOWN
 * Use DistanceCountAfterRampUp as ramp down count
 * Busy waits for stop
 */
void EncoderMotor::stopAllMotorsAndWaitUntilStopped() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->NextChangeMaxTargetCount = tEncoderMotorControlPointer->EncoderCount;
        tEncoderMotorControlPointer->TargetDistanceCount = tEncoderMotorControlPointer->EncoderCount
                + tEncoderMotorControlPointer->DistanceCountAfterRampUp;
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }

    /*
     * busy wait for stop
     */
    while (!allMotorsStopped()) {
        updateAllMotors();
    }
}

void EncoderMotor::waitUntilAllMotorsStopped(void (*aLoopCallback)(void)) {
    do {
        EncoderMotor::updateAllMotors();
        if (aLoopCallback != NULL) {
            aLoopCallback();
        }
    } while (!EncoderMotor::allMotorsStopped());
}

void EncoderMotor::stopAllMotors(uint8_t aStopMode) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->stop(aStopMode);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}
#endif
