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
#include "EncoderMotor.h"

volatile bool EncoderMotor::EncoderCountHasChanged;

EncoderMotor * sPointerForInt0ISR;
EncoderMotor * sPointerForInt1ISR;

EncoderMotor::EncoderMotor() : // @suppress("Class members should be properly initialized")
        PWMDcMotor() {
#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
    /*
     * The list version saves 100 bytes and is more flexible, compared with the array version
     */
    EncoderMotor::sNumberOfMotorControls++;
    NextMotorControl = NULL;
    if (sMotorControlListStart == NULL) {
        // first constructor
        sMotorControlListStart = this;
    } else {
        // put object in control list
        EncoderMotor *tObjectPointer = sMotorControlListStart;
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
/*
 * aMotorNumber from 1 to 2
 * If no parameter, we use a fixed assignment of rightCarMotor interrupts to INT0 / Pin2 and leftCarMotor to INT1 / Pin3
 * Currently motors 3 and 4 are not required/supported by own library for Adafruit Motor Shield
 */
void EncoderMotor::init(uint8_t aMotorNumber) {
    PWMDcMotor::init(aMotorNumber);  // create with the default frequency 1.6KHz
    resetEncoderControlValues();
}
void EncoderMotor::init(uint8_t aMotorNumber, uint8_t aInterruptNumber) {
    PWMDcMotor::init(aMotorNumber);  // create with the default frequency 1.6KHz
    resetEncoderControlValues();
    attachInterrupt(aInterruptNumber);
}
#else
void EncoderMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    PWMDcMotor::init(aForwardPin, aBackwardPin, aPWMPin);
    resetEncoderControlValues();
}

void EncoderMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aInterruptNumber) {
    PWMDcMotor::init(aForwardPin, aBackwardPin, aPWMPin);
    resetEncoderControlValues();
    attachInterrupt(aInterruptNumber);
}
#endif

/*
 * If motor is already running, adjust TargetDistanceCount to go to aRequestedDistanceCount
 */
void EncoderMotor::startGoDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount,
        uint8_t aRequestedDirection) {
    if (aRequestedDistanceCount == 0) {
        stop(DefaultStopMode); // In case motor was running
        return;
    }
    if (CurrentSpeed == 0) {
#ifdef SUPPORT_RAMP_UP
        startRampUp(aRequestedSpeed, aRequestedDirection);
#else
        setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
#endif
        TargetDistanceCount = aRequestedDistanceCount;
    } else {
        TargetDistanceCount = EncoderCount + aRequestedDistanceCount;
#ifdef SUPPORT_RAMP_UP
        if (MotorRampState == MOTOR_STATE_DRIVE_SPEED) {
            /*
             * prolong NextChangeMaxTargetCount for the new distance
             */
            MotorRampState = MOTOR_STATE_DRIVE_SPEED;
            uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
            // guarantee minimal ramp down length
            if (tDistanceCountForRampDown < 2 && TargetDistanceCount > 3) {
                tDistanceCountForRampDown = 2;
            }
            NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;
        }
        // else ramp is in mode MOTOR_STATE_RAMP_UP -> do nothing, let the ramp go on
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
        /*
         * Set ramp values
         */
        NextRampChangeMillis = tMillis + RAMP_UP_INTERVAL_MILLIS;
        /*
         * Start motor
         */
        tNewSpeed = StartSpeed; // start immediately with StartSpeed

        /*
         * Init Encoder values
         */
        LastRideEncoderCount = 0;
        EncoderCount = 0;
        DebugCount = 0;
        Debug = 0;
        NextChangeMaxTargetCount = TargetDistanceCount / 2;
        // initialize for timeout detection
        LastEncoderInterruptMillis = tMillis - ENCODER_SENSOR_RING_MILLIS - 1;
    }

    // do not use else if since state can be changed in code before
    if (MotorRampState == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed RAMP_UP_UPDATE_INTERVAL_STEPS times every RAMP_UP_INTERVAL_MILLIS milliseconds
         * or until more than half of distance is done
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_UP_INTERVAL_MILLIS;
            /*
             * Transition criteria is:
             * Drive Speed reached or more than half of distance is done
             */
            if (tNewSpeed == CurrentDriveSpeed || (MotorMovesFixedDistance && EncoderCount >= NextChangeMaxTargetCount)) {
                //  CurrentDriveSpeed reached switch to --> DRIVE_SPEED
                MotorRampState = MOTOR_STATE_DRIVE_SPEED;

                DistanceCountAfterRampUp = EncoderCount;
                uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
                // guarantee minimal ramp down length
                if (tDistanceCountForRampDown < 3 && TargetDistanceCount > 6) {
                    tDistanceCountForRampDown = 3;
                }
                NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;

            } else {
                tNewSpeed = tNewSpeed + RampDelta;
                // Clip value and check for 8 bit overflow
                if (tNewSpeed > CurrentDriveSpeed || tNewSpeed <= RampDelta) {
                    tNewSpeed = CurrentDriveSpeed;
                }
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
        }
    }
    // End of motor state machine

#  ifdef TRACE
        Serial.print(F("St="));
        Serial.println(MotorRampState);
#  endif
    if (tNewSpeed != CurrentSpeed) {
#  ifdef TRACE
        Serial.print(F("Ns="));
        Serial.println(tNewSpeed);
#  endif
        PWMDcMotor::setSpeed(tNewSpeed, CurrentDirectionOrBrakeMode);
    }
#endif // SUPPORT_RAMP_UP

    /*
     * Check if target count is reached or encoder tick timeout
     */
    if (CurrentSpeed > 0) {
        if (MotorMovesFixedDistance
                && (EncoderCount >= TargetDistanceCount || tMillis > (LastEncoderInterruptMillis + ENCODER_SENSOR_TIMEOUT_MILLIS))) {
#ifdef SUPPORT_RAMP_UP
            DebugSpeedAtTargetCountReached = CurrentSpeed;
#endif
            stop(MOTOR_BRAKE); // this sets MOTOR_STATE_STOPPED;
#ifdef DEBUG
            Serial.println(F("Reached"));
#endif
            return false; // need no more calls to update()
        }
        return true; // still running
    }
    return false; // current speed == 0
}

/*
 * Computes motor speed compensation value in order to go exactly straight ahead
 * Compensate only at forward direction
 */
void EncoderMotor::synchronizeMotor(EncoderMotor *aOtherMotorControl, unsigned int aCheckInterval) {
    if (CurrentDirectionOrBrakeMode != DIRECTION_FORWARD || aOtherMotorControl->CurrentDirectionOrBrakeMode != DIRECTION_FORWARD) {
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
#ifdef SUPPORT_RAMP_UP
        }
#endif
    }
}

/*************************
 * Direct motor control
 *************************/
/*
 * Reset all control values as distances, debug values etc. to 0x00
 */
void EncoderMotor::resetEncoderControlValues() {
    memset(reinterpret_cast<uint8_t*>(&TargetDistanceCount), 0,
            (((uint8_t *) &Debug) + sizeof(Debug)) - reinterpret_cast<uint8_t*>(&TargetDistanceCount));
// to force display of initial values
    EncoderCountHasChanged = true;
}

/***************************************************
 * Encoder functions
 ***************************************************/
/*
 * Attaches INT0 or INT1 interrupt to this EncoderMotor
 * Interrupt is enabled on rising edges
 * We can not use both edges since the on and off times of the opto interrupter are too different
 * aInterruptNumber can be one of INT0 (at pin D2) or INT1 (at pin D3) for Atmega328
 */
void EncoderMotor::attachInterrupt(uint8_t aInterruptNumber) {
    if (aInterruptNumber > 1) {
        return;
    }

    if (aInterruptNumber == 0) {
        sPointerForInt0ISR = this;
// interrupt on any logical change
        EICRA |= (_BV(ISC00) | _BV(ISC01));
// clear interrupt bit
        EIFR |= _BV(INTF0);
// enable interrupt on next change
        EIMSK |= _BV(INT0);
    } else {
        sPointerForInt1ISR = this;
        EICRA |= (_BV(ISC10) | _BV(ISC11));
        EIFR |= _BV(INTF1);
        EIMSK |= _BV(INT1);
    }
}

void EncoderMotor::resetSpeedValues() {
#ifdef SUPPORT_AVERAGE_SPEED
    memset((void *) &EncoderInterruptDeltaMillis, 0,
            ((uint8_t *) &AverageSpeedIsValid + sizeof(AverageSpeedIsValid)) - (uint8_t *) &EncoderInterruptDeltaMillis);
#else
    EncoderInterruptDeltaMillis = 0;
#endif
}

/*
 * Speed is in rpm for a 20 slot encoder disc
 */
int EncoderMotor::getSpeed() {
    /*
     * First check for timeout
     */
    if (millis() - LastEncoderInterruptMillis > ENCODER_SENSOR_TIMEOUT_MILLIS) {
        resetSpeedValues();
    }
    unsigned long tEncoderInterruptDeltaMillis = EncoderInterruptDeltaMillis;
    if (tEncoderInterruptDeltaMillis == 0) {
        return 0;
    }
    if (CurrentDirectionOrBrakeMode == DIRECTION_BACKWARD) {
        return (-(SPEED_SCALE_VALUE / tEncoderInterruptDeltaMillis));
    } else {
        return (SPEED_SCALE_VALUE / tEncoderInterruptDeltaMillis);
    }
}

#ifdef SUPPORT_AVERAGE_SPEED
/*
 * Speed is in rpm for a 20 slot encoder disc
 * Average is computed over the full revolution to compensate for unequal distances of the laser cut encoder discs.
 */
int EncoderMotor::getAverageSpeed() {
    int tAverageSpeed = 0;
    /*
     * First check for timeout
     */
    if (millis() - LastEncoderInterruptMillis > ENCODER_SENSOR_TIMEOUT_MILLIS) {
        resetSpeedValues();
    } else {
        int8_t tMillisArrayIndex = MillisArrayIndex;
        if (!AverageSpeedIsValid) {
            tMillisArrayIndex--;
            if (tMillisArrayIndex > 0) {
                // here MillisArray is not completely filled and MillisArrayIndex had no wrap around
                tAverageSpeed = (SPEED_SCALE_VALUE * tMillisArrayIndex)
                        / (EncoderInterruptMillisArray[tMillisArrayIndex] - EncoderInterruptMillisArray[0]);
            }
        } else {
            // tMillisArrayIndex points to the next value to write == the oldest value to overwrite
            unsigned long tOldestEncoderInterruptMillis = EncoderInterruptMillisArray[tMillisArrayIndex];

            // get index of current value
            tMillisArrayIndex--;
            if (tMillisArrayIndex < 0) {
                // wrap around
                tMillisArrayIndex = AVERAGE_SPEED_BUFFER_SIZE - 1;
            }
            tAverageSpeed = (SPEED_SCALE_VALUE * AVERAGE_SPEED_SAMPLE_SIZE)
                    / (EncoderInterruptMillisArray[tMillisArrayIndex] - tOldestEncoderInterruptMillis);
        }
    }
    if (CurrentDirectionOrBrakeMode == DIRECTION_BACKWARD) {
        return -tAverageSpeed;
    } else {
        return tAverageSpeed;
    }
}
#endif

void EncoderMotor::handleEncoderInterrupt() {
    long tMillis = millis();
    unsigned long tDeltaMillis = tMillis - LastEncoderInterruptMillis;
    if (tDeltaMillis <= ENCODER_SENSOR_RING_MILLIS) {
        // assume signal is ringing and do nothing
    } else {
        LastEncoderInterruptMillis = tMillis;
#ifdef SUPPORT_AVERAGE_SPEED
        uint8_t tMillisArrayIndex = MillisArrayIndex;
#endif
        if (tDeltaMillis < ENCODER_SENSOR_TIMEOUT_MILLIS) {
            EncoderInterruptDeltaMillis = tDeltaMillis;
        } else {
            // timeout
            EncoderInterruptDeltaMillis = 0;
#ifdef SUPPORT_AVERAGE_SPEED
            tMillisArrayIndex = 0;
            AverageSpeedIsValid = false;
#endif
        }
#ifdef SUPPORT_AVERAGE_SPEED
        EncoderInterruptMillisArray[tMillisArrayIndex++] = tMillis;
        if (tMillisArrayIndex >= AVERAGE_SPEED_BUFFER_SIZE) {
            tMillisArrayIndex = 0;
            AverageSpeedIsValid = true;
        }
        MillisArrayIndex = tMillisArrayIndex;
#endif

        EncoderCount++;
        LastRideEncoderCount++;
        EncoderCountHasChanged = true;
    }
}

// ISR for PIN PD2 / RIGHT
ISR(INT0_vect) {
    sPointerForInt0ISR->handleEncoderInterrupt();
}

// ISR for PIN PD3 / LEFT
ISR(INT1_vect) {
    sPointerForInt1ISR->handleEncoderInterrupt();
}

/******************************************************************************************
 * Static methods
 *****************************************************************************************/
/*
 * Enable both interrupts INT0/D2 or INT1/D3
 */
void EncoderMotor::enableINT0AndINT1InterruptsOnRisingEdge() {

// interrupt on any logical change
    EICRA |= (_BV(ISC00) | _BV(ISC01) | _BV(ISC10) | _BV(ISC11));
// clear interrupt bit
    EIFR |= (_BV(INTF0) | _BV(INTF1));
// enable interrupt on next change
    EIMSK |= (_BV(INT0) | _BV(INT1));
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startGoDistanceCount(aRequestedDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    waitUntilAllMotorsStopped(aLoopCallback);
}

bool EncoderMotor::allMotorsStarted() {
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
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
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->stop(aStopMode);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}
#endif
