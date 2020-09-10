/*
 * EncoderMotor.cpp
 *
 *  Functions for controlling a DC-motor which rotary encoder implemented by fork light barrier and an attached encoder disc (with 20 slots).
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
#include "EncoderMotor.h"

volatile bool EncoderMotor::EncoderTickCounterHasChanged;

EncoderMotor::EncoderMotor() : // @suppress("Class members should be properly initialized")
        PWMDcMotor() {
#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
    /*
     * The list version saves 100 bytes and is more flexible, compared with the array version
     */
    EncoderMotorNumber = EncoderMotor::sNumberOfMotorControls;
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
 * If motor is still running, add aDistanceCount to current target distance
 */
void EncoderMotor::initGoDistanceCount(uint16_t aDistanceCount, uint8_t aRequestedDirection) {
    aRequestedDirection &= DIRECTION_MASK;
    if (CurrentDirection != aRequestedDirection) {
        CurrentDirection = aRequestedDirection;
        if (State != MOTOR_STATE_STOPPED) {
#ifdef DEBUG
            Serial.print(F("Direction change to"));
            Serial.println(tRequestedDirection);
#endif
            /*
             * Direction change requested but motor still running-> first stop motor
             */
            stop(MOTOR_BRAKE);
            LastTargetDistanceCount = EncoderCount; // Reset LastTargetDistanceCount on direction change
        }
    }

    if (State == MOTOR_STATE_STOPPED) {
        CurrentDriveSpeed = DriveSpeed - SpeedCompensation;
        /*
         * Start the motor and compensate for last distance delta
         * Compensation is only valid if direction does not change.
         */
        // Positive if driven too far, negative if driven too short
        int8_t tLastDelta = (int) EncoderCount - (int) LastTargetDistanceCount;
        if (abs(tLastDelta) <= MAX_DISTANCE_DELTA && (int) aDistanceCount >= tLastDelta) {
            TargetDistanceCount = (int) aDistanceCount - tLastDelta;
        } else {
            TargetDistanceCount = aDistanceCount;
        }
        EncoderCount = 0;
    } else {
        /*
         * Increase the distance to go for running motor
         */
        TargetDistanceCount += aDistanceCount;
        NextChangeMaxTargetCount += aDistanceCount;
    }
    LastTargetDistanceCount = TargetDistanceCount;
}

/*
 * if aDistanceCount < 0 then use DIRECTION_BACKWARD
 * initialize motorInfo fields DirectionForward, CurrentDriveSpeed, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void EncoderMotor::initGoDistanceCount(int16_t aDistanceCount) {
    uint8_t tRequestedDirection = DIRECTION_FORWARD;

    if (aDistanceCount < 0) {
        aDistanceCount = -aDistanceCount;
        tRequestedDirection = DIRECTION_BACKWARD;
    }
    initGoDistanceCount(aDistanceCount, tRequestedDirection);
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool EncoderMotor::updateMotor() {
    unsigned long tMillis = millis();

    if (State == MOTOR_STATE_STOPPED) {
        if (TargetDistanceCount > 0) {
            //  --> RAMP_UP
            State = MOTOR_STATE_RAMP_UP;
            LastRideEncoderCount = 0;
            /*
             * Start motor
             */
            NextRampChangeMillis = tMillis + RAMP_UP_UPDATE_INTERVAL_MILLIS;
            CurrentSpeed = StartSpeed;
            // not really needed here since output is disabled during ramps
            MotorValuesHaveChanged = true;

            NextChangeMaxTargetCount = TargetDistanceCount / 2;
            // initialize for timeout detection
            EncoderTickLastMillis = tMillis - ENCODER_SENSOR_MASK_MILLIS - 1;

            RampDelta = RAMP_UP_VALUE_DELTA;
            if (RampDelta < 2) {
                RampDelta = 2;
            }
            DebugCount = 0;
            Debug = 0;

            PWMDcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }

    } else if (State == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_UP_UPDATE_INTERVAL_MILLIS;
            CurrentSpeed += RampDelta;
            // Clip value and check for 8 bit overflow
            if (CurrentSpeed > CurrentDriveSpeed || CurrentSpeed <= RampDelta) {
                CurrentSpeed = CurrentDriveSpeed;
            }
            MotorValuesHaveChanged = true;

            /*
             * Transition criteria is:
             * Max Speed reached or more than half of distance is done
             */
            if (CurrentSpeed == CurrentDriveSpeed || EncoderCount >= NextChangeMaxTargetCount) {
                //  --> FULL_SPEED
                State = MOTOR_STATE_FULL_SPEED;

                DistanceCountAfterRampUp = EncoderCount;
                uint8_t tDistanceCountForRampDown = DistanceCountAfterRampUp;
                // guarantee minimal ramp down length
                if (tDistanceCountForRampDown < 3 && TargetDistanceCount > 6) {
                    tDistanceCountForRampDown = 3;
                }
                NextChangeMaxTargetCount = TargetDistanceCount - tDistanceCountForRampDown;
            }
            PWMDcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }
    }

    // do not use else if since state can be changed in code before
    if (State == MOTOR_STATE_FULL_SPEED) {
        /*
         * Wait until ramp down count is reached
         */
        if (EncoderCount >= NextChangeMaxTargetCount) {
            NextChangeMaxTargetCount++;
            //  --> RAMP_DOWN
            State = MOTOR_STATE_RAMP_DOWN;
            /*
             * Ramp to reach StartSpeed after 1/2 of remaining distance
             */
            RampDeltaPerDistanceCount = ((CurrentSpeed - StartSpeed) * 2) / ((TargetDistanceCount - EncoderCount)) + 1;
            // brake
            if (CurrentSpeed > RampDeltaPerDistanceCount) {
                CurrentSpeed -= RampDeltaPerDistanceCount;
            } else {
                CurrentSpeed = StartSpeed;
            }
            MotorValuesHaveChanged = true;
            PWMDcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }

    } else if (State == MOTOR_STATE_RAMP_DOWN) {
        DebugCount++;

        /*
         * Decrease motor speed depending on distance to target count
         */
        if (EncoderCount >= NextChangeMaxTargetCount) {
            Debug++;
            NextChangeMaxTargetCount++;
            if (CurrentSpeed > RampDeltaPerDistanceCount) {
                CurrentSpeed -= RampDeltaPerDistanceCount;
            } else {
                CurrentSpeed = StartSpeed;
            }
            // safety net for slow speed
            if (CurrentSpeed < StartSpeed) {
                CurrentSpeed = StartSpeed;
            }
            // not really needed here since output is disabled during ramps
            MotorValuesHaveChanged = true;
        }
        /*
         * Check if target count is reached
         */
        if (EncoderCount >= TargetDistanceCount) {
            SpeedAtTargetCountReached = CurrentSpeed;
            stop(MOTOR_BRAKE);
            return false;
        } else {
            PWMDcMotor::setSpeed(CurrentSpeed, CurrentDirection);
        }
    }

    /*
     * Check for encoder tick timeout
     */
    if (State != MOTOR_STATE_STOPPED && tMillis > (EncoderTickLastMillis + RAMP_DOWN_TIMEOUT_MILLIS)) {
// No encoder tick in the last 500 ms -> stop motor
        SpeedAtTargetCountReached = CurrentSpeed;
        stop(MOTOR_BRAKE);
#ifdef DEBUG
        Serial.println(F("Encoder timeout -> stop motor"));
#endif
        return false;
    }
    return true;
}

/*
 * Computes motor speed compensation value in order to go exactly straight ahead
 * Compensate only at forward direction
 */
void EncoderMotor::synchronizeMotor(EncoderMotor * aOtherMotorControl, uint16_t aCheckInterval) {
    if (CurrentDirection == DIRECTION_BACKWARD) {
        return;
    }
    static long sNextMotorSyncMillis;
    long tMillis = millis();
    if (tMillis >= sNextMotorSyncMillis) {
        sNextMotorSyncMillis += aCheckInterval;
// only synchronize if manually operated or at full speed
        if ((State == MOTOR_STATE_STOPPED && aOtherMotorControl->State == MOTOR_STATE_STOPPED && CurrentSpeed > 0)
                || (State == MOTOR_STATE_FULL_SPEED && aOtherMotorControl->State == MOTOR_STATE_FULL_SPEED)) {

            MotorValuesHaveChanged = false;
            if (EncoderCount >= (aOtherMotorControl->EncoderCount + 2)) {
                EncoderCount = aOtherMotorControl->EncoderCount;
                /*
                 * This motor is too fast, first try to reduce other motors compensation
                 */
                if (aOtherMotorControl->SpeedCompensation >= 2) {
                    aOtherMotorControl->SpeedCompensation -= 2;
                    aOtherMotorControl->CurrentSpeed += 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                    EncoderCount = aOtherMotorControl->EncoderCount;
                } else if (CurrentSpeed > StartSpeed) {
                    /*
                     * else increase this motors compensation
                     */
                    SpeedCompensation += 2;
                    CurrentSpeed -= 2;
                    PWMDcMotor::setSpeed(CurrentSpeed, CurrentDirection);
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
                    PWMDcMotor::setSpeed(CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                } else if (aOtherMotorControl->CurrentSpeed > aOtherMotorControl->StartSpeed) {
                    /*
                     * else increase other motors compensation
                     */
                    aOtherMotorControl->SpeedCompensation += 2;
                    aOtherMotorControl->CurrentSpeed -= 2;
                    aOtherMotorControl->setSpeed(aOtherMotorControl->CurrentSpeed, CurrentDirection);
                    MotorValuesHaveChanged = true;
                }
            }

            if (MotorValuesHaveChanged && State == MOTOR_STATE_FULL_SPEED) {
                writeMotorvaluesToEeprom();
            }
        }
    }
}

/*************************
 * Direct motor control
 *************************/
void EncoderMotor::setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    if (aRequestedSpeed == 0) {
        stop();
    } else {
        PWMDcMotor::setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
    }
}

void EncoderMotor::setSpeedCompensated(int aRequestedSpeed) {
    uint8_t tDirection;
    if (aRequestedSpeed > 0) {
        tDirection = DIRECTION_FORWARD;
    } else {
        tDirection = DIRECTION_BACKWARD;
        aRequestedSpeed = -aRequestedSpeed;
    }
    setSpeedCompensated(aRequestedSpeed, tDirection);
}

/*
 * The one and only place where State is set to MOTOR_STATE_STOPPED
 * @param aStopMode STOP_MODE_KEEP (take previously defined StopMode) or MOTOR_BRAKE or MOTOR_RELEASE
 */
void EncoderMotor::stop(uint8_t aStopMode) {
    /*
     * Set state to MOTOR_STATE_STOPPED
     */
    State = MOTOR_STATE_STOPPED;
    TargetDistanceCount = 0;

    PWMDcMotor::stop(aStopMode);
}

/*
 * Reset all control values as distances, debug values to 0x00
 * Leave calibration and compensation values unaffected.
 */
void EncoderMotor::resetControlValues() {
    memset(&CurrentDriveSpeed, 0, (((uint8_t *) &Debug) + sizeof(Debug)) - &CurrentDriveSpeed);
// to force display of initial values
    EncoderTickCounterHasChanged = true;
}

/***************************************************
 * Encoder functions
 ***************************************************/
void EncoderMotor::handleEncoderInterrupt() {
    long tMillis = millis();
    uint16_t tDeltaMillis = tMillis - EncoderTickLastMillis;
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
    EICRA |= (1 << ISC00 | 1 << ISC10);
// clear interrupt bit
    EIFR |= (1 << INTF0 | 1 << INTF1);
// enable interrupt on next change
    EIMSK |= (1 << INT0 | 1 << INT1);
}

/*
 * aIntPinNumber can be one of INT0/D2 or INT1/D3 for Atmega328
 */
void EncoderMotor::enableInterruptOnBothEdges(uint8_t aIntPinNumber) {
    if (aIntPinNumber > 1) {
        return;
    }

    if (aIntPinNumber == 0) {
// interrupt on any logical change
        EICRA |= (1 << ISC00);
// clear interrupt bit
        EIFR |= 1 << INTF0;
// enable interrupt on next change
        EIMSK |= 1 << INT0;
    } else {
        EICRA |= (1 << ISC10);
        EIFR |= 1 << INTF1;
        EIMSK |= 1 << INT1;
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

void EncoderMotor::updateAllMotors() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->updateMotor();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::startAndWaitForFullSpeedForAll() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->initGoDistanceCount(INFINITE_DISTANCE_COUNT);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    do {
        EncoderMotor::updateAllMotors();
    } while (!EncoderMotor::allMotorsStarted());
}

void EncoderMotor::initGoDistanceCountForAll(int aDistanceCount) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->initGoDistanceCount(aDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::goDistanceCountForAll(int aDistanceCount, void (*aLoopCallback)(void)) {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->initGoDistanceCount(aDistanceCount);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    waitUntilAllMotorsStopped(aLoopCallback);
}

bool EncoderMotor::allMotorsStarted() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStarted = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->State != MOTOR_STATE_FULL_SPEED) {
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
        if (tEncoderMotorControlPointer->State != MOTOR_STATE_STOPPED) {
            tAllAreStopped = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStopped;
}

/*
 * Set NextChangeMaxTargetCount to change state from MOTOR_STATE_FULL_SPEED to MOTOR_STATE_RAMP_DOWN
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

void EncoderMotor::stopAllMotorsAndReset() {
    EncoderMotor * tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->stopMotorAndReset();
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
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
