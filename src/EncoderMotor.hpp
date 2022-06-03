/*
 * EncoderMotor.hpp
 *
 *  Functions for controlling a DC-motor which rotary encoder implemented by slot-type photo interrupter and an attached encoder disc (with 20 slots).
 *  Works with positive (unsigned) speed and direction or signed speed.
 *
 *  Contains functions to go a specified distance.
 *  These functions generates ramps for acceleration and deceleration if required and tries to stop at target distance.
 *  This enables deterministic turns for 2-Wheel Cars.  For 4-Wheel cars it is impossible
 *  to get deterministic turns, therefore I use approximated thumb values.
 *
 *  Tested for Adafruit Motor Shield and plain TB6612 breakout board.
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
#ifndef _ENCODER_MOTOR_CONTROL_HPP
#define _ENCODER_MOTOR_CONTROL_HPP

#if defined(USE_ENCODER_MOTOR_CONTROL)
#include "EncoderMotor.h"
#include "PWMDcMotor.hpp"

//#define TRACE
#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

EncoderMotor *sPointerForInt0ISR;
EncoderMotor *sPointerForInt1ISR;

EncoderMotor::EncoderMotor() : // @suppress("Class members should be properly initialized")
        PWMDcMotor() {
#if defined(ENABLE_MOTOR_LIST_FUNCTIONS)
    AddToMotorList();
#endif
}

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
/*
 * aMotorNumber from 1 to 2
 * If no parameter, we use a fixed assignment of rightCarMotor interrupts to INT0 / Pin2 and leftCarMotor to INT1 / Pin3
 * Currently motors 3 and 4 are not required/supported by own library for Adafruit Motor Shield
 */
void EncoderMotor::init(uint8_t aMotorNumber) {
    PWMDcMotor::init(aMotorNumber);  // create with the default frequency 1.6KHz
    resetEncoderMotorValues();
}
void EncoderMotor::init(uint8_t aMotorNumber, uint8_t aInterruptNumber) {
    PWMDcMotor::init(aMotorNumber);  // create with the default frequency 1.6KHz
    resetEncoderMotorValues();
    attachEncoderInterrupt(aInterruptNumber);
}
#else
EncoderMotor::EncoderMotor(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) : // @suppress("Class members should be properly initialized")
        PWMDcMotor(aForwardPin, aBackwardPin, aPWMPin) {
    resetEncoderMotorValues();
#if defined(ENABLE_MOTOR_LIST_FUNCTIONS)
    AddToMotorList();
#endif
}

void EncoderMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    PWMDcMotor::init(aForwardPin, aBackwardPin, aPWMPin);
    resetEncoderMotorValues();
}

void EncoderMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aInterruptNumber) {
    PWMDcMotor::init(aForwardPin, aBackwardPin, aPWMPin);
    resetEncoderMotorValues();
    attachEncoderInterrupt(aInterruptNumber);
}
#endif

/*
 * If motor is already running, adjust TargetDistanceMillimeter to go to aRequestedDistanceMillimeter
 */
void EncoderMotor::startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    if (aRequestedDistanceMillimeter == 0) {
        stop(DefaultStopMode); // In case motor was running
        return;
    }
    resetEncoderControlValues();
    if (RequestedSpeedPWM == 0) {
        TargetDistanceMillimeter = aRequestedDistanceMillimeter;
        setSpeedPWMAndDirectionWithRamp(aRequestedSpeedPWM, aRequestedDirection);
    } else {
        /*
         * Already moving
         */
#if !defined(DO_NOT_SUPPORT_RAMP)
        MotorRampState = MOTOR_STATE_DRIVE; // must be set, since we may be moving until now without ramp control
#endif
        TargetDistanceMillimeter = getDistanceMillimeter() + aRequestedDistanceMillimeter;
        PWMDcMotor::setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
    }
    LastTargetDistanceMillimeter = TargetDistanceMillimeter;
    CheckDistanceInUpdateMotor = true;
}

void EncoderMotor::startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * if aRequestedDistanceMillimeter < 0 then use DIRECTION_BACKWARD
 * initialize motorInfo fields DirectionForward, RequestedDriveSpeedPWM, DistanceTickCounter and optional NextChangeMaxTargetCount.
 */
void EncoderMotor::startGoDistanceMillimeter(int aRequestedDistanceMillimeter) {
    if (aRequestedDistanceMillimeter < 0) {
        aRequestedDistanceMillimeter = -aRequestedDistanceMillimeter;
        startGoDistanceMillimeter(aRequestedDistanceMillimeter, DIRECTION_BACKWARD);
    } else {
        startGoDistanceMillimeter(aRequestedDistanceMillimeter, DIRECTION_FORWARD);
    }
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool EncoderMotor::updateMotor() {
    unsigned long tMillis = millis();
    uint8_t tNewSpeedPWM = RequestedSpeedPWM;

    /*
     * Check if target distance is reached or encoder tick has timeout
     */
    if (tNewSpeedPWM > 0) {
        if (CheckDistanceInUpdateMotor
                && (getDistanceMillimeter() >= TargetDistanceMillimeter
                        || tMillis > (LastEncoderInterruptMillis + ENCODER_SENSOR_TIMEOUT_MILLIS))) {
            /*
             * Stop now
             */
            stop(STOP_MODE_BRAKE); // this sets MOTOR_STATE_STOPPED;
#if defined(LOCAL_DEBUG)
            Serial.print(PWMPin);
            if(tMillis > (LastEncoderInterruptMillis + ENCODER_SENSOR_TIMEOUT_MILLIS)){
                Serial.print(F(" Encoder timeout: dist="));
            } else{
                Serial.print(F(" Reached: dist="));
            }
            Serial.print(getDistanceMillimeter());
            Serial.print(F(" Breakdist="));
            Serial.println(getBrakingDistanceMillimeter());
#endif
            return false; // need no more calls to updateMotor()
        }
#if defined(DO_NOT_SUPPORT_RAMP)
        // We only need to check stop condition, if we have no ramp
        return true; // check again
#endif
    }
#if defined(DO_NOT_SUPPORT_RAMP)
    return false; // PWM is already 0, need no more calls to updateMotor()
#else
    if (MotorRampState == MOTOR_STATE_START) {
        NextRampChangeMillis = tMillis + RAMP_INTERVAL_MILLIS;
        /*
         * Start motor
         */
        if (RequestedDriveSpeedPWM > RAMP_UP_VALUE_OFFSET_SPEED_PWM) {
            // start with ramp to avoid spinning wheels
            tNewSpeedPWM = RAMP_UP_VALUE_OFFSET_SPEED_PWM; // start immediately with speed offset (2.3 volt)
            //  --> RAMP_UP
            MotorRampState = MOTOR_STATE_RAMP_UP;
        } else {
            // Motor ramp not required, go direct to drive speed.
            tNewSpeedPWM = RequestedDriveSpeedPWM;
            //  --> DRIVE
            MotorRampState = MOTOR_STATE_DRIVE;
        }


    } else if (MotorRampState == MOTOR_STATE_RAMP_UP) {
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_INTERVAL_MILLIS;
            /*
             * Increase motor speed by RAMP_VALUE_DELTA every RAMP_UPDATE_INTERVAL_MILLIS milliseconds
             * Transition criteria to next state is:
             * Drive speed reached or target distance - braking distance reached
             */
            if (tNewSpeedPWM == RequestedDriveSpeedPWM
                    || (CheckDistanceInUpdateMotor
                            && getDistanceMillimeter() + getBrakingDistanceMillimeter() >= TargetDistanceMillimeter)) {
                //  RequestedDriveSpeedPWM reached switch to --> DRIVE_SPEED_PWM and check immediately for next transition to RAMP_DOWN
                MotorRampState = MOTOR_STATE_DRIVE;
            } else {
                tNewSpeedPWM = tNewSpeedPWM + RAMP_UP_VALUE_DELTA;
                // Clip value and check for 8 bit overflow
                if (tNewSpeedPWM > RequestedDriveSpeedPWM || tNewSpeedPWM <= RAMP_UP_VALUE_DELTA) {
                    // do not change state here to let motor run at RequestedDriveSpeedPWM for one interval
                    tNewSpeedPWM = RequestedDriveSpeedPWM;
                }
            }
        }
    }

    // do not use "else if" since we must immediately check for next transition to RAMP_DOWN
    if (MotorRampState == MOTOR_STATE_DRIVE) {
        /*
         * Wait until target distance - braking distance reached
         */
        if (CheckDistanceInUpdateMotor && (getDistanceMillimeter() + getBrakingDistanceMillimeter() >= TargetDistanceMillimeter)) {
            if (RequestedSpeedPWM > RAMP_DOWN_VALUE_OFFSET_SPEED_PWM) {
                tNewSpeedPWM -= (RAMP_DOWN_VALUE_OFFSET_SPEED_PWM - RAMP_DOWN_VALUE_DELTA); // RAMP_VALUE_DELTA is immediately subtracted below
            } else {
                tNewSpeedPWM = RAMP_VALUE_MIN_SPEED_PWM;
            }
            //  --> RAMP_DOWN
            MotorRampState = MOTOR_STATE_RAMP_DOWN;
#if defined(LOCAL_DEBUG)
            Serial.print(PWMPin);
            Serial.print(F(" Speed="));
            Serial.print(getSpeed() * 10);
            Serial.print(F(" mm/s Dist="));
            Serial.print(getDistanceMillimeter());
            Serial.print(F(" mm Breakdist="));
            Serial.print(getBrakingDistanceMillimeter());
            Serial.println(F(" mm"));
#endif
        }
    }

    // do not use "else if" since we must immediately check for next transition to STOPPED
    if (MotorRampState == MOTOR_STATE_RAMP_DOWN) {
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis = tMillis + RAMP_INTERVAL_MILLIS;
            /*
             * Decrease motor speed RAMP_UPDATE_INTERVAL_STEPS times every RAMP_UPDATE_INTERVAL_MILLIS milliseconds
             * until RAMP_VALUE_MIN_SPEED_PWM is reached
             */
            if (tNewSpeedPWM == RAMP_VALUE_MIN_SPEED_PWM) {
                /*
                 * Ramp ended, last value was RAMP_VALUE_MIN_SPEED_PWM
                 */
                if(!CheckDistanceInUpdateMotor){
                    // can stop now
                    tNewSpeedPWM = 0;
                } else {
                    // continue to check distance a slow speed
                    MotorRampState = MOTOR_STATE_CHECK_DISTANCE;
                }
            } else {
                tNewSpeedPWM -= RAMP_DOWN_VALUE_DELTA;
                    if (tNewSpeedPWM < RAMP_VALUE_MIN_SPEED_PWM) {
                    // Clip at RAMP_VALUE_MIN_SPEED_PWM
                    tNewSpeedPWM = RAMP_VALUE_MIN_SPEED_PWM;
                }
            }
        }
    }

    /*
     * End of motor state machine, now set speed if changed
     */
    if (tNewSpeedPWM != RequestedSpeedPWM) {
#if defined(LOCAL_DEBUG)
        // The speed 0
        Serial.print(PWMPin);
        Serial.print(F(" St="));
        Serial.print(MotorRampState);
        Serial.print(F(" Ns="));
        Serial.println(tNewSpeedPWM);
#endif
        PWMDcMotor::setSpeedPWM(tNewSpeedPWM);
    }
    return (RequestedSpeedPWM > 0); // current speed == 0
#endif // !defined(DO_NOT_SUPPORT_RAMP)
}

/*
 * Computes motor speed compensation value in order to go exactly straight ahead
 * Compensate only at forward direction
 */
void EncoderMotor::synchronizeMotor(EncoderMotor *aOtherMotorControl, unsigned int aCheckInterval) {
    if (CurrentDirection != DIRECTION_FORWARD || aOtherMotorControl->CurrentDirection != DIRECTION_FORWARD) {
        return;
    }
    static long sNextMotorSyncMillis;
    long tMillis = millis();
    if (tMillis >= sNextMotorSyncMillis) {
        sNextMotorSyncMillis += aCheckInterval;
// only synchronize if manually operated or at full speed
#if !defined(DO_NOT_SUPPORT_RAMP)
        if ((MotorRampState == MOTOR_STATE_STOPPED && aOtherMotorControl->MotorRampState == MOTOR_STATE_STOPPED
                && CurrentCompensatedSpeedPWM > 0)
                || (MotorRampState == MOTOR_STATE_DRIVE && aOtherMotorControl->MotorRampState == MOTOR_STATE_DRIVE))
#endif
            {
            MotorControlValuesHaveChanged = false;
            if (EncoderCount >= (aOtherMotorControl->EncoderCount + 2)) {
                EncoderCount = aOtherMotorControl->EncoderCount;
                /*
                 * This motor is too fast, first try to reduce other motors compensation
                 */
                if (aOtherMotorControl->SpeedPWMCompensation >= 2) {
                    aOtherMotorControl->SpeedPWMCompensation -= 2;
                    aOtherMotorControl->CurrentCompensatedSpeedPWM += 2;
                    aOtherMotorControl->setSpeedPWMAndDirection(aOtherMotorControl->CurrentCompensatedSpeedPWM, DIRECTION_FORWARD);
                    MotorControlValuesHaveChanged = true;
                    EncoderCount = aOtherMotorControl->EncoderCount;
                } else if (CurrentCompensatedSpeedPWM > DriveSpeedPWM / 2) {
                    /*
                     * else increase this motors compensation
                     */
                    SpeedPWMCompensation += 2;
                    CurrentCompensatedSpeedPWM -= 2;
                    PWMDcMotor::setSpeedPWMAndDirection(CurrentCompensatedSpeedPWM, DIRECTION_FORWARD);
                    MotorControlValuesHaveChanged = true;
                }

            } else if (aOtherMotorControl->EncoderCount >= (EncoderCount + 2)) {
                aOtherMotorControl->EncoderCount = EncoderCount;
                /*
                 * Other motor is too fast, first try to reduce this motors compensation
                 */
                if (SpeedPWMCompensation >= 2) {
                    SpeedPWMCompensation -= 2;
                    CurrentCompensatedSpeedPWM += 2;
                    PWMDcMotor::setSpeedPWMAndDirection(CurrentCompensatedSpeedPWM, DIRECTION_FORWARD);
                    MotorControlValuesHaveChanged = true;
                } else if (aOtherMotorControl->CurrentCompensatedSpeedPWM > aOtherMotorControl->DriveSpeedPWM / 2) {
                    /*
                     * else increase other motors compensation
                     */
                    aOtherMotorControl->SpeedPWMCompensation += 2;
                    aOtherMotorControl->CurrentCompensatedSpeedPWM -= 2;
                    aOtherMotorControl->setSpeedPWMAndDirection(aOtherMotorControl->CurrentCompensatedSpeedPWM, DIRECTION_FORWARD);
                    MotorControlValuesHaveChanged = true;
                }
            }
        }
    }
}

/*************************
 * Direct motor control
 *************************/
/*
 * Reset ALL encoder class variables to 0x00. From TargetDistanceMillimeter up to EncoderInterruptDeltaMillis
 * Set LastEncoderInterruptMillis to a sensible value to avoid initial timeout
 */
void EncoderMotor::resetEncoderMotorValues() {
    memset(reinterpret_cast<uint8_t*>(&TargetDistanceMillimeter), 0,
            (((uint8_t*) &EncoderInterruptDeltaMillis) + sizeof(EncoderInterruptDeltaMillis)) - reinterpret_cast<uint8_t*>(&TargetDistanceMillimeter));
    resetEncoderControlValues();
// to force display of initial values
    SensorValuesHaveChanged = true;
}

/*
 * Reset EncoderInterruptDeltaMillis, EncoderInterruptMillisArray, EncoderInterruptMillisArrayIndex and AverageSpeedIsValid
 */
void EncoderMotor::resetSpeedValues() {
#if defined(_SUPPORT_AVERAGE_SPEED)
    memset((void*) &EncoderInterruptMillisArray, 0,
            ((uint8_t*) &EncoderInterruptDeltaMillis + sizeof(EncoderInterruptDeltaMillis)) - (uint8_t*) &EncoderInterruptMillisArray);
#else
    EncoderInterruptDeltaMillis = 0;
#endif
}

void EncoderMotor::resetEncoderControlValues() {
    EncoderCount = 0;
    LastRideEncoderCount = 0;
    LastEncoderInterruptMillis = millis() - ENCODER_SENSOR_RING_MILLIS - 1; // Set to a sensible value to avoid initial timeout
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
void EncoderMotor::attachEncoderInterrupt(uint8_t aInterruptNumber) {
#if defined(EICRA)
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
#else
#error Encoder interrupts for ESP32 not yet supported
    attachEncoderInterrupt(aInterruptNumber, handleEncoderInterrupt, RISING);
#endif
}


uint8_t EncoderMotor::getDirection() {
    return CurrentDirection;
}

unsigned int EncoderMotor::getDistanceMillimeter() {
    return LastRideEncoderCount * FACTOR_COUNT_TO_MILLIMETER_INTEGER_DEFAULT; // * 11
}

unsigned int EncoderMotor::getDistanceCentimeter() {
    return (LastRideEncoderCount * FACTOR_COUNT_TO_MILLIMETER_INTEGER_DEFAULT) / 10;
}

/*
 * Use physical formula of accelerated mass s = (v * v) / 2 * a
 */
unsigned int EncoderMotor::getBrakingDistanceMillimeter() {
    unsigned int tSpeedCmPerSecond = getSpeed();
//    return (tSpeedCmPerSecond * tSpeedCmPerSecond * 100) / RAMP_DECELERATION_TIMES_2; // overflow!
    // RAMP_DECELERATION_TIMES_2 / 100 instead of tSpeedCmPerSecond * 100 to avoid overflow
    return (tSpeedCmPerSecond * tSpeedCmPerSecond) / (RAMP_DECELERATION_TIMES_2 / 100);
}

/*
 * Speed is in cm/s for a 20 slot encoder disc
 * Reset speed values after 1 second
 */
unsigned int EncoderMotor::getSpeed() {
    if (millis() - LastEncoderInterruptMillis > SPEED_TIMEOUT_MILLIS) {
        resetSpeedValues(); // Reset speed values after 1 second
    }
    unsigned long tEncoderInterruptDeltaMillis = EncoderInterruptDeltaMillis;
    if (tEncoderInterruptDeltaMillis == 0) {
        return 0;
    }
    return (SPEED_SCALE_VALUE / tEncoderInterruptDeltaMillis);
}

/*
 * Speed is in cm/s for a 20 slot encoder disc
 * Average is computed over the full revolution to compensate for unequal distances of the laser cut encoder discs.
 * If we do not have 21 timestamps, average is computed over the existing ones
 * if DO_NOT_SUPPORT_AVERAGE_SPEED is defined, it is reduced to getSpeed()
 */
unsigned int EncoderMotor::getAverageSpeed() {
#if defined(DO_NOT_SUPPORT_AVERAGE_SPEED)
    return getSpeed();
#else
    int tAverageSpeed = 0;
    /*
     * First check for timeout
     */
    if (millis() - LastEncoderInterruptMillis > SPEED_TIMEOUT_MILLIS) {
        resetSpeedValues(); // Reset speed values after 1 second
    } else {
        int8_t tEncoderInterruptMillisArrayIndex = EncoderInterruptMillisArrayIndex;
        if (!AverageSpeedIsValid) {
            tEncoderInterruptMillisArrayIndex--;
            if (tEncoderInterruptMillisArrayIndex > 0) {
                // here MillisArray is not completely filled and EncoderInterruptMillisArrayIndex had no wrap around
                tAverageSpeed = (SPEED_SCALE_VALUE * tEncoderInterruptMillisArrayIndex)
                        / (EncoderInterruptMillisArray[tEncoderInterruptMillisArrayIndex] - EncoderInterruptMillisArray[0]);
            }
        } else {
            // tEncoderInterruptMillisArrayIndex points to the next value to write == the oldest value to overwrite
            unsigned long tOldestEncoderInterruptMillis = EncoderInterruptMillisArray[tEncoderInterruptMillisArrayIndex];

            // get index of current value
            tEncoderInterruptMillisArrayIndex--;
            if (tEncoderInterruptMillisArrayIndex < 0) {
                // wrap around
                tEncoderInterruptMillisArrayIndex = AVERAGE_SPEED_BUFFER_SIZE - 1;
            }
            tAverageSpeed = (SPEED_SCALE_VALUE * AVERAGE_SPEED_SAMPLE_SIZE)
                    / (EncoderInterruptMillisArray[tEncoderInterruptMillisArrayIndex] - tOldestEncoderInterruptMillis);
        }
    }
    return tAverageSpeed;
#endif
}

/*
 * Smaller function with no automatic fill recognition.
 * @param aLengthOfAverage only values from 1 to 20 are valid!
 */
unsigned int EncoderMotor::getAverageSpeed(uint8_t aLengthOfAverage) {
#if defined(DO_NOT_SUPPORT_AVERAGE_SPEED)
    (void) aLengthOfAverage;
    return getSpeed();
#else
    if (!AverageSpeedIsValid && EncoderInterruptMillisArrayIndex < aLengthOfAverage) {
        // cannot compute requested average
        return 0;
    }
    // get index of aLengthOfAverage counts before
    int8_t tHistoricIndex = (EncoderInterruptMillisArrayIndex - 1) - aLengthOfAverage;
    if (tHistoricIndex < 0) {
        // wrap around
        tHistoricIndex += AVERAGE_SPEED_BUFFER_SIZE;
    }
    int tAverageSpeed = (SPEED_SCALE_VALUE * aLengthOfAverage)
            / (LastEncoderInterruptMillis - EncoderInterruptMillisArray[tHistoricIndex]);

    return tAverageSpeed;
#endif
}

/*
 * Print caption for Serial Plotter
 * Space but NO println() at the end, to enable additional information to be printed
 */
void EncoderMotor::printEncoderDataCaption(Print *aSerial) {
//    aSerial->print(F("Speed[cm/s] SpeedAvg.Of10[cm/s] Count[22cm/20LSB] "));
    aSerial->print(F("Speed[cm/s] Distance[cm] "));
}

bool EncoderMotor::printEncoderDataPeriodically(Print *aSerial, uint16_t aPeriodMillis) {
    static unsigned long sLastPrintMillis;

    if ((millis() - sLastPrintMillis) >= aPeriodMillis) { // print data every n ms
        sLastPrintMillis = millis();
        printEncoderData(aSerial);
        return true;
    }
    return false;
}

void EncoderMotor::printEncoderData(Print *aSerial) {
    aSerial->print(getSpeed());
    aSerial->print(" ");
#if defined(_SUPPORT_AVERAGE_SPEED)
//    aSerial->print(getAverageSpeed(10));
//    aSerial->print(" ");
#endif
    aSerial->print(getDistanceCentimeter());
    aSerial->print(" ");
}

#if defined ESP32
void IRAM_ATTR EncoderMotor::handleEncoderInterrupt() {
#else
void EncoderMotor::handleEncoderInterrupt() {
#endif
    long tMillis = millis();
    unsigned long tDeltaMillis = tMillis - LastEncoderInterruptMillis;
    if (tDeltaMillis <= ENCODER_SENSOR_RING_MILLIS) {
        // assume signal is ringing and do nothing
    } else {
        LastEncoderInterruptMillis = tMillis;
#if defined(_SUPPORT_AVERAGE_SPEED)
        uint8_t tEncoderInterruptMillisArrayIndex = EncoderInterruptMillisArrayIndex;
#endif
        if (tDeltaMillis < ENCODER_SENSOR_TIMEOUT_MILLIS) {
            EncoderInterruptDeltaMillis = tDeltaMillis;
        } else {
            // timeout
            EncoderInterruptDeltaMillis = 0;
#if defined(_SUPPORT_AVERAGE_SPEED)
            tEncoderInterruptMillisArrayIndex = 0;
            AverageSpeedIsValid = false;
#endif
        }
#if defined(_SUPPORT_AVERAGE_SPEED)
        EncoderInterruptMillisArray[tEncoderInterruptMillisArrayIndex++] = tMillis;
        if (tEncoderInterruptMillisArrayIndex >= AVERAGE_SPEED_BUFFER_SIZE) {
            tEncoderInterruptMillisArrayIndex = 0;
            AverageSpeedIsValid = true;
        }
        EncoderInterruptMillisArrayIndex = tEncoderInterruptMillisArrayIndex;
#endif

        EncoderCount++;
        LastRideEncoderCount++;
        SensorValuesHaveChanged = true;
    }
}

#if defined(INT0_vect)
// ISR for PIN PD2 / RIGHT
ISR(INT0_vect) {
    sPointerForInt0ISR->handleEncoderInterrupt();
}

// ISR for PIN PD3 / LEFT
ISR(INT1_vect) {
    sPointerForInt1ISR->handleEncoderInterrupt();
}
#endif

/******************************************************************************************
 * Static methods
 *****************************************************************************************/
/*
 * Enable both interrupts INT0/D2 or INT1/D3
 */
void EncoderMotor::enableINT0AndINT1InterruptsOnRisingEdge() {
#if defined(EICRA)
// interrupt on any logical change
    EICRA |= (_BV(ISC00) | _BV(ISC01) | _BV(ISC10) | _BV(ISC11));
// clear interrupt bit
    EIFR |= (_BV(INTF0) | _BV(INTF1));
// enable interrupt on next change
    EIMSK |= (_BV(INT0) | _BV(INT1));
#endif
}

#if defined(ENABLE_MOTOR_LIST_FUNCTIONS)
/*
 * The list version saves 100 bytes and is more flexible, compared with the array version
 */
uint8_t EncoderMotor::sNumberOfMotorControls = 0;
EncoderMotor *EncoderMotor::sMotorControlListStart = NULL;

void EncoderMotor::AddToMotorList() {
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
}
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

#if !defined(DO_NOT_SUPPORT_RAMP)
/*
 * Waits until distance is reached
 */
void EncoderMotor::startRampUpAndWaitForDriveSpeedPWMForAll(uint8_t aRequestedDirection, void (*aLoopCallback)(void)) {
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

bool EncoderMotor::allMotorsStarted() {
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStarted = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->MotorRampState != MOTOR_STATE_DRIVE) {
            tAllAreStarted = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStarted;
}
#endif

void EncoderMotor::startGoDistanceMillimeterForAll(int aRequestedDistanceMillimeter) {
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startGoDistanceMillimeter(aRequestedDistanceMillimeter);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
}

/*
 * Waits until distance is reached
 */
void EncoderMotor::goDistanceMillimeterForAll(int aRequestedDistanceMillimeter, void (*aLoopCallback)(void)) {
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startGoDistanceMillimeter(aRequestedDistanceMillimeter);
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    waitUntilAllMotorsStopped(aLoopCallback);
}


bool EncoderMotor::allMotorsStopped() {
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
    bool tAllAreStopped = true;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        if (tEncoderMotorControlPointer->isStopped()) {
            tAllAreStopped = false;
        }
        tEncoderMotorControlPointer = tEncoderMotorControlPointer->NextMotorControl;
    }
    return tAllAreStopped;
}

/*
 * Start ramp down and busy wait for stop
 */
void EncoderMotor::stopAllMotorsAndWaitUntilStopped() {
    EncoderMotor *tEncoderMotorControlPointer = sMotorControlListStart;
// walk through list
    while (tEncoderMotorControlPointer != NULL) {
        tEncoderMotorControlPointer->startRampDown();
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
#endif // #if defined(ENABLE_MOTOR_LIST_FUNCTIONS)
#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // #if defined(USE_ENCODER_MOTOR_CONTROL)
#endif // _ENCODER_MOTOR_CONTROL_HPP
