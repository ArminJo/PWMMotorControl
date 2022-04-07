/*
 * PWMDcMotor.hpp
 *
 * Low level motor control for Adafruit_MotorShield OR breakout board with TB6612 or L298 driver IC for two DC motors.
 *
 * Motor control has 2 parameters:
 * 1. SpeedPWM / PWM which is ignored for BRAKE or RELEASE. This library also accepts signed speed (including the direction as sign).
 * 2. Direction / MotorDriverMode. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 *
 * PWM period is 600 us for Adafruit Motor Shield V2 using PCA9685.
 * PWM period is 1030 us for using AnalogWrite on pin 5 + 6.
 *
 * Distance is computed in 3 different ways.
 * Without IMU or Encoder: - distance is converted to a time for riding.
 * With IMU: - distance is measured by IMU.
 * With encoder: - distance is measured by Encoder.
 *
 * A fixed speed compensation PWM value to be subtracted can be specified.
 *
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
#ifndef _PWM_DC_MOTOR_HPP
#define _PWM_DC_MOTOR_HPP

#include <Arduino.h>

#include "PWMDcMotor.h"

#if defined(ESP32)
#include "analogWrite.h" // from e.g. ESP32Servo library
#endif

//#define TRACE
//#define DEBUG

char sDirectionCharArray[3] = { 'S', 'F', 'B' };
const char *sDirectionStringArray[3] = { "stop", "forward", "backward" };

// Flags e.g. for display update control
#if defined(USE_MPU6050_IMU) || defined(USE_ENCODER_MOTOR_CONTROL)
volatile bool PWMDcMotor::SensorValuesHaveChanged; // true if encoder count and derived encoder speed, or one of the TMU data have changed
#endif
bool PWMDcMotor::MotorControlValuesHaveChanged; // true if DefaultStopMode, DriveSpeedPWM or SpeedPWMCompensation have changed
bool PWMDcMotor::MotorPWMHasChanged;              // true if CompensatedSpeedPWM has changed

PWMDcMotor::PWMDcMotor() { // @suppress("Class members should be properly initialized")
}

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
void PWMDcMotor::PCA9685WriteByte(uint8_t aAddress, uint8_t aData) {
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    Wire.write(aAddress);
    Wire.write(aData);
    Wire.endTransmission(true);
}

void PWMDcMotor::PCA9685SetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff) {
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    Wire.write((PCA9685_FIRST_PWM_REGISTER) + 4 * aPin);
    Wire.write(aOn);
    Wire.write(aOn >> 8);
    Wire.write(aOff);
    Wire.write(aOff >> 8);
    Wire.endTransmission(true);
}

void PWMDcMotor::PCA9685SetPin(uint8_t aPin, bool aSetToOn) {
    if (aSetToOn) {
        PCA9685SetPWM(aPin, 4096, 0);
    } else {
        PCA9685SetPWM(aPin, 0, 0);
    }
}

#  else
// Create the motor shield object with the default I2C address
Adafruit_MotorShield sAdafruitMotorShield = Adafruit_MotorShield();
#  endif // USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD

/*
 * aMotorNumber from 1 to 2
 * Currently motors 3 and 4 are not required/supported by own library for Adafruit Motor Shield
 */
void PWMDcMotor::init(uint8_t aMotorNumber) {
#  if defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    if (aMotorNumber == 1) {
        // Set PCA9685 channel numbers for Adafruit Motor Shield
        PWMPin = 8;
        BackwardPin = 9;
        ForwardPin = 10;
    } else {
        PWMPin = 13;
        BackwardPin = 12;
        ForwardPin = 11;
    }

    Wire.begin();
    Wire.setClock(400000);
#    if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(5000); // Sets timeout to 5 ms. default is 25 ms.
#    endif
#if defined(TRACE)
    Serial.print(PWMPin);
    Serial.print(F(" MotorNumber="));
    Serial.println(aMotorNumber);
#endif
    // Reset PCA9685
    Wire.beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
    Wire.write(PCA9685_SOFTWARE_RESET);
    Wire.endTransmission(true);
    // Set expander to 1600 HZ
    PCA9685WriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_SLEEP)); // go to sleep
    PCA9685WriteByte(PCA9685_PRESCALE_REGISTER, PCA9685_PRESCALER_FOR_1600_HZ); // set the prescaler
    delay(2); // > 500 us before the restart bit according to datasheet
    PCA9685WriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_RESTART) | _BV(PCA9685_MODE_1_AUTOINCREMENT)); // reset sleep and enable auto increment

#  else
    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber);
    sAdafruitMotorShield.begin();
#  endif

    setDefaultsForFixedDistanceDriving();
    stop(STOP_MODE_KEEP);
}

#else // USE_ADAFRUIT_MOTOR_SHIELD
/*
 * @param aForwardPin the pin, which is high if direction is forward
 * @param aBackwardPin the pin, which is high if direction is backward
 */
PWMDcMotor::PWMDcMotor(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    init(aForwardPin, aBackwardPin, aPWMPin);
}

/*
 * @param aForwardPin the pin, which is high if direction is forward
 * @param aBackwardPin the pin, which is high if direction is backward
 */
void PWMDcMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin) {
    ForwardPin = aForwardPin;
    BackwardPin = aBackwardPin;
    PWMPin = aPWMPin;
    DefaultStopMode = DEFAULT_STOP_MODE;

    pinMode(aForwardPin, OUTPUT);
    pinMode(aBackwardPin, OUTPUT);
    pinMode(aPWMPin, OUTPUT);

    // Set DriveSpeedPWM, SpeedPWMCompensation and MillisPerCentimeter defaults
    setDefaultsForFixedDistanceDriving();
    stop(DEFAULT_STOP_MODE);
}

#endif // USE_ADAFRUIT_MOTOR_SHIELD

/*
 *  @brief  Control the DC motor driver direction and stop mode
 *  @param  aMotorDriverMode The mode can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 */
void PWMDcMotor::setDirection(uint8_t aMotorDirection) {
    setMotorDriverMode(aMotorDirection);
}

void PWMDcMotor::setMotorDriverMode(uint8_t aMotorDriverMode) {
    CurrentDirection = aMotorDriverMode;
    if (aMotorDriverMode == STOP_MODE_RELEASE) {
        // We want to store only directions, no brake mode
        CurrentDirection = DIRECTION_STOP;
    }
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    // until here DIRECTION_FORWARD is 0 back is 1, Adafruit library starts with 1
#  if defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    switch (aMotorDriverMode) {
    case DIRECTION_FORWARD:
        PCA9685SetPin(BackwardPin, LOW); // take low first to avoid 'break'
        PCA9685SetPin(ForwardPin, HIGH);
        break;
    case DIRECTION_BACKWARD:
        PCA9685SetPin(ForwardPin, LOW); // take low first to avoid 'break'
        PCA9685SetPin(BackwardPin, HIGH);
        break;
    case STOP_MODE_BRAKE:
        PCA9685SetPin(ForwardPin, HIGH);
        PCA9685SetPin(BackwardPin, HIGH);
        break;
    case STOP_MODE_RELEASE:
        PCA9685SetPin(ForwardPin, LOW);
        PCA9685SetPin(BackwardPin, LOW);
        break;
    }
#  else
    Adafruit_MotorShield_DcMotor->run(aMotorDriverMode + CONVERSION_FOR_ADAFRUIT_API);
#  endif

#else
    switch (aMotorDriverMode) {
    case DIRECTION_FORWARD:
        digitalWrite(BackwardPin, LOW); // take low first to avoid 'break'
        digitalWrite(ForwardPin, HIGH);
        break;
    case DIRECTION_BACKWARD:
        digitalWrite(ForwardPin, LOW); // take low first to avoid 'break'
        digitalWrite(BackwardPin, HIGH);
        break;
    case STOP_MODE_BRAKE:
        digitalWrite(ForwardPin, HIGH);
        digitalWrite(BackwardPin, HIGH);
        break;
    case STOP_MODE_RELEASE:
        digitalWrite(ForwardPin, LOW);
        digitalWrite(BackwardPin, LOW);
        break;
    }
#endif // USE_ADAFRUIT_MOTOR_SHIELD
}

uint8_t PWMDcMotor::getDirection() {
    return CurrentDirection;
}

float PWMDcMotor::getMotorVoltageforPWMAndMillivolt(uint8_t aSpeedPWM, uint16_t aFullBridgeInputVoltageMillivolt) {
    // if aFullBridgeInputVoltageMillivolt is constant, this can be optimized well
    return aSpeedPWM * ((aFullBridgeInputVoltageMillivolt - FULL_BRIDGE_LOSS_MILLIVOLT) / (1000.0 * MAX_SPEED_PWM));
}

float PWMDcMotor::getMotorVoltageforPWM(uint8_t aSpeedPWM, float aFullBridgeInputVoltage) {
    return aSpeedPWM * ((aFullBridgeInputVoltage - FULL_BRIDGE_LOSS_MILLIVOLT) / (1000.0 * MAX_SPEED_PWM));
}

void PWMDcMotor::printDirectionString(Print *aSerial, uint8_t aDirection) {
    aSerial->print(sDirectionStringArray[aDirection]);
}

/*
 * Sets active PWM and handles speed compensation
 */
void PWMDcMotor::setSpeedPWM(uint8_t aRequestedSpeedPWM) {
    RequestedSpeedPWM = aRequestedSpeedPWM;
    /*
     * Handle speed compensation
     */
    uint8_t tCompensatedSpeedPWM;
    if (aRequestedSpeedPWM > SpeedPWMCompensation) {
        tCompensatedSpeedPWM = aRequestedSpeedPWM - SpeedPWMCompensation; // The only statement which sets CompensatedSpeedPWM to a value != 0
    } else {
        tCompensatedSpeedPWM = 0; // no stop mode here
    }
#if defined(TRACE)
        Serial.print(PWMPin);
        Serial.print(F(" RequestedSpeedPWM="));
        Serial.print(aRequestedSpeedPWM);
        Serial.print(F(" CompensatedSpeedPWM="));
        Serial.println(tCompensatedSpeedPWM);
#endif
    if (CompensatedSpeedPWM != tCompensatedSpeedPWM) {
        CompensatedSpeedPWM = tCompensatedSpeedPWM;
        MotorPWMHasChanged = true;
        /*
         * Write to hardware
         */
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
        PCA9685SetPWM(PWMPin, 0, 16 * tCompensatedSpeedPWM);
#  else
        Adafruit_MotorShield_DcMotor->setSpeedPWMAndDirection(tCompensatedSpeedPWM);
#  endif
#else
        analogWrite(PWMPin, tCompensatedSpeedPWM);
#endif
    }
}

/*
 * @return true if direction has changed AND motor was stopped
 */
bool PWMDcMotor::checkAndHandleDirectionChange(uint8_t aRequestedDirection) {
    /*
     * Reduce to STOP, FORWARD or BACKWARD
     */
    uint8_t tRequestedDirection = aRequestedDirection & DIRECTION_MASK;
    bool tReturnValue = false;
    if (CurrentDirection != tRequestedDirection) {
        if (!isStopped()) {
            /*
             * Direction change requested but motor still running-> first stop motor
             */
            stop(STOP_MODE_BRAKE);
            tReturnValue = true;
        }
#if defined(DEBUG)
        Serial.print(PWMPin);
        Serial.print(F(" Change motor mode from "));
        Serial.print(sDirectionCharArray[CurrentDirection]);
        Serial.print(F(" to "));
        Serial.println(sDirectionCharArray[tRequestedDirection]);
#endif
        setDirection(tRequestedDirection); // this in turn sets CurrentDirection
    }
    return tReturnValue;
}

/**
 *  @brief  Control the DC Motor speed/throttle. Subtracts SpeedPWMCompensation from aRequestedSpeedPWM before applying
 *
 *  @param  speed The 8-bit PWM value, 0 is off, 255 is on forward -255 is on backward
 *  @param  aRequestedDirection is DIRECTION_FORWARD or DIRECTION_BACKWARD
 *  First set driver mode, then set PWM
 *  PWM period is 600 us for Adafruit Motor Shield V2 using PCA9685.
 *  PWM period is 1030 us for using AnalogWrite on pin 5 + 6.
 */
void PWMDcMotor::setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
    if (aRequestedSpeedPWM == 0) {
        stop(STOP_MODE_KEEP);
    } else {
        checkAndHandleDirectionChange(aRequestedDirection);
        setSpeedPWM(aRequestedSpeedPWM);
    }
}

/*
 * Keeps direction and sets new speed only if not stopped
 */
void PWMDcMotor::changeSpeedPWM(uint8_t aRequestedSpeedPWM) {
    if (!isStopped()) {
        setSpeedPWMAndDirection(aRequestedSpeedPWM, CurrentDirection); // output PWM value to motor
    }
}

/*
 * Signed speed
 */
void PWMDcMotor::setSpeedPWMAndDirection(int aRequestedSpeedPWM) {
    if (aRequestedSpeedPWM < 0) {
        aRequestedSpeedPWM = -aRequestedSpeedPWM;
        setSpeedPWMAndDirection(aRequestedSpeedPWM, DIRECTION_BACKWARD);
    } else {
        setSpeedPWMAndDirection(aRequestedSpeedPWM, DIRECTION_FORWARD);
    }
}

/*
 * Starts motor using current drive speed
 */
void PWMDcMotor::start(uint8_t aRequestedDirection) {
    setSpeedPWMAndDirection(DriveSpeedPWM, aRequestedDirection);
}

/*
 *  CompensatedSpeedPWM == 0, should be equivalent to MotorRampState == MOTOR_STATE_STOPPED
 */
bool PWMDcMotor::isStopped() {
    return (RequestedSpeedPWM == 0);
}

/*
 * First set PWM to 0 then disable driver
 * @param aStopMode STOP_MODE_KEEP (take previously defined DefaultStopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void PWMDcMotor::stop(uint8_t aStopMode) {
    RequestedSpeedPWM = 0;
    CompensatedSpeedPWM = 0;
    MotorPWMHasChanged = true;
    CheckDistanceInUpdateMotor = false;
#if !defined(DO_NOT_SUPPORT_RAMP)
    MotorRampState = MOTOR_STATE_STOPPED;
#endif

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    PCA9685SetPWM(PWMPin, 0, 0);
#  else
    Adafruit_MotorShield_DcMotor->setSpeedPWMAndDirection(0);
#  endif
#else
    analogWrite(PWMPin, 0);
#endif
    if (aStopMode == STOP_MODE_KEEP) {
        aStopMode = DefaultStopMode;
    }
    setMotorDriverMode(aStopMode);
#if defined(DEBUG)
    Serial.print(PWMPin);
    Serial.print(F(" Stop motor StopMode="));
    Serial.println(aStopMode);
#endif
}

/*
 * @param aStopMode used for speed == 0 or STOP_MODE_KEEP: STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void PWMDcMotor::setStopMode(uint8_t aStopMode) {
    DefaultStopMode = aStopMode;
    MotorControlValuesHaveChanged = true;
}

/*
 * Set DriveSpeedPWM, SpeedPWMCompensation and MillisPerCentimeter defaults
 * setDefaultsForFixedDistanceDriving() is called at init
 * Values depend on FULL_BRIDGE_OUTPUT_MILLIVOLT
 */
void PWMDcMotor::setDefaultsForFixedDistanceDriving() {
    DriveSpeedPWM = DEFAULT_DRIVE_SPEED_PWM;
    SpeedPWMCompensation = 0;
#if !defined(USE_ENCODER_MOTOR_CONTROL)
    MillisPerCentimeter = DEFAULT_MILLIS_PER_CENTIMETER;
#endif
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM, uint8_t aSpeedPWMCompensation) {
    DriveSpeedPWM = aDriveSpeedPWM;
    setSpeedPWMCompensation(aSpeedPWMCompensation);
}

void PWMDcMotor::setDriveSpeedPWM(uint8_t aDriveSpeedPWM) {
    DriveSpeedPWM = aDriveSpeedPWM;
    MotorControlValuesHaveChanged = true;
}

/*
 * 64 bytes program memory
 */
void PWMDcMotor::setSpeedPWMCompensation(uint8_t aSpeedPWMCompensation) {
    SpeedPWMCompensation = aSpeedPWMCompensation;
    setSpeedPWM(RequestedSpeedPWM);
    MotorControlValuesHaveChanged = true;
}

/**
 * Update DriveSpeedPWM value, i.e. if moving set new CompensatedSpeedPWM and apply it to the motor
 */
void PWMDcMotor::updateDriveSpeedPWM(uint8_t aDriveSpeedPWM) {
    DriveSpeedPWM = aDriveSpeedPWM;
    MotorControlValuesHaveChanged = true;
// DriveSpeedPWM = 0 makes no sense and just stops the car
    if (!isStopped() && aDriveSpeedPWM != 0) {
        setSpeedPWMAndDirectionWithRamp(aDriveSpeedPWM, CurrentDirection);
    }
}

/*
 * Changes RequestedDriveSpeedPWM and uses ramp for transitions if it makes sense
 */
void PWMDcMotor::setSpeedPWMAndDirectionWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
#if defined(DEBUG)
    Serial.print(PWMPin);
    Serial.print(F(" Set PWM to "));
    Serial.print(aRequestedSpeedPWM);
    Serial.print(F(" Dir="));
    Serial.print(aRequestedDirection);
    Serial.print(F(" CompensatedSpeedPWM="));
    Serial.print(CompensatedSpeedPWM);
    Serial.print(F(" MotorMode="));
    Serial.print(sDirectionCharArray[CurrentDirection]);
    Serial.println();
#endif
#if defined(DO_NOT_SUPPORT_RAMP)
    setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
#else
    if (isStopped() || CurrentDirection != aRequestedDirection) {
        checkAndHandleDirectionChange(aRequestedDirection);
        MotorRampState = MOTOR_STATE_START;
        /*
         * Stopped -> set target speed for ramp up
         */
        RequestedDriveSpeedPWM = aRequestedSpeedPWM;
    } else if (MotorRampState == MOTOR_STATE_DRIVE) {
        /*
         * motor is running -> just change speed
         */
        setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
    }
// else ramp is in mode MOTOR_STATE_RAMP_UP -> do nothing, let the ramp go on
#  if defined(DEBUG)
    Serial.print(F("MotorRampState="));
    Serial.print(MotorRampState);
    Serial.println();
#  endif
#endif // defined(DO_NOT_SUPPORT_RAMP)
}

/*
 * Takes the current DriveSpeedPWM for ramp up
 */
void PWMDcMotor::startRampUp(uint8_t aRequestedDirection) {
    setSpeedPWMAndDirectionWithRamp(DriveSpeedPWM, aRequestedDirection);
}

void PWMDcMotor::startRampDown() {
#if defined(DO_NOT_SUPPORT_RAMP)
    stop(STOP_MODE_KEEP);
#else
    MotorRampState = MOTOR_STATE_RAMP_DOWN;
// set only the variables for later evaluation in updateMotor() below
    if (CompensatedSpeedPWM > (RAMP_VALUE_OFFSET_SPEED_PWM - RAMP_VALUE_DELTA)) {
        CompensatedSpeedPWM -= (RAMP_VALUE_OFFSET_SPEED_PWM - RAMP_VALUE_DELTA); // RAMP_VALUE_DELTA is immediately subtracted below
    } else {
        CompensatedSpeedPWM = RAMP_VALUE_MIN_SPEED_PWM;
    }
#endif
}

#if !defined(USE_ENCODER_MOTOR_CONTROL)
/*
 * @return true if not stopped (motor expects another update)
 */
bool PWMDcMotor::updateMotor() {
    uint8_t tNewSpeedPWM = CompensatedSpeedPWM;

    /*
     * Check if target milliseconds are reached
     */
    if (tNewSpeedPWM > 0) {
        if (CheckDistanceInUpdateMotor && millis() > computedMillisOfMotorStopForDistance) {
            stop(STOP_MODE_KEEP); // resets CheckDistanceInUpdateMotor and sets MOTOR_STATE_STOPPED;
            return false;
        }
    }

#if !defined(DO_NOT_SUPPORT_RAMP)
    unsigned long tMillis = millis();

    if (MotorRampState == MOTOR_STATE_START) {
        NextRampChangeMillis = tMillis + RAMP_INTERVAL_MILLIS;
        /*
         * Start motor
         */
        if (RequestedDriveSpeedPWM > RAMP_VALUE_OFFSET_SPEED_PWM) {
            // Start with ramp to avoid spinning wheels
            tNewSpeedPWM = RAMP_VALUE_OFFSET_SPEED_PWM; // start immediately with speed offset (3 volt)
            //  --> RAMP_UP
            MotorRampState = MOTOR_STATE_RAMP_UP;
        } else {
            // Motor ramp not required, go direct to drive speed.
            tNewSpeedPWM = RequestedDriveSpeedPWM;
            //  --> DRIVE
            MotorRampState = MOTOR_STATE_DRIVE;
        }
    } else if (MotorRampState == MOTOR_STATE_RAMP_UP) {
        /*
         * Increase motor speed by RAMP_VALUE_DELTA every RAMP_UPDATE_INTERVAL_MILLIS milliseconds
         * Only used if RequestedDriveSpeedPWM > RAMP_VALUE_OFFSET_SPEED_PWM to avoid spinning wheels
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_INTERVAL_MILLIS;
            tNewSpeedPWM = tNewSpeedPWM + RAMP_VALUE_DELTA;
            /*
             * Transition criteria is: RequestedDriveSpeedPWM reached.
             * Then check immediately for timeout
             */
            // Clip value and check for 8 bit overflow
            if (tNewSpeedPWM >= RequestedDriveSpeedPWM || tNewSpeedPWM <= RAMP_VALUE_DELTA) {
                tNewSpeedPWM = RequestedDriveSpeedPWM;
                //  --> DRIVE
                MotorRampState = MOTOR_STATE_DRIVE;
            }
        }
    }
    if (MotorRampState == MOTOR_STATE_DRIVE) {
        /*
         * Time based distance. Ramp down is included in time formula.
         */
        if (CheckDistanceInUpdateMotor && millis() >= computedMillisOfMotorStopForDistance) {
            /*
             * "Distance" reached -> stop now
             */

            if (tNewSpeedPWM > RAMP_VALUE_OFFSET_SPEED_PWM) {
                // Stop with ramp to avoid blocking wheels
                tNewSpeedPWM -= (RAMP_VALUE_OFFSET_SPEED_PWM - RAMP_VALUE_DELTA); // RAMP_VALUE_DELTA is immediately subtracted below
                //  --> RAMP_DOWN
                MotorRampState = MOTOR_STATE_RAMP_DOWN;
            } else {
                tNewSpeedPWM = 0; // This in turn changes state to MOTOR_STATE_STOPPED
            }
        }
    }

    if (MotorRampState == MOTOR_STATE_RAMP_DOWN) {
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis = tMillis + RAMP_INTERVAL_MILLIS;
            /*
             * Decrease motor speed RAMP_UPDATE_INTERVAL_STEPS times every RAMP_UPDATE_INTERVAL_MILLIS milliseconds
             * Only used if CompensatedSpeedPWM > RAMP_VALUE_OFFSET_SPEED_PWM to avoid blocking wheels
             */
            if (tNewSpeedPWM > (RAMP_VALUE_DELTA + RAMP_VALUE_MIN_SPEED_PWM)) {
                tNewSpeedPWM -= RAMP_VALUE_DELTA;
            } else {
                tNewSpeedPWM = RAMP_VALUE_MIN_SPEED_PWM;
            }
        }
    }
// End of motor state machine

#if defined(TRACE)
        Serial.print(F("St="));
        Serial.println(MotorRampState);
#endif
    if (tNewSpeedPWM != CompensatedSpeedPWM) {
#if defined(TRACE)
        Serial.print(F("Ns="));
        Serial.println(tNewSpeedPWM);
#endif
        PWMDcMotor::setSpeedPWMAndDirection(tNewSpeedPWM, CurrentDirection); // sets MOTOR_STATE_STOPPED if speed is 0
    }
#endif // #if defined(DO_NOT_SUPPORT_RAMP)
    return (CompensatedSpeedPWM > 0); // current speed == 0
}

/********************************************************************************************
 * Fixed distance driving functions
 ********************************************************************************************/
/*
 * Required for non encoder motors to estimate duration for a fixed distance
 */
void PWMDcMotor::setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond) {
    MillisPerCentimeter = MILLIS_IN_ONE_SECOND * MILLIMETER_IN_ONE_CENTIMETER / aMillimeterPerSecond;
}

void PWMDcMotor::goDistanceMillimeter(int aRequestedDistanceMillimeter) {
    uint8_t tRequestedDirection = DIRECTION_FORWARD;
    if (aRequestedDistanceMillimeter < 0) {
        tRequestedDirection = DIRECTION_BACKWARD;
    }
    goDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, tRequestedDirection);
}

void PWMDcMotor::goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    goDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

void PWMDcMotor::goDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
#if defined(DO_NOT_SUPPORT_RAMP)
    delay(computedMillisOfMotorStopForDistance - millis());
#else
    while (millis() <= computedMillisOfMotorStopForDistance) {
        updateMotor();
    }
#endif
    stop(STOP_MODE_KEEP);
}

void PWMDcMotor::startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * Signed DistanceCount
 */
void PWMDcMotor::startGoDistanceMillimeter(int aRequestedDistanceMillimeter) {
    if (aRequestedDistanceMillimeter < 0) {
        aRequestedDistanceMillimeter = -aRequestedDistanceMillimeter;
        startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_BACKWARD);
    } else {
        startGoDistanceMillimeter(DriveSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_FORWARD);
    }
}

/*
 * If motor is already running, just update speed and new stop time
 */
void PWMDcMotor::startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    if (aRequestedDistanceMillimeter == 0) {
        stop(STOP_MODE_BRAKE); // In case motor was running
        return;
    }

    setSpeedPWMAndDirectionWithRamp(aRequestedSpeedPWM, aRequestedDirection);
    /*
     * Estimate duration for given distance
     * use 32 bit intermediate to avoid overflow (this also saves around 50 bytes of program memory by using slower functions instead of faster inline code)
     */
    uint32_t tComputedMillisOfMotorStopForDistance =
            (((uint32_t) aRequestedDistanceMillimeter * MillisPerCentimeter * DriveSpeedPWM) / (MILLIMETER_IN_ONE_CENTIMETER * DEFAULT_DRIVE_SPEED_PWM));

    if (isStopped()) {
        // add startup time
        tComputedMillisOfMotorStopForDistance += DEFAULT_MOTOR_START_TIME_MILLIS;
    }

#if defined(DEBUG)
    Serial.print(F("Go for distance "));
    Serial.print(aRequestedDistanceMillimeter);
    Serial.print(F(" mm -> "));
    Serial.print(tComputedMillisOfMotorStopForDistance);
    Serial.println(F(" ms"));

#endif
    tComputedMillisOfMotorStopForDistance += millis();

    computedMillisOfMotorStopForDistance = tComputedMillisOfMotorStopForDistance;

    CheckDistanceInUpdateMotor = true;
}

#endif // !defined(USE_ENCODER_MOTOR_CONTROL)

#if defined(E2END)
/********************************************************************************************
 * EEPROM functions
 * Uses the start of EEPROM for storage of EepromMotorInfoStruct's for motor number 1 to n
 ********************************************************************************************/
void PWMDcMotor::readMotorValuesFromEeprom(uint8_t aMotorValuesEepromStorageNumber) {
    EepromMotorInfoStruct tEepromMotorInfo;
    eeprom_read_block((void*) &tEepromMotorInfo, (void*) ((aMotorValuesEepromStorageNumber) * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));

    /*
     * Overwrite with values if valid
     */
    if (tEepromMotorInfo.DriveSpeedPWM < 222 && tEepromMotorInfo.DriveSpeedPWM > 40) {
        DriveSpeedPWM = tEepromMotorInfo.DriveSpeedPWM;
        if (tEepromMotorInfo.SpeedPWMCompensation < 24) {
            SpeedPWMCompensation = tEepromMotorInfo.SpeedPWMCompensation;
        }
    }
    MotorControlValuesHaveChanged = true;
}

void PWMDcMotor::writeMotorValuesToEeprom(uint8_t aMotorValuesEepromStorageNumber) {
    EepromMotorInfoStruct tEepromMotorInfo;
    tEepromMotorInfo.DriveSpeedPWM = DriveSpeedPWM;
    tEepromMotorInfo.SpeedPWMCompensation = SpeedPWMCompensation;

    eeprom_write_block((void*) &tEepromMotorInfo, (void*) ((aMotorValuesEepromStorageNumber) * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));
}
#endif // defined(E2END)

void PWMDcMotor::printValues(Print *aSerial) {
    aSerial->print(PWMPin);
    aSerial->print(F(" CompensatedSpeedPWM="));
    aSerial->print(CompensatedSpeedPWM);
    aSerial->print(F(" DriveSpeedPWM="));
    aSerial->print(DriveSpeedPWM);
    aSerial->print(F(" SpeedPWMCompensation="));
    aSerial->print(SpeedPWMCompensation);
    aSerial->print(F(" CurrentDirection="));
    aSerial->print(sDirectionCharArray[CurrentDirection]);
    aSerial->println();
}

const char StringNot[] PROGMEM = { " not" };
const char StringDefined[] PROGMEM = { " defined" };

void PWMDcMotor::printCompileOptions(Print *aSerial) {
    aSerial->println();
    aSerial->println(F("Settings (from PWMDcMotor.h):"));

    aSerial->print(F("USE_ENCODER_MOTOR_CONTROL:"));
#if !defined(USE_ENCODER_MOTOR_CONTROL)
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));

    aSerial->print(F("USE_ADAFRUIT_MOTOR_SHIELD:"));
#if !defined(USE_ADAFRUIT_MOTOR_SHIELD)
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    aSerial->print(F("USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD:"));
#if !defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    aSerial->print(reinterpret_cast<const __FlashStringHelper *>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));
#endif

    aSerial->print(F("FULL_BRIDGE_OUTPUT_MILLIVOLT="));
    aSerial->print(FULL_BRIDGE_OUTPUT_MILLIVOLT);
    aSerial->print(F("mV (= FULL_BRIDGE_INPUT_MILLIVOLT|"));
    aSerial->print(FULL_BRIDGE_INPUT_MILLIVOLT);
    aSerial->print(F("mV - FULL_BRIDGE_LOSS_MILLIVOLT|"));
    aSerial->print(FULL_BRIDGE_LOSS_MILLIVOLT);
    aSerial->println(F("mV)"));

    aSerial->print(F("DEFAULT_START_SPEED_PWM="));
    aSerial->print(DEFAULT_START_SPEED_PWM);
    aSerial->print(F(", DEFAULT_DRIVE_SPEED_PWM="));
    aSerial->println(DEFAULT_DRIVE_SPEED_PWM);

    aSerial->print(F("DEFAULT_MOTOR_START_TIME_MILLIS="));
    aSerial->println(DEFAULT_MOTOR_START_TIME_MILLIS);

    aSerial->print(F("DEFAULT_MILLIS_PER_MILLIMETER="));
    aSerial->println(DEFAULT_MILLIS_PER_CENTIMETER);
    aSerial->println();
}

//void PanicWithLed(unsigned int aDelay, uint8_t aCount) {
//    for (uint_fast8_t i = 0; i < aCount; ++i) {
//        digitalWrite(LED_BUILTIN, HIGH);
//        delay(aDelay);
//        digitalWrite(LED_BUILTIN, LOW);
//        delay(aDelay);
//    }
//}

#endif // _PWM_DC_MOTOR_HPP
#pragma once
