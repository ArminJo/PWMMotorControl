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
 * PWM period is 1024 us / 976,56 Hz for using AnalogWrite on pin 5 + 6. (Fast PWM, internal prescaler of 64)
 * PWM period is 2048 us / 488 Hz for using AnalogWrite on pin 3, 9, 10 + 11. (Phase correct PWM, internal prescaler of 64)
 *
 * Distance is computed in 3 different ways.
 * Without IMU or Encoder: - distance is converted to a time for riding.
 * With IMU: - distance is measured by IMU.
 * With encoder: - distance is measured by Encoder.
 *
 * A fixed speed compensation PWM value to be subtracted can be specified.
 *
 *
 *  Copyright (C) 2019-2024  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */
#ifndef _PWM_DC_MOTOR_HPP
#define _PWM_DC_MOTOR_HPP

#include "PWMDcMotor.h"

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
//#define USE_SOFT_I2C_MASTER       // Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
#  if defined(USE_SOFT_I2C_MASTER)
#include "SoftI2CMasterConfig.h"
#include "SoftI2CMaster.h"
#  else
#include "Wire.h"
#  endif // defined(USE_SOFT_I2C_MASTER)
#endif // defined(USE_ADAFRUIT_MOTOR_SHIELD)

//#define TRACE
#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

char sDirectionCharArray[3] = { 'S', 'F', 'B' };
const char *sDirectionStringArray[4] = { "stop", "forward", "backward", "unknown" };

// Flags e.g. for display update control
#if defined(USE_MPU6050_IMU) || defined(USE_ENCODER_MOTOR_CONTROL)
volatile bool PWMDcMotor::SensorValuesHaveChanged; // true if encoder count and derived encoder speed, or one of the TMU data have changed
#endif
bool PWMDcMotor::MotorControlValuesHaveChanged; // true if DefaultStopMode, DriveSpeedPWM or SpeedPWMCompensation have changed
bool PWMDcMotor::MotorPWMHasChanged;              // true if CurrentCompensatedSpeedPWM has changed

PWMDcMotor::PWMDcMotor() { // @suppress("Class members should be properly initialized")
}

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
void PWMDcMotor::PCA9685WriteByte(uint8_t aAddress, uint8_t aData) {
#if defined(USE_SOFT_I2C_MASTER)
    i2c_start(PCA9685_DEFAULT_ADDRESS << 1);
    i2c_write(aAddress);
    i2c_write(aData);
    i2c_stop();
#else
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    Wire.write(aAddress);
    Wire.write(aData);
    Wire.endTransmission(true);
#endif
}

void PWMDcMotor::PCA9685SetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff) {
#if defined(USE_SOFT_I2C_MASTER)
    i2c_start(PCA9685_DEFAULT_ADDRESS << 1);
    i2c_write((PCA9685_FIRST_PWM_REGISTER) + 4 * aPin);
    i2c_write(aOn);
    i2c_write(aOn >> 8);
    i2c_write(aOff);
    i2c_write(aOff >> 8);
    i2c_stop();
#else
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    Wire.write((PCA9685_FIRST_PWM_REGISTER) + 4 * aPin);
    Wire.write(aOn);
    Wire.write(aOn >> 8);
    Wire.write(aOff);
    Wire.write(aOff >> 8);
    Wire.endTransmission(true);
#endif
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
#  endif // _USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD

/*
 * aMotorNumber from 1 to 2
 * Currently motors 3 and 4 are not required/supported by own library for Adafruit Motor Shield
 */
void PWMDcMotor::init(uint8_t aMotorNumber) {
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
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
#    if defined(USE_SOFT_I2C_MASTER)
    i2c_init(); // Initialize everything and check for bus lockup
#    else
    Wire.begin();
    Wire.setClock(400000);
#      if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(5000); // Sets timeout to 5 ms. default is 25 ms.
#      endif
#    endif

#if defined(TRACE)
    Serial.print(PWMPin);
    Serial.print(F(" MotorNumber="));
    Serial.println(aMotorNumber);
#endif
    // Reset PCA9685
#if defined(USE_SOFT_I2C_MASTER)
    i2c_start(PCA9685_GENERAL_CALL_ADDRESS << 1);
    i2c_write(PCA9685_SOFTWARE_RESET);
    i2c_stop();
#else
    Wire.beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
    Wire.write(PCA9685_SOFTWARE_RESET);
    Wire.endTransmission(true);
#endif
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
#if defined(ESP32)
    ledcAttachChannel(aPWMPin, ESP32_LEDC_MOTOR_CHANNEL_FREQUENCY, ESP32_LEDC_MOTOR_CHANNEL_RESOLUTION, ESP32_LEDC_MOTOR_CHANNEL); // 1000 Hz, 8 bit, channel 2
#endif
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
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
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

uint16_t PWMDcMotor::getMotorVoltageMillivoltforPWMAndMillivolt(uint8_t aSpeedPWM, uint16_t aFullBridgeInputVoltageMillivolt) {
    // if aFullBridgeInputVoltageMillivolt is constant, this can be optimized well
    return ((uint32_t) (aSpeedPWM * ((aFullBridgeInputVoltageMillivolt - FULL_BRIDGE_LOSS_MILLIVOLT))) / MAX_SPEED_PWM);
}

float PWMDcMotor::getMotorVoltageforPWM(uint8_t aSpeedPWM, float aFullBridgeInputVoltage) {
    return aSpeedPWM * ((aFullBridgeInputVoltage - (FULL_BRIDGE_LOSS_MILLIVOLT / 1000.0)) / MAX_SPEED_PWM);
}

void PWMDcMotor::printDirectionString(Print *aSerial, uint8_t aDirection) {
    if (aDirection > 3) {
        aDirection = 3;
    }
    aSerial->print(sDirectionStringArray[aDirection]);
}

/*
 * Sets active PWM and handles speed compensation and stop of motor
 *  @param  aRequestedSpeedPWM The 8-bit PWM value, 0 is off, 255 is on forward
 */
void PWMDcMotor::setSpeedPWM(uint8_t aRequestedSpeedPWM) {
    RequestedSpeedPWM = aRequestedSpeedPWM;
    if (aRequestedSpeedPWM == 0) {
        stop(STOP_MODE_KEEP);
        return;
    }
    /*
     * Handle speed compensation
     */
    uint8_t tCompensatedSpeedPWM;
    if (aRequestedSpeedPWM > SpeedPWMCompensation) {
        tCompensatedSpeedPWM = aRequestedSpeedPWM - SpeedPWMCompensation; // The only statement which sets CurrentCompensatedSpeedPWM to a value != 0
    } else {
        tCompensatedSpeedPWM = 0; // no stop mode here
    }
#if defined(TRACE)
        Serial.print(PWMPin);
        Serial.print(F(" RequestedSpeedPWM="));
        Serial.print(aRequestedSpeedPWM);
        Serial.print(F(" CurrentCompensatedSpeedPWM="));
        Serial.println(tCompensatedSpeedPWM);
#endif
    if (CurrentCompensatedSpeedPWM != tCompensatedSpeedPWM) {
        CurrentCompensatedSpeedPWM = tCompensatedSpeedPWM;
        MotorPWMHasChanged = true;
        /*
         * Write to hardware
         */
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
        PCA9685SetPWM(PWMPin, 0, 16 * tCompensatedSpeedPWM);
#  else
        Adafruit_MotorShield_DcMotor->setSpeedPWM(tCompensatedSpeedPWM);
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
    uint8_t tRequestedDirection = aRequestedDirection & DIRECTION_FORWARD_BACKWARD_MASK;
    bool tReturnValue = false;
    if (CurrentDirection != tRequestedDirection) {
        if (!isStopped()) {
            /*
             * Direction change requested but motor still running-> first stop motor
             */
            stop(STOP_MODE_BRAKE);
            tReturnValue = true;
        }
#if defined(LOCAL_DEBUG)
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
 *  @param  aRequestedSpeedPWM The 8-bit PWM value, 0 is off, 255 is on forward
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
        setSpeedPWM(aRequestedSpeedPWM); // output PWM value to motor
    }
}

/*
 * Signed speed
 *  @param  aRequestedSpeedPWM The 8-bit PWM value, 0 is off, 255 is on forward -255 is on backward
 */
void PWMDcMotor::setSpeedPWMAndDirection(int aSignedRequestedSpeedPWM) {
    uint8_t tDirection;
    if (aSignedRequestedSpeedPWM < 0) {
        aSignedRequestedSpeedPWM = -aSignedRequestedSpeedPWM;
        tDirection = DIRECTION_BACKWARD;
    } else {
        tDirection = DIRECTION_FORWARD;
    }

    if (aSignedRequestedSpeedPWM > MAX_SPEED_PWM) {
        aSignedRequestedSpeedPWM = MAX_SPEED_PWM;
    }
    setSpeedPWMAndDirection(aSignedRequestedSpeedPWM, tDirection);
}

/*
 * Starts motor using current drive speed
 */
void PWMDcMotor::start(uint8_t aRequestedDirection) {
    setSpeedPWMAndDirection(DriveSpeedPWM, aRequestedDirection);
}

/*
 *  RequestedSpeedPWM == 0, should be equivalent to MotorRampState == MOTOR_STATE_STOPPED
 */
bool PWMDcMotor::isStopped() {
    return (RequestedSpeedPWM == 0);
}

/*
 * First set PWM to 0 then set driver to stop mode
 * @param aStopMode STOP_MODE_KEEP (take previously defined DefaultStopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
 */
void PWMDcMotor::stop(uint8_t aStopMode) {
    RequestedSpeedPWM = 0;
    CurrentCompensatedSpeedPWM = 0;
    MotorPWMHasChanged = true;
    CheckStopConditionInUpdateMotor = false;
#if !defined(DO_NOT_SUPPORT_RAMP)
    MotorRampState = MOTOR_STATE_STOPPED;
#endif

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    PCA9685SetPWM(PWMPin, 0, 0);
#  else
    Adafruit_MotorShield_DcMotor->setSpeedPWM(0);
#  endif
#else
    analogWrite(PWMPin, 0);
#endif
    if (aStopMode == STOP_MODE_KEEP) {
        aStopMode = DefaultStopMode;
    }
    setMotorDriverMode(aStopMode);
#if defined(LOCAL_DEBUG)
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
    DriveSpeedPWMFor2Volt = DEFAULT_DRIVE_SPEED_PWM;
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
 * @param aFullBridgeInputVoltageMillivolt is used to compute the (reference) PWM for 2 volt.
 * Formula is: 2VPWM = (2000mV / tBridgeMillivolt) * MAX_SPEED_PWM
 */
void PWMDcMotor::setDriveSpeedPWMFor2Volt(uint16_t aFullBridgeInputVoltageMillivolt) {
    uint16_t tBridgeMillivolt = aFullBridgeInputVoltageMillivolt - FULL_BRIDGE_LOSS_MILLIVOLT;
    DriveSpeedPWMFor2Volt = (2000 * MAX_SPEED_PWM) / tBridgeMillivolt;
    DriveSpeedPWM = DriveSpeedPWMFor2Volt;
    MotorControlValuesHaveChanged = true;
}
void PWMDcMotor::setDriveSpeedPWMFor2Volt(float aFullBridgeInputVoltage) {
    float tBridgeVolt = aFullBridgeInputVoltage - (FULL_BRIDGE_LOSS_MILLIVOLT / 1000.0);
    DriveSpeedPWMFor2Volt = (2 * MAX_SPEED_PWM) / tBridgeVolt;
    DriveSpeedPWM = DriveSpeedPWMFor2Volt;
    MotorControlValuesHaveChanged = true;
}

/*
 * Can be used to get real value for DEFAULT_START_SPEED_PWM, etc. which are computed using the value FULL_BRIDGE_OUTPUT_MILLIVOLT
 */
uint8_t PWMDcMotor::getVoltageAdjustedSpeedPWM(uint8_t aSpeedPWM, uint16_t aFullBridgeInputVoltageMillivolt) {
    uint16_t tSpeedPWM = ((uint32_t) (aSpeedPWM * FULL_BRIDGE_OUTPUT_MILLIVOLT))
            / (aFullBridgeInputVoltageMillivolt - FULL_BRIDGE_LOSS_MILLIVOLT);
    if (tSpeedPWM > MAX_SPEED_PWM) {
        return MAX_SPEED_PWM;
    }
    return tSpeedPWM;
}
uint8_t PWMDcMotor::getVoltageAdjustedSpeedPWM(uint8_t aSpeedPWM, float aFullBridgeInputVoltage) {
    uint16_t tSpeedPWM = (aSpeedPWM * (FULL_BRIDGE_OUTPUT_MILLIVOLT / 1000.0))
            / (aFullBridgeInputVoltage - (FULL_BRIDGE_LOSS_MILLIVOLT / 1000.0));
    if (tSpeedPWM > MAX_SPEED_PWM) {
        return MAX_SPEED_PWM;
    }
    return tSpeedPWM;
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
 * Update DriveSpeedPWM value, i.e. if moving set new RequestedSpeedPWM and apply it to the motor
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
 * If motor was stooped or changed direction, starts ramp if enabled
 * Else call setSpeedPWMAndDirection() directly, which sets CurrentCompensatedSpeedPWM
 */
void PWMDcMotor::setSpeedPWMAndDirectionWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection) {
#if defined(LOCAL_DEBUG)
    Serial.print(PWMPin);
    Serial.print(F(" Set PWM to "));
    Serial.print(aRequestedSpeedPWM);
    Serial.print(F(" Dir="));
    Serial.print(sDirectionCharArray[aRequestedDirection]);
    Serial.print(F(" CurrentCompensatedSpeedPWM="));
    Serial.print(CurrentCompensatedSpeedPWM);
    Serial.print(F(" MotorMode="));
    Serial.print(sDirectionCharArray[CurrentDirection]);
    Serial.println();
#endif
#if defined(DO_NOT_SUPPORT_RAMP)
    setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection); // reduced to setSpeedPWMAndDirection()
#else
    if (aRequestedSpeedPWM <= RAMP_UP_VALUE_OFFSET_SPEED_PWM) {
        // Here ramp makes no sense, since requested PWM does not lead to spinning wheels
        setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
    } else {
        if (MotorRampState == MOTOR_STATE_DRIVE && CurrentDirection == aRequestedDirection) {
            /*
             * motor is running -> just change drive speed
             */
            setSpeedPWMAndDirection(aRequestedSpeedPWM, aRequestedDirection);
        } else {
            checkAndHandleDirectionChange(aRequestedDirection);
            /*
             * Stopped here, now set target speed for ramp up
             */
            MotorRampState = MOTOR_STATE_START;
            RequestedDriveSpeedPWM = aRequestedSpeedPWM;
        }
#  if defined(LOCAL_DEBUG)
        Serial.print(F("MotorRampState="));
        Serial.print(MotorRampState);
        Serial.println();
#  endif
    }
#endif // defined(DO_NOT_SUPPORT_RAMP)
}

/*
 * Takes the current DriveSpeedPWM for ramp up
 */
void PWMDcMotor::startRampUp(uint8_t aRequestedDirection) {
    setSpeedPWMAndDirectionWithRamp(DriveSpeedPWM, aRequestedDirection);
}

/*
 * Starts ramp down immediately
 * Its a copy of the code in updateMotor() but this is fairly good optimized :-)
 */
void PWMDcMotor::startRampDown() {
#if defined(DO_NOT_SUPPORT_RAMP)
    stop(STOP_MODE_KEEP);
#else
    //  --> RAMP_DOWN
    MotorRampState = MOTOR_STATE_RAMP_DOWN;
    uint8_t tNewSpeedPWM;
    if (RequestedSpeedPWM > RAMP_DOWN_VALUE_OFFSET_SPEED_PWM) {
        tNewSpeedPWM = RequestedSpeedPWM - RAMP_DOWN_VALUE_OFFSET_SPEED_PWM;
    } else {
        tNewSpeedPWM = RAMP_VALUE_MIN_SPEED_PWM;
    }

#  if defined(TRACE)
    Serial.print(PWMPin);
    Serial.print(F(" St="));
    Serial.print(MotorRampState);
    Serial.print(F(" Ns="));
    Serial.print(tNewSpeedPWM);
#  endif
    PWMDcMotor::setSpeedPWM(tNewSpeedPWM);
    NextRampChangeMillis = millis() + RAMP_INTERVAL_MILLIS;
#endif
}

/*
 * Guarantees, that both motors start ramp down at the same time
 */
void PWMDcMotor::synchronizeRampDown(PWMDcMotor *aOtherMotorControl) {
#if !defined(DO_NOT_SUPPORT_RAMP)
    if (MotorRampState == MOTOR_STATE_RAMP_DOWN && aOtherMotorControl->MotorRampState == MOTOR_STATE_DRIVE) {
        aOtherMotorControl->startRampDown();
    }
    if (MotorRampState == MOTOR_STATE_DRIVE && aOtherMotorControl->MotorRampState == MOTOR_STATE_RAMP_DOWN) {
        startRampDown();
    }
#else
    (void) aOtherMotorControl;
#endif
}

#if !defined(USE_ENCODER_MOTOR_CONTROL)
/*
 * @return true if not stopped (motor expects another update)
 */
bool PWMDcMotor::updateMotor() {
    uint8_t tNewSpeedPWM = RequestedSpeedPWM;

    /*
     * Check if target milliseconds are reached
     */
    if (tNewSpeedPWM > 0) {
        if (CheckStopConditionInUpdateMotor && millis() > computedMillisOfMotorStopForDistance) {
            stop(STOP_MODE_KEEP); // resets CheckStopConditionInUpdateMotor and sets MOTOR_STATE_STOPPED;
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
        if (RequestedDriveSpeedPWM > RAMP_UP_VALUE_OFFSET_SPEED_PWM) {
            // Start with ramp to avoid spinning wheels
            tNewSpeedPWM = RAMP_UP_VALUE_OFFSET_SPEED_PWM; // start immediately with speed offset (3 volt)
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
         * Only used if RequestedDriveSpeedPWM > RAMP_UP_VALUE_OFFSET_SPEED_PWM to avoid spinning wheels
         */
        if (tMillis >= NextRampChangeMillis) {
            NextRampChangeMillis += RAMP_INTERVAL_MILLIS;
            tNewSpeedPWM = tNewSpeedPWM + RAMP_UP_VALUE_DELTA;
            /*
             * Transition criteria is: RequestedDriveSpeedPWM reached.
             * Then check immediately for timeout
             */
            // Clip value and check for 8 bit overflow
            if (tNewSpeedPWM >= RequestedDriveSpeedPWM || tNewSpeedPWM <= RAMP_UP_VALUE_DELTA) {
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
        if (CheckStopConditionInUpdateMotor && millis() >= computedMillisOfMotorStopForDistance) {
            /*
             * "Distance" reached -> stop now
             */

            if (tNewSpeedPWM > RAMP_DOWN_VALUE_OFFSET_SPEED_PWM) {
                // Stop with ramp to avoid blocking wheels
                tNewSpeedPWM -= (RAMP_DOWN_VALUE_OFFSET_SPEED_PWM - RAMP_DOWN_VALUE_DELTA); // RAMP_VALUE_DELTA is immediately subtracted below
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
             * until RAMP_VALUE_MIN_SPEED_PWM is reached
             */
            if (tNewSpeedPWM == RAMP_VALUE_MIN_SPEED_PWM) {
                /*
                 * Ramp ended, last value was RAMP_VALUE_MIN_SPEED_PWM
                 */
                if (!CheckStopConditionInUpdateMotor) {
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
        Serial.print(PWMPin);
        Serial.print(F(" St="));
        Serial.print(MotorRampState);
        Serial.print(F(" Ns="));
        Serial.println(tNewSpeedPWM);
#endif
        PWMDcMotor::setSpeedPWM(tNewSpeedPWM); // sets MOTOR_STATE_STOPPED if speed is 0
    }
#endif // #if defined(DO_NOT_SUPPORT_RAMP)
    return (RequestedSpeedPWM > 0); // current requested speed == 0
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

/*
 * Blocking call
 * @param aRequestedDistanceMillimeter positive is forward, negative is backward
 */
void PWMDcMotor::goDistanceMillimeter(int aRequestedDistanceMillimeter) {
    uint8_t tRequestedDirection = DIRECTION_FORWARD;
    if (aRequestedDistanceMillimeter < 0) {
        tRequestedDirection = DIRECTION_BACKWARD;
    }
    goDistanceMillimeter(DriveSpeedPWMFor2Volt, aRequestedDistanceMillimeter, tRequestedDirection);
}

/*
 * Blocking call
 */
void PWMDcMotor::goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    goDistanceMillimeter(DriveSpeedPWMFor2Volt, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * Blocking call
 */
void PWMDcMotor::goDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    startGoDistanceMillimeterWithSpeed(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
#if defined(DO_NOT_SUPPORT_RAMP)
    delay(computedMillisOfMotorForDistance);
#else
    while (millis() <= computedMillisOfMotorStopForDistance) {
        updateMotor();
    }
#endif
    stop(STOP_MODE_KEEP);
}

void PWMDcMotor::startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection) {
    startGoDistanceMillimeterWithSpeed(DriveSpeedPWMFor2Volt, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * Signed DistanceCount
 */
void PWMDcMotor::startGoDistanceMillimeter(int aRequestedDistanceMillimeter) {
    if (aRequestedDistanceMillimeter < 0) {
        aRequestedDistanceMillimeter = -aRequestedDistanceMillimeter;
        startGoDistanceMillimeterWithSpeed(DriveSpeedPWMFor2Volt, aRequestedDistanceMillimeter, DIRECTION_BACKWARD);
    } else {
        startGoDistanceMillimeterWithSpeed(DriveSpeedPWMFor2Volt, aRequestedDistanceMillimeter, DIRECTION_FORWARD);
    }
}

void PWMDcMotor::startGoDistanceMillimeterWithSpeed(uint8_t aRequestedSpeedPWM, int aRequestedDistanceMillimeter) {
    if (aRequestedDistanceMillimeter < 0) {
        aRequestedDistanceMillimeter = -aRequestedDistanceMillimeter;
        startGoDistanceMillimeterWithSpeed(aRequestedSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_BACKWARD);
    } else {
        startGoDistanceMillimeterWithSpeed(aRequestedSpeedPWM, aRequestedDistanceMillimeter, DIRECTION_FORWARD);
    }
}

uint32_t PWMDcMotor::convertMillimeterToMillis(uint8_t aSpeedPWM, unsigned int aRequestedDistanceMillimeter) {
    return ((uint32_t) aRequestedDistanceMillimeter * MillisPerCentimeter * DriveSpeedPWMFor2Volt)
            / ((uint_fast16_t) MILLIMETER_IN_ONE_CENTIMETER * aSpeedPWM);
}

unsigned int PWMDcMotor::convertMillisToMillimeter(uint8_t aSpeedPWM, unsigned int aMillis) {
    return ((uint32_t) aMillis * aSpeedPWM * MILLIMETER_IN_ONE_CENTIMETER)
            / ((uint_fast16_t) MillisPerCentimeter * DriveSpeedPWMFor2Volt);
}

unsigned int PWMDcMotor::convertMillisToCentimeterFor2Volt(unsigned int aMillis) {
    return aMillis / MillisPerCentimeter;
}

void PWMDcMotor::startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    startGoDistanceMillimeterWithSpeed(aRequestedSpeedPWM, aRequestedDistanceMillimeter, aRequestedDirection);
}

/*
 * If motor is already running, just update speed and new stop time
 */
void PWMDcMotor::startGoDistanceMillimeterWithSpeed(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
        uint8_t aRequestedDirection) {
    if (aRequestedDistanceMillimeter == 0) {
        stop(STOP_MODE_BRAKE); // In case motor was running
        return;
    }

    /*
     * Estimate duration for given distance
     * use 32 bit intermediate to avoid overflow (this also saves around 50 bytes of program memory by using slower functions instead of faster inline code)
     */
    unsigned int tDistanceMillimeterDriveSpeed; // The distance we drive with nominal speed i.e. without the start / stop acceleration happening in one cm
    if (aRequestedDistanceMillimeter > 10) {
        tDistanceMillimeterDriveSpeed = aRequestedDistanceMillimeter - 10;
    } else {
        tDistanceMillimeterDriveSpeed = 0;
    }
    /*
     * Compute milliseconds for distance by using MillisPerCentimeter (which is measured for 2 volt)
     * and scaling it for the requested aRequestedSpeedPWM
     */
    uint32_t tComputedMillisOfMotorForDistance = convertMillimeterToMillis(aRequestedSpeedPWM, tDistanceMillimeterDriveSpeed);

    if (isStopped()) {
        // add startup time for the centimeter, we subtracted above
        tComputedMillisOfMotorForDistance += DEFAULT_MILLIS_FOR_FIRST_CENTIMETER;
    }
    // after check of isStopped(), set PWM
    setSpeedPWMAndDirectionWithRamp(aRequestedSpeedPWM, aRequestedDirection);

#if defined(LOCAL_DEBUG)
    Serial.print(F("Go for distance "));
    Serial.print(aRequestedDistanceMillimeter);
    Serial.print(F(" mm -> "));
    Serial.print(tComputedMillisOfMotorForDistance);
    Serial.println(F(" ms"));
#endif

    computedMillisOfMotorForDistance = tComputedMillisOfMotorForDistance;
    computedMillisOfMotorStopForDistance = tComputedMillisOfMotorForDistance + millis();
    CheckStopConditionInUpdateMotor = true;
}

#endif // !defined(USE_ENCODER_MOTOR_CONTROL)

/*
 * @return true, if both values were valid
 */
bool PWMDcMotor::readMotorValuesFromInfoStructure(EepromMotorInfoStruct *aEepromMotorInfo) {
    /*
     * Overwrite with EEPROM values if valid
     */
    if (aEepromMotorInfo->DriveSpeedPWMFor2Volt < 222 && aEepromMotorInfo->DriveSpeedPWMFor2Volt > 40
            && aEepromMotorInfo->SpeedPWMCompensation < 24) {
        DriveSpeedPWMFor2Volt = aEepromMotorInfo->DriveSpeedPWMFor2Volt;
        SpeedPWMCompensation = aEepromMotorInfo->SpeedPWMCompensation;
        MotorControlValuesHaveChanged = true;
        return true;
    }
    return false;
}

void PWMDcMotor::writeMotorValuesToInfoStructure(EepromMotorInfoStruct *aEepromMotorInfo) {
    aEepromMotorInfo->DriveSpeedPWMFor2Volt = DriveSpeedPWMFor2Volt;
    aEepromMotorInfo->SpeedPWMCompensation = SpeedPWMCompensation;
}

/********************************************************************************************
 * EEPROM functions
 * Uses the start of EEPROM for storage of EepromMotorInfoStruct's for motor number 1 to n
 ********************************************************************************************/
void PWMDcMotor::readMotorValuesFromEeprom(uint8_t aMotorValuesEepromStorageNumber) {
#if defined(E2END)
    EepromMotorInfoStruct tEepromMotorInfo;
    eeprom_read_block((void*) &tEepromMotorInfo, (void*) ((aMotorValuesEepromStorageNumber) * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));

    readMotorValuesFromInfoStructure(&tEepromMotorInfo);
#endif // defined(E2END)
}

void PWMDcMotor::writeMotorValuesToEeprom(uint8_t aMotorValuesEepromStorageNumber) {
#if defined(E2END)
    EepromMotorInfoStruct tEepromMotorInfo;
    writeMotorValuesToInfoStructure(&tEepromMotorInfo);

    eeprom_write_block((void*) &tEepromMotorInfo, (void*) ((aMotorValuesEepromStorageNumber) * sizeof(EepromMotorInfoStruct)),
            sizeof(EepromMotorInfoStruct));
#endif // defined(E2END)
}

void PWMDcMotor::printValues(Print *aSerial) {
#if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    aSerial->print(PWMPin);
#endif
    aSerial->print(F(" CurrentCompensatedSpeedPWM="));
    aSerial->print(CurrentCompensatedSpeedPWM);
    aSerial->print(F(" DriveSpeedPWM="));
    aSerial->print(DriveSpeedPWM);
    aSerial->print(F(" DriveSpeedPWMFor2Volt="));
    aSerial->print(DriveSpeedPWMFor2Volt);
    aSerial->print(F(" SpeedPWMCompensation="));
    aSerial->print(SpeedPWMCompensation);
    aSerial->print(F(" CurrentDirection="));
    aSerial->print(sDirectionCharArray[CurrentDirection]);
    aSerial->println();
}

const char StringNot[] PROGMEM = {" not"};
const char StringDefined[] PROGMEM = {" defined"};

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
    aSerial->print(F("_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD:"));
#if !defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));
#endif

    aSerial->print(F("FULL_BRIDGE_OUTPUT_MILLIVOLT="));
    aSerial->print(FULL_BRIDGE_OUTPUT_MILLIVOLT);
    aSerial->println(
            F(
                    "mV (= FULL_BRIDGE_INPUT_MILLIVOLT|" STR(FULL_BRIDGE_INPUT_MILLIVOLT) "mV - FULL_BRIDGE_LOSS_MILLIVOLT|" STR(FULL_BRIDGE_LOSS_MILLIVOLT "mV)")));

    aSerial->print(F("DEFAULT_START_SPEED_PWM="));
    aSerial->print(DEFAULT_START_SPEED_PWM);
    aSerial->print(F(", DEFAULT_DRIVE_SPEED_PWM="));
    aSerial->println(DEFAULT_DRIVE_SPEED_PWM);

    aSerial->println(F("DEFAULT_MILLIS_FOR_FIRST_CENTIMETER=" STR(DEFAULT_MILLIS_FOR_FIRST_CENTIMETER)));

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

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _PWM_DC_MOTOR_HPP
