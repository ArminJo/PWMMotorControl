/*
 * PWMDcMotor.cpp
 *
 * Low level motor control for Adafruit_MotorShield OR breakout board with TB6612 or L298 driver IC for two DC motors.
 *
 * Motor control has 2 parameters:
 * 1. Speed / PWM which is ignored for BRAKE or RELEASE. This library also accepts signed speed (including the direction as sign).
 * 2. Direction / MotorDriverMode. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 *
 *  Created on: 12.05.2019
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

#include "PWMDcMotor.h"

bool PWMDcMotor::MotorValuesHaveChanged; // for printing

PWMDcMotor::PWMDcMotor() { // @suppress("Class members should be properly initialized")
}

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
void PWMDcMotor::I2CWriteByte(uint8_t aAddress, uint8_t aData) {
    Wire.beginTransmission(0x60);
    Wire.write(aAddress);
    Wire.write(aData);
    Wire.endTransmission();
}

void PWMDcMotor::I2CSetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff) {
    Wire.beginTransmission(0x60);
    Wire.write((PCA9685_FIRST_PWM_REGISTER) + 4 * aPin);
    Wire.write(aOn);
    Wire.write(aOn >> 8);
    Wire.write(aOff);
    Wire.write(aOff >> 8);
    Wire.endTransmission();
}

void PWMDcMotor::I2CSetPin(uint8_t aPin, bool aSetToOn) {
    if (aSetToOn) {
        I2CSetPWM(aPin, 4096, 0);
    } else {
        I2CSetPWM(aPin, 0, 0);
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
void PWMDcMotor::init(uint8_t aMotorNumber, bool aReadFromEeprom) {

#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
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
#    if defined (ARDUINO_ARCH_AVR) // Other platforms do not have this new function
    Wire.setWireTimeout(5000); // Sets timeout to 5 ms. default is 25 ms.
#    endif
    // Reset PCA9685
    Wire.beginTransmission(PCA9685_GENERAL_CALL_ADDRESS);
    Wire.write(PCA9685_SOFTWARE_RESET);
    Wire.endTransmission();
    // Set expander to 1600 HZ
    I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_SLEEP)); // go to sleep
    I2CWriteByte(PCA9685_PRESCALE_REGISTER, PCA9685_PRESCALER_FOR_1600_HZ); // set the prescaler
    delay(2); // > 500 us before the restart bit according to datasheet
    I2CWriteByte(PCA9685_MODE1_REGISTER, _BV(PCA9685_MODE_1_RESTART) | _BV(PCA9685_MODE_1_AUTOINCREMENT)); // reset sleep and enable auto increment

#  else
    Adafruit_MotorShield_DcMotor = sAdafruitMotorShield.getMotor(aMotorNumber);
    sAdafruitMotorShield.begin();
#  endif

    // set defaults
    setDefaultsForFixedDistanceDriving();

    if (!aReadFromEeprom) {
        MotorValuesEepromStorageNumber = 0;
    } else {
        MotorValuesEepromStorageNumber = aMotorNumber;
        readMotorValuesFromEeprom();
    }
    stop(DEFAULT_STOP_MODE);
}

#else // USE_ADAFRUIT_MOTOR_SHIELD
/*
 * @param aMotorNumber if 0 do not read from EEPROM, otherwise block number
 */
void PWMDcMotor::init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aMotorNumber) {
    MotorValuesEepromStorageNumber = aMotorNumber;
    ForwardPin = aForwardPin;
    BackwardPin = aBackwardPin;
    PWMPin = aPWMPin;
    DefaultStopMode = MOTOR_RELEASE;

    pinMode(aForwardPin, OUTPUT);
    pinMode(aBackwardPin, OUTPUT);
    pinMode(aPWMPin, OUTPUT);

    // set defaults
    setDefaultsForFixedDistanceDriving();

    readMotorValuesFromEeprom(); // checks MotorValuesEepromStorageNumber
    stop(DEFAULT_STOP_MODE);
}

#endif // USE_ADAFRUIT_MOTOR_SHIELD

/*
 *  @brief  Control the DC motor driver direction and stop mode
 *  @param  aMotorDriverMode The mode can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 */
void PWMDcMotor::setMotorDriverMode(uint8_t aMotorDriverMode) {
    CurrentDirectionOrBrakeMode = aMotorDriverMode; // The only statement which changes CurrentDirectionOrBrakeMode
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // until here DIRECTION_FORWARD is 0 back is 1, Adafruit library starts with 1
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    switch (aMotorDriverMode) {
    case DIRECTION_FORWARD:
        I2CSetPin(BackwardPin, LOW); // take low first to avoid 'break'
        I2CSetPin(ForwardPin, HIGH);
        break;
    case DIRECTION_BACKWARD:
        I2CSetPin(ForwardPin, LOW); // take low first to avoid 'break'
        I2CSetPin(BackwardPin, HIGH);
        break;
    case MOTOR_BRAKE:
        I2CSetPin(ForwardPin, HIGH);
        I2CSetPin(BackwardPin, HIGH);
        break;
    case MOTOR_RELEASE:
        I2CSetPin(ForwardPin, LOW);
        I2CSetPin(BackwardPin, LOW);
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
    case MOTOR_BRAKE:
        digitalWrite(ForwardPin, HIGH);
        digitalWrite(BackwardPin, HIGH);
        break;
    case MOTOR_RELEASE:
        digitalWrite(ForwardPin, LOW);
        digitalWrite(BackwardPin, LOW);
        break;
    }
#endif // USE_ADAFRUIT_MOTOR_SHIELD
}

/*
 * @return true if direction has changed and motor has stopped
 */
bool PWMDcMotor::checkAndHandleDirectionChange(uint8_t aRequestedDirection) {
    bool tReturnValue = false;
    uint8_t tRequestedDirection = aRequestedDirection & DIRECTION_MASK;
    if (CurrentDirectionOrBrakeMode != tRequestedDirection) {
        if (CurrentSpeed != 0) {
#ifdef DEBUG
            Serial.print(F("Motor mode change to "));
            Serial.println(tRequestedDirection);
#endif
            /*
             * Direction change requested but motor still running-> first stop motor
             */
            stop(MOTOR_BRAKE);
            tReturnValue = true;
        }
        setMotorDriverMode(tRequestedDirection); // this in turn sets CurrentDirectionOrBrakeMode
    }
    return tReturnValue;
}

/*
 *  @brief  Control the DC Motor speed/throttle
 *  @param  speed The 8-bit PWM value, 0 is off, 255 is on forward -255 is on backward
 *  First set driver mode, then set PWM
 */
void PWMDcMotor::setSpeed(uint8_t aSpeedRequested, uint8_t aRequestedDirection) {
    if (aSpeedRequested == 0) {
        stop(DefaultStopMode);
    } else {
        checkAndHandleDirectionChange(aRequestedDirection);
        MotorValuesHaveChanged = true;
        if (CurrentSpeed != aSpeedRequested) {
            CurrentSpeed = aSpeedRequested; // The only statement which sets CurrentSpeed to a value != 0
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
            I2CSetPWM(PWMPin, 0, 16 * aSpeedRequested);
#  else
            Adafruit_MotorShield_DcMotor->setSpeed(aSpeedRequested);
#  endif
#else
            analogWrite(PWMPin, aSpeedRequested);
#endif
        }
    }
}

/*
 * Subtracts SpeedCompensation from aRequestedSpeed before applying
 */
void PWMDcMotor::setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    // avoid underflow
    uint8_t tCurrentSpeed;
    if (aRequestedSpeed > SpeedCompensation) {
        tCurrentSpeed = aRequestedSpeed - SpeedCompensation;
    } else {
        tCurrentSpeed = 0;
    }
    setSpeed(tCurrentSpeed, aRequestedDirection); // output PWM value to motor
}

/*
 * Signed speed
 */
void PWMDcMotor::setSpeed(int aSpeedRequested) {
    if (aSpeedRequested < 0) {
        aSpeedRequested = -aSpeedRequested;
        setSpeed(aSpeedRequested, DIRECTION_BACKWARD);
    } else {
        setSpeed(aSpeedRequested, DIRECTION_FORWARD);
    }
}

void PWMDcMotor::setSpeedCompensated(int aRequestedSpeed) {
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
 * First set PWM to 0 then disable driver
 * @param aStopMode STOP_MODE_KEEP (take previously defined DefaultStopMode) or MOTOR_BRAKE or MOTOR_RELEASE
 */
void PWMDcMotor::stop(uint8_t aStopMode) {

    CurrentSpeed = 0; // The only statement which sets CurrentSpeed to 0
    MotorValuesHaveChanged = true;
    MotorMovesFixedDistance = false;
#ifdef SUPPORT_RAMP_UP
    MotorRampState = MOTOR_STATE_STOPPED;
#endif
#ifndef USE_ENCODER_MOTOR_CONTROL
    MotorMovesFixedDistance = false;
#endif
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    I2CSetPWM(PWMPin, 0, 0);
#  else
        Adafruit_MotorShield_DcMotor->setSpeed(0);
#  endif
#else
    analogWrite(PWMPin, 0);
#endif
    if (aStopMode == STOP_MODE_KEEP) {
        aStopMode = DefaultStopMode;
    }
    setMotorDriverMode(CheckStopMODE(aStopMode));
}

/*
 * @param aStopMode used for speed == 0 or STOP_MODE_KEEP: MOTOR_BRAKE or MOTOR_RELEASE
 */
void PWMDcMotor::setStopMode(uint8_t aStopMode) {
    DefaultStopMode = CheckStopMODE(aStopMode);
}

/******************************************************************************************
 * Distance functions
 *****************************************************************************************/
/*
 * StartSpeed (at which car starts to move) for 8 volt is appr. 35 to 40, for 4.3 volt (USB supply) is appr. 90 to 100
 */
void PWMDcMotor::setDefaultsForFixedDistanceDriving() {
    StartSpeed = DEFAULT_START_SPEED;
    DriveSpeed = DEFAULT_DRIVE_SPEED;
    SpeedCompensation = 0;
#ifndef USE_ENCODER_MOTOR_CONTROL
    DistanceToTimeFactor = DEFAULT_DISTANCE_TO_TIME_FACTOR;
#endif
}

void PWMDcMotor::setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, uint8_t aSpeedCompensation) {
    StartSpeed = aStartSpeed;
    DriveSpeed = aDriveSpeed;
    SpeedCompensation = aSpeedCompensation;
}

void PWMDcMotor::setDriveSpeed(uint8_t aDriveSpeed) {
    DriveSpeed = aDriveSpeed;
}

#ifdef SUPPORT_RAMP_UP
void PWMDcMotor::startRampUp(uint8_t aRequestedSpeed, uint8_t aRequestedDirection) {
    checkAndHandleDirectionChange(aRequestedDirection);
    if (MotorRampState == MOTOR_STATE_STOPPED) {
        MotorRampState = MOTOR_STATE_START;

        // avoid underflow
        uint8_t tCurrentDriveSpeed;
        if (aRequestedSpeed > SpeedCompensation) {
            tCurrentDriveSpeed = aRequestedSpeed - SpeedCompensation;
        } else {
            tCurrentDriveSpeed = 0;
        }
        CurrentDriveSpeed = tCurrentDriveSpeed;
    } else if (MotorRampState == MOTOR_STATE_DRIVE_SPEED) {
        setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
    }
}

void PWMDcMotor::startRampUp(uint8_t aRequestedDirection) {
    startRampUp(DriveSpeed, aRequestedDirection);
}
#endif

#if !defined(USE_ENCODER_MOTOR_CONTROL)
/*
 * Required for non encoder motors to estimate duration for a fixed distance
 */
void PWMDcMotor::setDistanceToTimeFactorForFixedDistanceDriving(unsigned int aDistanceToTimeFactor) {
    DistanceToTimeFactor = aDistanceToTimeFactor;
}

/*
 * @param aRequestedDistanceCount distance in 5mm resolution (to be compatible with 20 slot encoder discs and 20 cm wheel circumference)
 * If motor is already running just update speed and new time
 */
void PWMDcMotor::startGoDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection) {
//    if (aRequestedDistanceCount > DEFAULT_COUNTS_PER_FULL_ROTATION * 10) {
//        PanicWithLed(400, 22);
//    }
    if (aRequestedDistanceCount == 0) {
        return;
    }

    if (CurrentSpeed == 0) {
#ifdef SUPPORT_RAMP_UP
        MotorRampState = MOTOR_STATE_START;
        CurrentDriveSpeed = aRequestedSpeed;
        setMotorDriverMode(aRequestedDirection); // this in turn sets CurrentDirectionOrBrakeMode
#else
        setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
#endif
        /*
         * Estimate duration for given distance
         * use 32 bit intermediate to avoid overflow (this also saves around 50 bytes of program memory by using slower functions instead of faster inline code)
         */
        computedMillisOfMotorStopForDistance = millis() + DEFAULT_MOTOR_START_UP_TIME_MILLIS
                + (10 * (((uint32_t) aRequestedDistanceCount * DistanceToTimeFactor) / DriveSpeed));
    } else {
#ifdef SUPPORT_RAMP_UP
        MotorRampState = MOTOR_STATE_START;
        CurrentDriveSpeed = aRequestedSpeed;
#endif
        setSpeedCompensated(aRequestedSpeed, aRequestedDirection);
        /*
         * Estimate duration for given distance
         * use 32 bit intermediate to avoid overflow (this also saves around 50 bytes of program memory by using slower functions instead of faster inline code)
         */
        computedMillisOfMotorStopForDistance = millis()
                + (10 * (((uint32_t) aRequestedDistanceCount * DistanceToTimeFactor) / DriveSpeed));
    }
    MotorMovesFixedDistance = true;
}

void PWMDcMotor::startGoDistanceCount(unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection) {
    startGoDistanceCount(DriveSpeed, aRequestedDistanceCount, aRequestedDirection);
}

/*
 * Signed DistanceCount
 */
void PWMDcMotor::startGoDistanceCount(int aRequestedDistanceCount) {
    if (aRequestedDistanceCount < 0) {
        aRequestedDistanceCount = -aRequestedDistanceCount;
        startGoDistanceCount(DriveSpeed, aRequestedDistanceCount, DIRECTION_BACKWARD);
    } else {
        startGoDistanceCount(DriveSpeed, aRequestedDistanceCount, DIRECTION_FORWARD);
    }
}

/*
 * Not used by CarControl
 */
void PWMDcMotor::goDistanceCount(unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection) {
    goDistanceCount(DriveSpeed, aRequestedDistanceCount, aRequestedDirection);
}

void PWMDcMotor::goDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection) {
    startGoDistanceCount(aRequestedSpeed, aRequestedDistanceCount, aRequestedDirection);
    while (millis() <= computedMillisOfMotorStopForDistance) {
#ifdef SUPPORT_RAMP_UP
        updateMotor();
#endif
    }
    stop(DefaultStopMode);
}

/*
 * @return true if not stopped (motor expects another update)
 */
bool PWMDcMotor::updateMotor() {
#ifdef SUPPORT_RAMP_UP
    unsigned long tMillis = millis();
    uint8_t tNewSpeed = CurrentSpeed;

    if (MotorRampState == MOTOR_STATE_START) {
        //  --> RAMP_UP
        MotorRampState = MOTOR_STATE_RAMP_UP;
        /*
         * Set ramp values, 16 steps a 16 millis for ramp up => 256 milliseconds
         */
        NextRampChangeMillis = tMillis + RAMP_UP_UPDATE_INTERVAL_MILLIS;
        RampDelta = RAMP_UP_VALUE_DELTA; // ((CurrentDriveSpeed - StartSpeed) / RAMP_UP_UPDATE_INTERVAL_STEPS)
        if (RampDelta < 2) {
            RampDelta = 2;
        }
        /*
         * Start motor
         */
        tNewSpeed = StartSpeed;
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
            if (tNewSpeed == CurrentDriveSpeed) {
                //  --> DRIVE_SPEED
                MotorRampState = MOTOR_STATE_DRIVE_SPEED;
            }
        }
    }
    // End of motor state machine

    if (tNewSpeed != CurrentSpeed) {
        PWMDcMotor::setSpeed(tNewSpeed, CurrentDirectionOrBrakeMode);
    }
#endif

    /*
     * Check if target milliseconds are reached
     */
    if (CurrentSpeed > 0) {
        if (MotorMovesFixedDistance && millis() > computedMillisOfMotorStopForDistance) {
            stop(DefaultStopMode); // resets MotorMovesFixedDistance
            return false;
        }
    }
    return true;
}
#endif // !defined(USE_ENCODER_MOTOR_CONTROL)

/********************************************************************************************
 * EEPROM functions
 * uUses the start of EEPROM for storage of EepromMotorInfoStruct's for motor number 1 to n
 ********************************************************************************************/
void PWMDcMotor::readMotorValuesFromEeprom() {
    if (MotorValuesEepromStorageNumber != 0) {
        EepromMotorInfoStruct tEepromMotorInfo;
        eeprom_read_block((void*) &tEepromMotorInfo, (void*) ((MotorValuesEepromStorageNumber - 1) * sizeof(EepromMotorInfoStruct)),
                sizeof(EepromMotorInfoStruct));

        /*
         * Overwrite with values if valid
         */
        if (tEepromMotorInfo.StartSpeed < 100 && tEepromMotorInfo.StartSpeed > 10) {
            StartSpeed = tEepromMotorInfo.StartSpeed;
            if (tEepromMotorInfo.DriveSpeed > 40) {
                DriveSpeed = tEepromMotorInfo.DriveSpeed;
            }
            if (tEepromMotorInfo.SpeedCompensation < 24) {
                SpeedCompensation = tEepromMotorInfo.SpeedCompensation;
            }
        }
        MotorValuesHaveChanged = true;
    }
}

void PWMDcMotor::writeMotorvaluesToEeprom() {
    if (MotorValuesEepromStorageNumber != 0) {
        EepromMotorInfoStruct tEepromMotorInfo;
        tEepromMotorInfo.StartSpeed = StartSpeed;
        tEepromMotorInfo.DriveSpeed = DriveSpeed;
        tEepromMotorInfo.SpeedCompensation = SpeedCompensation;

        eeprom_write_block((void*) &tEepromMotorInfo,
                (void*) ((MotorValuesEepromStorageNumber - 1) * sizeof(EepromMotorInfoStruct)), sizeof(EepromMotorInfoStruct));
    }
}

//void PanicWithLed(unsigned int aDelay, uint8_t aCount) {
//    for (uint8_t i = 0; i < aCount; ++i) {
//        digitalWrite(LED_BUILTIN, HIGH);
//        delay(aDelay);
//        digitalWrite(LED_BUILTIN, LOW);
//        delay(aDelay);
//    }
//}
