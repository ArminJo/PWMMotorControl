/*
 * PWMDCMotor.h
 *
 * Motor control has 2 technical dimensions
 * 1. Motor driver control. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 * 2. Speed / PWM which is ignored for BRAKE or RELEASE
 *
 *  Created on: 12.05.2019
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
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

#ifndef PWMDCMOTOR_H_
#define PWMDCMOTOR_H_

#include <stdint.h>

#define VERSION_PWMMOTORCONTROL "1.1.0"
#define VERSION_PWMMOTORCONTROL_MAJOR 1
#define VERSION_PWMMOTORCONTROL_MINOR 1

/*
 * Comment this out, if you have encoder interrupts attached at pin 2 and 3
 * and want to use the methods of the EncoderMotor class for fixed distance / closed loop driving.
 * Enabling it will disable no longer required PWMDCMotor class variables and functions
 * and force the usage of the EncoderMotor class in CarMotorControl.
 */
//#define USE_ENCODER_MOTOR_CONTROL
//
/*
 * Use Adafruit Motor Shield v2 connected by I2C instead of simple TB6612 or L298 breakout board.
 * This disables tone output by using motor as loudspeaker, but requires only 2 I2C/TWI pins in contrast to the 6 pins used for the full bridge.
 * For full bridge, analogWrite the millis() timer0 is used since we use pin 5 & 6.
 */
//#define USE_ADAFRUIT_MOTOR_SHIELD
//
/*
 * Own library saves me 694 bytes program memory
 */
#if ! defined(USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD) // if defined (externally), forces using Adafruit library
#define USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
#endif

/*
 * Disabling SUPPORT_RAMP_UP saves 7 bytes RAM per motor and around 300 bytes program memory
 */
#if ! DO_NOT_SUPPORT_RAMP_UP // if defined (externally), disables ramp up support
#define SUPPORT_RAMP_UP      // 256 milliseconds for ramp, see below
#endif

/*
 * Comment this out, if you use default settings and 2 LiPo Cells (around 7.4 volt) as Motor supply.
 */
//#define VIN_2_LIPO
#define MAX_SPEED   255

/*
 * This values are chosen to be compatible with 20 slot encoder discs, giving 20 on and 20 off counts per full rotation.
 * At a circumference of around 20 cm (21.5 cm actual) this gives 5 mm per count.
 */
#define DEFAULT_COUNTS_PER_FULL_ROTATION    40
#define DEFAULT_MILLIMETER_PER_COUNT         5

#define DEFAULT_MOTOR_START_UP_TIME_MILLIS 150 // constant value for the for the formula below
/*
 * DEFAULT_DISTANCE_TO_TIME_FACTOR is the factor used to convert distance in 5mm steps to motor on time in milliseconds. I depends on motor supply voltage.
 * Currently formula is:
 * computedMillisOfMotorStopForDistance = DEFAULT_MOTOR_START_UP_TIME_MILLIS + (10 * ((aRequestedDistanceCount * DistanceToTimeFactor) / DriveSpeed));
 *
 * DEFAULT_START_SPEED is the speed PWM value at which car starts to move. For 8 volt is appr. 35 to 40, for 3,6 volt (USB supply) is appr. 70 to 100
 */
#define DEFAULT_START_SPEED_7_4_VOLT                45
#define DEFAULT_DRIVE_SPEED_7_4_VOLT                80
#define DEFAULT_DISTANCE_TO_TIME_FACTOR_7_4_VOLT   135 // for 2 x LIPO batteries (7.4 volt).

#define DEFAULT_START_SPEED_6_VOLT                  150
#define DEFAULT_DRIVE_SPEED_6_VOLT                  255
#define DEFAULT_DISTANCE_TO_TIME_FACTOR_6_VOLT      300 // for 4 x AA batteries (6 volt).

// Default values - used if EEPROM values are invalid
#if defined(VIN_2_LIPO)
#define DEFAULT_START_SPEED                 DEFAULT_START_SPEED_7_4_VOLT
#define DEFAULT_DRIVE_SPEED                 DEFAULT_DRIVE_SPEED_7_4_VOLT
#define DEFAULT_DISTANCE_TO_TIME_FACTOR     DEFAULT_DISTANCE_TO_TIME_FACTOR_7_4_VOLT

#else
#  if ! defined(DEFAULT_START_SPEED)
#define DEFAULT_START_SPEED                 DEFAULT_START_SPEED_6_VOLT
#  endif
#  if ! defined(DEFAULT_DRIVE_SPEED)
#define DEFAULT_DRIVE_SPEED                 DEFAULT_DRIVE_SPEED_6_VOLT
#  endif
#  if ! defined(DEFAULT_DISTANCE_TO_TIME_FACTOR)
#define DEFAULT_DISTANCE_TO_TIME_FACTOR     DEFAULT_DISTANCE_TO_TIME_FACTOR_6_VOLT
#  endif
#endif

// Motor directions and stop modes. Are used for parameter aMotorDriverMode and sequence is determined by the Adafruit library API.
#define DIRECTION_FORWARD   0
#define DIRECTION_BACKWARD  1
#define DIRECTION_MASK      1
#define oppositeDIRECTION(aDirection) (aDirection ^ DIRECTION_BACKWARD)

#define MOTOR_BRAKE         2
#define MOTOR_RELEASE       3
#define STOP_MODE_KEEP      0
#define STOP_MODE_AND_MASK  0x03
#define STOP_MODE_OR_MASK   0x02
#define DEFAULT_STOP_MODE   MOTOR_RELEASE
#define CheckStopMODE(aStopMode) ((aStopMode & STOP_MODE_AND_MASK) | STOP_MODE_OR_MASK)

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#include <Wire.h>
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
// some PCA9685 specific constants
#define PCA9685_GENERAL_CALL_ADDRESS 0x00
#define PCA9685_SOFTWARE_RESET      6
#define PCA9685_DEFAULT_ADDRESS     0x40
#define PCA9685_MAX_CHANNELS        16 // 16 PWM channels on each PCA9685 expansion module
#define PCA9685_MODE1_REGISTER      0x0
#define PCA9685_MODE_1_RESTART        7
#define PCA9685_MODE_1_AUTOINCREMENT  5
#define PCA9685_MODE_1_SLEEP          4
#define PCA9685_FIRST_PWM_REGISTER  0x06
#define PCA9685_PRESCALE_REGISTER   0xFE

#define PCA9685_PRESCALER_FOR_1600_HZ ((25000000L /(4096L * 1600))-1) // = 3 at 1600 Hz

#  else
#include <Adafruit_MotorShield.h>
#define CONVERSION_FOR_ADAFRUIT_API 1
#  endif // USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
#endif // USE_ADAFRUIT_MOTOR_SHIELD

struct EepromMotorInfoStruct {
    uint8_t StartSpeed;
    uint8_t StopSpeed;
    uint8_t DriveSpeed;
    uint8_t SpeedCompensation;
};

/*
 * For ramp control
 */
#define MOTOR_STATE_STOPPED     0
#define MOTOR_STATE_START       1
#define MOTOR_STATE_RAMP_UP     2
#define MOTOR_STATE_DRIVE_SPEED  3
#define MOTOR_STATE_RAMP_DOWN   4

#ifdef SUPPORT_RAMP_UP
#define RAMP_UP_UPDATE_INTERVAL_MILLIS 16 // The smaller the value the steeper the ramp
#define RAMP_UP_UPDATE_INTERVAL_STEPS  16 // Results in a ramp up time of 16 steps * 16 millis = 256 milliseconds
#define RAMP_UP_VALUE_DELTA ((CurrentDriveSpeed - StartSpeed) / RAMP_UP_UPDATE_INTERVAL_STEPS)
#endif

class PWMDcMotor {
public:
    PWMDcMotor();

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    void init(uint8_t aMotorNumber, bool aReadFromEeprom = false);
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    /*
     * Own internal functions for communicating with the PCA9685 Expander IC on the Adafruit motor shield
     */
    void I2CWriteByte(uint8_t aAddress, uint8_t aData);
    void I2CSetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff);
    void I2CSetPin(uint8_t aPin, bool aSetToOn);
#  else
    Adafruit_DCMotor * Adafruit_MotorShield_DcMotor;
#  endif
#else
    void init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aMotorNumber = 0);
#endif

    /*
     * Basic motor commands
     */
    void setSpeed(int aRequestedSpeed);
    void setSpeed(uint8_t aSpeedRequested, uint8_t aRequestedDirection);
    void setSpeedCompensated(int aRequestedSpeed);
    void setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);

    void stop(uint8_t aStopMode = STOP_MODE_KEEP); // STOP_MODE_KEEP (take previously defined DefaultStopMode) or MOTOR_BRAKE or MOTOR_RELEASE
    void setStopMode(uint8_t aStopMode); // mode for Speed==0 or STOP_MODE_KEEP: MOTOR_BRAKE or MOTOR_RELEASE

    /*
     * Fixed distance driving
     */
    void setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, uint8_t aSpeedCompensation = 0);
    void setDefaultsForFixedDistanceDriving();
    void setDriveSpeed(uint8_t aDriveSpeed);

#ifdef SUPPORT_RAMP_UP
    void startRampUp(uint8_t aRequestedDirection);
    void startRampUp(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);
#endif

#ifndef USE_ENCODER_MOTOR_CONTROL
    // This function only makes sense for non encoder motors
    void setDistanceToTimeFactorForFixedDistanceDriving(unsigned int aDistanceToTimeFactor);

    // These functions are implemented by encoder motor too
    void startGoDistanceCount(int aRequestedDistanceCount); // Signed distance count
    void startGoDistanceCount(unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection);
    void startGoDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection);
    bool updateMotor();

    /*
     * Implementation for non encoder motors. Not used by CarControl.
     */
    void goDistanceCount(unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection);
    void goDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection);
#endif

    /*
     * EEPROM functions to read and store calibration and other control values (DriveSpeed, SpeedCompensation)
     */
    void readMotorValuesFromEeprom();
    void writeMotorvaluesToEeprom();

    /*
     * Internal functions
     */
    void setMotorDriverMode(uint8_t cmd);
    bool checkAndHandleDirectionChange(uint8_t aRequestedDirection);

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) || defined(USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    uint8_t PWMPin;     // PWM output pin / PCA9685 channel of Adafruit Motor Shield
    uint8_t ForwardPin; // if high, motor runs forward
    uint8_t BackwardPin;
#endif

    int8_t MotorValuesEepromStorageNumber; // number of EEPROM EepromMotorInfoStruct block address for one PWMDcMotor. 0 means no corresponding EEPROM block. Set by init.
    /**********************************
     * Start of EEPROM values
     *********************************/
    /*
     * Minimum speed setting at which motor starts moving. Depend on current voltage, load and surface.
     * Is set by calibrate() and then stored (with the other values) in eeprom.
     */
    uint8_t StartSpeed; // Speed PWM value at which car starts to move. For 8 volt is appr. 35 to 40, for 4.3 volt (USB supply) is appr. 90 to 100
    uint8_t DriveSpeed; // Speed PWM value used for going fixed distance.

    /*
     * Positive value to be subtracted from TargetSpeed to get CurrentSpeed to compensate for different left and right motors
     * Currently SpeedCompensation is in steps of 2 and only one motor can have a positive value, the other is set to zero.
     * Value is computed in synchronizeMotor()
     */
    uint8_t SpeedCompensation;
    /**********************************
     * End of EEPROM values
     *********************************/
    uint8_t CurrentSpeed;
    uint8_t CurrentDirectionOrBrakeMode; // (of CurrentSpeed etc.) DIRECTION_FORWARD, DIRECTION_BACKWARD, MOTOR_BRAKE, MOTOR_RELEASE
    static bool MotorValuesHaveChanged;

    uint8_t DefaultStopMode; // used for speed == 0 and STOP_MODE_KEEP
    bool MotorMovesFixedDistance; // if true, stop if end distance condition reached

#ifdef SUPPORT_RAMP_UP
    /*
     * For ramp control
     */
    uint8_t MotorRampState; // MOTOR_STATE_STOPPED, MOTOR_STATE_START, MOTOR_STATE_RAMP_UP, MOTOR_STATE_DRIVE_SPEED, MOTOR_STATE_RAMP_DOWN
    uint8_t CurrentDriveSpeed; // DriveSpeed - SpeedCompensation; The DriveSpeed used for current movement. Can be set for eg. turning which better performs with reduced DriveSpeed

    uint8_t RampDelta;
    unsigned long NextRampChangeMillis;
#endif

#ifndef USE_ENCODER_MOTOR_CONTROL
    uint32_t computedMillisOfMotorStopForDistance; // Since we have no distance sensing, we must estimate a duration instead
    unsigned int DistanceToTimeFactor; // Required for non encoder motors to estimate duration for a fixed distance
#endif

};

//void PanicWithLed(unsigned int aDelay, uint8_t aCount);
/*
 * Version 1.1.0 - 10/2020
 * - Added and renamed functions.
 *
 * Version 1.0.0 - 9/2020
 * - Initial version.
 */
#endif /* PWMDCMOTOR_H_ */

#pragma once
