/*
 * PWMDCMotor.h
 *
 * Motor control has 2 parameters:
 * 1. SpeedPWM / PWM which is ignored for BRAKE or RELEASE. This library also accepts signed speed (including the direction as sign).
 * 2. Direction / MotorDriverMode. Can be FORWARD, BACKWARD (BRAKE motor connection are shortened) or RELEASE ( motor connections are high impedance)
 *
 * PWM period is 600 us for Adafruit Motor Shield V2 using PCA9685.
 * PWM period is 1030 us for using full bridge PWM generation by analogWrite() with millis() timer0 on pin 5 & 6.
 *
 * Distance is computed in 3 different ways.
 * Without IMU or Encoder: - distance is converted to a time for riding.
 * With IMU: - distance is measured by IMU.
 * With encoder: - distance is measured by Encoder.
 *
 *
 *  Created on: 12.05.2019
 *  Copyright (C) 2019-2021  Armin Joachimsmeyer
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

#ifndef PWMDCMOTOR_H_
#define PWMDCMOTOR_H_

#include <stdint.h>

#define VERSION_PWMMOTORCONTROL "2.1.0"
#define VERSION_PWMMOTORCONTROL_MAJOR 2
#define VERSION_PWMMOTORCONTROL_MINOR 1
// The change log is at the bottom of the file

#define MILLIS_IN_ONE_SECOND 1000L

/*
 * Activate this, if you have encoder interrupts attached at pin 2 and 3
 * and want to use the methods of the EncoderMotor class for fixed distance / closed loop driving.
 * Enabling it will overload some PWMDCMotor class functions and force the usage of the EncoderMotor class in CarPWMMotorControl.
 */
//#define USE_ENCODER_MOTOR_CONTROL

//#define USE_ADAFRUIT_MOTOR_SHIELD // Activate this if you use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
//This disables using motor as buzzer, but requires only 2 I2C/TWI pins in contrast to the 6 pins used for the full bridge.
#  if !defined(MOSFET_BRIDGE_USED)
#define MOSFET_BRIDGE_USED
#  endif
#  if !defined(FULL_BRIDGE_LOSS_MILLIVOLT)
#define FULL_BRIDGE_LOSS_MILLIVOLT             0
#  endif
#endif

//#define USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD // Activate this to force using of Adafruit library. Requires 694 bytes program memory.
#if ! defined(USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
#define USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
#endif

//#define VIN_2_LIPO // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO // Or if you use a Mosfet bridge, 1 LIPO may be sufficient.
/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define MAX_SPEED_PWM   255

/*
 * Circumference of my smart car wheel
 */
#define DEFAULT_CIRCUMFERENCE_MILLIMETER   220

#if !defined(FULL_BRIDGE_INPUT_MILLIVOLT)
#  if defined(VIN_2_LIPO)
#define FULL_BRIDGE_INPUT_MILLIVOLT         7400 // for 2 x LIPO batteries (7.4 volt).
#  elif defined(VIN_1_LIPO)
#define FULL_BRIDGE_INPUT_MILLIVOLT         3700 // for 1 x LIPO battery (3.7 volt).
#  else
#define FULL_BRIDGE_INPUT_MILLIVOLT         6000  // Default. For 4 x AA batteries (6 volt).
#  endif
#endif

#if !defined(FULL_BRIDGE_LOSS_MILLIVOLT)
#  if defined(MOSFET_BRIDGE_USED)
// Speed is almost linear to 1/2 PWM in cm/s without any offset, only with dead band
#define FULL_BRIDGE_LOSS_MILLIVOLT             0
#  else
// Speed is not linear to PWM and has an offset
// Effective voltage loss includes loss by switching to high impedance at inactive for bipolar full bridges like L298
#define FULL_BRIDGE_LOSS_MILLIVOLT          2200 // Effective voltage loss
#  endif
#endif

#if !defined(FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define FULL_BRIDGE_OUTPUT_MILLIVOLT        (FULL_BRIDGE_INPUT_MILLIVOLT - FULL_BRIDGE_LOSS_MILLIVOLT)
#endif

#define DEFAULT_START_MILLIVOLT_MOSFET      1000 // Voltage where motors start to turn
#define DEFAULT_START_MILLIVOLT_L298        1500 // For L298 the start voltage is higher (because of a higher ESR of the L298 bridge?)
#define DEFAULT_DRIVE_MILLIVOLT             2000 // Drive voltage -motors default speed- is 2.0 volt
#define SPEED_PWM_FOR_1_VOLT                ((1000L * MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define SPEED_FOR_8_VOLT                    ((8000L * MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)

// Default values - used if EEPROM values are invalid or nor available
#if !defined(DEFAULT_DRIVE_SPEED_PWM)
// At 2 volt I measured around 32 cm/s. PWM=127 for 4 volt, 68 for 7.4 volt
#define DEFAULT_DRIVE_SPEED_PWM             ((DEFAULT_DRIVE_MILLIVOLT * (long)MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#endif

#if !defined(DEFAULT_START_SPEED_PWM)
#  if defined(MOSFET_BRIDGE_USED)
#define DEFAULT_START_SPEED_PWM             ((DEFAULT_START_MILLIVOLT_MOSFET * (long)MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#  else
#define DEFAULT_START_SPEED_PWM             ((DEFAULT_START_MILLIVOLT_L298 * (long)MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#  endif
#endif

#if !defined(DEFAULT_MILLIMETER_PER_SECOND)
// At 2 volt (DEFAULT_DRIVE_MILLIVOLT) we have around 1.5 rotation per second, 29 distance/encoder counts per second -> 32 cm / second
#define DEFAULT_MILLIMETER_PER_SECOND       320 // at DEFAULT_DRIVE_MILLIVOLT (2.0 V) motor supply
#define DEFAULT_MILLIS_PER_MILLIMETER       (MILLIS_IN_ONE_SECOND / DEFAULT_MILLIMETER_PER_SECOND)
#endif
/*
 * Currently formula used to convert distance in 11 mm steps to motor on time in milliseconds is:
 * computedMillisOfMotorStopForDistance = DEFAULT_MOTOR_START_TIME_MILLIS + (((aRequestedDistanceCount * MillisPerMillimeter) / DriveSpeedPWM));
 */

/*
 * RAMP values
 * I measured maximum positive acceleration with spinning wheels as 250 cm/s^2 on varnished wood.
 * I measured maximum negative acceleration with blocking wheels as 350 cm/s^2 on varnished wood.
 * This corresponds to 5 cm/s every 20 ms
 */
#define RAMP_INTERVAL_MILLIS               20 // The smaller the value the steeper the ramp
#define RAMP_VALUE_OFFSET_MILLIVOLT      2300 // Start positive or negative acceleration with this voltage offset in order to get a reasonable acceleration for ramps
#define RAMP_VALUE_OFFSET_SPEED_PWM      ((RAMP_VALUE_OFFSET_MILLIVOLT * (long)MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define RAMP_VALUE_MIN_SPEED_PWM         (DEFAULT_DRIVE_SPEED_PWM / 2) // Minimal speed, if motor is still turning
#define RAMP_VALUE_DELTA                 (SPEED_FOR_8_VOLT / (MILLIS_IN_ONE_SECOND / RAMP_INTERVAL_MILLIS)) // Results in a ramp up voltage of 8V/s = 0.8 volt per 100 ms
#define RAMP_DECELERATION_TIMES_2        3500 // Take half of the observed maximum. This depends on the type of tires and the mass of the car

#define DEFAULT_MOTOR_START_TIME_MILLIS 15 // 15 to 20, constant value for the for the formula below

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
#define ForceStopMODE(aStopMode) ((aStopMode & STOP_MODE_AND_MASK) | STOP_MODE_OR_MASK)

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
#include <Wire.h>
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
// some PCA9685 specific constants
#define PCA9685_DEFAULT_ADDRESS      0x60
#define PCA9685_GENERAL_CALL_ADDRESS 0x00
#define PCA9685_SOFTWARE_RESET          6
#define PCA9685_MAX_CHANNELS           16 // 16 PWM channels on each PCA9685 expansion module
#define PCA9685_MODE1_REGISTER       0x00
#define PCA9685_MODE_1_RESTART          7
#define PCA9685_MODE_1_AUTOINCREMENT    5
#define PCA9685_MODE_1_SLEEP            4
#define PCA9685_FIRST_PWM_REGISTER   0x06
#define PCA9685_PRESCALE_REGISTER    0xFE

#define PCA9685_PRESCALER_FOR_1600_HZ ((25000000L /(4096L * 1600))-1) // = 3 at 1600 Hz

#  else
#include <Adafruit_MotorShield.h>
#define CONVERSION_FOR_ADAFRUIT_API 1
#  endif // USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
#endif // USE_ADAFRUIT_MOTOR_SHIELD

struct EepromMotorInfoStruct {
    uint8_t DriveSpeedPWM;
    uint8_t SpeedPWMCompensation;
};

/*
 * Ramp control
 */
#define MOTOR_STATE_STOPPED     0
#define MOTOR_STATE_START       1
#define MOTOR_STATE_RAMP_UP     2
#define MOTOR_STATE_DRIVE       3
#define MOTOR_STATE_RAMP_DOWN   4

class PWMDcMotor {
public:
    PWMDcMotor();

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    void init(uint8_t aMotorNumber);
#  ifdef USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
    /*
     * Own internal functions for communicating with the PCA9685 Expander IC on the Adafruit motor shield
     */
    void PCA9685WriteByte(uint8_t aAddress, uint8_t aData);
    void PCA9685SetPWM(uint8_t aPin, uint16_t aOn, uint16_t aOff);
    void PCA9685SetPin(uint8_t aPin, bool aSetToOn);
#  else
    Adafruit_DCMotor *Adafruit_MotorShield_DcMotor;
#  endif
#else
    PWMDcMotor(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin);
    void init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin);
#endif

    /*
     * Basic motor commands
     */
    void setSpeedPWM(int aRequestedSpeedPWM);
    void changeSpeedPWM(uint8_t aRequestedSpeedPWM); // Keeps direction
    void setSpeedPWM(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void setSpeedPWMWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);

    void setSpeedPWMCompensation(uint8_t aSpeedPWMCompensation);

    void start(uint8_t aRequestedDirection);
    void stop(uint8_t aStopMode = STOP_MODE_KEEP); // STOP_MODE_KEEP (take previously defined DefaultStopMode) or MOTOR_BRAKE or MOTOR_RELEASE
    void setStopMode(uint8_t aStopMode); // mode for SpeedPWM==0 or STOP_MODE_KEEP: MOTOR_BRAKE or MOTOR_RELEASE
    bool isStopped(); // checks for SpeedPWM==0
    /*
     * Fixed distance driving functions
     */
    void setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM, uint8_t aSpeedPWMCompensation = 0);
    void setDefaultsForFixedDistanceDriving();
    void setDriveSpeedPWM(uint8_t aDriveSpeedPWM);
    void updateDriveSpeedPWM(uint8_t aDriveSpeedPWM); // if running update also current speed

    void startRampUp(uint8_t aRequestedDirection);
    void startRampDown();

#ifndef USE_ENCODER_MOTOR_CONTROL // required here, since we cannot access the computedMillisOfMotorStopForDistance and MillisPerMillimeter for the functions below
    // This function only makes sense for non encoder motors
    void setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond);

    // These functions are implemented by encoder motor too
    void startGoDistanceMillimeter(int aRequestedDistanceMillimeter); // Signed distance
    void startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
    void startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
            uint8_t aRequestedDirection);
    bool updateMotor();

    /*
     * Implementation for non encoder motors. Not used by CarControl.
     */
    void goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
    void goDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
#endif

    /*
     * EEPROM functions to read and store control values (DriveSpeedPWM, SpeedPWMCompensation)
     */
    void readMotorValuesFromEeprom(uint8_t aMotorValuesEepromStorageNumber);
    void writeMotorValuesToEeprom(uint8_t aMotorValuesEepromStorageNumber);

    static void printSettings(Print *aSerial);

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

    /**********************************
     * Start of values for EEPROM
     *********************************/
    uint8_t DriveSpeedPWM; // SpeedPWM value used for going fixed distance.

    /*
     * Positive value to be subtracted from TargetPWM to get CurrentSpeedPWM to compensate for different left and right motors
     * Currently SpeedPWMCompensation is in steps of 2 and only one motor can have a positive value, the other is set to zero.
     * Value is computed in EncoderMotor::synchronizeMotor()
     */
    uint8_t SpeedPWMCompensation;
    /**********************************
     * End of EEPROM values
     *********************************/
    uint8_t DefaultStopMode; // used for PWM == 0 and STOP_MODE_KEEP
    static bool MotorControlValuesHaveChanged; // true if DefaultStopMode, DriveSpeedPWM or SpeedPWMCompensation have changed - for printing
#if defined(USE_MPU6050_IMU) || defined(USE_ENCODER_MOTOR_CONTROL)
    volatile static bool SensorValuesHaveChanged; // true if encoder data or IMU data have changed
#endif

    uint8_t CurrentSpeedPWM; // stopped if CurrentSpeedPWM == 0
    uint8_t CurrentDirectionOrBrakeMode; // (of CurrentSpeedPWM etc.) DIRECTION_FORWARD, DIRECTION_BACKWARD, if motor stopped, then: MOTOR_BRAKE, MOTOR_RELEASE
    uint8_t LastDirection; // Used for speed and distance. Contains  DIRECTION_FORWARD, DIRECTION_BACKWARD but not MOTOR_BRAKE, MOTOR_RELEASE.
    static bool MotorPWMHasChanged;

    bool CheckDistanceInUpdateMotor;

    /*
     * For ramp control
     */
    uint8_t MotorRampState; // MOTOR_STATE_STOPPED, MOTOR_STATE_START, MOTOR_STATE_RAMP_UP, MOTOR_STATE_DRIVE, MOTOR_STATE_RAMP_DOWN
    uint8_t RequestedDriveSpeedPWM; // DriveSpeedPWM - SpeedPWMCompensation; The DriveSpeedPWM used for current movement. Can be set for eg. turning which better performs with reduced DriveSpeedPWM

    unsigned long NextRampChangeMillis;

#ifndef USE_ENCODER_MOTOR_CONTROL // this saves 5 bytes ram if we know, that we do not use the simple PWMDcMotor distance functions
    uint32_t computedMillisOfMotorStopForDistance; // Since we have no distance sensing, we must estimate a duration instead
    uint8_t MillisPerMillimeter; // Value for 2 volt motor effective voltage at DEFAULT_DRIVE_SPEED_PWM. Required for non encoder motors to estimate duration for a fixed distance
#endif

};

/*
 * Version 2.1.0 - 10/2021
 * - Removed all *Compensated functions, compensation now is always active.
 * - Removed StopSpeed from EepromMotorinfoStruct.
 * - Removed StartSpeed.
 * - Renamed *.cpp to *.hpp.
 * - Added and renamed functions.
 *
 * Version 2.0.0 - 11/2020
 * - IMU / MPU6050 support.
 * - Support of off the shelf smart cars.
 * - Added and renamed functions.
 *
 * Version 1.0.0 - 9/2020
 * - Initial version.
 */
#endif /* PWMDCMOTOR_H_ */

#pragma once
