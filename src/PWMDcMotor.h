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

#ifndef _PWM_DC_MOTOR_H
#define _PWM_DC_MOTOR_H

#include <stdint.h>

#define VERSION_PWMMOTORCONTROL "2.0.0"
#define VERSION_PWMMOTORCONTROL_MAJOR 2
#define VERSION_PWMMOTORCONTROL_MINOR 0
#define VERSION_PWMMOTORCONTROL_PATCH 0
// The change log is at the bottom of the file

/*
 * Macro to convert 3 version parts into an integer
 * To be used in preprocessor comparisons, such as #if VERSION_SERVO_EASING_HEX >= VERSION_HEX_VALUE(3, 0, 0)
 */
#define VERSION_HEX_VALUE(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#define VERSION_PWMMOTORCONTROL_HEX  VERSION_HEX_VALUE(VERSION_PWMMOTORCONTROL_MAJOR, VERSION_PWMMOTORCONTROL_MINOR, VERSION_PWMMOTORCONTROL_PATCH)

#define MILLIS_IN_ONE_SECOND            1000L
#define MILLIMETER_IN_ONE_CENTIMETER    10L

/*
 * Parameters for PWMMotorControl
 */
//Enabling it will overload some PWMDCMotor class functions and force the usage of the EncoderMotor class in CarPWMMotorControl.
//#define USE_ENCODER_MOTOR_CONTROL       // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 with 2xTB6612 connected by I2C instead of external TB6612 or L298 breakout board.
//#define USE_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                    // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT 6000// Default. For 4 x AA batteries (6 volt).
//#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define DEFAULT_DRIVE_MILLIVOLT   2000  // Drive voltage / motors default speed. Default value is 2.0 volt.
//#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
//#define DO_NOT_SUPPORT_AVERAGE_SPEED    // Disables the encoder function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.

//#define USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD // Activate this to force using of Adafruit library. Requires 694 bytes program memory.
#if !defined(USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
#define _USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD // to avoid double negations
#endif

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
//This disables using motor as buzzer, but requires only 2 I2C/TWI pins in contrast to the 6 pins used for the full bridge.
#  if !defined(FULL_BRIDGE_LOSS_MILLIVOLT)
#define FULL_BRIDGE_LOSS_MILLIVOLT             0
#  endif
#endif

/*
 * Helper macro for getting a macro definition as string
 */
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

#define MAX_SPEED_PWM                        255L // Long constant, otherwise we get "integer overflow in expression"

/*
 * Propagate debug level, we have no INFO output in the sources
 */
#if defined(TRACE)    // Information you need to understand details of a function or if you hunt a bug.
#  if !defined(DEBUG)
#define DEBUG   // Information need to understand the operating of your program. E.g. function calls and values of control variables.
#  endif
#endif

/********************************************
 * Car and motor driver characteristics
 ********************************************/
#if !defined(DEFAULT_CIRCUMFERENCE_MILLIMETER)
#define DEFAULT_CIRCUMFERENCE_MILLIMETER     220
#endif

#if !defined(FULL_BRIDGE_INPUT_MILLIVOLT)
#  if defined(VIN_2_LI_ION)
#define FULL_BRIDGE_INPUT_MILLIVOLT         7400 // for 2 x LIPO batteries (7.4 volt).
#  elif defined(VIN_1_LI_ION)
#define FULL_BRIDGE_INPUT_MILLIVOLT         3700 // for 1 x LIPO battery (3.7 volt).
#  else
//#define FULL_BRIDGE_INPUT_MILLIVOLT         6000  // Default. For 4 x AA batteries (6 volt).
#define FULL_BRIDGE_INPUT_MILLIVOLT         4800  // Default. For 4 x AA rechargeable batteries (4.8 volt).
#  endif
#endif

#if !defined(FULL_BRIDGE_LOSS_MILLIVOLT)
#  if defined(USE_L298_BRIDGE)
// Speed is not linear to PWM and has an offset
// Effective voltage loss includes loss by switching to high impedance at inactive for bipolar full bridges like L298
#define FULL_BRIDGE_LOSS_MILLIVOLT          2000 // Effective voltage loss
#  else
// Speed is almost linear to 1/2 PWM in cm/s without any offset, only with dead band
#define FULL_BRIDGE_LOSS_MILLIVOLT             0
#  endif
#endif

#if !defined(FULL_BRIDGE_OUTPUT_MILLIVOLT)
// Effective voltage available for the motor
#define FULL_BRIDGE_OUTPUT_MILLIVOLT        (FULL_BRIDGE_INPUT_MILLIVOLT - FULL_BRIDGE_LOSS_MILLIVOLT)
#endif

/********************************************
 * Motor speed voltages
 ********************************************/
#define DEFAULT_STOP_MILLIVOLT_MOSFET       700 // Voltage where spinning motors start to stop
#define DEFAULT_START_MILLIVOLT_MOSFET      1000 // Voltage where motors start to turn
#define DEFAULT_STOP_MILLIVOLT_L298         750  // Voltage where spinning motors start to stop
#define DEFAULT_START_MILLIVOLT_L298        1700 // For L298 the start voltage is higher (because of a higher ESR of the L298 bridge?)
#define DEFAULT_DRIVE_MILLIVOLT             2000 // Drive voltage -motors default speed- is 2.0 volt


// Default values - used if EEPROM values are invalid or not available
#if !defined(DEFAULT_DRIVE_SPEED_PWM)
// Corresponds to 2 volt. At 2 volt I measured around 32 cm/s. PWM=127 for 4 volt VCC, 68 for 7.4 volt VCC
#define DEFAULT_DRIVE_SPEED_PWM             (((DEFAULT_DRIVE_MILLIVOLT * MAX_SPEED_PWM) + (FULL_BRIDGE_OUTPUT_MILLIVOLT / 2)) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#endif

#if !defined(DEFAULT_START_SPEED_PWM)
#  if defined(USE_L298_BRIDGE)
#define DEFAULT_START_SPEED_PWM             (((DEFAULT_START_MILLIVOLT_L298 * MAX_SPEED_PWM) + (FULL_BRIDGE_OUTPUT_MILLIVOLT / 2)) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define DEFAULT_STOP_SPEED_PWM              (((DEFAULT_STOP_MILLIVOLT_L298 * MAX_SPEED_PWM)  + (FULL_BRIDGE_OUTPUT_MILLIVOLT / 2)) / FULL_BRIDGE_OUTPUT_MILLIVOLT)  // 24 for 7.4 volt
#  else
#define DEFAULT_START_SPEED_PWM             (((DEFAULT_START_MILLIVOLT_MOSFET * MAX_SPEED_PWM)  + (FULL_BRIDGE_OUTPUT_MILLIVOLT / 2)) / FULL_BRIDGE_OUTPUT_MILLIVOLT) // 24 for 7.4 volt
#define DEFAULT_STOP_SPEED_PWM              (((DEFAULT_STOP_MILLIVOLT_MOSFET * MAX_SPEED_PWM)  + (FULL_BRIDGE_OUTPUT_MILLIVOLT / 2)) / FULL_BRIDGE_OUTPUT_MILLIVOLT)  // 24 for 7.4 volt
#  endif
#endif

/********************************************
 * PWM to voltage conversion
 ********************************************/
#define SPEED_PWM_FOR_1_VOLT                ((1000 * MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define SPEED_PWM_FOR_8_VOLT                ((8000 * MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)

#if !defined(DEFAULT_MILLIMETER_PER_SECOND)
#  if defined(CAR_HAS_4_MECANUM_WHEELS)
#define DEFAULT_MILLIMETER_PER_SECOND            200 // At DEFAULT_DRIVE_MILLIVOLT (2.0 V) motor supply
#define DEFAULT_MILLIS_FOR_FIRST_CENTIMETER       75 // Time for start stop in (guessed) one cm. 50 -> 10 mm at 200 mm/second
#  else
#define DEFAULT_MILLIMETER_PER_SECOND            230 // At DEFAULT_DRIVE_MILLIVOLT (2.0 V) motor supply
#define DEFAULT_MILLIS_FOR_FIRST_CENTIMETER       85 // Time for start stop in (guessed) one cm. 50 -> 10 mm at 200 mm/second
#define SPEED_PER_VOLT                           130 // Only for documentation, not used. mm/s after accelerating. Up to 145 mm/s @7.4V, 50% PWM
#  endif
#endif
/*
 * Use MILLIS_PER_CENTIMETER instead of MILLIS_PER_MILLIMETER to get a reasonable resolution
 */
#define DEFAULT_MILLIS_PER_CENTIMETER       ((MILLIS_IN_ONE_SECOND * MILLIMETER_IN_ONE_CENTIMETER) / DEFAULT_MILLIMETER_PER_SECOND)
/*
 * Currently formula used to convert distance in mm to motor on time in milliseconds is:
 * computedMillisOfMotorStopForDistance = DEFAULT_MILLIS_FOR_FIRST_CENTIMETER + ((((aRequestedDistanceMillimeter - 10) * MillisPerCentimeter) / MILLIMETER_IN_ONE_CENTIMETER * DriveSpeedPWM));
 */

/*******************************************************
 * RAMP values for an offset of 2.3V and a ramp of 10V/s
 *******************************************************/
#define SPEED_PWM_FOR_1_VOLT             ((1000 * MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define RAMP_UP_VOLTAGE_PER_SECOND       12 // 12 * 130 mm/s = 1560 mm/s ^2
#define RAMP_DOWN_VOLTAGE_PER_SECOND     14 // 14 * 130 mm/s = 1820 mm/s ^2

#define RAMP_INTERVAL_MILLIS             20
/*
 * Start positive or negative acceleration with this voltage offset in order to get a reasonable acceleration for ramps
 * The value must be low enough to avoid spinning wheels
 * I measured maximum brake acceleration with blocking wheels as 320 to 350 cm/s^2 on varnished wood. 6 to 7 cm/s every 20 ms.
 * I measured maximum positive acceleration with spinning wheels as 2000 to 2500 mm/s^2 on varnished wood. 4 to 5 cm/s every 20 ms.
 * Measured values up:   1V -> 1600mm/s^2, 2.5V -> 2000mm/s^2, the optimum.
 * Measured values down: 2.5V -> 2500mm/s^2
 */
#define RAMP_UP_VALUE_OFFSET_MILLIVOLT   2000 // Experimental value, 2500 seems to be optimum. 3000 leads to spinning wheels.
#define RAMP_UP_VALUE_OFFSET_SPEED_PWM   ((RAMP_UP_VALUE_OFFSET_MILLIVOLT * (long)MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define RAMP_DOWN_VALUE_OFFSET_MILLIVOLT 2500 // Experimental value. 3000 may be optimum.
#define RAMP_DOWN_VALUE_OFFSET_SPEED_PWM ((RAMP_UP_VALUE_OFFSET_MILLIVOLT * (long)MAX_SPEED_PWM) / FULL_BRIDGE_OUTPUT_MILLIVOLT)
#define RAMP_VALUE_MIN_SPEED_PWM         DEFAULT_DRIVE_SPEED_PWM // Maximal speed, where motor can be stopped immediately
#define RAMP_UP_VALUE_DELTA              ((SPEED_PWM_FOR_1_VOLT * RAMP_UP_VOLTAGE_PER_SECOND) / (MILLIS_IN_ONE_SECOND / RAMP_INTERVAL_MILLIS))
#define RAMP_DOWN_VALUE_DELTA            ((SPEED_PWM_FOR_1_VOLT * RAMP_DOWN_VOLTAGE_PER_SECOND) / (MILLIS_IN_ONE_SECOND / RAMP_INTERVAL_MILLIS))
#if (RAMP_DOWN_VALUE_DELTA > RAMP_VALUE_MIN_SPEED_PWM)
#error RAMP_DOWN_VALUE_DELTA must be smaller than RAMP_VALUE_MIN_SPEED_PWM !
#endif
#define RAMP_DECELERATION_TIMES_2        (2000 * 2) // 2000 was measured by IMU for 14V/s and 2500 mV offset.

/********************************************
 * Program defines
 ********************************************/
// Motor directions and stop modes. Are used for parameter aMotorDriverMode and sequence is determined by the Adafruit library API.
#define DIRECTION_STOP                  0x00
#define DIRECTION_FORWARD               0x01
#define DIRECTION_BACKWARD              0x02
#define DIRECTION_MASK                  (DIRECTION_FORWARD | DIRECTION_BACKWARD)
#define oppositeDIRECTION(aDirection)   (aDirection ^ DIRECTION_MASK) // invert every bit
/*
 * Stop mode definitions
 */
#define STOP_MODE_BRAKE                 0x00
#define STOP_MODE_RELEASE               0x03
#define DEFAULT_STOP_MODE               STOP_MODE_BRAKE
#define STOP_MODE_KEEP                  1 // Take DefaultStopMode - used only as parameter for stop()

/*
 * Extension for mecanum wheel movements
 * They are coded as bit positions in the upper nibble
 */
#define DIRECTION_STRAIGHT                  0x00
#define DIRECTION_LEFT                      0x10
#define DIRECTION_DIAGONAL_LEFT_FORWARD     0x11
#define DIRECTION_DIAGONAL_LEFT_BACKWARD    0x12
#define DIRECTION_RIGHT                     0x20
#define DIRECTION_DIAGONAL_RIGHT_FORWARD    0x21
#define DIRECTION_DIAGONAL_RIGHT_BACKWARD   0x22
#define DIRECTION_LEFT_RIGHT_MASK           (DIRECTION_LEFT | DIRECTION_RIGHT)
#define oppositeSIDE(aSide)                 (aSide ^ DIRECTION_LEFT_RIGHT_MASK)
#define DIRECTION_TURN                      0x40
#define DIRECTION_TURN_LEFT                 0x50
#define DIRECTION_TURN_RIGHT                0x60
#define DIRECTION_TURN_MASK                 DIRECTION_TURN
#define DIRECTION_NOT_TURN                  0x00

#if defined(DEBUG)
extern char sDirectionCharArray[3];
#endif
extern const char *sDirectionStringArray[4];

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
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
#  endif // _USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD
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
#define MOTOR_STATE_CHECK_DISTANCE 5

class PWMDcMotor {
public:
    PWMDcMotor();

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    void init(uint8_t aMotorNumber);
#  if defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
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
    void setDirection(uint8_t aMotorDirection); // alias for setMotorDriverMode()
    void setSpeedPWM(uint8_t aRequestedSpeedPWM);

    void setSpeedPWMAndDirection(int aRequestedSpeedPWM); // sign us used for direction
    void changeSpeedPWM(uint8_t aRequestedSpeedPWM); // Keeps direction
    void setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void setSpeedPWMAndDirectionWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);

    void setSpeedPWMCompensation(uint8_t aSpeedPWMCompensation);

    static float getMotorVoltageforPWMAndMillivolt(uint8_t aSpeedPWM, uint16_t aFullBridgeInputVoltageMillivolt);
    static uint16_t getMotorVoltageMillivoltforPWMAndMillivolt(uint8_t aSpeedPWM, uint16_t aFullBridgeInputVoltageMillivolt);
    static float getMotorVoltageforPWM(uint8_t aSpeedPWM, float aFullBridgeInputVoltage);
    static uint8_t getVoltageAdjustedSpeedPWM(uint8_t aSpeedPWM, uint16_t aFullBridgeInputVoltageMillivolt);
    static uint8_t getVoltageAdjustedSpeedPWM(uint8_t aSpeedPWM, float aFullBridgeInputVoltage);
    uint8_t getDirection();
    static void printDirectionString(Print *aSerial, uint8_t aDirection);

    void start(uint8_t aRequestedDirection);
    void stop(uint8_t aStopMode = STOP_MODE_KEEP); // STOP_MODE_KEEP (take previously defined DefaultStopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
    void setStopMode(uint8_t aStopMode); // mode for SpeedPWM==0 or STOP_MODE_KEEP: STOP_MODE_BRAKE or STOP_MODE_RELEASE
    bool isStopped(); // checks for SpeedPWM==0
    /*
     * Fixed distance driving functions
     */
    void setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM, uint8_t aSpeedPWMCompensation = 0);
    void setDefaultsForFixedDistanceDriving();
    void setDriveSpeedPWM(uint8_t aDriveSpeedPWM);
    void setDriveSpeedPWMFor2Volt(uint16_t aFullBridgeInputVoltageMillivolt);
    void setDriveSpeedPWMFor2Volt(float aFullBridgeInputVoltage);
    void updateDriveSpeedPWM(uint8_t aDriveSpeedPWM); // if running update also current speed

    void startRampUp(uint8_t aRequestedDirection);
    void startRampDown();
    void synchronizeRampDown(PWMDcMotor *aOtherMotorControl);

#if !defined(USE_ENCODER_MOTOR_CONTROL) // Guard required here, since we cannot access the computedMillisOfMotorStopForDistance and MillisPerCentimeter for the functions below
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
    void goDistanceMillimeter(int aRequestedDistanceMillimeter);
    void goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
    void goDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
#endif

    /*
     * EEPROM functions to read and store control values (DriveSpeedPWM, SpeedPWMCompensation)
     */
    void readMotorValuesFromEeprom(uint8_t aMotorValuesEepromStorageNumber);
    void writeMotorValuesToEeprom(uint8_t aMotorValuesEepromStorageNumber);

    void printValues(Print *aSerial);
    static void printCompileOptions(Print *aSerial);

    /*
     * Internal functions
     */
    void setMotorDriverMode(uint8_t aMotorDriverMode);
    bool checkAndHandleDirectionChange(uint8_t aRequestedDirection);

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) || defined(_USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD)
    uint8_t PWMPin;     // PWM output pin / PCA9685 channel of Adafruit Motor Shield
    uint8_t ForwardPin; // if high, motor runs forward
    uint8_t BackwardPin;
#endif

    /**********************************
     * Start of values for EEPROM
     *********************************/
    uint8_t DriveSpeedPWM; // SpeedPWM value used internally for moving. Default is a PWM value which corresponds to 2 volt.
    uint8_t DriveSpeedPWMFor2Volt; // SpeedPWM value which corresponds to 2 volt. Used only in startGoDistanceMillimeter() for scaling.

    /**********************************
     * End of EEPROM values
     *********************************/
    uint8_t DefaultStopMode; // used for PWM == 0 and STOP_MODE_KEEP
    static bool MotorControlValuesHaveChanged; // true if DefaultStopMode, DriveSpeedPWM or SpeedPWMCompensation have changed - for printing
#if defined(USE_MPU6050_IMU) || defined(USE_ENCODER_MOTOR_CONTROL)
    volatile static bool SensorValuesHaveChanged; // true if encoder data or IMU data have changed
#endif

    /*
     * Positive value to be subtracted from TargetPWM to get CurrentCompensatedSpeedPWM to compensate for different left and right motors
     * Currently SpeedPWMCompensation is in steps of 2 and only one motor can have a positive value, the other is set to zero.
     * Value is computed in EncoderMotor::synchronizeMotor()
     */
    uint8_t SpeedPWMCompensation;   // Positive value!
    uint8_t RequestedSpeedPWM;      // Last PWM requested for motor. Stopped if RequestedSpeedPWM == 0. It is always >= CurrentCompensatedSpeedPWM
    uint8_t CurrentCompensatedSpeedPWM; // RequestedSpeedPWM - SpeedPWMCompensation.
    uint8_t CurrentDirection; // Used for speed and distance. Contains DIRECTION_FORWARD, DIRECTION_BACKWARD but NOT STOP_MODE_BRAKE, STOP_MODE_RELEASE.
    static bool MotorPWMHasChanged;

    bool CheckDistanceInUpdateMotor;

#if !defined(DO_NOT_SUPPORT_RAMP)
    /*
     * For ramp control
     */
    uint8_t MotorRampState; // MOTOR_STATE_STOPPED, MOTOR_STATE_START, MOTOR_STATE_RAMP_UP, MOTOR_STATE_DRIVE, MOTOR_STATE_RAMP_DOWN
    uint8_t RequestedDriveSpeedPWM; // DriveSpeedPWM - SpeedPWMCompensation; The DriveSpeedPWM used for current movement. Can be set for eg. turning which better performs with reduced DriveSpeedPWM

    unsigned long NextRampChangeMillis;
#endif

#if !defined(USE_ENCODER_MOTOR_CONTROL) // this saves 5 bytes ram if we know, that we do not use the simple PWMDcMotor distance functions
    uint32_t computedMillisOfMotorStopForDistance; // Since we have no distance sensing, we must estimate a duration instead
    // MillisPerMillimeter values were in the range of 3 and 4, thus use MillisPerCentimeter for better resolution
    uint8_t MillisPerCentimeter; // Value for 2 volt motor effective voltage at DEFAULT_DRIVE_SPEED_PWM. Required for non encoder motors to estimate duration for a fixed distance
#endif

};

/*
 * Version 2.0.0 - 06/2022
 * - Renamed instance from RobotCarPWMMotorControl to RobotCar.
 * - MecanumWheelCar support.
 * - IMUCarData improved.
 * - Added Voltage handling functions like getVoltageAdjustedSpeedPWM() etc.
 *
 * Version 1.9.0 - 04/2022
 * - Removed all *Compensated functions, compensation now is always active.
 * - Removed StopSpeed from EepromMotorinfoStruct.
 * - Removed StartSpeed.
 * - Renamed *.cpp to *.hpp.
 * - Added and renamed functions.
 * - IMU / MPU6050 support.
 * - Support of off the shelf smart cars.
 * - Added and renamed functions.
 *
 * Version 1.0.0 - 9/2020
 * - Initial version.
 */
#endif // _PWM_DC_MOTOR_H
