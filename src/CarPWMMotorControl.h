/*
 * CarPWMMotorControl.h
 *
 *  Motor control for a car with 2 encoder motors
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _CAR_PWM_MOTOR_CONTROL_H
#define _CAR_PWM_MOTOR_CONTROL_H

#include "EncoderMotor.h"

/*
 * Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning and speed / distance calibration.
 * Connectors point to the rear.
 */
#if defined(USE_MPU6050_IMU)
#include "IMUCarData.h"
#endif
#include <stdint.h>

/*
 * Values for 20 slot encoder discs. Circumference of the wheel is 22.0 cm
 * Distance between two wheels is around 14 cm -> 360 degree are 82 cm
 */
#define MILLIMETER_PER_256_DEGREE_2WD_CARIN_PLACE           320 // we drive both motors here
#define MILLIMETER_PER_256_DEGREE_2WD_CAR                   750
#define MILLIMETER_PER_256_DEGREE_4WD_CAR_IN_PLACE          600 //  670, 860 for NIMH car.
#define MILLIMETER_PER_256_DEGREE_4WD_CAR                  1100 // 1075, 1350 for NIMH car.
#define MILLIMETER_PER_256_DEGREE_4WD_MECANUM_CAR_IN_PLACE  600 // For turns with 4 wheels.
#define MILLIMETER_PER_256_DEGREE_4WD_MECANUM_CAR          1550 // For turns with 2 wheels.

#if !defined(MILLIMETER_PER_256_DEGREE)
#  if defined(CAR_HAS_4_MECANUM_WHEELS)
#define DEFAULT_MILLIMETER_PER_256_DEGREE_IN_PLACE    MILLIMETER_PER_256_DEGREE_4WD_MECANUM_CAR_IN_PLACE
#define DEFAULT_MILLIMETER_PER_256_DEGREE             MILLIMETER_PER_256_DEGREE_4WD_MECANUM_CAR
#  elif defined(CAR_HAS_4_WHEELS)
#define DEFAULT_MILLIMETER_PER_256_DEGREE_IN_PLACE    MILLIMETER_PER_256_DEGREE_4WD_CAR_IN_PLACE
#define DEFAULT_MILLIMETER_PER_256_DEGREE             MILLIMETER_PER_256_DEGREE_4WD_CAR
#  else
#define DEFAULT_MILLIMETER_PER_256_DEGREE_IN_PLACE    (MILLIMETER_PER_256_DEGREE_2WD_CAR / 2)
#define DEFAULT_MILLIMETER_PER_256_DEGREE             MILLIMETER_PER_256_DEGREE_2WD_CAR
#  endif
#endif

#define EEPROM_CAR_INFO_VALID_MARKER_VALUE  A5
struct EepromCarInfoStruct {
    EepromMotorInfoStruct rightMotorInfo;
    EepromMotorInfoStruct leftMotorInfo;
#if !defined(USE_MPU6050_IMU)
    uint16_t MillimeterPer256Degree;
    uint16_t MillimeterPer256DegreeInPlace;
#endif
    uint8_t ValidMarker; // must be A5
};

// turn directions
typedef enum turn_direction {
    TURN_IN_PLACE = DIRECTION_STOP, TURN_FORWARD = DIRECTION_FORWARD, TURN_BACKWARD = DIRECTION_BACKWARD
} turn_direction_t;
//#define TURN_FORWARD    DIRECTION_FORWARD  // 1
//#define TURN_BACKWARD   DIRECTION_BACKWARD // 2
//#define TURN_IN_PLACE   0

class CarPWMMotorControl {
public:

    CarPWMMotorControl();
//    virtual ~CarPWMMotorControl();

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    void init();
#else
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin, uint8_t aLeftMotorForwardPin,
            uint8_t aLeftMotorBackwardPin, uint8_t aLeftMotorPWMPin);
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin, uint8_t aRightInterruptNumber,
            uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin, uint8_t aLeftInterruptNumber);
#endif // USE_ADAFRUIT_MOTOR_SHIELD

    void setDefaultsForFixedDistanceDriving();
    void setDriveSpeedAndSpeedCompensationPWM(uint8_t aDriveSpeedPWM, int8_t aSpeedPWMCompensationRight);
    void setSpeedPWMCompensation(int8_t aSpeedPWMCompensationRight);
    void changeSpeedPWMCompensation(int8_t aSpeedPWMCompensationRightDelta);
    void setDriveSpeedPWM(uint8_t aDriveSpeedPWM);
    void setDriveSpeedPWMFor2Volt(uint16_t aFullBridgeInputVoltageMillivolt);
    void setDriveSpeedPWMFor2Volt(float aFullBridgeInputVoltageMillivolt);

#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
    void getStartSpeedPWM(void (*aLoopCallback)(void)); // aLoopCallback must call readCarDataFromMPU6050Fifo()
    unsigned int getBrakingDistanceMillimeter();
    uint8_t getTurnDistanceHalfDegree();
#endif

#if defined(USE_MPU6050_IMU)
    void updateIMUData();
    void calculateAndPrintIMUOffsets(Print *aSerial);
#endif

#if defined(USE_ENCODER_MOTOR_CONTROL)
    // retrieves values from right motor
    unsigned int getDistanceCount();
    unsigned int getDistanceMillimeter();
#else
    // makes no sense for encoder motor
    void setMillimeterPerSecondForFixedDistanceDriving(uint16_t aMillimeterPerSecond);
#endif

    // If ramp up is not supported, these functions just sets the speed and return immediately
    void startRampUp(uint8_t aRequestedDirection = DIRECTION_FORWARD);
    void setSpeedPWMWithRamp(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void waitForDriveSpeedPWM(void (*aLoopCallback)(void) = nullptr);
    void startRampUpAndWait(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection = DIRECTION_FORWARD,
            void (*aLoopCallback)(void) = nullptr);
    void startRampUpAndWaitForDriveSpeedPWM(uint8_t aRequestedDirection = DIRECTION_FORWARD, void (*aLoopCallback)(void) = nullptr);

    /*
     * For car direction handling
     */
    uint8_t getCarDirection();
    uint8_t CarDirection; // One of DIRECTION_STOP, DIRECTION_FORWARD, DIRECTION_BACKWARD. Exclusively changed by checkAndHandleDirectionChange()

    /*
     * Functions for moving a fixed distance
     */
    // With signed distance
    void startGoDistanceMillimeter(int aRequestedDistanceMillimeter); // only setup values, no movement -> use updateMotors()
    void startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection); // only setup values
    void startGoDistanceMillimeter(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
    uint8_t aRequestedDirection)__attribute__ ((deprecated ("Renamed to startGoDistanceMillimeterWithSpeed()."))); // only setup values
    void goDistanceMillimeter(int aRequestedDistanceMillimeter, void (*aLoopCallback)(void) = nullptr); // Blocking function, uses waitUntilStopped
    void goDistanceMillimeterWithSpeed(uint8_t aRequestedSpeedPWM, int aRequestedDistanceMillimeter, void (*aLoopCallback)(void) = nullptr); // Blocking function, uses waitUntilStopped
    void goDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection,
            void (*aLoopCallback)(void) = nullptr); // Blocking function, uses waitUntilStopped
    void startGoDistanceMillimeterWithSpeed(uint8_t aRequestedSpeedPWM, int aRequestedDistanceMillimeter); // only setup values
    void startGoDistanceMillimeterWithSpeed(uint8_t aRequestedSpeedPWM, unsigned int aRequestedDistanceMillimeter,
            uint8_t aRequestedDirection); // only setup values

    bool checkAndHandleDirectionChange(uint8_t aRequestedDirection); // used internally

    /*
     * Functions for rotation
     */
    void startRotate(int aRotationDegrees, turn_direction_t aTurnDirection, bool aUseSlowSpeed = false);
    void rotate(int aRotationDegrees, turn_direction_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = false,
            void (*aLoopCallback)(void) = nullptr);
    void setMillimeterPer256Degree(uint16_t aMillimeterPerDegree);
    void setMillimeterPer256DegreeInPlace(uint16_t aMillimeterPerDegreeInPlace);

    uint16_t MillimeterPer256Degree; // Use value for 256 degree to have a better resolution and faster division
    uint16_t MillimeterPer256DegreeInPlace;

    bool readCarValuesFromEeprom();
    void writeCarValuesToEeprom();
    void printCalibrationValues(Print *aSerial);

#if defined(USE_MPU6050_IMU)
    IMUCarData IMUData;
    int CarRequestedRotationDegrees; // 0 -> car is moving forward / backward
    int CarTurnAngleHalfDegreesFromIMU; // Read from Gyroscope
    int8_t CarTurn2DegreesPerSecondFromIMU; // Read from Gyroscope
    // Read from Accelerator
    unsigned int CarSpeedCmPerSecondFromIMU;
    unsigned int CarRequestedDistanceMillimeter;
    unsigned int CarDistanceMillimeterFromIMU;
#endif

    bool updateMotors();
    bool updateMotors(void (*aLoopCallback)(void));
    void delayAndUpdateMotors(unsigned int aDelayMillis);

    /*
     * Start/Stop functions
     */
    void startRampDown();
    void stopAndWaitForIt(void (*aLoopCallback)(void) = nullptr); // uses waitUntilStopped()
    void waitUntilStopped(void (*aLoopCallback)(void) = nullptr);

    /*
     * Check motor state functions
     */
    bool isStopped();
    bool isState(uint8_t aState);
    bool isStateRamp(); // MOTOR_STATE_RAMP_UP OR MOTOR_STATE_RAMP_DOWN

    void resetEncoderControlValues();

    /*
     * Functions, which directly call motor functions for both motors
     */
    void setSpeedPWM(uint8_t aRequestedSpeedPWM);
    void setSpeedPWM(int aSignedRequestedSpeedPWMForLeftMotor, int aSignedRequestedSpeedPWMForRightMotor); // cannot be renamed to setSpeedPWMAndDirection()!
    void setDirection(uint8_t aRequestedDirection);

    void setSpeedPWMAndDirection(int aSignedRequestedSpeedPWM);
    void setSpeedPWMAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection);
    void setSpeedPWMWithDeltaAndDirection(uint8_t aRequestedSpeedPWM, uint8_t aRequestedDirection,
            int8_t aSpeedPWMCompensationRightDelta);
    void changeSpeedPWM(uint8_t aRequestedSpeedPWM); // Keeps direction

    void stop(uint8_t aStopMode = STOP_MODE_KEEP); // STOP_MODE_KEEP (take previously defined DefaultStopMode) or STOP_MODE_BRAKE or STOP_MODE_RELEASE
    void setStopMode(uint8_t aStopMode);

#if defined(USE_ENCODER_MOTOR_CONTROL)
    EncoderMotor rightCarMotor; // 40 bytes RAM
    EncoderMotor leftCarMotor;
#else
    PWMDcMotor rightCarMotor;
    PWMDcMotor leftCarMotor;
#endif
};

#if !defined(CAR_HAS_4_MECANUM_WHEELS)
extern CarPWMMotorControl RobotCar;
#endif

#endif // _CAR_PWM_MOTOR_CONTROL_H
