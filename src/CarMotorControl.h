/*
 * CarMotorControl.h
 *
 *  Motor control for a car with 2 encoder motors
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

#ifndef CARMOTORCONTROL_H_
#define CARMOTORCONTROL_H_

#include "EncoderMotor.h"
#include <stdint.h>

/*
 * Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning and speed / distance calibration.
 * Connectors point to the rear.
 */
//#define USE_MPU6050_IMU

/*
 * Comment this out if the y axis of the GY-521 MPU6050 breakout board points forward / backward, i.e. connectors are at the right.
 */
//#define USE_ACCELERATOR_Y_FOR_SPEED
/*
 * Some factors depending on wheel diameter and encoder resolution
 */
#if ! defined(FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT)
// Exact value is 215 mm / 20 but integer saves program space and time
#define FACTOR_MILLIMETER_TO_COUNT_INTEGER_DEFAULT  (DEFAULT_CIRCUMFERENCE_MILLIMETER / ENCODER_COUNTS_PER_FULL_ROTATION) // = 21
#define FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT  (FACTOR_MILLIMETER_TO_COUNT_INTEGER_DEFAULT / 10) // = 2
#endif
/*
 * Values for 20 slot encoder discs. Circumference of the wheel is 21.5 cm
 * Distance between two wheels is around 14 cm -> 360 degree are 82 cm
 * 360 degree are (82 / 21.5) * 20 counts
 */
#define FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT      0.2118863
#define FACTOR_DEGREE_TO_COUNT_4WD_CAR_DEFAULT      0.4 // estimated, with slip

#if ! defined(FACTOR_DEGREE_TO_COUNT_DEFAULT)
#  if defined(CAR_HAS_4_WHEELS)
#define FACTOR_DEGREE_TO_COUNT_DEFAULT  FACTOR_DEGREE_TO_COUNT_4WD_CAR_DEFAULT
#  else
#define FACTOR_DEGREE_TO_COUNT_DEFAULT  FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT
#  endif
#endif

// turn directions
#define TURN_FORWARD    DIRECTION_FORWARD  // 0
#define TURN_BACKWARD   DIRECTION_BACKWARD // 1
#define TURN_IN_PLACE   2

class CarMotorControl {
public:

    CarMotorControl();
//    virtual ~CarMotorControl();

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    void init();
#else
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin, uint8_t aLeftMotorForwardPin,
            uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin);
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin, uint8_t aRightInterruptNumber,
            uint8_t aLeftMotorForwardPin, uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin, uint8_t aLeftInterruptNumber);
#endif // USE_ADAFRUIT_MOTOR_SHIELD

#ifdef USE_MPU6050_IMU
    void initIMU();
    void printIMUOffsets(Print *aSerial);
#endif

    void setDefaultsForFixedDistanceDriving();
    void setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, int8_t aSpeedCompensationRight);
    void changeSpeedCompensation(int8_t aSpeedCompensationRight);
    void setDriveSpeed(uint8_t aDriveSpeed);

    void writeMotorValuesToEeprom();
    void readMotorValuesFromEeprom();

#ifdef USE_ENCODER_MOTOR_CONTROL
    void calibrate();
    // retrieves values from right motor
    unsigned int getDistanceCount();
    int getDistanceCentimeter();
#else
    // makes no sense for encoder motor
    void setMillisPerDistanceCountForFixedDistanceDriving(uint8_t aMillisPerDistanceCount);
#endif

#ifdef SUPPORT_RAMP_UP
    void startRampUp(uint8_t aRequestedDirection = DIRECTION_FORWARD);
    void startRampUp(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);
    void waitForDriveSpeed(void (*aLoopCallback)(void) = NULL);
#endif
    // If ramp up is not supported, these functions just sets the speed and return immediately
    void startRampUpAndWait(uint8_t aRequestedSpeed, uint8_t aRequestedDirection = DIRECTION_FORWARD,
            void (*aLoopCallback)(void) = NULL);
    void startRampUpAndWaitForDriveSpeed(uint8_t aRequestedDirection = DIRECTION_FORWARD, void (*aLoopCallback)(void) = NULL);

    /*
     * For car direction handling
     */
    uint8_t getCarDirectionOrBrakeMode();
    uint8_t CarDirectionOrBrakeMode;

    /*
     * Functions for moving a fixed distance
     */
    // With signed distance
    void startGoDistanceCentimeter(uint8_t aRequestedSpeed, unsigned int aDistanceCentimeter, uint8_t aRequestedDirection); // only setup values
    void startGoDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection); // only setup values
    void startGoDistanceCentimeter(int aDistanceCentimeter); // only setup values, no movement -> use updateMotors()

    void goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilStopped
    void goDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilStopped

    bool checkAndHandleDirectionChange(uint8_t aRequestedDirection); // used internally

    /*
     * Functions for rotation
     */
    void setFactorDegreeToCount(float aFactorDegreeToCount);
    void startRotate(int aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed);
    void rotate(int aRotationDegrees, uint8_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = true,
            void (*aLoopCallback)(void) = NULL);
#ifdef USE_MPU6050_IMU
    int CarRotationDegrees; // 0 -> car is moving forward / backward
#else
    float FactorDegreeToCount;
#endif

    bool updateMotors();
    bool updateMotors(void (*aLoopCallback)(void));
    void delayAndUpdateMotors(unsigned int aDelayMillis);

    /*
     * Start/Stop functions
     */
    void stopAndWaitForIt(void (*aLoopCallback)(void) = NULL); // uses waitUntilStopped()
    void waitUntilStopped(void (*aLoopCallback)(void) = NULL);

    /*
     * Check motor state functions
     */
    bool isStopped();
    bool isState(uint8_t aState);
    bool isStateRamp(); // MOTOR_STATE_RAMP_UP OR MOTOR_STATE_RAMP_DOWN

    void resetControlValues();

    /*
     * Functions, which directly call motor functions for both motors
     */
    void setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);
    void setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection, int8_t aLeftRightSpeed);
    void stop(uint8_t aStopMode = STOP_MODE_KEEP); // STOP_MODE_KEEP (take previously defined DefaultStopMode) or MOTOR_BRAKE or MOTOR_RELEASE

    void setSpeed(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);
    void setStopMode(uint8_t aStopMode);
    void setSpeed(int aRequestedSpeed);
    void setSpeedCompensated(int aRequestedSpeed);

#ifdef USE_ENCODER_MOTOR_CONTROL
    EncoderMotor rightCarMotor; // 40 bytes RAM
    EncoderMotor leftCarMotor;
#else
    PWMDcMotor rightCarMotor;
    PWMDcMotor leftCarMotor;
#endif
};

#endif /* CARMOTORCONTROL_H_ */

#pragma once

