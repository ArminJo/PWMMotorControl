/*
 * EncoderMotor.h
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

#ifndef SRC_ENCODERMOTORCONTROL_H_
#define SRC_ENCODERMOTORCONTROL_H_

#include "PWMDcMotor.h"
#include <stdint.h>

#define SUPPORT_AVERAGE_SPEED
#define AVERAGE_SPEED_SAMPLE_SIZE 20
#define AVERAGE_SPEED_BUFFER_SIZE 21 // one more than samples, because speed is the difference between 2 samples

// maybe useful especially for more than 2 motors
//#define ENABLE_MOTOR_LIST_FUNCTIONS

/*
 * 20 slot Encoder generates 4 to 5 Hz at min speed and 110 Hz at max speed => 200 to 8 ms per period
 */
#define ENCODER_COUNTS_PER_FULL_ROTATION    20
#define ENCODER_SENSOR_TIMEOUT_MILLIS       400L // Timeout for encoder ticks if motor is running
#define ENCODER_SENSOR_RING_MILLIS          4

/*
 * Some factors depending on wheel diameter and encoder resolution
 */
#if ! defined(FACTOR_COUNT_TO_MILLIMETER_INTEGER_DEFAULT)
// Exact value is 220 mm / 20
#define FACTOR_COUNT_TO_MILLIMETER_INTEGER_DEFAULT  ((DEFAULT_CIRCUMFERENCE_MILLIMETER + (ENCODER_COUNTS_PER_FULL_ROTATION / 2)) / ENCODER_COUNTS_PER_FULL_ROTATION) // = 11
#endif

/*
 * The millis per tick have the unit [ms]/ (circumference[cm]/countsPerCircumference) -> ms/cm
 * To get cm/s, use (circumference[cm]/countsPerCircumference) * 1000 / millis per tick
 */
#define SPEED_SCALE_VALUE ((100L * DEFAULT_CIRCUMFERENCE_MILLIMETER) / ENCODER_COUNTS_PER_FULL_ROTATION) // 1100

class EncoderMotor: public PWMDcMotor {
public:

    EncoderMotor();
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    void init(uint8_t aMotorNumber);
    void init(uint8_t aMotorNumber, uint8_t aInterruptNumber);
#else
    void init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin);
    void init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aInterruptNumber);
#endif
//    virtual ~EncoderMotor();

    /*
     * Functions for going a fixed distance, they "overwrite" PWMDCMotor functions
     */
    void startGoDistanceMillimeter(int aRequestedDistanceMillimeter); // Signed distance count
    void startGoDistanceMillimeter(unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
    void startGoDistanceMillimeter(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceMillimeter, uint8_t aRequestedDirection);
    bool updateMotor();

    /*
     * Functions especially for encoder motors
     */
    void synchronizeMotor(EncoderMotor *aOtherMotorControl, unsigned int aCheckInterval); // Computes motor speed compensation value in order to go exactly straight ahead
    static void calibrate(); // Generates a rising ramp and detects the first movement -> this sets StartSpeed / dead band

    /*
     * Encoder interrupt handling
     */
    void handleEncoderInterrupt();
    void attachInterrupt(uint8_t aInterruptPinNumber);
    static void enableINT0AndINT1InterruptsOnRisingEdge();

    uint8_t getDirection();
    unsigned int getDistanceMillimeter();
    unsigned int getDistanceCentimeter();
    unsigned int getBrakingDistanceMillimeter();

    unsigned int getSpeed();
#ifdef SUPPORT_AVERAGE_SPEED
    unsigned int getAverageSpeed();
    unsigned int getAverageSpeed(uint8_t aLengthOfAverage);
#endif

    void printEncoderDataCaption(Print *aSerial);
    bool printEncoderDataPeriodically(Print *aSerial, uint16_t aPeriodMillis);
    void printEncoderData(Print *aSerial);

    void resetEncoderControlValues();
    void resetSpeedValues();

#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
    /*
     * Static convenience functions affecting all motors. If you have 2 motors, better use CarControl
     */
    static bool updateAllMotors();

    static void startGoDistanceMillimeterForAll(int aRequestedDistanceMillimeter);
    static void goDistanceMillimeterForAll(int aRequestedDistanceMillimeter, void (*aLoopCallback)(void));
    static void startRampUpAndWaitForDriveSpeedForAll(uint8_t aRequestedDirection, void (*aLoopCallback)(void));

    static void stopAllMotors(uint8_t aStopMode);
    static void waitUntilAllMotorsStopped(void (*aLoopCallback)(void));
    static void stopAllMotorsAndWaitUntilStopped();

    static bool allMotorsStarted();
    static bool allMotorsStopped();

    /*
     * List for access to all motorControls
     */
    static EncoderMotor *sMotorControlListStart; // Root pointer to list of all motorControls
    static uint8_t sNumberOfMotorControls;

    EncoderMotor * NextMotorControl;
#endif

    /**************************************************************
     * Variables required for going a fixed distance with encoder
     **************************************************************/
    /*
     * Reset() resets all members from TargetDistanceCount to (including) Debug to 0
     */
    unsigned int TargetDistanceMillimeter;
    unsigned int LastTargetDistanceMillimeter;

    /*
     * Distance optocoupler impulse counter. It is reset at startGoDistanceCount if motor was stopped.
     */
    volatile unsigned int EncoderCount;
    volatile unsigned int LastRideEncoderCount; // count of last ride - from start of MOTOR_STATE_RAMP_UP to next MOTOR_STATE_RAMP_UP
    // Flag e.g. for display update control
    volatile unsigned long LastEncoderInterruptMillis; // used internal for debouncing and lock/timeout detection

    /*
     * for speed computation
     * Do not rearrange, since reset is done with memset().
     */
    volatile unsigned long EncoderInterruptDeltaMillis; // Used to get speed
#ifdef SUPPORT_AVERAGE_SPEED
    volatile unsigned int EncoderInterruptMillisArray[AVERAGE_SPEED_BUFFER_SIZE]; // store for 20 deltas
    volatile uint8_t MillisArrayIndex; // Index of the next value to write  == the oldest value to overwrite. 0 to 20|(AVERAGE_SPEED_BUFFER_SIZE-1)
    volatile bool AverageSpeedIsValid; // true if 11 values are written since last timeout
#endif

    // do not delete it!!! It must be the last element in structure and is required for stopMotorAndReset()
    unsigned int Debug;

};

extern EncoderMotor * sPointerForInt0ISR;
extern EncoderMotor * sPointerForInt1ISR;

#endif /* SRC_ENCODERMOTORCONTROL_H_ */

#pragma once

