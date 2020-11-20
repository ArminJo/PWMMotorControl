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
 * 20 slot Encoder generates 10 Hz at min speed and 110 Hz at max speed => 100 to 8 ms per period
 */
#define ENCODER_COUNTS_PER_FULL_ROTATION    20
#define ENCODER_SENSOR_TIMEOUT_MILLIS       200L // Timeout for encoder ticks if motor is running
#define ENCODER_SENSOR_RING_MILLIS          4

#define SPEED_SCALE_VALUE (60000L / ENCODER_COUNTS_PER_FULL_ROTATION) // for computing of CurrentSpeed in rpm

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
    void startGoDistanceCount(int aRequestedDistanceCount); // Signed distance count
    void startGoDistanceCount(unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection);
    void startGoDistanceCount(uint8_t aRequestedSpeed, unsigned int aRequestedDistanceCount, uint8_t aRequestedDirection);
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

    int getSpeed();
#ifdef SUPPORT_AVERAGE_SPEED
    int getAverageSpeed();
#endif

    void resetEncoderControlValues();
    void resetSpeedValues();

#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
    /*
     * Static convenience functions affecting all motors. If you have 2 motors, better use CarControl
     */
    static bool updateAllMotors();

    static void startGoDistanceCountForAll(int aRequestedDistanceCount);
    static void goDistanceCountForAll(int aRequestedDistanceCount, void (*aLoopCallback)(void));
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
    unsigned int TargetDistanceCount;
    unsigned int LastTargetDistanceCount;

    /*
     * Distance optocoupler impulse counter. It is reset at startGoDistanceCount if motor was stopped.
     */
    volatile unsigned int EncoderCount;
    volatile unsigned int LastRideEncoderCount; // count of last ride - from start of MOTOR_STATE_RAMP_UP to next MOTOR_STATE_RAMP_UP
    // Flag e.g. for display update control
    volatile static bool EncoderCountHasChanged;
    volatile unsigned long LastEncoderInterruptMillis; // used internal for debouncing and lock/timeout detection

    /*
     * for speed computation
     */
    volatile unsigned long EncoderInterruptDeltaMillis; // Used to get Speed which is 1/EncoderInterruptDeltaMillis
#ifdef SUPPORT_AVERAGE_SPEED
    volatile unsigned int EncoderInterruptMillisArray[AVERAGE_SPEED_BUFFER_SIZE]; // store for 20 deltas
    volatile uint8_t MillisArrayIndex; // Index of the next value to write  == the oldest value to overwrite
    volatile bool AverageSpeedIsValid; // true if 11 values are written since last timeout
#endif

#ifdef SUPPORT_RAMP_UP
    /*
     * For ramp and state control
     */
    uint8_t RampDeltaPerDistanceCount;
    unsigned int NextChangeMaxTargetCount;      // target count at which next change must be done

    uint8_t DistanceCountAfterRampUp; // number of ticks at the transition from MOTOR_STATE_RAMP_UP to MOTOR_STATE_FULL_SPEED to be used for computing ramp down start ticks
    unsigned int DebugCount;                    // for different debug purposes of ramp optimisation
    uint8_t DebugSpeedAtTargetCountReached;     // for debug of ramp down
#endif

    // do not delete it!!! It must be the last element in structure and is required for stopMotorAndReset()
    unsigned int Debug;

};

extern EncoderMotor * sPointerForInt0ISR;
extern EncoderMotor * sPointerForInt1ISR;

#endif /* SRC_ENCODERMOTORCONTROL_H_ */

#pragma once

