/*
 * EncoderMotor.h
 *
 *  Created on: 16.09.2016
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

#ifndef SRC_ENCODERMOTORCONTROL_H_
#define SRC_ENCODERMOTORCONTROL_H_

#include "PWMDcMotor.h"
#include <stdint.h>

// maybe useful especially for more than 2 motors
//#define ENABLE_MOTOR_LIST_FUNCTIONS

/*
 * Encoders generate 110 Hz at max speed => 8 ms per period
 * since duty cycle of encoder disk impulse is 1/3 choose 3 millis as ringing mask.
 * 3 ms means 2.1 to 3.0 ms depending on the Arduino ms trigger.
 */
#define ENCODER_SENSOR_MASK_MILLIS 3
#define VELOCITY_SCALE_VALUE 500 // for computing of CurrentVelocity
// Timeout for encoder ticks if motor is running
#define ENCODER_TICKS_TIMEOUT_MILLIS 500

class EncoderMotor: public PWMDcMotor {
public:

    EncoderMotor();
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    void init(uint8_t aMotorNumber, bool aReadFromEeprom = false);
#else
    void init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin, uint8_t aMotorNumber = 0);
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
    void synchronizeMotor(EncoderMotor * aOtherMotorControl, unsigned int aCheckInterval); // Computes motor speed compensation value in order to go exactly straight ahead
    static void calibrate(); // Generates a rising ramp and detects the first movement -> this sets StartSpeed / dead band

    /*
     * Encoder interrupt handling
     */
    void handleEncoderInterrupt();
    static void enableINT0AndINT1Interrupts();
    static void enableInterruptOnBothEdges(uint8_t aIntPinNumber);

    void resetControlValues(); // Shutdown and reset all control values and sets direction to forward

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

    /*
     * Flags for display update control
     */
    volatile static bool EncoderTickCounterHasChanged;

    /**********************************************************
     * Variables required for going a fixed distance
     **********************************************************/
    /*
     * Reset() resets all members from CurrentVelocity to (including) Debug to 0
     */
    int CurrentVelocity; // VELOCITY_SCALE_VALUE / tDeltaMillis

    unsigned int TargetDistanceCount;
    unsigned int LastTargetDistanceCount;

    /*
     * Distance optocoupler impulse counter. It is reset at startGoDistanceCount if motor was stopped.
     */
    volatile unsigned int EncoderCount;
    volatile unsigned int LastRideEncoderCount; // count of last ride - from start of MOTOR_STATE_RAMP_UP to next MOTOR_STATE_RAMP_UP
    // The next value is volatile, but volatile increases the code size by 20 bites without any logical improvement
    unsigned long EncoderTickLastMillis; // used for debouncing and lock/timeout detection

#ifdef SUPPORT_RAMP_UP
    /*
     * For ramp and state control
     */
    uint8_t RampDeltaPerDistanceCount;
    unsigned int NextChangeMaxTargetCount;      // target count at which next change must be done

    uint8_t DistanceCountAfterRampUp; // number of ticks at the transition from MOTOR_STATE_RAMP_UP to MOTOR_STATE_FULL_SPEED to be used for computing ramp down start ticks
    unsigned int DebugCount;                    // for different debug purposes of ramp optimisation
    uint8_t DebugSpeedAtTargetCountReached; // for debug of ramp down
#endif

    // do not delete it!!! It must be the last element in structure and is required for stopMotorAndReset()
    unsigned int Debug;

};

#endif /* SRC_ENCODERMOTORCONTROL_H_ */

#pragma once

