/*
 * EncoderMotor.h
 *
 *  Created on: 16.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
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

/*
 * Encoders generate 110 Hz at max speed => 8 ms per period
 * since duty cycle of encoder disk impulse is 1/3 choose 3 millis as ringing mask.
 * 3 ms means 2.1 to 3.0 ms depending on the Arduino ms trigger.
 */
#define ENCODER_SENSOR_MASK_MILLIS 3
#define VELOCITY_SCALE_VALUE 500

/*
 * Motor Control
 */
// the smaller the value the steeper the ramp
#define RAMP_UP_UPDATE_INTERVAL_MILLIS 16
// gives 16 steps a 16 millis for ramp up => 256 milliseconds
#define RAMP_UP_VALUE_DELTA ((CurrentDriveSpeed - StartSpeed) / 16)

// timeout after ramp down was finished to switch off motor
#define RAMP_DOWN_TIMEOUT_MILLIS 500

// Ticks for ramp down if external stop requested
#define RAMP_DOWN_MIN_TICKS 3

// Safety net. If difference between targetCount and current distanceCount is less than, adjust new targetCount
#define MAX_DISTANCE_DELTA 8

#define INFINITE_DISTANCE_COUNT (32000) // we have an int that is multiplied with FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT and

#define MOTOR_STATE_STOPPED 0
#define MOTOR_STATE_RAMP_UP 1
#define MOTOR_STATE_FULL_SPEED 2
#define MOTOR_STATE_RAMP_DOWN 3

//#define MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS 500
#define MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS 100

//#define SERVO_CURRENT_LOW_THRESHOLD 100
#define SERVO_INITIAL_DELAY 5

#define SERVO_CURRENT_LOW_MILLIS_FOR_SERVO_STOPPED 12

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
     * Functions for going a fixed distance
     */
    void initGoDistanceCount(uint16_t aDistanceCount, uint8_t aRequestedDirection);
    // not used yet
    void initGoDistanceCount(int16_t aDistanceCount);

    bool updateMotor();
    void synchronizeMotor(EncoderMotor * aOtherMotorControl, uint16_t aCheckInterval); // Computes motor speed compensation value in order to go exactly straight ahead
    static void calibrate(); // Generates a rising ramp and detects the first movement -> this sets StartSpeed / dead band

    /*
     * Encoder interrupt handling
     */
    void handleEncoderInterrupt();
    static void enableINT0AndINT1Interrupts();
    static void enableInterruptOnBothEdges(uint8_t aIntPinNumber);

    /*
     * Direct motor control
     */
    void setSpeedCompensated(uint8_t aRequestedSpeed, uint8_t aRequestedDirection);
    // not used yet
    void setSpeedCompensated(int aRequestedSpeed);

    /**
     * Sets CurrentSpeed (PWM) to 0 and driver mode to aStopMode
     * Resets State and TargetDistanceCount variables.
     * @param aStopMode STOP_MODE_KEEP (take previously defined StopMode) or MOTOR_BRAKE or MOTOR_RELEASE
     */
    void stop(uint8_t aStopMode = STOP_MODE_KEEP);
    void resetControlValues(); // Shutdown and reset all control values and sets direction to forward

#ifdef ENABLE_MOTOR_LIST_FUNCTIONS
    /*
     * Static convenience functions affecting all motors. If you have 2 motors, better use CarControl
     */
    static void updateAllMotors();

    static void initGoDistanceCountForAll(int aDistanceCount);
    static void goDistanceCountForAll(int aDistanceCount, void (*aLoopCallback)(void));
    static void startAndWaitForFullSpeedForAll(void);

    static void stopAllMotors(uint8_t aStopMode);
    static void stopAllMotorsAndReset();
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
     * Reset() resets all members from CurrentDriveSpeed to (including) Debug to 0
     */
    uint8_t CurrentDriveSpeed; // DriveSpeed - SpeedCompensation; The DriveSpeed used for current movement. Can be set for eg. turning which better performs with reduced DriveSpeed
    volatile int16_t CurrentVelocity;

    uint8_t State; // MOTOR_STATE_STOPPED, MOTOR_STATE_RAMP_UP, MOTOR_STATE_FULL_SPEED, MOTOR_STATE_RAMP_DOWN
    uint16_t TargetDistanceCount;
    uint16_t LastTargetDistanceCount;

    /*
     * Distance optocoupler impulse counter. It is reset at initGoDistanceCount if motor was stopped.
     */
    volatile uint16_t EncoderCount;
    uint16_t LastRideEncoderCount; // count of last ride - from start of MOTOR_STATE_RAMP_UP to next MOTOR_STATE_RAMP_UP
    unsigned long EncoderTickLastMillis; // used for debouncing and lock/timeout detection

    /*
     * For ramp and state control
     */
    uint8_t RampDelta;
    uint8_t RampDeltaPerDistanceCount;
    uint16_t NextChangeMaxTargetCount; // target count at which next change must be done
    unsigned long NextRampChangeMillis;

    uint8_t DistanceCountAfterRampUp; // number of ticks at the transition from MOTOR_STATE_RAMP_UP to MOTOR_STATE_FULL_SPEED to be used for computing ramp down start ticks
    uint16_t DebugCount;
    uint8_t SpeedAtTargetCountReached;

    // do not delete it!!! It must be the last element in structure and is required for stopMotorAndReset()
    uint16_t Debug;

};

#endif /* SRC_ENCODERMOTORCONTROL_H_ */
