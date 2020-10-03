/*
 * CarMotorControl.h
 *
 *  Motor control for a car with 2 encoder motors
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

#ifndef CARMOTORCONTROL_H_
#define CARMOTORCONTROL_H_

#include "EncoderMotor.h"
#include <stdint.h>

/*
 * Some factors depending on wheel diameter and encoder resolution
 */
#if ! defined(FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT)
#define FACTOR_CENTIMETER_TO_COUNT_INTEGER_DEFAULT  2 // Exact value is 1.86, but integer saves program space and time
#endif
// Values for 20 slot encoder discs -> 40 ticks per turn. Circumference of the wheel is 21.5 cm, Distance between two wheels is around 13 cm
#define FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT      0.4277777
#define FACTOR_DEGREE_TO_COUNT_4WD_CAR_DEFAULT      0.8 // estimated, with slip

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
    void init(bool aReadFromEeprom = false);
#else
    void init(uint8_t aRightMotorForwardPin, uint8_t aRightMotorBackwardPin, uint8_t aRightPWMPin, uint8_t aLeftMotorForwardPin,
            uint8_t LeftMotorBackwardPin, uint8_t aLeftMotorPWMPin, bool aReadFromEeprom = false);
#endif

    void setDefaultsForFixedDistanceDriving();
    void setValuesForFixedDistanceDriving(uint8_t aStartSpeed, uint8_t aDriveSpeed, int8_t aSpeedCompensationRight);
    void setDriveSpeed(uint8_t aDriveSpeed);

#ifdef USE_ENCODER_MOTOR_CONTROL
    void calibrate();
    // retrieves values from right motor
    unsigned int getDistanceCount();
    int getDistanceCentimeter();
#else
    // makes no sense for encoder motor
    void setDistanceToTimeFactorForFixedDistanceDriving(unsigned int aDistanceToTimeFactor);
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

    void goDistanceCentimeter(int aDistanceCentimeter, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilCarStopped
    void goDistanceCentimeter(unsigned int aDistanceCentimeter, uint8_t aRequestedDirection, void (*aLoopCallback)(void) = NULL); // Blocking function, uses waitUntilCarStopped

    bool checkAndHandleDirectionChange(uint8_t aRequestedDirection); // used internally

    /*
     * Functions for rotation
     */
    void setFactorDegreeToCount(float aFactorDegreeToCount);
    void startRotateCar(int aRotationDegrees, uint8_t aTurnDirection, bool aUseSlowSpeed = true);
    void rotateCar(int aRotationDegrees, uint8_t aTurnDirection = TURN_IN_PLACE, bool aUseSlowSpeed = true,
            void (*aLoopCallback)(void) = NULL);
    float FactorDegreeToCount;

    bool updateMotors();
    void delayAndUpdateMotors(unsigned int aDelayMillis);

    /*
     * Start/Stop functions for infinite distance
     */
    void stopCarAndWaitForIt(void (*aLoopCallback)(void) = NULL); // uses waitUntilCarStopped()
    void waitUntilCarStopped(void (*aLoopCallback)(void) = NULL);

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
    void stopMotors(uint8_t aStopMode = STOP_MODE_KEEP);

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

// Pointer to the last and only! instance for use by ISR
extern CarMotorControl * sCarMotorControlPointerForISR;

#endif /* CARMOTORCONTROL_H_ */

#pragma once

