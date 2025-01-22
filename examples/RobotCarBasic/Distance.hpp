/*
 * Distance.hpp
 *
 *  Contains all distance measurement functions.
 *
 *  Copyright (C) 2020-2024  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  PWMMotorControl and Arduino-RobotCar are free software: you can redistribute it and/or modify
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

#ifndef _ROBOT_CAR_DISTANCE_HPP
#define _ROBOT_CAR_DISTANCE_HPP

#if !defined(DISTANCE_CHANGE_THRESHOLD_CENTIMETER)
#define DISTANCE_CHANGE_THRESHOLD_CENTIMETER    0 // set sEffectiveDistanceJustChanged to true if distance changed more than this threshold
#endif
#include "Distance.h"
#include "HCSR04.hpp" // include sources
#include "RobotCarConfigurations.h" // helps the pretty printer / Ctrl F

#if defined(CAR_HAS_DISTANCE_SERVO)
#  if defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
#include "LightweightServo.hpp"
#    if defined(_LIGHTWEIGHT_SERVO_HPP)
LightweightServo DistanceServo;    // The pan servo instance for distance sensor
#    else // LightweightServo is not applicable for this CPU
#undef USE_LIGHTWEIGHT_SERVO_LIBRARY
#    endif
#  endif // defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)

#  if !defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
Servo DistanceServo;    // The pan servo instance for distance sensor
#  endif
uint8_t sLastDistanceServoAngleInDegrees; // 0 - 180 required for optimized delay for servo repositioning. Only set by DistanceServoWriteAndWaitForStop()
#endif // defined(CAR_HAS_DISTANCE_SERVO)

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
// Default is IR. One of DISTANCE_SOURCE_MODE_MINIMUM, DISTANCE_SOURCE_MODE_MAXIMUM, DISTANCE_SOURCE_MODE_US or DISTANCE_SOURCE_MODE_IR_OR_TOF
uint8_t sDistanceSourceMode = DISTANCE_SOURCE_MODE_DEFAULT;
#endif

#if defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#  if !defined(TOF_OFFSET_MILLIMETER)
//#define TOF_OFFSET_MILLIMETER   10 // The offset measured manually or by calibrateOffset(). Offset = RealDistance - MeasuredDistance
#  endif

#  if defined(__AVR__) && defined(USE_SOFT_I2C_MASTER)
#undef USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE // just in case...
#include "SoftI2CMasterConfig.h"
#include "SoftI2CMaster.h"
VL53L1X sToFDistanceSensor(-1, -1); // 100 kHz
#  else
// removing usage of SFEVL53L1X wrapper class saves 794 bytes
VL53L1X sToFDistanceSensor(&Wire, -1, -1); // 100 kHz
#  endif
#endif // defined(CAR_HAS_TOF_DISTANCE_SENSOR)

#include "pitches.h"    // for pentatonic feedback mode

bool sDoSlowScan = false;

uint8_t sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
uint8_t sUSDistanceTimeoutCentimeter; // The value to display if sUSDistanceCentimeter == DISTANCE_TIMEOUT_RESULT, we have different timeouts
uint8_t sIROrTofDistanceCentimeter;
uint8_t sEffectiveDistanceCentimeter;   // Depends on sDistanceSourceMode
bool sEffectiveDistanceJustChanged;     // Is set and reset by getDistanceAsCentimeter() and scanTarget()

uint8_t sRawForwardDistancesArray[3];   // From 0 (70 degree, right) to 2 (110 degree, left) with steps of 20 degrees
int8_t sComputedRotation;

ForwardDistancesInfoStruct sForwardDistancesInfo;

/*
 * This initializes the pins too
 */
void initDistance() {
    getDistanceModesFromPins();

#if defined(CAR_HAS_DISTANCE_SERVO)
    DistanceServo.attach(DISTANCE_SERVO_PIN);
#endif

#if defined(US_SENSOR_SUPPORTS_1_PIN_MODE)
    initUSDistancePin (TRIGGER_OUT_PIN);
#else
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);
#endif

#if defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#  if defined(USE_BLUE_DISPLAY_GUI)
    if (sToFDistanceSensor.VL53L1X_SensorInit() != 0) { //Begin returns 0 on a good init
        BlueDisplay1.debug("ToF sensor connect failed!");
    }
#  endif
    // Short mode max distance is limited to 1.3 m but better ambient immunity. Above 1.3 meter we get error 4 (wrap around).
    sToFDistanceSensor.VL53L1X_SetDistanceMode(1); // 1 for Mode short
    //sToFDistanceSensor.setDistanceModeLong(); // default
#if defined(TOF_OFFSET_MILLIMETER)
    sToFDistanceSensor.VL53L1X_SetOffset(TOF_OFFSET_MILLIMETER);
#endif

    /*
     * The minimum timing budget is 20 ms for short distance mode and 33 ms for medium and long distance modes.
     * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
     * This function must be called after SetDistanceMode.
     */
    sToFDistanceSensor.VL53L1X_SetTimingBudgetInMs(33);
    sToFDistanceSensor.VL53L1X_StartRanging();
#endif
}

/*
 * Check if tone has to be changed and play according to sDistanceFeedbackMode
 */
void playDistanceFeedbackTone(uint8_t aCentimeter) {
    /*
     * Update tone according to sDistanceFeedbackMode
     */
    if (sDistanceFeedbackMode != DISTANCE_FEEDBACK_NO_TONE) {
        if (aCentimeter == DISTANCE_TIMEOUT_RESULT) {
            noTone(BUZZER_PIN);
        } else {
            /*
             * Compute frequency
             */
            int tFrequency;
            if (sDistanceFeedbackMode == DISTANCE_FEEDBACK_PENTATONIC) {
                /*
                 * Map distance to an index in a pentatonic pitch table
                 */
                uint8_t tIndex = map(aCentimeter, 0, 100, 0, ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1);
                if (tIndex > ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1) {
                    tIndex = ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1;
                }
                tFrequency = NoteC5ToC7Pentatonic[tIndex]; // 523 to 2093 Hz for 0 to 100 cm
            } else {
                /*
                 * DISTANCE_FEEDBACK_CONTINUOUSLY. Play feedback tone proportional to measured distance
                 */
#if defined(FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) && defined(FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER)
                if (aCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
                    tFrequency = map(aCentimeter, 0, FOLLOWER_DISTANCE_MINIMUM_CENTIMETER, 100, 200);
                } else if (aCentimeter > FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
                    tFrequency = map(aCentimeter, FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER, 200, 2000, 6000);
                } else {
                    tFrequency = map(aCentimeter, FOLLOWER_DISTANCE_MINIMUM_CENTIMETER, FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER, 440,
                            880); // 1 octaves for gap
                }
#else
                tFrequency = map(aCentimeter, 0, 100, 110, 7040); // 6 octaves per meter
#endif
            }
            /*
             * Generate tone
             */
            tone(BUZZER_PIN, tFrequency);
        }
    }
}

/*
 * Print without a newline
 * @return true if sEffectiveDistanceJustChanged is true, i.e. something was printed
 */
bool printDistanceIfChanged(Print *aSerial) {
    if (sEffectiveDistanceJustChanged) {
        if (sEffectiveDistanceCentimeter == DISTANCE_TIMEOUT_RESULT) {
            aSerial->println("Distance timeout ");
        } else {
            aSerial->print("Distance=");
            aSerial->print(sEffectiveDistanceCentimeter);
            aSerial->print("cm ");
        }
    }
    return sEffectiveDistanceJustChanged;
}

/**
 * Get distance from US, IR and TOF sensors.
 * If two sensors are available, sDistanceSourceMode defines, which value to take.
 * Modes are US, IR_OR_TOF, MINIMUM and MAXIMUM. Default is IR_OR_TOF.
 * @param   aDistanceTimeoutCentimeter   - The maximum distance acquired.
 * @param   aWaitForCurrentMeasurmentToEnd   - Used for IR Distance sensors: If true, wait for the current measurement to end, since the sensor was recently moved.
 * @param   aMinimumUSDistanceForMinimumMode   - In cm. Only for DISTANCE_SOURCE_MODE_MINIMUM. If US distance is lower, take IR Distance.
 * @param   aDoShow   - Show as distance bar(s) on GUI
 * @return  Distance in centimeter @20 degree celsius (time in us/58.25)
 *          0 / DISTANCE_TIMEOUT_RESULT if timeout or pins are not initialized*
 *          sUSDistanceCentimeter
 *          sIROrTofDistanceCentimeter
 *          sEffectiveDistanceCentimeter
 *          sEffectiveDistanceJustChanged
 *          sUSDistanceTimeoutCentimeter
 */
unsigned int getDistanceAsCentimeter(uint8_t aDistanceTimeoutCentimeter, bool aWaitForCurrentMeasurementToEnd,
        uint8_t aMinimumUSDistanceForMinimumMode, bool aDoShow) {
    (void) aDoShow; // to avoid compiler warnings

#if !defined(CAR_HAS_IR_DISTANCE_SENSOR)
    (void) aWaitForCurrentMeasurementToEnd; // suppress compiler warnings
#endif
#if !(defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
    (void) aMinimumUSDistanceForMinimumMode; // suppress compiler warnings
#endif
#if (defined(CAR_HAS_TOF_DISTANCE_SENSOR))
    if (sDistanceSourceMode != DISTANCE_SOURCE_MODE_US) {
        sToFDistanceSensor.VL53L1X_StartRanging(); // start TOF measurement at start of function
    }
#endif

    /*
     * Always get US distance
     */
    sUSDistanceTimeoutCentimeter = aDistanceTimeoutCentimeter;
    uint8_t tUSCentimeter = getUSDistanceAsCentimeterWithCentimeterTimeout(aDistanceTimeoutCentimeter);
    if (tUSCentimeter == DISTANCE_TIMEOUT_RESULT) {
        tUSCentimeter = aDistanceTimeoutCentimeter;
    }
    sUSDistanceCentimeter = tUSCentimeter; // overwrite value set by getUSDistanceAsCentimeterWithCentimeterTimeout()

#if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
    uint8_t tIRCentimeter;
#  if !defined(USE_BLUE_DISPLAY_GUI)
    /*
     * Only get IR or TOF distance if used later
     */
    if (sDistanceSourceMode != DISTANCE_SOURCE_MODE_US) {
#  endif
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
        tIRCentimeter = getIRDistanceAsCentimeter(aWaitForCurrentMeasurementToEnd);
        if (tIRCentimeter == DISTANCE_TIMEOUT_RESULT) {
            tIRCentimeter = aDistanceTimeoutCentimeter;
        }
        sIROrTofDistanceCentimeter = tIRCentimeter;

#  elif defined(CAR_HAS_TOF_DISTANCE_SENSOR)
        tIRCentimeter = readToFDistanceAsCentimeter();
        sIROrTofDistanceCentimeter = tIRCentimeter;
#  endif
#  if !defined(USE_BLUE_DISPLAY_GUI)
    }
#  else

    if (aDoShow) {
        /*
         * Show distances as bars on GUI
         */
#    if defined(CAR_HAS_US_DISTANCE_SENSOR)
        showUSDistance();       // Here we have a dedicated US distance bar
#    endif
        showIROrTofDistance();  // Here we always have IR or TOF sensor bar
    }
#  endif // !defined(USE_BLUE_DISPLAY_GUI)

    /*
     * IR or TOF sensor is used, we check for distance source mode
     * (take IR or minimum or maximum of the US and (IR or TOF) values)
     */
    uint8_t tCentimeterToReturn = sUSDistanceCentimeter; // using sUSDistanceCentimeter here instead of tUSCentimeter saves 6 bytes
    if (sDistanceSourceMode == DISTANCE_SOURCE_MODE_IR_OR_TOF) {
        tCentimeterToReturn = tIRCentimeter;
    } else if (sDistanceSourceMode == DISTANCE_SOURCE_MODE_MINIMUM) {
        // Scan mode MINIMUM => Take the minimum of the US and IR or TOF values
        if (tCentimeterToReturn > tIRCentimeter || tCentimeterToReturn <= aMinimumUSDistanceForMinimumMode) {
            tCentimeterToReturn = tIRCentimeter;
        }
    } else {
        // Scan mode MAXIMUM => Take the maximum of the US and IR or TOF values
        if (tCentimeterToReturn < tIRCentimeter) {
            tCentimeterToReturn = tIRCentimeter;
        }
    }
#else
#  if defined(USE_BLUE_DISPLAY_GUI) && defined(CAR_HAS_US_DISTANCE_SENSOR)
    showUSDistance();       // Here we have a dedicated US distance bar
#  endif
    uint8_t tCentimeterToReturn = tUSCentimeter;
#endif // (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))

    auto tLastEffectiveDistanceCentimeter = sEffectiveDistanceCentimeter;
    if (abs(tLastEffectiveDistanceCentimeter - tCentimeterToReturn) > DISTANCE_CHANGE_THRESHOLD_CENTIMETER) {
        sEffectiveDistanceJustChanged = true;
        sEffectiveDistanceCentimeter = tCentimeterToReturn;
    } else {
        sEffectiveDistanceJustChanged = false;
    }
    return tCentimeterToReturn;
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
#if !defined(DISTANCE_TIMEOUT_RESULT)
#define DISTANCE_TIMEOUT_RESULT                   0
#endif
/**
 * The Sharp 1080 takes 39 ms for each measurement cycle
 * @return     DISTANCE_TIMEOUT_RESULT on values > IR_SENSOR_TIMEOUT_CENTIMETER
 */
uint8_t getIRDistanceAsCentimeter(bool aWaitForCurrentMeasurementToEnd) {
    if (aWaitForCurrentMeasurementToEnd) {
        /*
         * Check for a voltage change which indicates that a new measurement is started
         */
        int16_t tOldValue = analogRead(IR_DISTANCE_SENSOR_PIN); // 100 us
        uint32_t tStartMillis = millis();
        do {
            int16_t tNewValue = analogRead(IR_DISTANCE_SENSOR_PIN); // 100 us
            if (abs(tOldValue-tNewValue) > IR_SENSOR_NEW_MEASUREMENT_THRESHOLD) {
                // assume, that voltage has changed because of the end of a measurement
                break;
            }
#  if defined(USE_BLUE_DISPLAY_GUI)
            loopGUI();
#  endif
        } while (millis() - tStartMillis <= IR_SENSOR_MEASUREMENT_TIME_MILLIS);
        // now a new measurement has started, wait for the result
#if defined(USE_BLUE_DISPLAY_GUI)
        delayAndLoopGUI(IR_SENSOR_NEW_MEASUREMENT_THRESHOLD); // the IR sensor takes 39 ms for one measurement
#  else
        delay(IR_SENSOR_NEW_MEASUREMENT_THRESHOLD); // the IR sensor takes 39 ms for one measurement
#  endif
    }

    float tVolt = analogRead(IR_DISTANCE_SENSOR_PIN); // 100 us
    // tVolt * 0.004887585 = 5(V) for tVolt == 1023
#  if defined(IR_SENSOR_TYPE_100550)
    uint16_t tDistanceCentimeter;
#  else
    uint8_t tDistanceCentimeter;
#  endif
#  if defined(IR_SENSOR_TYPE_430) // 4 to 30 cm, 18 ms, GP2YA41SK0F
    tDistanceCentimeter =  (12.08 * pow(tVolt * 0.004887585, -1.058)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
    if(tDistanceCentimeter > IR_SENSOR_TIMEOUT_CENTIMETER) { //35 cm
        tDistanceCentimeter = DISTANCE_TIMEOUT_RESULT;
    }
#  elif defined(IR_SENSOR_TYPE_1080)    // 10 to 80 cm, GP2Y0A21YK0F
    tDistanceCentimeter = (29.988 * pow(tVolt * 0.004887585, -1.173)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
    if (tDistanceCentimeter > IR_SENSOR_TIMEOUT_CENTIMETER) { // 90 cm
        tDistanceCentimeter = DISTANCE_TIMEOUT_RESULT;
    }
//    return 4800/(analogRead(IR_DISTANCE_SENSOR_PIN)-20);    // see https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp

#  elif defined(IR_SENSOR_TYPE_20150) // 20 to 150 cm, 18 ms, GP2Y0A02YK0F
    // Model 20150 - Do not forget to add at least 100uF capacitor between the Vcc and GND connections on the sensor
    tDistanceCentimeter =  (60.374 * pow(tVolt * 0.004887585, -1.16)) + 0.5;// see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
    if(tDistanceCentimeter > IR_SENSOR_TIMEOUT_CENTIMETER) { // 160 cm
        tDistanceCentimeter = DISTANCE_TIMEOUT_RESULT;
    }
#  elif defined(IR_SENSOR_TYPE_100550) // 100 to 550 cm, 18 ms, GP2Y0A710K0F
    tDistanceCentimeter =  1.0 / (((tVolt * 0.004887585 - 1.1250)) / 137.5);
    if(tDistanceCentimeter > IR_SENSOR_TIMEOUT_CENTIMETER) { // 255 cm to guarantee an 8 bit result
        tDistanceCentimeter = DISTANCE_TIMEOUT_RESULT;
    }
#  else
#error Define one of IR_SENSOR_TYPE_430, IR_SENSOR_TYPE_1080, IR_SENSOR_TYPE_20150 or IR_SENSOR_TYPE_100550
#  endif
    return tDistanceCentimeter;
}
#endif // CAR_HAS_IR_DISTANCE_SENSOR

#if defined(CAR_HAS_TOF_DISTANCE_SENSOR)
/*
 * No start of measurement, just read result.
 */
uint8_t readToFDistanceAsCentimeter() {
    uint8_t i = 0; // for timeout

    // we have set  a timing budget of 33 ms
    while (i < 20) {
        uint8_t tDataReady;
//        sToFDistanceSensor.VL53L1X_CheckForDataReady(&tDataReady); // checkForDataReady needs 1.1 ms
        sToFDistanceSensor.VL53L1_RdByte(sToFDistanceSensor.Device, GPIO__TIO_HV_STATUS, &tDataReady);
        if (tDataReady & 1) {
            break;
        }
#  if defined(USE_BLUE_DISPLAY_GUI)
        delayAndLoopGUI(4);
#else
        delay(4);
#endif
        i++;
    }

    // Here GPIO__TIO_HV_STATUS is 0x03
    sToFDistanceSensor.VL53L1X_ClearInterrupt(); // I2C 0086 + 0x01
    // Now GPIO__TIO_HV_STATUS is 0x02
    uint8_t tStatus;
//    sToFDistanceSensor.VL53L1X_GetRangeStatus(&tStatus);
    uint16_t tDistance;
    sToFDistanceSensor.VL53L1X_GetDistance(&tDistance); //Get the result of the measurement from the sensor in millimeter
    sToFDistanceSensor.VL53L1X_GetRangeStatus(&tStatus); // I2c 0x0089 -> 0x09
    if (tStatus != 0) {
        if (tStatus == 4) {
            // Wrap around in mode short -> more than 130 cm
            tDistance = 1300;
        } else {
            tDistance = 10;
        }
    }
//    tone(SPEAKER_PIN, tDistance + 500);
    return tDistance / 10;
}

uint8_t getToFDistanceAsCentimeter() {
//    sToFDistanceSensor.VL53L1X_StartRanging();
    return readToFDistanceAsCentimeter();
}

#endif // CAR_HAS_TOF_DISTANCE_SENSOR

#if defined(CAR_HAS_DISTANCE_SERVO)
/*
 * Moves servo, wait for stop and get distance with
 */
unsigned int moveServoAndGetDistance(uint8_t aTargetDegrees, uint8_t aDistanceTimeoutCentimeter) {
    DistanceServoWriteAndWaitForStop(aTargetDegrees, true);
    return getDistanceAsCentimeter(aDistanceTimeoutCentimeter, true);
}

//#define USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
/**
 * Handles overflow, no movement
 * servo trim value, servo mounted head down, and then does a Servo.write().
 * Overshoot handling is is still experimental and disabled by default.
 * If doWaitForStop a trailing delay of 8 or 16 times delta degree depending on value of sDoSlowScan.
 * Sets and uses sLastDistanceServoAngleInDegrees to enable optimized servo movement and delay.
 *
 * SG90 Micro Servo has reached its end position if the current (200 mA) is low for more than 11 to 14 ms
 * No action if aTargetDegrees == sLastDistanceServoAngleInDegrees
 * @param aTargetDegrees    0 is left, 180 is right
 */
void DistanceServoWriteAndWaitForStop(uint8_t aTargetDegrees, bool doWaitForStop) {
    if (sLastDistanceServoAngleInDegrees == aTargetDegrees) {
        return;
    }

    if (aTargetDegrees > 220) {
        // handle underflow
        aTargetDegrees = 0;
    } else if (aTargetDegrees > 180) {
        // handle overflow
        aTargetDegrees = 180;
    }

    uint8_t tDeltaDegrees;
#if defined(USE_OVERSHOOT_FOR_FAST_SERVO_MOVING)
    int8_t tOvershootDegrees; // Experimental
#endif

    uint8_t tLastServoAngleInDegrees = sLastDistanceServoAngleInDegrees;
    sLastDistanceServoAngleInDegrees = aTargetDegrees;

    if (aTargetDegrees > tLastServoAngleInDegrees) {
        tDeltaDegrees = aTargetDegrees - tLastServoAngleInDegrees;
#if defined(USE_OVERSHOOT_FOR_FAST_SERVO_MOVING)
        tOvershootDegrees = 3; // Experimental
#endif
    } else {
        tDeltaDegrees = tLastServoAngleInDegrees - aTargetDegrees;
#if defined(USE_OVERSHOOT_FOR_FAST_SERVO_MOVING)
        tOvershootDegrees = -3; // Experimental
#endif
    }

    /*
     * The central place where the servo is moved
     */
#if defined(USE_OVERSHOOT_FOR_FAST_SERVO_MOVING)
    /*
     * Experimental!
     * Compensate (set target to more degrees) for fast servo speed
     * Reasonable value is between 2 and 3 at 20 degrees and tWaitDelayforServo = tDeltaDegrees * 5
     * Reasonable value is between 10 and 20 degrees and tWaitDelayforServo = tDeltaDegrees * 4 => avoid it
     */
    if (!sDoSlowScan) {
        aTargetDegrees += tOvershootDegrees;
    }
#endif
#if defined(DISTANCE_SERVO_TRIM_DEGREE)
    aTargetDegrees += DISTANCE_SERVO_TRIM_DEGREE;
#endif
#if defined(DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN)
    // The servo is top down and therefore inverted
    aTargetDegrees = 180 - aTargetDegrees;
#endif
    DistanceServo.write(aTargetDegrees);


    /*
     * Delay until stopped
     */
    if (doWaitForStop) {
// Datasheet says: SG90 Micro Servo needs 100 millis per 60 degrees angle => 300 ms per 180
// I measured: SG90 Micro Servo needs 400 per 180 degrees and 400 per 2*90 degree, but 540 millis per 9*20 degree
// 60-80 ms for 20 degrees

//        // wait at least 5 ms for the servo to receive signal
//        delay(SERVO_INITIAL_DELAY);
//        digitalWrite(DEBUG_OUT_PIN, LOW);

        /*
         * Factor 8 gives a fairly reproducible US result, but some dropouts for IR
         * factor 7 gives some strange (to small) values for US.
         */
        uint16_t tWaitDelayforServo;
        if (sDoSlowScan) {
            tWaitDelayforServo = tDeltaDegrees * 16; // 16 => 288 ms for 18 degrees
        } else {
#if defined(USE_OVERSHOOT_FOR_FAST_SERVO_MOVING)
            tWaitDelayforServo = tDeltaDegrees * 5;
#else
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)  // TODO really required?
            tWaitDelayforServo = tDeltaDegrees * 9; // 9 => 162 ms for 18 degrees
#  else
            tWaitDelayforServo = tDeltaDegrees * 8; // 7 => 128|140 ms, 8 => 144|160 for 18|20 degrees
#  endif
#endif
        }
#if defined(USE_BLUE_DISPLAY_GUI)
        delayAndLoopGUI(tWaitDelayforServo);
#else
        delay(tWaitDelayforServo);
#endif
    }
}

/*
 * Scan 70 (right), 90 and 110 (left) degree and return NextDegreesToTurn.
 * Around 330 ms for 20 degree per scan step in fast mode.
 * If BlueDisplay GUI is enabled, display values, otherwise print them.
 * If we get a different distance than last time, we let the LED_BUILTIN flash and read again.
 * @param   aMaximumTargetDistance  - Distances above are taken as timeouts (but displayed by GUI).
 *                                  - Must be < FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER, otherwise timeout is handled as found.
 * @return  0 -> no turn, positive -> turn left (counterclockwise), negative -> turn right
 *          sEffectiveDistanceJustChanged is true, if forward distance is <= aMaximumTargetDistance, i.e. target was found ahead.
 *          sComputedRotation is set.
 *          sRawForwardDistancesArray[] contains values between 1 and FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER
 */
int8_t scanForTargetAndPrint(uint8_t aMaximumTargetDistance) {
    uint8_t tCentimeter;
    uint8_t tServoDegreeToScan;
    int8_t tDeltaDegree;
    uint8_t tIndex;
    uint8_t tMinDistance = 0xFF;
    int tRotationDegree = 0;

    /*
     * Set start values according to last servo position
     */
    if (sLastDistanceServoAngleInDegrees < 90) {
        // Start searching at right
        tServoDegreeToScan = 70;
        tDeltaDegree = DEGREES_PER_TARGET_SCAN_STEP;
        tIndex = INDEX_TARGET_RIGHT;
    } else {
        // Start searching at left
        tServoDegreeToScan = 110;
        tDeltaDegree = -DEGREES_PER_TARGET_SCAN_STEP;
        tIndex = INDEX_TARGET_LEFT;
    }

    /*
     * Scan 3 distances
     */
    for (uint_fast8_t i = 0; i < sizeof(sRawForwardDistancesArray); ++i) {
        /*
         * Get distance
         * Here we allow greater values than max target distance, because they are displayed or converted to a tone
         * Using a constant timeout saves 36 bytes
         */
        tCentimeter = moveServoAndGetDistance(tServoDegreeToScan, FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER);

        if (tCentimeter == DISTANCE_TIMEOUT_RESULT) {
            tCentimeter = FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER;
        }
//        if (abs(tCentimeter - sRawForwardDistancesArray[tIndex]) > 10) {
//            /*
//             * Read again, if different from last value.
//             * Tone / LED feedback for signaling this "exception".
//             */
//            digitalWrite(LED_BUILTIN, HIGH);
////            tone(BUZZER_PIN, 2200);
//            delay(20); // 10 ms is 1.7 m
//            tCentimeter = getDistanceAsCentimeter(FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER, true);
//            if (tCentimeter == DISTANCE_TIMEOUT_RESULT) {
//                tCentimeter = FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER;
//            }
//            digitalWrite(LED_BUILTIN, LOW);
////            noTone (BUZZER_PIN);
//        }

        if (tMinDistance > tCentimeter) {
            tMinDistance = tCentimeter;
            tRotationDegree = tServoDegreeToScan - 90;
        }

#if defined(USE_BLUE_DISPLAY_GUI)
        /*
         * Display distances for BlueDisplay
         */
        if (sCurrentPage == PAGE_AUTOMATIC_CONTROL && BlueDisplay1.isConnectionEstablished()) {
            /*
             * Determine color and draw distance line
             */
            color16_t tColor;
            tColor = COLOR16_CYAN; // 40 < tCentimeter <= 60
            if (tCentimeter <= FOLLOWER_TARGET_DISTANCE_TIMEOUT_CENTIMETER) {
                tColor = COLOR16_GREEN;
            } else if (tCentimeter < FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
                tColor = COLOR16_YELLOW;
            } else if (tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
                tColor = COLOR16_YELLOW;
            }

            // Clear old line - use drawVectorDegreeWithAliasing to be able to clear line without residual
            BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y,
                    sRawForwardDistancesArray[tIndex], tServoDegreeToScan, COLOR16_WHITE, 3);
            // draw new one and store value in distances array for clearing at next scan
            BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y, tCentimeter, tServoDegreeToScan,
                    tColor, 3);
            sRawForwardDistancesArray[tIndex] = tCentimeter;
        }
#endif
        /*
         * Store distance in sRawForwardDistancesArray and compute new array index
         */
        sRawForwardDistancesArray[tIndex] = tCentimeter;
        if (tDeltaDegree > 0) {
            tIndex++;
        } else {
            tIndex--;
        }

        // compute next scan degree
#if defined(USE_BLUE_DISPLAY_GUI)
        loopGUI();
#endif
        tServoDegreeToScan += tDeltaDegree;
    } // scan 3 distances

    /*
     * Check if forward distance is in the right range for a target
     */
    if (sRawForwardDistancesArray[INDEX_TARGET_FORWARD] <= aMaximumTargetDistance) {
        // target found in forward direction
        tRotationDegree = 0; // no rotation
        sEffectiveDistanceJustChanged = true; // force movement
        DistanceServoWriteAndWaitForStop(90); // sets sLastDistanceServoAngleInDegrees
    }

    /*
     * Do no rotation if minimum distance is NOT in the right range for a target
     */
    if (tMinDistance > aMaximumTargetDistance) {
        // Target not found at any side
        tRotationDegree = 0; // no rotation, no movement
    }

    /*
     * Print results
     */
#if defined(USE_BLUE_DISPLAY_GUI)
    snprintf_P(sBDStringBuffer, sizeof(sBDStringBuffer), PSTR("rotation:%3d\xB0 distance:%3dcm"), tRotationDegree, tMinDistance); // \xB0 is degree character
    BlueDisplay1.drawText(BUTTON_WIDTH_3_5_POS_2, US_DISTANCE_MAP_START_Y + TEXT_SIZE_11, sBDStringBuffer, TEXT_SIZE_11,
            COLOR16_BLACK, COLOR16_WHITE);
#else
    printForwardDistanceInfo(&Serial);
    Serial.print(F(" -> "));
    Serial.print(tRotationDegree);
    Serial.print(F(" degree "));
#endif

    sComputedRotation = tRotationDegree;
    return tRotationDegree;
}

void printPadded(uint8_t aByte, Print *aSerial) {
    if (aByte < 10) {
        aSerial->print(' ');
    }
    if (aByte < 100) {
        aSerial->print(' ');
    }
    aSerial->print(aByte);
}

/*
 * End with a space and do not print a newline to allow the range character to be appended.
 */
void printForwardDistanceInfo(Print *aSerial) {
    /*
     * Print turn direction indicators
     */
    if (sComputedRotation > 0) {
        aSerial->print('+');
    } else if (sComputedRotation == 0) {
        aSerial->print('|');
    } else {
        aSerial->print('-');
    }

    uint8_t tDegree = 70;
    for (uint_fast8_t i = 0; i < sizeof(sRawForwardDistancesArray); ++i) {
        printPadded(sRawForwardDistancesArray[i], aSerial);
        aSerial->print(F("cm@"));
        aSerial->print(tDegree);
        aSerial->print(' ');
        tDegree += DEGREES_PER_TARGET_SCAN_STEP;
    }
}

distance_range_t getDistanceRange(uint8_t aCentimeter) {
    if (aCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
        return DISTANCE_TO_SMALL;
    }
    if (aCentimeter < FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
        return DISTANCE_OK;
    }
    if (aCentimeter < FOLLOWER_TARGET_DISTANCE_TIMEOUT_CENTIMETER) {
        return DISTANCE_TO_GREAT;
    }
    return DISTANCE_TARGET_NOT_FOUND;
}

const char RangeCharacterArray[] = { '=', '<', '>', '!' };
void printDistanceRangeCharacter(distance_range_t aRange, Print *aSerial) {
    aSerial->print(RangeCharacterArray[aRange]);
}

#if defined(USE_BLUE_DISPLAY_GUI)
/**
 * Get 7 distances starting at 9 degrees (right) increasing by 18 degrees up to 171 degrees (left)
 * Avoid 0 and 180 degrees since at this position the US sensor might see the wheels of the car as an obstacle.
 * @param aDoFirstValue if false, skip first value since it is the same as last value of last measurement in continuous mode.
 *
 * Wall detection:
 * If 2 or 3 adjacent values are quite short and the surrounding values are quite far,
 * then assume a wall which cannot reflect the pulse for the surrounding values.
 *
 * @param aDoFirstValue Get also first value, i.e. do a complete scan
 * @param aForceScan    If true, do not prematurely return if stop was requested i.e. sRuningAutonomousDrive is false
 * @return true if user cancellation requested.
 */
bool __attribute__((weak)) fillAndShowForwardDistancesInfo(bool aDoFirstValue, bool aForceScan) {

    color16_t tColor;

// Values for forward scanning
    uint8_t tCurrentDegrees = START_DEGREES;
    int8_t tDeltaDegree = DEGREES_PER_STEP;
    int8_t tIndex = INDEX_RIGHT;
    int8_t tIndexDelta = 1;

    if (sLastDistanceServoAngleInDegrees >= 180 - (START_DEGREES + 2)) {
// values for backward scanning
        tCurrentDegrees = 180 - START_DEGREES;
        tDeltaDegree = -(tDeltaDegree);
        tIndex = INDEX_LEFT;
        tIndexDelta = -1;
    }
    if (!aDoFirstValue) {
// skip first value, since it is equal to last value of last measurement
        tIndex += tIndexDelta;
        tCurrentDegrees += tDeltaDegree;
    }

    sBDEventJustReceived = false;
    while (tIndex >= 0 && tIndex < NUMBER_OF_DISTANCES) {
        /*
         * rotate servo, wait with delayAndLoopGUI() and get distance
         */
        DistanceServoWriteAndWaitForStop(tCurrentDegrees, true); // this calls loopGui() during delay, which in turn may update distance sliders.
        if (!aForceScan && sBDEventJustReceived) {
            // User sent an event -> stop and return now
            return true;
        }
        uint8_t tMinimumUSDistanceForMinimumMode = (tIndex == 0 || tIndex == NUMBER_OF_DISTANCES - 1) ? 6 : 0;
        // Show only forward info as bar
        auto tCentimeter = getDistanceAsCentimeter(AUTONOMOUS_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER, true,
                tMinimumUSDistanceForMinimumMode, tIndex == INDEX_FORWARD_1);
        if (tCentimeter == DISTANCE_TIMEOUT_RESULT) {
            // timeout here
            tCentimeter = AUTONOMOUS_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER;
        }
        if ((tIndex == INDEX_FORWARD_1 || tIndex == INDEX_FORWARD_2) && tCentimeter <= sCentimetersDrivenPerScan * 2) {
            /*
             * Emergency motor stop if index is forward and measured distance is less than distance driven during two scans
             */
            RobotCar.stop();
        }

        if (sCurrentPage == PAGE_AUTOMATIC_CONTROL && BlueDisplay1.isConnectionEstablished()) {
            /*
             * Determine color
             */
            if (tCentimeter >= AUTONOMOUS_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER) {
                tColor = DISTANCE_TIMEOUT_COLOR; // Cyan
            } else if (tCentimeter >= sCentimetersDrivenPerScan * 2) {
                tColor = COLOR16_GREEN;
            } else if (tCentimeter >= sCentimetersDrivenPerScan) {
                tColor = COLOR16_YELLOW;
            } else {
                tColor = COLOR16_RED; // tCentimeter < sCentimeterDrivenPerScan
            }

            /*
             * Clear old and draw new distance line
             */
            BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y,
                    sForwardDistancesInfo.RawDistancesArray[tIndex], tCurrentDegrees, COLOR16_WHITE, 3);
            BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y, tCentimeter, tCurrentDegrees, tColor,
                    3);
        }

        sForwardDistancesInfo.RawDistancesArray[tIndex] = tCentimeter;

        tIndex += tIndexDelta;
        tCurrentDegrees += tDeltaDegree;
    }
    return false;
}

/*
 * Draw values of ActualDistancesArray as vectors
 * Not used yet
 */
void drawForwardDistancesInfos() {
    color16_t tColor;
    uint8_t tCurrentDegrees = 0;
    /*
     * Clear drawing area
     */
    clearPrintedForwardDistancesInfos(true);
    for (int i = 0; i < NUMBER_OF_DISTANCES; ++i) {
        /*
         * Determine color
         */
        uint8_t tDistance = sForwardDistancesInfo.RawDistancesArray[i];
        tColor = COLOR16_ORANGE;
        if (tDistance >= AUTONOMOUS_DRIVE_DISTANCE_TIMEOUT_CENTIMETER) {
            tColor = COLOR16_GREEN;
        }
        if (tDistance > (DISTANCE_MAX_FOR_WALL_DETECTION_CM / 2)) {
            tColor = COLOR16_GREEN;
        } else if (tDistance < (DISTANCE_MAX_FOR_WALL_DETECTION_CM / 2)) {
            tColor = COLOR16_RED;
        }

        /*
         * Draw line
         */
        BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y, tDistance, tCurrentDegrees, tColor, 3);
        tCurrentDegrees += DEGREES_PER_STEP;
    }
}
#endif

/*
 * This documentation assumes 20 degrees stepping.
 * By changing DEGREES_PER_STEP it can easily adopted to other degrees values.
 *
 * Assume the value of 20 and 40 degrees are distances to a wall.
 * @return the clipped (aClipValue) distance to the wall of the vector at 0 degree.
 *
 * @param [out] aDegreeOfEndpointConnectingLine: The angle of the line from endpoint 0 degrees to given endpoints
 * 0 means x values of given endpoints are the same >= wall is parallel / rectangular to the vector at 0 degree
 * Positive means wall is more ore less in front, to avoid we must turn positive angle
 * 90 means y values are the same =>  wall is in front
 * Negative means we are heading away from wall
 */
uint8_t computeNeigbourValue(uint8_t aDegreesPerStepValue, uint8_t a2DegreesPerStepValue, uint8_t aClipValue,
        int8_t *aDegreeOfEndpointConnectingLine) {

    /*
     * Name of the variables are for DEGREES_PER_STEP = 20 only for better understanding.
     * The computation of course works for other values of DEGREES_PER_STEP!
     */
    float tYat20degrees = sin((PI / 180) * DEGREES_PER_STEP) * aDegreesPerStepValue; // e.g. 20 Degree
// assume current = 40 Degree
    float tYat40degrees = sin((PI / 180) * (DEGREES_PER_STEP * 2)) * a2DegreesPerStepValue; // e.g. 40 Degree

//    char tStringBuffer[] = "A=_______ L=_______";
//    dtostrf(tYat40degrees, 7, 2, &tStringBuffer[2]);
//    tStringBuffer[9] = ' ';
//    dtostrf(tYat20degrees, 7, 2, &tStringBuffer[12]);
//    BlueDisplay1.debugMessage(tStringBuffer);

    uint8_t tDistanceAtZeroDegree = aClipValue;

    /*
     * if tY40degrees == tY20degrees the tInvGradient is infinite (distance at 0 is infinite)
     */
    if (tYat40degrees > tYat20degrees) {
        float tXat20degrees = cos((PI / 180) * DEGREES_PER_STEP) * aDegreesPerStepValue; // 20 Degree
        float tXat40degrees = cos((PI / 180) * (DEGREES_PER_STEP * 2)) * a2DegreesPerStepValue; // 40 Degree

//        dtostrf(tXat40degrees, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXat20degrees, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        /*  Here the graphic for 90 and 60 degrees (since we have no ASCII graphic symbols for 20 and 40 degree)
         *  In this function we have e.g. 40 and 20 degrees and compute 0 degrees!
         *      90 degrees value
         *      |\ \==wall
         *      | \  60 degrees value
         *      | /\
 *      |/__\ 0 degrees value to be computed
         */

        /*
         * InvGradient line represents the wall
         * if tXat20degrees > tXat40degrees InvGradient is negative => X0 value is bigger than X20 one / right wall is in front if we look in 90 degrees direction
         * if tXat20degrees == tXat40degrees InvGradient is 0 / wall is parallel right / 0 degree
         * if tXat20degrees < tXat40degrees InvGradient is positive / right wall is behind / degrees is negative (from direction front which is 90 degrees)
         */
        float tInvGradient = (tXat40degrees - tXat20degrees) / (tYat40degrees - tYat20degrees);
        float tXatZeroDegree = tXat20degrees - (tInvGradient * tYat20degrees);
        *aDegreeOfEndpointConnectingLine = -(atan(tInvGradient) * RAD_TO_DEG);
//        tStringBuffer[0] = 'G';
//        tStringBuffer[10] = 'B';
//        dtostrf(tInvGradient, 7, 2, &tStringBuffer[2]);
//        tStringBuffer[9] = ' ';
//        dtostrf(tXZeroDegree, 7, 2, &tStringBuffer[12]);
//        BlueDisplay1.debugMessage(tStringBuffer);

        if (tXatZeroDegree < 255) {
            tDistanceAtZeroDegree = tXatZeroDegree + 0.5;
            if (tDistanceAtZeroDegree > aClipValue) {
                tDistanceAtZeroDegree = aClipValue;
            }
        }
    }
    return tDistanceAtZeroDegree;
}

/*
 * The problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor,
 * if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
 * For other angels the reflected ultrasonic beam can not reach the receiver, which leads to unrealistic great distances.
 *
 * Therefore I take samples every 18 degrees and if I get 2 adjacent short (< DISTANCE_MAX_FOR_WALL_DETECTION_CM) distances, I assume a wall determined by these 2 samples.
 * The (invalid) values 18 degrees right and left of these samples are then extrapolated by computeNeigbourValue().
 *
 * Modifies values in sForwardDistancesInfo.ProcessedDistancesArray[]
 */
//#define FUNCTION_TRACE // only used for this function
void doWallDetection() {
    uint8_t tCurrentAngleToCheck = START_DEGREES + (2 * DEGREES_PER_STEP); // first angle to adjust at index 2
    int8_t tDegreeOfConnectingLine;
    sForwardDistancesInfo.WallRightAngleDegrees = 0;
    sForwardDistancesInfo.WallLeftAngleDegrees = 0;
//    sForwardDistancesInfo.WallRightDistance = UINT8_MAX;
//    sForwardDistancesInfo.WallLeftDistance = UINT8_MAX;

    /*
     * Parse the array from 0 to STEPS_PER_SCAN
     * Check values at i and i-1 and adjust value at i+1
     * i is index of CurrentDistance
     */
    uint8_t tLastDistance = sForwardDistancesInfo.ProcessedDistancesArray[0];
    uint8_t tCurrentDistance = sForwardDistancesInfo.ProcessedDistancesArray[1];
    for (uint_fast8_t i = 1; i < STEPS_PER_SCAN; ++i) {
        uint8_t tNextDistanceOriginal = sForwardDistancesInfo.ProcessedDistancesArray[i + 1];
        if (tLastDistance < DISTANCE_MAX_FOR_WALL_DETECTION_CM && tCurrentDistance < DISTANCE_MAX_FOR_WALL_DETECTION_CM) {
            /*
             * 2 adjacent short distances -> assume a wall -> adjust adjacent values
             */

            /*
             * Use computeNeigbourValue the other way round
             * i.e. put 20 degrees to 40 degrees parameter and vice versa in order to use the 0 degree value as the 60 degrees one
             */
            uint8_t tNextDistanceComputed = computeNeigbourValue(tCurrentDistance, tLastDistance,
            AUTONOMOUS_DRIVE_DISTANCE_TIMEOUT_CENTIMETER, &tDegreeOfConnectingLine);
#if defined(FUNCTION_TRACE) && defined(USE_BLUE_DISPLAY_GUI)
            BlueDisplay1.debug("i=", i);
            BlueDisplay1.debug("AngleToCheck @i+1=", tCurrentAngleToCheck);
            BlueDisplay1.debug("Original distance @i+1=", tNextDistanceOriginal);
            BlueDisplay1.debug("New distance @i+1=", tNextDistanceComputed);
            BlueDisplay1.debug("Connecting degrees @i+1=", tDegreeOfConnectingLine);
#endif

            if (tNextDistanceOriginal > tNextDistanceComputed + 5) {
                /*
                 * Adjust and draw next value if computed value is less than original value - 5
                 *
                 * Start with i=1 and adjust for 2 at (2 * DEGREES_PER_STEP) + START_DEGREES.
                 * Since we use computeNeigbourValue the other way round, we must change sign of tDegreeOfConnectingLine!
                 * The formula is 90 - (180->sum of degrees in triangle - tCurrentAngleToCheck - (90 - tDegreeOfConnectingLine)->since we use it the other way round)
                 * Which leads to -90 + tCurrentAngleToCheck + 90 - tDegreeOfConnectingLine.
                 * If we then get a tDegreeOfConnectingLine value of 0 we have a wall at right rectangular to the vector at 2,
                 * Negative raw values means the wall is more in front / the the wall angle is greater,
                 */
                int tDegreeOfWallAngle = tCurrentAngleToCheck - tDegreeOfConnectingLine;
#if defined(FUNCTION_TRACE) && defined(USE_BLUE_DISPLAY_GUI)
                BlueDisplay1.debug("tDegreeOfWallAngle=", tDegreeOfWallAngle);
#endif
                if (tDegreeOfWallAngle <= 90) {
                    // wall at right
                    sForwardDistancesInfo.WallRightAngleDegrees = tDegreeOfWallAngle;
//                    sForwardDistancesInfo.WallRightDistance = tCurrentDistance;
                } else {
                    // wall at left
                    sForwardDistancesInfo.WallLeftAngleDegrees = 180 - tDegreeOfWallAngle;
//                    sForwardDistancesInfo.WallLeftDistance = tCurrentDistance;
                }

                // store and draw adjusted value
                sForwardDistancesInfo.ProcessedDistancesArray[i + 1] = tNextDistanceComputed;
                tNextDistanceOriginal = tNextDistanceComputed;
#if defined(USE_BLUE_DISPLAY_GUI)
                if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
                    BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y, tNextDistanceComputed,
                            tCurrentAngleToCheck, COLOR16_WHITE, 1);
                }
#endif
            }
        }
        tLastDistance = tCurrentDistance;
        tCurrentDistance = tNextDistanceOriginal;
        tCurrentAngleToCheck += DEGREES_PER_STEP;
    }

    /*
     * Go backwards through the array
     */
    tLastDistance = sForwardDistancesInfo.ProcessedDistancesArray[STEPS_PER_SCAN];
    tCurrentDistance = sForwardDistancesInfo.ProcessedDistancesArray[STEPS_PER_SCAN - 1];
    tCurrentAngleToCheck = 180 - (START_DEGREES + (2 * DEGREES_PER_STEP));

    /*
     * check values at i and i+1 and adjust value at i-1
     */
    for (uint_fast8_t i = STEPS_PER_SCAN - 1; i > 0; --i) {
        uint8_t tNextValue = sForwardDistancesInfo.ProcessedDistancesArray[i - 1];

// Do it only if none of the 3 values are processed before
        if (sForwardDistancesInfo.ProcessedDistancesArray[i + 1] == sForwardDistancesInfo.RawDistancesArray[i + 1]
                && sForwardDistancesInfo.ProcessedDistancesArray[i] == sForwardDistancesInfo.RawDistancesArray[i]
                && tNextValue == sForwardDistancesInfo.RawDistancesArray[i - 1]) {

            /*
             * check values at i+1 and i and adjust value at i-1
             */
            if (tLastDistance < DISTANCE_MAX_FOR_WALL_DETECTION_CM && tCurrentDistance < DISTANCE_MAX_FOR_WALL_DETECTION_CM) {
                /*
                 * Wall detected -> adjust adjacent values
                 * Use computeNeigbourValue in the intended way, so do not change sign of tDegreeOfConnectingLine!
                 */
                uint8_t tNextValueComputed = computeNeigbourValue(tCurrentDistance, tLastDistance,
                AUTONOMOUS_DRIVE_DISTANCE_TIMEOUT_CENTIMETER, &tDegreeOfConnectingLine);
#if defined(FUNCTION_TRACE) && defined(USE_BLUE_DISPLAY_GUI)
                BlueDisplay1.debug("i=", i);
                BlueDisplay1.debug("AngleToCheck @i+1=", tCurrentAngleToCheck);
                BlueDisplay1.debug("Original distance @i-1=", tNextValue);
                BlueDisplay1.debug("New distance @i-1=", tNextValueComputed);
                BlueDisplay1.debug("Connecting degrees @i-1=", tDegreeOfConnectingLine);
#endif
                if (tNextValue > tNextValueComputed + 5) {
                    // start with i = 8 and adjust for 7
                    // degrees at index i-1 are ((i - 1) * DEGREES_PER_STEP) + START_DEGREES
                    int tWallBackwardDegrees = tCurrentAngleToCheck + tDegreeOfConnectingLine;
#if defined(FUNCTION_TRACE) && defined(USE_BLUE_DISPLAY_GUI)
                    BlueDisplay1.debug("tWallBackwardDegrees=", tWallBackwardDegrees);
#endif
                    if (tWallBackwardDegrees <= 90) {
                        // wall at right - overwrite only if greater
                        if (sForwardDistancesInfo.WallRightAngleDegrees < tWallBackwardDegrees) {
                            sForwardDistancesInfo.WallRightAngleDegrees = tWallBackwardDegrees;
#if defined(FUNCTION_TRACE) && defined(USE_BLUE_DISPLAY_GUI)
                            BlueDisplay1.debug("WallRightAngleDegrees=", sForwardDistancesInfo.WallRightAngleDegrees);
#endif
                        }
                    } else if (sForwardDistancesInfo.WallLeftAngleDegrees < (180 - tWallBackwardDegrees)) {
                        // wall at right - overwrite only if greater
                        sForwardDistancesInfo.WallLeftAngleDegrees = 180 - tWallBackwardDegrees;
#if defined(TRACE) && defined(USE_BLUE_DISPLAY_GUI)
                        BlueDisplay1.debug("WallLeftAngleDegrees=", sForwardDistancesInfo.WallLeftAngleDegrees);
#endif

                    }
                    //Adjust and draw next value if original value is greater
                    sForwardDistancesInfo.ProcessedDistancesArray[i - 1] = tNextValueComputed;
                    tNextValue = tNextValueComputed;
#if defined(USE_BLUE_DISPLAY_GUI)
                    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
                        BlueDisplay1.drawVectorDegreeWithAliasing(US_DISTANCE_MAP_START_X, US_DISTANCE_MAP_START_Y, tNextValueComputed,
                                tCurrentAngleToCheck, COLOR16_WHITE, 1);
                    }
#endif
                }
            }
        }
        tLastDistance = tCurrentDistance;
        tCurrentDistance = tNextValue;
        tCurrentAngleToCheck -= DEGREES_PER_STEP;

    }
}
#if defined(FUNCTION_TRACE)
#undef FUNCTION_TRACE
#endif

/*
 * Evaluates ProcessedDistancesArray.
 * Find min and max value and one index > aDistanceThreshold. Prefer the headmost value if we have more than one choice
 * Do not use the values at 0 and 9 for minimum, since sometimes we measure the distance to the own wheels at this degrees.
 */
void postProcessDistances(uint8_t aDistanceThreshold) {
    unsigned int tMax = 0;
    unsigned int tMin = UINT16_MAX; // = 65535
    sForwardDistancesInfo.DegreeOfDistanceGreaterThanThreshold = INVALID_DEGREE; // mark as invalid
    sForwardDistancesInfo.DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold = INVALID_DEGREE; // mark as invalid
// scan simultaneously from 0 to 4 and 9 to 5 to prefer headmost values/indexes, if distances are the same.
    for (uint_fast8_t i = 0; i < (NUMBER_OF_DISTANCES + 1) / 2; ++i) {
        uint8_t tIndex = i;
        for (uint_fast8_t j = 0; j < 2; ++j) {
            /*
             * Do it first for i and then for STEPS_PER_SCAN - i
             */
            uint8_t tDistance;
            tDistance = sForwardDistancesInfo.ProcessedDistancesArray[tIndex];
            if (tDistance >= tMax) {
                tMax = tDistance;
                sForwardDistancesInfo.DegreeOfMaxDistance = IndexToDegree(tIndex);
                sForwardDistancesInfo.MaxDistance = tDistance;
            }
            if (tDistance <= tMin && i != 0) {
                tMin = tDistance;
                sForwardDistancesInfo.DegreeOfMinDistance = IndexToDegree(tIndex);
                sForwardDistancesInfo.MinDistance = tDistance;
            }
            if (tDistance >= aDistanceThreshold) {
                sForwardDistancesInfo.DegreeOfDistanceGreaterThanThreshold = IndexToDegree(tIndex);
                if (tDistance >= aDistanceThreshold * 2) {
                    if (j == 0) {
                        if (sForwardDistancesInfo.ProcessedDistancesArray[tIndex + 1] >= aDistanceThreshold * 2) {
                            sForwardDistancesInfo.DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold =
                                    ((tIndex * DEGREES_PER_STEP) + START_DEGREES + (DEGREES_PER_STEP / 2)) - 90;
                        }
                    } else if (sForwardDistancesInfo.ProcessedDistancesArray[tIndex - 1] >= aDistanceThreshold * 2) {
                        sForwardDistancesInfo.DegreeOf2ConsecutiveDistancesGreaterThanTwoThreshold = ((tIndex * DEGREES_PER_STEP)
                                + START_DEGREES - (DEGREES_PER_STEP / 2)) - 90;
                    }
                }
            }

            tIndex = STEPS_PER_SCAN - i;
        }
    }
}
#endif // defined(CAR_HAS_DISTANCE_SERVO)

/*
 * Evaluates the US_DISTANCE_SENSOR_ENABLE_PIN switching between IR and US sensor.
 * Evaluates the DISTANCE_TONE_FEEDBACK_ENABLE_PIN to suppress tone feedback.
 */
void getDistanceModesFromPins() {
#  if defined(US_DISTANCE_SENSOR_ENABLE_PIN) && (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
// If US_DISTANCE_SENSOR_ENABLE_PIN is connected to ground we use the US distance as fallback.
// Useful for testing the difference between both sensors.
    if (digitalRead(US_DISTANCE_SENSOR_ENABLE_PIN) == LOW) {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_US;
    } else {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_IR_OR_TOF;
    }
#  endif
#  if defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && defined(DISTANCE_FEEDBACK_MODE) // If this pin is connected to ground, enable distance feedback
    if (digitalRead(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) == LOW) {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_MODE;
    } else {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
        noTone(BUZZER_PIN);
    }
#  endif
}
#endif // _ROBOT_CAR_DISTANCE_HPP
