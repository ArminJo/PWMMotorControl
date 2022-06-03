/*
 * Distance.hpp
 *
 *  Contains all distance measurement functions.
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  Arduino-RobotCar is free software: you can redistribute it and/or modify
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

#ifndef _ROBOT_CAR_DISTANCE_HPP
#define _ROBOT_CAR_DISTANCE_HPP

#include "Distance.h"

#if defined(CAR_HAS_SERVO) && defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
#define DISABLE_SERVO_TIMER_AUTO_INITIALIZE // saves 70 bytes program space
#include "LightweightServo.hpp"
#endif

#include "pitches.h"

bool sDoSlowScan = false;

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
// Default is US. One of DISTANCE_SOURCE_MODE_MINIMUM, DISTANCE_SOURCE_MODE_MAXIMUM, DISTANCE_SOURCE_MODE_US or DISTANCE_SOURCE_MODE_IR_OR_TOF
uint8_t sDistanceSourceMode =  DISTANCE_SOURCE_MODE_DEFAULT;
#endif

uint8_t sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
uint8_t sUSDistanceCentimeter;
uint8_t sUSDistanceTimeoutCentimeter; // The value to display if sUSDistanceCentimeter == DISTANCE_TIMEOUT_RESULT
uint8_t sIROrTofDistanceCentimeter;
uint8_t sEffectiveDistanceCentimeter; // depends on sDistanceSourceMode
uint8_t sLastEffectiveDistanceCentimeter; // depends on sDistanceSourceMode
bool sDistanceJustChanged;

ForwardDistancesInfoStruct sForwardDistancesInfo;

#if defined(CAR_HAS_DISTANCE_SERVO)
#  if !defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
Servo DistanceServo;
#  endif
uint8_t sLastDistanceServoAngleInDegrees; // 0 - 180 needed for optimized delay for servo repositioning. Only set by DistanceServoWriteAndDelay()
#endif

#if defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#  if defined(AVR) && defined(USE_SOFT_I2C_MASTER)
#undef USE_SOFT_I2C_MASTER_H_AS_PLAIN_INCLUDE // just in case...
#include "SoftI2CMasterConfig.h"
#include "SoftI2CMaster.h"
VL53L1X sToFDistanceSensor(-1, -1); // 100 kHz
#  else
// removing usage of SFEVL53L1X wrapper class saves 794 bytes
VL53L1X sToFDistanceSensor(&Wire, -1, -1); // 100 kHz
#  endif
#endif

/*
 * This initializes the pins too
 */
void initDistance() {
#if defined(CAR_HAS_DISTANCE_SERVO) && !defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
    DistanceServo.attach(PIN_DISTANCE_SERVO);
#endif

#if defined(US_SENSOR_SUPPORTS_1_PIN_MODE)
    initUSDistancePin (PIN_TRIGGER_OUT);
#else
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);
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
    sToFDistanceSensor.VL53L1X_SetOffset(OFFSET_MILLIMETER);

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
 * Get distance, optionally display it and check if tone has to be changed
 * @return  Distance in centimeter @20 degree celsius (time in us/58.25)
 *          0 / DISTANCE_TIMEOUT_RESULT if timeout or pins are not initialized
 */
unsigned int getDistanceAsCentimeterAndPlayTone(uint8_t aDistanceTimeoutCentimeter, bool aWaitForCurrentMeasurementToEnd) {
    unsigned int tCentimeter = getDistanceAsCentimeter(aDistanceTimeoutCentimeter, aWaitForCurrentMeasurementToEnd);
#if defined(USE_BLUE_DISPLAY_GUI)
#  if defined(CAR_HAS_US_DISTANCE_SENSOR)
            showUSDistance();
#  endif
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
            showIROrTofDistance();
#  endif
#endif // defined(USE_BLUE_DISPLAY_GUI)

    if (sDistanceJustChanged) {
        /*
         * Distance has changed here
         * Update tone according to sDistanceFeedbackMode
         */
        if (sDistanceFeedbackMode != DISTANCE_FEEDBACK_NO_TONE) {
            if (tCentimeter == DISTANCE_TIMEOUT_RESULT) {
                noTone(PIN_BUZZER);
            } else {
                int tFrequency;
                if (sDistanceFeedbackMode == DISTANCE_FEEDBACK_PENTATONIC) {
                    /*
                     * Map distance to an index in a pentatonic pitch table
                     */
                    uint8_t tIndex = map(tCentimeter, 0, 100, 0, ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1);
                    if (tIndex > ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1) {
                        tIndex = ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1;
                    }
                    tFrequency = NoteC5ToC7Pentatonic[tIndex]; // 523 to 2093 Hz for 0 to 100 cm
                } else {
                    /*
                     * DISTANCE_FEEDBACK_CONTINUOUSLY. Play feedback tone proportional to measured distance
                     */
#if defined(FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) && defined(FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER)
                if(tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER){
                    tFrequency = map(tCentimeter, 0, FOLLOWER_DISTANCE_MINIMUM_CENTIMETER, 100, 200);
                } else if(tCentimeter > FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER){
                    tFrequency = map(tCentimeter, FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER, 200, 2000, 6000);
                } else {
                    tFrequency = map(tCentimeter, FOLLOWER_DISTANCE_MINIMUM_CENTIMETER, FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER, 440, 880); // 1 octaves for gap
                }
#else
                    tFrequency = map(tCentimeter, 0, 100, 110, 7040); // 6 octaves per meter
#endif
                }
                tone(PIN_BUZZER, tFrequency);
            }
        }
    }
    return tCentimeter;
}

/*
 * Print without a newline
 */
void printDistanceIfChanged(Print *aSerial) {
    if (sDistanceJustChanged) {
        if (sEffectiveDistanceCentimeter == DISTANCE_TIMEOUT_RESULT) {
            aSerial->println("Distance timeout");
        } else {
            aSerial->print("Distance=");
            aSerial->print(sEffectiveDistanceCentimeter);
            aSerial->print("cm");
        }
    }
}

/*
 * @param aWaitForCurrentMeasurmentToEnd  for IR Distance sensors if true, wait for the current measurement to end, since the sensor was recently moved.
 * @param doShow show distance value in the GUI
 * @return 0 / DISTANCE_TIMEOUT_RESULT for timeout
 */
unsigned int getDistanceAsCentimeter(uint8_t aDistanceTimeoutCentimeter, bool aWaitForCurrentMeasurementToEnd) {
#if !defined(CAR_HAS_IR_DISTANCE_SENSOR)
    (void) aWaitForCurrentMeasurementToEnd; // suppress compiler warnings
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
    uint8_t tCentimeterToReturn = getUSDistanceAsCentimeterWithCentimeterTimeout(aDistanceTimeoutCentimeter);
    sUSDistanceCentimeter = tCentimeterToReturn;

#if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    sIROrTofDistanceCentimeter = getIRDistanceAsCentimeter(aWaitForCurrentMeasurementToEnd);
#  elif defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    sIROrTofDistanceCentimeter = readToFDistanceAsCentimeter();
#  endif
    /*
     * IR or TOF sensor is used, we check for distance source mode
     * (take IR or minimum or maximum of the US and (IR or TOF) values)
     */
    if (sDistanceSourceMode == DISTANCE_SOURCE_MODE_IR_OR_TOF) {
        tCentimeterToReturn = sIROrTofDistanceCentimeter;
    } else if (sDistanceSourceMode == DISTANCE_SOURCE_MODE_MINIMUM) {
        // Scan mode MINIMUM => Take the minimum of the US and IR or TOF values
        if (tCentimeterToReturn > sIROrTofDistanceCentimeter) {
            tCentimeterToReturn = sIROrTofDistanceCentimeter;
        }
    } else if (sDistanceSourceMode == DISTANCE_SOURCE_MODE_MAXIMUM) {
        // Scan mode MAXIMUM => Take the maximum of the US and IR or TOF values
        if (tCentimeterToReturn < sIROrTofDistanceCentimeter) {
            tCentimeterToReturn = sIROrTofDistanceCentimeter;
        }
    }
#endif
    sEffectiveDistanceCentimeter = tCentimeterToReturn;
    if (sLastEffectiveDistanceCentimeter != tCentimeterToReturn) {
        sLastEffectiveDistanceCentimeter = tCentimeterToReturn;
        sDistanceJustChanged = true;
    } else {
        sDistanceJustChanged = false;
    }
    return tCentimeterToReturn;
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
#  if !defined(IR_SENSOR_TYPE_100550) && !defined(IR_SENSOR_TYPE_20150) && !defined(IR_SENSOR_TYPE_1080) && !defined(IR_SENSOR_TYPE_430)
#define IR_SENSOR_TYPE_1080
#  endif
/*
 * The Sharp 1080 takes 39 ms for each measurement cycle
 */
uint8_t getIRDistanceAsCentimeter(bool aWaitForCurrentMeasurementToEnd) {
    if (aWaitForCurrentMeasurementToEnd) {
        /*
         * Check for a voltage change which indicates that a new measurement is started
         */
        int16_t tOldValue = analogRead(PIN_IR_DISTANCE_SENSOR); // 100 us
        uint32_t tStartMillis = millis();
        do {
            int16_t tNewValue = analogRead(PIN_IR_DISTANCE_SENSOR); // 100 us
            if (abs(tOldValue-tNewValue) > IR_SENSOR_NEW_MEASUREMENT_THRESHOLD) {
                // assume, that voltage has changed because of the end of a measurement
                break;
            }
#if defined(USE_BLUE_DISPLAY_GUI)
            loopGUI();
#endif
        } while (millis() - tStartMillis <= IR_SENSOR_MEASUREMENT_TIME_MILLIS);
        // now a new measurement has started, wait for the result
#if defined(USE_BLUE_DISPLAY_GUI)
        delayAndLoopGUI(IR_SENSOR_NEW_MEASUREMENT_THRESHOLD); // the IR sensor takes 39 ms for one measurement
#else
        delay(IR_SENSOR_NEW_MEASUREMENT_THRESHOLD); // the IR sensor takes 39 ms for one measurement
#endif
    }

    float tVolt = analogRead(PIN_IR_DISTANCE_SENSOR); // 100 us
    // tVolt * 0.004887585 = 5(V) for tVolt == 1023

#  if defined(IR_SENSOR_TYPE_430) // 4 to 30 cm, 18 ms, GP2YA41SK0F
    return (12.08 * pow(tVolt * 0.004887585, -1.058)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp

#  elif defined(IR_SENSOR_TYPE_1080) // 10 to 80 cm, GP2Y0A21YK0F
    return (29.988 * pow(tVolt * 0.004887585, -1.173)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
//    return 4800/(analogRead(PIN_IR_DISTANCE_SENSOR)-20);    // see https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp

#  elif defined(IR_SENSOR_TYPE_20150) // 20 to 150 cm, 18 ms, GP2Y0A02YK0F
    // Model 20150 - Do not forget to add at least 100uF capacitor between the Vcc and GND connections on the sensor
    return (60.374 * pow(tVolt * 0.004887585, -1.16)) + 0.5;// see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp

#  elif defined(IR_SENSOR_TYPE_100550) // GP2Y0A02YK0F // 100 to 550 cm, 18 ms, GP2Y0A710K0F
    return 1.0 / (((tVolt * 0.004887585 - 1.1250)) / 137.5);
#  endif
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
//#define USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
/*
 * sets also sLastDistanceServoAngleInDegrees to enable optimized servo movement and delays
 * SG90 Micro Servo has reached its end position if the current (200 mA) is low for more than 11 to 14 ms
 * No action if aTargetDegrees == sLastDistanceServoAngleInDegrees
 */
void DistanceServoWriteAndDelay(uint8_t aTargetDegrees, bool doDelay) {

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

    if (tLastServoAngleInDegrees == aTargetDegrees) {
        return;
    } else if (aTargetDegrees > tLastServoAngleInDegrees) {
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
     * Move servo
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

#if defined(DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN)
    // The servo is top down and therefore inverted
    aTargetDegrees = 180 - aTargetDegrees;
#endif
#if defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
    write10(aTargetDegrees);
#else
    DistanceServo.write(aTargetDegrees);
#endif

    /*
     * Delay
     */
    if (doDelay) {
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
            tWaitDelayforServo = tDeltaDegrees * 8; // 7 => 128 ms, 8 => 144 for 18 degrees
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
 * Stop, scan 70, 90 and 110 degree for moved target and returns NextDegreesToTurn.
 * Display values if BlueDisplay GUI is enabled.
 * @return NO_TARGET_FOUND (360) if no target found
 */
int scanForTarget(unsigned int aMaximumTargetDistance) {
    noTone(PIN_BUZZER); // suppress distance tone at scanning

    uint8_t tServoDegreeOfFoundTarget = 0;
    uint8_t tCentimeter; // We have set the timeout to less than 255 cm
    uint8_t tServoDegreeToScan;
    int tDeltaDegree;
    /*
     * Set start values according to last servo position
     */
    if (sLastDistanceServoAngleInDegrees < 90) {
        // Start searching at right
        tServoDegreeToScan = 70;
        tDeltaDegree = 20;
    } else {
        // Start searching at left
        tServoDegreeToScan = 110;
        tDeltaDegree = -20;
    }
    /*
     * Scan (and display) 3 distances
     */
    for (uint_fast8_t i = 0; i < 3; ++i) {
        DistanceServoWriteAndDelay(tServoDegreeToScan, true);
        tCentimeter = getDistanceAsCentimeter(DISTANCE_TIMEOUT_CM_FOLLOWER, true);
        if(tCentimeter == DISTANCE_TIMEOUT_RESULT) {
            tCentimeter = DISTANCE_TIMEOUT_CM_FOLLOWER;
        }
#if defined(USE_BLUE_DISPLAY_GUI)
        if (sCurrentPage == PAGE_AUTOMATIC_CONTROL && BlueDisplay1.isConnectionEstablished()) {
            /*
             * Determine color and draw distance line
             */
            color16_t tColor;
            tColor = COLOR16_RED; // tCentimeter <= sCentimeterDrivenPerScan
            if (tCentimeter <= FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) {
                tColor = COLOR16_GREEN;
            } else if (tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
                tColor = COLOR16_YELLOW;
            }

            uint8_t tCurrentIndex;
            if (tDeltaDegree > 0) {
                // scan from 70 to 110 -> 0 contains value for 70 degrees, 1 for 90 and 2 for 110
                tCurrentIndex = i;
            } else {
                tCurrentIndex = 2 - i;
            }

            // Clear old line
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y,
                    sForwardDistancesInfo.RawDistancesArray[tCurrentIndex] / 2, tServoDegreeToScan, COLOR16_WHITE, 3);
            // draw new one and store value in distances array for next scan
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tCentimeter / 2, tServoDegreeToScan, tColor, 3);
            sForwardDistancesInfo.RawDistancesArray[tCurrentIndex] = tCentimeter;
        }
#else

        Serial.print(tCentimeter);
        Serial.print(F("cm@"));
        Serial.print(tServoDegreeToScan);
        Serial.print(' ');
#endif
        if (tCentimeter <= aMaximumTargetDistance) {
            tServoDegreeOfFoundTarget = tServoDegreeToScan;
            break;
        }
        // prepare for next degree
#if defined(USE_BLUE_DISPLAY_GUI)
        loopGUI();
#endif
        tServoDegreeToScan += tDeltaDegree;
    }

    if (tServoDegreeOfFoundTarget != 0) {
        int tRotationDegree = tServoDegreeToScan - 90;
        /*
         * Target found -> print turn info
         */
#if defined(USE_BLUE_DISPLAY_GUI)
        sprintf_P(sStringBuffer, PSTR("rotation:%3d\xB0 min:%2dcm"), tRotationDegree, tCentimeter); // \xB0 is degree character
        BlueDisplay1.drawText(BUTTON_WIDTH_3_5_POS_2, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11, sStringBuffer, TEXT_SIZE_11,
                COLOR16_BLACK, COLOR16_WHITE);
#else
        Serial.print(F("rotate "));
        Serial.println(tRotationDegree);
#endif

        return tRotationDegree;
    } else {
#if !defined(USE_BLUE_DISPLAY_GUI)
        Serial.println(); // terminate scan result line without rotate info
#endif
        return NO_TARGET_FOUND;
    }
}

#if defined(USE_BLUE_DISPLAY_GUI)
/*
 * Get 7 distances starting at 9 degrees (right) increasing by 18 degrees up to 171 degrees (left)
 * Avoid 0 and 180 degrees since at this position the US sensor might see the wheels of the car as an obstacle.
 * @param aDoFirstValue if false, skip first value since it is the same as last value of last measurement in continuous mode.
 *
 * Wall detection:
 * If 2 or 3 adjacent values are quite short and the surrounding values are quite far,
 * then assume a wall which cannot reflect the pulse for the surrounding values.
 *
 * aForceScan if true, do not return if sRuningAutonomousDrive is false / autonomous drive is stopped
 *
 * return true if user cancellation requested.
 */
bool __attribute__((weak)) fillAndShowForwardDistancesInfo(bool aDoFirstValue, bool aForceScan) {

    color16_t tColor;

// Values for forward scanning
    uint8_t tCurrentDegrees = START_DEGREES;
    int8_t tDegreeIncrement = DEGREES_PER_STEP;
    int8_t tIndex = 0;
    int8_t tIndexDelta = 1;

    // mark ProcessedDistancesArray as invalid
    sForwardDistancesInfo.ProcessedDistancesArray[0] = 0;

    if (sLastDistanceServoAngleInDegrees >= 180 - (START_DEGREES + 2)) {
// values for backward scanning
        tCurrentDegrees = 180 - START_DEGREES;
        tDegreeIncrement = -(tDegreeIncrement);
        tIndex = STEPS_PER_SCAN;
        tIndexDelta = -1;
    }
    if (!aDoFirstValue) {
// skip first value, since it is equal to last value of last measurement
        tIndex += tIndexDelta;
        tCurrentDegrees += tDegreeIncrement;
    }

    sBDEventJustReceived = false;
    while (tIndex >= 0 && tIndex < NUMBER_OF_DISTANCES) {
        /*
         * rotate servo, wait with delayAndLoopGUI() and get distance
         */
        DistanceServoWriteAndDelay(tCurrentDegrees, true); // this calls loopGui() during delay, which in turn may update distance sliders.
        if (!aForceScan && sBDEventJustReceived) {
            // User sent an event -> stop and return now
            return true;
        }
        unsigned int tCentimeter = getDistanceAsCentimeter(DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE, true);
        if(tCentimeter == DISTANCE_TIMEOUT_RESULT){
            // timeout here
            tCentimeter = DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE;
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
            tColor = COLOR16_RED; // tCentimeter <= sCentimeterDrivenPerScan
            if (tCentimeter >= DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE) {
                tColor = DISTANCE_TIMEOUT_COLOR;
            } else if (tCentimeter > sCentimetersDrivenPerScan * 2) {
                tColor = COLOR16_GREEN;
            } else if (tCentimeter > sCentimetersDrivenPerScan) {
                tColor = COLOR16_YELLOW;
            }

            /*
             * Clear old and draw new distance line
             */
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y,
                    sForwardDistancesInfo.RawDistancesArray[tIndex], tCurrentDegrees, COLOR16_WHITE, 3);
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tCentimeter, tCurrentDegrees, tColor,
                    3);
        }
        /*
         * Store value and search for min and max
         */
        sForwardDistancesInfo.RawDistancesArray[tIndex] = tCentimeter;

        tIndex += tIndexDelta;
        tCurrentDegrees += tDegreeIncrement;
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
        if (tDistance >= DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE) {
            tDistance = DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE;
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
        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tCurrentDegrees, tColor, 3);
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
 * The problem of the ultrasonic values is, that you can only detect a wall with the ultrasonic sensor if the angle of the wall relative to sensor axis is approximately between 70 and 110 degree.
 * For other angels the reflected ultrasonic beam can not not reach the receiver which leads to unrealistic great distances.
 *
 * Therefore I take samples every 18 degrees and if I get 2 adjacent short (< DISTANCE_MAX_FOR_WALL_DETECTION_CM) distances, I assume a wall determined by these 2 samples.
 * The (invalid) values 18 degrees right and left of these samples are then extrapolated by computeNeigbourValue().
 */
//#define TRACE // only used for this function
void doWallDetection() {
    uint8_t tTempDistancesArray[NUMBER_OF_DISTANCES];
    /*
     * First copy all raw values
     */
    memcpy(tTempDistancesArray, sForwardDistancesInfo.RawDistancesArray, NUMBER_OF_DISTANCES);

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
    uint8_t tLastDistance = tTempDistancesArray[0];
    uint8_t tCurrentDistance = tTempDistancesArray[1];
    for (uint_fast8_t i = 1; i < STEPS_PER_SCAN; ++i) {
        uint8_t tNextDistanceOriginal = tTempDistancesArray[i + 1];
        if (tLastDistance < DISTANCE_MAX_FOR_WALL_DETECTION_CM && tCurrentDistance < DISTANCE_MAX_FOR_WALL_DETECTION_CM) {
            /*
             * 2 adjacent short distances -> assume a wall -> adjust adjacent values
             */

            /*
             * Use computeNeigbourValue the other way round
             * i.e. put 20 degrees to 40 degrees parameter and vice versa in order to use the 0 degree value as the 60 degrees one
             */
            uint8_t tNextDistanceComputed = computeNeigbourValue(tCurrentDistance, tLastDistance,
            DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE, &tDegreeOfConnectingLine);
#if defined(TRACE) && defined(USE_BLUE_DISPLAY_GUI)
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
#if defined(TRACE) && defined(USE_BLUE_DISPLAY_GUI)
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
                tTempDistancesArray[i + 1] = tNextDistanceComputed;
                tNextDistanceOriginal = tNextDistanceComputed;
#if defined(USE_BLUE_DISPLAY_GUI)
                if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
                    BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextDistanceComputed,
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
    memcpy(sForwardDistancesInfo.ProcessedDistancesArray, tTempDistancesArray, NUMBER_OF_DISTANCES);

    tLastDistance = tTempDistancesArray[STEPS_PER_SCAN];
    tCurrentDistance = tTempDistancesArray[STEPS_PER_SCAN - 1];
    tCurrentAngleToCheck = 180 - (START_DEGREES + (2 * DEGREES_PER_STEP));

    /*
     * check values at i and i+1 and adjust value at i-1
     */
    for (uint_fast8_t i = STEPS_PER_SCAN - 1; i > 0; --i) {
        uint8_t tNextValue = tTempDistancesArray[i - 1];

// Do it only if none of the 3 values are processed before
        if (tTempDistancesArray[i + 1] == sForwardDistancesInfo.RawDistancesArray[i + 1]
                && tTempDistancesArray[i] == sForwardDistancesInfo.RawDistancesArray[i]
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
                DISTANCE_TIMEOUT_CM_AUTONOMOUS_DRIVE, &tDegreeOfConnectingLine);
#if defined(TRACE) && defined(USE_BLUE_DISPLAY_GUI)
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
#if defined(TRACE) && defined(USE_BLUE_DISPLAY_GUI)
                    BlueDisplay1.debug("tWallBackwardDegrees=", tWallBackwardDegrees);
#endif
                    if (tWallBackwardDegrees <= 90) {
                        // wall at right - overwrite only if greater
                        if (sForwardDistancesInfo.WallRightAngleDegrees < tWallBackwardDegrees) {
                            sForwardDistancesInfo.WallRightAngleDegrees = tWallBackwardDegrees;
#if defined(TRACE) && defined(USE_BLUE_DISPLAY_GUI)
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
                        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tNextValueComputed,
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

/*
 * Find min and max value. Prefer the headmost value if we have more than one choice
 * Do not use the values at 0 and 9 for minimum, since sometimes we measure the distance to the own wheels at this degrees.
 */
void postProcessDistances(uint8_t aDistanceThreshold) {
    unsigned int tMax = 0;
    unsigned int tMin = UINT16_MAX; // = 65535
    sForwardDistancesInfo.IndexOfDistanceGreaterThanThreshold = 0xFF;
    // scan simultaneously from 0 to 4 and 9 to 5 to prefer headmost values/indexes, if distances are the same.
    for (uint_fast8_t i = 0; i < (NUMBER_OF_DISTANCES + 1) / 2; ++i) {
        uint8_t tIndex = i;
        for (uint_fast8_t j = 0; j < 2; ++j) {
            uint8_t tDistance;
            // Check if we have processed distances, otherwise take the raw ones
            if (sForwardDistancesInfo.ProcessedDistancesArray[0] != 0) {
                tDistance = sForwardDistancesInfo.ProcessedDistancesArray[tIndex];
            } else {
                tDistance = sForwardDistancesInfo.RawDistancesArray[tIndex];
            }
            if (tDistance >= tMax) {
                tMax = tDistance;
                sForwardDistancesInfo.IndexOfMaxDistance = tIndex;
                sForwardDistancesInfo.MaxDistance = tDistance;
            }
            if (tDistance <= tMin && i != 0) {
                tMin = tDistance;
                sForwardDistancesInfo.IndexOfMinDistance = tIndex;
                sForwardDistancesInfo.MinDistance = tDistance;
            }
            if (tDistance >= aDistanceThreshold) {
                sForwardDistancesInfo.IndexOfDistanceGreaterThanThreshold = tIndex;
            }
            tIndex = STEPS_PER_SCAN - i;
        }
    }
}
#endif // defined(CAR_HAS_DISTANCE_SERVO)

#endif // _ROBOT_CAR_DISTANCE_HPP
