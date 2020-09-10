/*
 * Distance.cpp
 *
 *  Contains all distance measurement functions.
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include "Distance.h"
#include "RobotCar.h"
#include "RobotCarGui.h"

#include "HCSR04.h"

ForwardDistancesInfoStruct sForwardDistancesInfo;

Servo DistanceServo;
uint8_t sLastServoAngleInDegrees; // 0 - 180 needed for optimized delay for servo repositioning

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
// removing usage of SFEVL53L1X wrapper class saves 794 bytes
VL53L1X sToFDistanceSensor(&Wire, -1, -1); // 100 kHz
#endif

/*
 * This initializes the pins too
 */
void initDistance() {
#ifdef USE_US_SENSOR_1_PIN_MODE
    initUSDistancePin(PIN_TRIGGER_OUT);
#else
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);
#endif

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
    if (sToFDistanceSensor.VL53L1X_SensorInit() != 0) { //Begin returns 0 on a good init
        BlueDisplay1.debug("ToF sensor connect failed!");
    }
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

//#define USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
/*
 * sets also sLastServoAngleInDegrees to enable optimized servo movement and delays
 * SG90 Micro Servo has reached its end position if the current (200 mA) is low for more than 11 to 14 ms
 * No action if aTargetDegrees == sLastServoAngleInDegrees
 */
void DistanceServoWriteAndDelay(uint8_t aTargetDegrees, bool doDelay) {

    if (aTargetDegrees > 220) {
        // handle underflow
        aTargetDegrees = 0;
    } else if (aTargetDegrees > 180) {
        // handle underflow
        aTargetDegrees = 180;
    }

    uint8_t tDeltaDegrees;
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
    int8_t tOvershootDegrees; // Experimental
#endif
    uint8_t tLastServoAngleInDegrees = sLastServoAngleInDegrees;
    sLastServoAngleInDegrees = aTargetDegrees;

    if (tLastServoAngleInDegrees == aTargetDegrees) {
        return;
    } else if (aTargetDegrees > tLastServoAngleInDegrees) {
        tDeltaDegrees = aTargetDegrees - tLastServoAngleInDegrees;
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
        tOvershootDegrees = 3; // Experimental
#endif
    } else {
        tDeltaDegrees = tLastServoAngleInDegrees - aTargetDegrees;
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
        tOvershootDegrees = -3; // Experimental
#endif
    }

    /*
     * Move servo
     */
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
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

#ifdef DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN
    // The servo is top down and therefore inverted
    aTargetDegrees = 180 - aTargetDegrees;
#endif
    DistanceServo.write(aTargetDegrees);

    /*
     * Delay
     */
    if (doDelay) {
#ifdef USE_ENCODER_MOTOR_CONTROL
// Synchronize before doing delay
        rightCarMotor.synchronizeMotor(&leftCarMotor, MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
#endif
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
#ifdef USE_OVERSHOOT_FOR_FAST_SERVO_MOVING
            tWaitDelayforServo = tDeltaDegrees * 5;
#else
#  ifdef CAR_HAS_IR_DISTANCE_SENSOR
            tWaitDelayforServo = tDeltaDegrees * 9; // 9 => 162 ms for 18 degrees
#  else
            tWaitDelayforServo = tDeltaDegrees * 8; // 7 => 128 ms, 8 => 144 for 18 degrees
#  endif
#endif
        }
        delayAndLoopGUI(tWaitDelayforServo);
    }
}

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

    if (sLastServoAngleInDegrees >= 180 - (START_DEGREES + 2)) {
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
        DistanceServoWriteAndDelay(tCurrentDegrees, true);
        if (!aForceScan && sBDEventJustReceived) {
            // User sent an event -> stop and return now
            return true;
        }

        unsigned int tCentimeter = getDistanceAsCentiMeter();
        if ((tIndex == INDEX_FORWARD_1 || tIndex == INDEX_FORWARD_2) && tCentimeter <= sCentimeterPerScanTimesTwo) {
            /*
             * Emergency motor stop if index is forward and measured distance is less than distance driven during two scans
             */
            RobotCarMotorControl.stopCarAndWaitForIt();
        }

        if (sCurrentPage == PAGE_AUTOMATIC_CONTROL && BlueDisplay1.isConnectionEstablished()) {
            /*
             * Determine color
             */
            tColor = COLOR_RED; // tCentimeter <= sCentimeterPerScan
            if (tCentimeter >= DISTANCE_TIMEOUT_CM) {
                tColor = DISTANCE_TIMEOUT_COLOR;
            } else if (tCentimeter > sCentimeterPerScanTimesTwo) {
                tColor = COLOR_GREEN;
            } else if (tCentimeter > sCentimeterPerScan) {
                tColor = COLOR_YELLOW;
            }

            /*
             * Clear old and draw new line
             */
            BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y,
                    sForwardDistancesInfo.RawDistancesArray[tIndex], tCurrentDegrees, COLOR_WHITE, 3);
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
 * Find min and max value. Prefer the headmost value if we have more than one choice
 * Do not use the values at 0 and 9 for minimum, since sometimes we measure the distance to the own wheels at this degrees.
 */
void postProcessDistances() {
    unsigned int tMax = 0;
    unsigned int tMin = __UINT16_MAX__; // = 65535
    // scan simultaneously from 0 to 4 and 9 to 5 to prefer headmost values/indexes, if distances are the same.
    for (uint8_t i = 0; i < (NUMBER_OF_DISTANCES + 1) / 2; ++i) {
        uint8_t tIndex = i;
        for (uint8_t j = 0; j < 2; ++j) {
            uint8_t tDistance;
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
            tIndex = STEPS_PER_SCAN - i;
        }
    }
}

void checkAndShowDistancePeriodically(uint16_t aPeriodMillis) {
    // Do not show distanced during (time critical) acceleration or deceleration
    if (!RobotCarMotorControl.needsFastUpdates()) {
        static long sLastUSMeasurementMillis;
        long tMillis = millis();
        if (sLastUSMeasurementMillis + aPeriodMillis < tMillis) {
            sLastUSMeasurementMillis = tMillis;
#ifdef CAR_HAS_IR_DISTANCE_SENSOR
            showIRDistance(getIRDistanceAsCentimeter());
#elif CAR_HAS_TOF_DISTANCE_SENSOR
            showIRDistance(getToFDistanceAsCentimeter());
#endif
            // feedback as slider length
            showUSDistance(getUSDistanceAsCentiMeterWithCentimeterTimeout(300));
        }
    }
}

/*
 * Timeout is DISTANCE_TIMEOUT_CM (1 meter)
 */
unsigned int getDistanceAsCentiMeter() {
#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
    if (sScanMode != SCAN_MODE_US) {
        sToFDistanceSensor.VL53L1X_StartRanging();
    }
#endif

    unsigned int tCentimeter = getUSDistanceAsCentiMeterWithCentimeterTimeout(DISTANCE_TIMEOUT_CM);
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    unsigned int tIRCentimeter;
    if (sScanMode != SCAN_MODE_US) {
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
        if (sScanMode != SCAN_MODE_US) {
            tIRCentimeter = getIRDistanceAsCentimeter();
        }
#  elif defined(CAR_HAS_TOF_DISTANCE_SENSOR)
        if (sScanMode != SCAN_MODE_US) {
            tIRCentimeter = readToFDistanceAsCentimeter();
        }
#  endif
        if (sScanMode == SCAN_MODE_IR) {
            tCentimeter = tIRCentimeter;
        } else if (sScanMode == SCAN_MODE_MINIMUM) {
            // Scan mode MINIMUM => Take the minimum of the two values
            if (tCentimeter > tIRCentimeter) {
                tCentimeter = tIRCentimeter;
            }
        } else if (sScanMode == SCAN_MODE_MAXIMUM) {
            // Scan mode MAXIMUM => Take the maximum of the two values
            if (tCentimeter < tIRCentimeter) {
                tCentimeter = tIRCentimeter;
            }
        }
    }
#endif
    return tCentimeter;
}

#ifdef CAR_HAS_IR_DISTANCE_SENSOR
/*
 * The 1080 needs 39 ms for each measurement cycle
 */
uint8_t getIRDistanceAsCentimeter() {
    float tVolt = analogRead(PIN_IR_DISTANCE_SENSOR);
    // * 0.004887585 for 1023 = 5V
    // Model 1080 / GP2Y0A21YK0F
    return (29.988 * pow(tVolt * 0.004887585, -1.173)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp

    // Model 20150 - Do not forget to add at least 100uF capacitor across the Vcc and GND connections on the sensor
//    return (60.374 * pow(tVolt * 0.004887585, -1.16)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
}
#endif

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
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
        delayAndLoopGUI(4);
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

#endif
