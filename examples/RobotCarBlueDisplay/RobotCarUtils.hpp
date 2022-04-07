/*
 * RobotCarUtils.hpp
 *
 *  Contains miscellaneous convenience utility functions for the robot cars.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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

#ifndef _ROBOT_CAR_UTILS_HPP
#define _ROBOT_CAR_UTILS_HPP

#include "RobotCarUtils.h"

void printConfigInfo() {
#if defined(BASIC_CONFIG_NAME)
    Serial.print(F("Car configuration is: " BASIC_CONFIG_NAME));
#endif
#if defined(CONFIG_NAME)
    Serial.print(F(CONFIG_NAME));
#endif
    Serial.println();
}

/*
 * Call RobotCarPWMMotorControl.init() with different sets of parameters
 */
void initRobotCarPWMMotorControl() {
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    RobotCarPWMMotorControl.init();
#elif defined(CAR_HAS_4_MECANUM_WHEELS)
    RobotCarPWMMotorControl.init(FRONT_RIGHT_MOTOR_FORWARD_PIN, FRONT_RIGHT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN,
    FRONT_LEFT_MOTOR_FORWARD_PIN, FRONT_LEFT_MOTOR_BACKWARD_PIN, BACK_RIGHT_MOTOR_FORWARD_PIN, BACK_RIGHT_MOTOR_BACKWARD_PIN,
    BACK_LEFT_MOTOR_FORWARD_PIN, BACK_LEFT_MOTOR_BACKWARD_PIN);
#else
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
            LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#endif
}

#if defined(CAR_HAS_US_DISTANCE_SENSOR) || defined(CAR_HAS_IR_DISTANCE_SENSOR)
unsigned int getDistanceAndPlayTone() {
    /*
     * Get distance
     */
#if !defined(USE_IR_REMOTE)
#  if defined(US_DISTANCE_SENSOR_ENABLE_PIN)
    // if US_DISTANCE_SENSOR_ENABLE_PIN is connected to ground we use the US distance as fallback. Useful for testing the difference between both sensors.
    if (digitalRead(US_DISTANCE_SENSOR_ENABLE_PIN) == HIGH) {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_IR_OR_TOF;
    } else {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_US;
    }
#  endif
    sDistanceFeedbackMode = DISTANCE_FEEDBACK_CONTINUOUSLY; // DISTANCE_FEEDBACK_CONTINUOUSLY or DISTANCE_FEEDBACK_PENTATONIC
#endif

    unsigned int tCentimeter = getDistanceAsCentimeterAndPlayTone(150, true); // timeout at 150 cm
    return tCentimeter;
}
#endif

#if defined(MONITOR_VIN_VOLTAGE)
/*
 * Check VIN every 2 seconds (PRINT_VOLTAGE_PERIOD_MILLIS) and print if changed
 * Resolution is 10 mV
 */
void checkVinPeriodicallyAndPrintIfChanged() {
#  if defined(ESP32)
    // On ESP32 currently not supported
#  else
    static uint32_t sMillisOfLastVCCInfo;
    static uint16_t  sLastVINRaw;
    uint32_t tMillis = millis();

    if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
        sMillisOfLastVCCInfo = tMillis;
#    if defined(CAR_HAS_IR_DISTANCE_SENSOR)
        // We need to change ADC channel and reference from the values of reading IR distance sensor, so we have to wait for the signal to settle
        uint16_t tVINRaw = waitAndReadADCChannelWithReferenceAndRestoreADMUX(VIN_11TH_IN_CHANNEL, INTERNAL);
#    else
        // Here we have no IR distance sensor, so we can keep it simple
        analogReference(INTERNAL);
        uint16_t tVINRaw = analogRead(PIN_VIN_11TH_IN); // this switches the reference and ADC-mux
        // to be tested
//        delay(10);                                      // wait to settle signal
//        tVINRaw = analogRead(PIN_VIN_11TH_IN);
#    endif
//        Serial.print(F("VINRaw="));
//        Serial.println(tVINRaw);
        /*
         * Compute and print if changed
         */
        if(abs(sLastVINRaw - tVINRaw) > 2) {
            sLastVINRaw = tVINRaw;
            /*
             * Assume resistor network of 1MOhm / 100kOhm (divider by 11)
             * tVIN = tVINRaw * 0,01182795 => resolution is 0.01 volt
             */
#    if defined(VIN_VOLTAGE_CORRECTION)
            // we have a diode  which requires around 0.8 volt between LIPO and VIN
            float tVIN = (tVINRaw * ((11.0 * 1.1) / 1023)) + VIN_VOLTAGE_CORRECTION;
#    else
            float tVIN = tVINRaw * ((11.0 * 1.1) / 1023);
#    endif
            Serial.print(F("VIN="));
            Serial.print(tVIN);
            Serial.println(F("V"));
        }
    }
#  endif // defined(ESP32)
}

#endif // MONITOR_VIN_VOLTAGE

#endif // _ROBOT_CAR_UTILS_HPP
#pragma once
