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
#include "CarPWMMotorControl.h"
#include "digitalWriteFast.h"

#if !defined(VOLTAGE_DIVIDER_DIVISOR)
#define VOLTAGE_DIVIDER_DIVISOR   11.0  // VIN/11 by 1MOhm to VIN and 100kOhm to ground.
#endif

#if !defined(PRINT_VOLTAGE_PERIOD_MILLIS)
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000
#endif

#include "ADCUtils.hpp"
uint16_t sVINRawSum;   // Sum of NUMBER_OF_VIN_SAMPLES raw readings of ADC
float sVINVoltage;

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

#if defined(CAR_HAS_DISTANCE_SENSOR)
#include "Distance.h"
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

    unsigned int tCentimeter = getDistanceAsCentimeterAndPlayTone(DISTANCE_TIMEOUT_CM_FOLLOWER, true); // timeout at 150 cm
    return tCentimeter;
}
#endif

/*
 * For MONITOR_VIN_VOLTAGE
 */
/*
 * @return true, if voltage divider attached
 */
bool isVINVoltageDividerAttached(uint8_t aPin) {
#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
    return true; // if we know it by compile option just return true, this saves 160 bytes program memory
#else
    pinModeFast(aPin, OUTPUT);
    digitalWriteFast(aPin, LOW); // discharge any charge at pin
    pinModeFast(aPin, INPUT);
    readVINVoltage();
    bool tDividerAttached = sVINVoltage > 3.0;
    Serial.print(F("VIN voltage divider"));
    if (!tDividerAttached) {
        sVINVoltage = FULL_BRIDGE_INPUT_MILLIVOLT / 1000; // set default value for later use
        Serial.print(F("not "));
    }
    Serial.println(F(" attached"));

    return tDividerAttached; // if voltage measured > 3 volt, assume that we have a voltage divider attached
#endif
}

#if (MOTOR_PWM_PIN == 5) || (MOTOR_PWM_PIN == 6)
#define NUMBER_OF_VIN_SAMPLES   10 // Get 10 samples lasting 1030 us, which is almost the PWM period of 1024 us.
#else
#define NUMBER_OF_VIN_SAMPLES   20 // Get 20 samples lasting 2060 us, which is almost the PWM period of 2048 us.
#endif
/*
 * Read multiple samples during a PWM period
 */
void readVINVoltage() {
#if defined(ESP32)
    // On ESP32 currently not supported
#else
#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    // Here we have also other channels than VIN, that we convert during the loop
    uint8_t tOldADMUX = checkAndWaitForReferenceAndChannelToSwitch(VIN_ATTENUATED_INPUT_CHANNEL, INTERNAL);
#endif
    /*
     * Here VIN is the only channel we convert.
     * Get 10 samples lasting 1030 us, which is almost the PWM period of 1024 us.
     */
    sVINRawSum = readADCChannelWithReferenceMultiSamples(VIN_ATTENUATED_INPUT_CHANNEL, INTERNAL, NUMBER_OF_VIN_SAMPLES); // 10 samples
#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    checkAndWaitForReferenceAndChannelToSwitch(tOldADMUX & MASK_FOR_ADC_CHANNELS, tOldADMUX >> SHIFT_VALUE_FOR_REFERENCE);
#endif

//    BlueDisplay1.debug("VIN Raw=", tVINRaw);

// assume resistor network of 1MOhm / 100kOhm (divider by 11)
// tVIN * 0,01182795
#if defined(VIN_VOLTAGE_CORRECTION)
    // we have a diode (requires 0.8 to 0.9 volt) between LIPO and VIN
    sVINVoltage = (sVINRawSum * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023)) + VIN_VOLTAGE_CORRECTION;
#else
    sVINVoltage = sVINRawSum
            * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023);
#endif
#endif // defined(ESP32)
}

/*
 * Check VIN every 2 seconds (PRINT_VOLTAGE_PERIOD_MILLIS) and print if changed
 * Resolution is 10 mV
 */
void checkVinPeriodicallyAndPrintIfChanged() {
#if defined(ESP32)
    // On ESP32 currently not supported
#else
    static uint16_t sLastVINRawSumPrinted;
    static uint32_t sMillisOfLastVCCInfo;
    uint32_t tMillis = millis();

    if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
        sMillisOfLastVCCInfo = tMillis;
        readVINVoltage(); // sets sVINRawSum
        /*
         * Check if voltage has changed (44 bytes)
         */
        if (abs(sLastVINRawSumPrinted - sVINRawSum) > NUMBER_OF_VIN_SAMPLES) {
            sLastVINRawSumPrinted = sVINRawSum;
            Serial.print(F("VIN="));
            Serial.print(sVINVoltage);
            Serial.println(F("V"));
        }
    }
#endif // defined(ESP32)
}

#endif // _ROBOT_CAR_UTILS_HPP
