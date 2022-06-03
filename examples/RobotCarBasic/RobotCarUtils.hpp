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

#if defined(MONITOR_VIN_VOLTAGE)
#  if !defined(PRINT_VOLTAGE_PERIOD_MILLIS)
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000
#  endif

#  if !defined(PIN_VIN_ATTENUATED_INPUT)
#warning MONITOR_VIN_VOLTAGE is defined, but PIN_VIN_ATTENUATED_INPUT is NOT defined. -> we disable MONITOR_VIN_VOLTAGE now.
#undef MONITOR_VIN_VOLTAGE
#  endif

#  if !defined(VOLTAGE_DIVIDER_DIVISOR)
#define VOLTAGE_DIVIDER_DIVISOR   11.0  // VIN/11 by 1MOhm to VIN and 100kOhm to ground.
#  endif

#  if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
#define sVINVoltageDividerIsAttached true
#  else
bool sVINVoltageDividerIsAttached;      // State is determined at init()
#  endif

#include "ADCUtils.hpp"
uint16_t sVINRawSum;   // Sum of NUMBER_OF_VIN_SAMPLES raw readings of ADC
float sVINVoltage = FULL_BRIDGE_INPUT_MILLIVOLT / 1000; // set default value for later use
#endif // defined(MONITOR_VIN_VOLTAGE)

void printConfigInfo(Print *aSerial) {
#if defined(BASIC_CONFIG_NAME)
    aSerial->print(F("Car configuration is: " BASIC_CONFIG_NAME));
#endif
#if defined(CONFIG_NAME)
    aSerial->print(F(CONFIG_NAME));
#endif
    aSerial->println();
}

void printProgramOptions(Print *aSerial) {
    aSerial->println();
    aSerial->println(F("Settings:"));

    aSerial->print(F("ENABLE_RTTTL_FOR_CAR:"));
#if !defined(ENABLE_RTTTL_FOR_CAR)
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));

    aSerial->print(F("MONITOR_VIN_VOLTAGE:"));
#if !defined(MONITOR_VIN_VOLTAGE)
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));

#if !defined(VIN_VOLTAGE_CORRECTION)
    aSerial->print(F("VIN_VOLTAGE_CORRECTION:"
            ""));
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));
#else
    aSerial->println(F("VIN_VOLTAGE_CORRECTION=" STR(VIN_VOLTAGE_CORRECTION) "V"));
#endif

    aSerial->println(F("ADC_INTERNAL_REFERENCE_MILLIVOLT=" STR(ADC_INTERNAL_REFERENCE_MILLIVOLT) " mV"));
    aSerial->println();
}

/*
 * Call RobotCar.init() with different sets of parameters
 */
void initRobotCarPWMMotorControl() {
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    RobotCar.init();
#elif defined(CAR_HAS_4_MECANUM_WHEELS)
    RobotCar.init(FRONT_RIGHT_MOTOR_FORWARD_PIN, FRONT_RIGHT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN,
    FRONT_LEFT_MOTOR_FORWARD_PIN, FRONT_LEFT_MOTOR_BACKWARD_PIN, BACK_RIGHT_MOTOR_FORWARD_PIN, BACK_RIGHT_MOTOR_BACKWARD_PIN,
    BACK_LEFT_MOTOR_FORWARD_PIN, BACK_LEFT_MOTOR_BACKWARD_PIN);
#else
    RobotCar.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
            LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#endif

#if defined(MONITOR_VIN_VOLTAGE)
#  if !defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
    sVINVoltageDividerIsAttached = isVINVoltageDividerAttached(PIN_VIN_ATTENUATED_INPUT);
#  endif
    readVINVoltageAndAdjustDriveSpeed(); // This value might not correct, but it sets the channel and reference for VIN initially
    readVINVoltageAndAdjustDriveSpeed(); // 2. call costs only 2 bytes :-)
#endif
}

#if defined(CAR_HAS_DISTANCE_SENSOR)
#include "Distance.h"
unsigned int getDistanceAndPlayTone() {
    /*
     * Get distance
     */
#  if !defined(USE_IR_REMOTE)
#    if defined(US_DISTANCE_SENSOR_ENABLE_PIN)
    // if US_DISTANCE_SENSOR_ENABLE_PIN is connected to ground we use the US distance as fallback. Useful for testing the difference between both sensors.
    if (digitalRead(US_DISTANCE_SENSOR_ENABLE_PIN) == HIGH) {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_IR_OR_TOF;
    } else {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_US;
    }
#    endif
#    if defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && defined(DISTANCE_FEEDBACK_MODE) // If this pin is connected to ground, enable distance feedback
    if (digitalRead(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) == HIGH) {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
        noTone(PIN_BUZZER);
    } else {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_MODE;
    }
#    endif
#  endif // !defined(USE_IR_REMOTE)

    unsigned int tCentimeter = getDistanceAsCentimeterAndPlayTone(DISTANCE_TIMEOUT_CM_FOLLOWER, true); // timeout at 150 cm
    return tCentimeter;
}
#endif

/************************************
 * Functions to monitor VIN voltage
 ************************************/
/*
 * @return true, if voltage divider attached
 */
bool isVINVoltageDividerAttached(uint8_t aPin) {
#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
    (void) aPin;
    return true; // if we know it by compile option just return true, this saves 160 bytes program memory
#else
    pinModeFast(aPin, OUTPUT);
    digitalWriteFast(aPin, LOW); // discharge any charge at pin
    pinModeFast(aPin, INPUT);
    readVINVoltageAndAdjustDriveSpeed();
    bool tDividerAttached = sVINVoltage > 3.0;
#  if defined(APPLICATON_INFO) // requires 1504 bytes program space
    Serial.print(F("VIN voltage divider"));
    if (!tDividerAttached) {
        Serial.print(F("not "));
    }
    Serial.println(F(" attached"));
#  endif
    return tDividerAttached; // if voltage measured > 3 volt, assume that we have a voltage divider attached
#endif
}

#if (MOTOR_PWM_PIN == 5) || (MOTOR_PWM_PIN == 6)
#define NUMBER_OF_VIN_SAMPLES   10 // Get 10 samples lasting 1030 us, which is almost the PWM period of 1024 us for UNO/Nano pin 5 and 6.
#else
#define NUMBER_OF_VIN_SAMPLES   20 // Get 20 samples lasting 2060 us, which is almost the PWM period of 2048 us.
#endif
/*
 * Read multiple samples during a PWM period and adjust DriveSpeedPWMFor2Volt
 */
void readVINVoltageAndAdjustDriveSpeed() {
#if defined(MONITOR_VIN_VOLTAGE)
#if defined(ESP32)
    // On ESP32 currently not supported
#else
    if (sVINVoltageDividerIsAttached) { // sVINVoltageDividerIsAttached is constant true if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    // Here we have also other channels than VIN, that we convert during the loop
    uint8_t tOldADMUX = checkAndWaitForReferenceAndChannelToSwitch(VIN_ATTENUATED_INPUT_CHANNEL, INTERNAL);
#  endif
        /*
         * Here VIN is the only channel we convert.
         * Get 10 samples lasting 1030 us, which is almost the PWM period of 1024 us.
         */
        sVINRawSum =
                readADCChannelWithReferenceMultiSamples(VIN_ATTENUATED_INPUT_CHANNEL, INTERNAL, NUMBER_OF_VIN_SAMPLES); // 10 samples
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    checkAndWaitForReferenceAndChannelToSwitch(tOldADMUX & MASK_FOR_ADC_CHANNELS, tOldADMUX >> SHIFT_VALUE_FOR_REFERENCE);
#  endif

//    BlueDisplay1.debug("VIN Raw=", tVINRaw);
#if defined(DEBUG)
    Serial.print(F("sVINRawSum="));
    Serial.println(sVINRawSum);
#endif

// assume resistor network of 1MOhm / 100kOhm (divider by 11)
// tVIN * 0,01182795
#  if defined(VIN_VOLTAGE_CORRECTION)
    // we have a diode (requires 0.8 to 0.9 volt) between LIPO and VIN
        sVINVoltage = (sVINRawSum * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023)) + VIN_VOLTAGE_CORRECTION;
#  else
        // sVINRawSum * 0.000591
        sVINVoltage = sVINRawSum
                * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023);
#  endif
        /*
         * Adjust DriveSpeedPWMFor2Volt according to voltage
         */
        RobotCar.setDriveSpeedPWMFor2Volt(sVINVoltage);
    }
#endif // defined(ESP32)
#endif // defined(MONITOR_VIN_VOLTAGE)
}

/*
 * Start motors direction forward, get voltage after 400 ms and call setDriveSpeedPWMFor2Volt()
 */
void calibrateDriveSpeedPWM() {
#if defined(MONITOR_VIN_VOLTAGE)
    if (sVINVoltageDividerIsAttached) { // sVINVoltageDividerIsAttached is constant true if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
        RobotCar.setSpeedPWMAndDirection(MAX_SPEED_PWM / 2);
        delay(400);
        readVINVoltageAndAdjustDriveSpeed();
        RobotCar.stop();
    }
#endif
}

/*
 * Check VIN every 2 seconds (PRINT_VOLTAGE_PERIOD_MILLIS) and print if changed
 * Resolution is 10 mV
 */
void checkVinPeriodicallyAndPrintIfChanged() {
#if defined(MONITOR_VIN_VOLTAGE)
#  if defined(ESP32)
    // On ESP32 currently not supported
#  else
    static uint16_t sLastVINRawSumPrinted;
    static uint32_t sMillisOfLastVCCInfo;

    if (sVINVoltageDividerIsAttached) { // sVINVoltageDividerIsAttached is constant true if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
        uint32_t tMillis = millis();

        if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
            sMillisOfLastVCCInfo = tMillis;
            readVINVoltageAndAdjustDriveSpeed(); // sets sVINRawSum
            /*
             * Check if voltage has changed (44 bytes)
             */
            if (abs(sLastVINRawSumPrinted - sVINRawSum) > NUMBER_OF_VIN_SAMPLES) {
                sLastVINRawSumPrinted = sVINRawSum;
#    if defined(APPLICATON_INFO) // requires 1504 bytes program space
                Serial.print(F("VIN="));
                Serial.print(sVINVoltage);
                Serial.println(F("V"));
#    endif
            }
        }
    }
#  endif // defined(ESP32)
#endif
}

#endif // _ROBOT_CAR_UTILS_HPP
