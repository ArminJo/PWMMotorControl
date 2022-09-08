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

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

#if defined(ROBOT_CAR_BLUE_DISPLAY)
#include "RobotCarGui.h"
bool doCalibration = false;
bool receivedStopForCalibration = false;
#endif

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
uint16_t sLastVINRawSum;   // Sum of NUMBER_OF_VIN_SAMPLES raw readings of ADC, used to determine if voltage has changed and must be displayed.
float sVINVoltage = FULL_BRIDGE_INPUT_MILLIVOLT / 1000; // set default value for later use
uint32_t sMillisOfLastVCCInfo;
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

#if !defined(ROBOT_CAR_BLUE_DISPLAY)
#  if defined(USE_IR_REMOTE)
    // For available IR commands see IRCommandMapping.h https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_INPUT_PIN)));
#  else
    Serial.println(F("USE_IR_REMOTE: not defined"));
#  endif
#endif

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
    aSerial->println(F("VIN_VOLTAGE_CORRECTION: 0 V"));
#else
    aSerial->println(F("VIN_VOLTAGE_CORRECTION=" STR(VIN_VOLTAGE_CORRECTION) " V"));
#endif

    aSerial->println(F("ADC_INTERNAL_REFERENCE_MILLIVOLT: " STR(ADC_INTERNAL_REFERENCE_MILLIVOLT) " mV"));
    aSerial->println();

#if defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && defined(DISTANCE_FEEDBACK_MODE) // If this pin is connected to ground, enable distance feedback
    aSerial->print(F("Pin " STR(DISTANCE_TONE_FEEDBACK_ENABLE_PIN)));
    if (digitalRead(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) == HIGH) {
        aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
    }
    aSerial->print(F(" connected to ground. Tone feedback"));
    if (digitalRead(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) == HIGH) {
        aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
    }
    aSerial->println(F(" enabled."));
#endif

#if defined(ROBOT_CAR_BLUE_DISPLAY)
    aSerial->println(F("If not connected, run follower demo after " STR(TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS) " ms"));
#endif
#if defined(ROBOT_CAR_BLUE_DISPLAY) || !defined(USE_IR_REMOTE)
    aSerial->println(
            F(
                    "Keep distance between " STR(FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) " and " STR(FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) " cm. Rescan if distance > " STR(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER) " cm"));
#endif
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
    readVINVoltage(); // This sets the channel and reference for VIN initially
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
    readVINVoltageAndAdjustDriveSpeedAndPrint();
    bool tDividerAttached = sVINVoltage > 3.0;
#  if !defined(NO_APPLICATON_INFO) // requires 1504 bytes program space
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
 * @return true if voltage changed
 */
bool readVINVoltage() {
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
        auto tVINRawSum =
                readADCChannelWithReferenceMultiSamples(VIN_ATTENUATED_INPUT_CHANNEL, INTERNAL, NUMBER_OF_VIN_SAMPLES); // 10 samples
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    checkAndWaitForReferenceAndChannelToSwitch(tOldADMUX & MASK_FOR_ADC_CHANNELS, tOldADMUX >> SHIFT_VALUE_FOR_REFERENCE);
#  endif

#if defined(TRACE)
#  if defined(ROBOT_CAR_BLUE_DISPLAY)
    BlueDisplay1.debug("VINRawSum=", tVINRawSum);
#  else
    Serial.print(F("VINRawSum="));
    Serial.println(tVINRawSum);
#  endif
#endif

// assume resistor network of 1MOhm / 100kOhm (divider by 11)
// tVIN * 0,01182795
#  if defined(VIN_VOLTAGE_CORRECTION)
    // we have a diode (requires 0.8 to 0.9 volt) between LIPO and VIN
        sVINVoltage = (tVINRawSum * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023)) + VIN_VOLTAGE_CORRECTION;
#  else
        // Voltage correction is 0 volt. sLastVINRawSum * 0.000591
        sVINVoltage = tVINRawSum
                * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023);
#  endif
        // resolution is about 5 mV and we display in a 10 mV resolution -> compare with (2 * NUMBER_OF_VIN_SAMPLES)
        if(abs(sLastVINRawSum - tVINRawSum) > (2 * NUMBER_OF_VIN_SAMPLES)) {
            sLastVINRawSum = tVINRawSum;
            return true;
        }
    }
#endif // defined(ESP32)
#endif // defined(MONITOR_VIN_VOLTAGE)
    return false;
}

/*
 * Read multiple samples during a PWM period and adjust DriveSpeedPWMFor2Volt
 */
void readVINVoltageAndAdjustDriveSpeedAndPrint() {
#if defined(MONITOR_VIN_VOLTAGE)
#if defined(ESP32)
    // On ESP32 currently not supported
#else
    if (sVINVoltageDividerIsAttached) { // sVINVoltageDividerIsAttached is constant true if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
        readVINVoltage();
        /*
         * Adjust DriveSpeedPWMFor2Volt according to voltage
         */
#  if defined(ROBOT_CAR_BLUE_DISPLAY)
        uint8_t tOldDriveSpeedPWM = RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt;
        RobotCar.setDriveSpeedPWMFor2Volt(sVINVoltage);
        sprintf_P(sStringBuffer, PSTR("2 volt PWM %3d -> %3d"), tOldDriveSpeedPWM, RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
        BlueDisplay1.debug(sStringBuffer);
#  else
        Serial.print(F("2 volt PWM: "));
        Serial.print(RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
        Serial.print(F(" -> "));
        RobotCar.setDriveSpeedPWMFor2Volt(sVINVoltage);
        Serial.println(RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
#  endif
    }
#endif // defined(ESP32)
#endif // defined(MONITOR_VIN_VOLTAGE)
}

/*
 * Start motors direction forward, get voltage after 400 ms and call setDriveSpeedPWMFor2Volt()
 */
void calibrateDriveSpeedPWMAndPrint() {
#if defined(MONITOR_VIN_VOLTAGE)
    if (sVINVoltageDividerIsAttached) { // sVINVoltageDividerIsAttached is constant true if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
        RobotCar.setSpeedPWMAndDirection(MAX_SPEED_PWM / 2);
        delay(400);
        readVINVoltageAndAdjustDriveSpeedAndPrint();
        RobotCar.stop();
#  if defined(ROBOT_CAR_BLUE_DISPLAY)
        isPWMCalibrated = true;
#  endif
    }
#endif
}

/*
 * For remote control, but not for 2WD car with encoder motor.
 */
#if (defined(USE_IR_REMOTE) || defined(ROBOT_CAR_BLUE_DISPLAY)) && !defined(USE_MPU6050_IMU) \
    && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))
/*
 * Start a 720 degree turn to be sure to reach 360 degree.
 * The user should press the stop button when 360 degree was reached.
 * Then compute the MillimeterPer256Degreee value used for rotations.
 * If a IR stop command is received in the first 4 seconds after start of rotation, it is taken as abort command.
 * @return true if aborted or timeout
 */
bool calibrateRotation(turn_direction_t aTurnDirection) {
#if defined(ROBOT_CAR_BLUE_DISPLAY)
#  if VERSION_BLUE_DISPLAY_HEX >= VERSION_HEX_VALUE(3, 0, 3)
#    if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
    BlueDisplay1.debug(F("Press stop button at 360 deg.")); // Message must be less than 32 bytes
#    else
    BlueDisplay1.debug(F("Press stop button at 720 deg.")); // Message must be less than 32 bytes
#    endif
    TouchButtonRobotCarStartStop.setValueAndDraw(BUTTON_AUTO_RED_GREEN_VALUE_FOR_GREEN);
#  else
#    if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
    BlueDisplay1.debug("Press stop button at 360 deg."); // Message must be less than 32 bytes
#    else
    BlueDisplay1.debug("Press stop button at 720 deg."); // Message must be less than 32 bytes
#    endif
    TouchButtonRobotCarStartStop.setValueAndDraw(true);
#  endif
    receivedStopForCalibration = false; // Init
#else // defined(ROBOT_CAR_BLUE_DISPLAY)
#  if !defined(NO_APPLICATON_INFO) || defined(LOCAL_DEBUG)
// requires 1504 bytes program space
#    if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
    Serial.println(F("Press stop button at 360 degree"));
#    else
    Serial.println(F("Press stop button at 720 degree"));
#    endif
#  endif
    IRDispatcher.requestToStopReceived = false;
#endif // defined(ROBOT_CAR_BLUE_DISPLAY)
    /*
     * Initialize
     */
#if !defined(USE_ENCODER_MOTOR_CONTROL)
    auto tStartMillis = millis();
#endif
    /*
     * Start turn
     */
#if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
    RobotCar.startRotate(720, aTurnDirection); // around 6 seconds for 360 degree
#else
    RobotCar.startRotate(1440, aTurnDirection);
#endif
    while (RobotCar.updateMotors()) { // wait for 720 degree
#if defined(ROBOT_CAR_BLUE_DISPLAY)
        if (receivedStopForCalibration) {
#else
        if (IRDispatcher.requestToStopReceived) {
#endif
            /*
             * RequestToStopReceived is set by ISR and must not polled or enabled
             * Here we received a stop key from user. The stop command doStop() is NOT executed, it is scheduled.
             */
#if defined(USE_ENCODER_MOTOR_CONTROL)
            uint16_t tNewMillimeterPer256Degree = RobotCar.rightCarMotor.getCurrentDistanceMillimeter();
#else
            unsigned long tMillisPer360Degree = millis() - tStartMillis;
#  if defined(LOCAL_DEBUG) && !defined(ROBOT_CAR_BLUE_DISPLAY)
            Serial.print(F("Millis for 360 degree="));
            Serial.println(tMillisPer360Degree);
#  endif
            /*
             * Plausi
             */
            if (tMillisPer360Degree < 2500) {
                return true; // we received stop in the first 2.5 seconds -> interpret it as abort
            }
            /*
             * Set millimeter per degree and not millis per degree, since startRotate() calls startGoDistanceMillimeter() and therefore requires millimeter.
             * Example: tMillisPer360Degree = 6000 => 16.666 millis per degree 4266 millis per 256 degree.
             * => with 43 MillisPerCentimeter we get 992 MillimeterPer256Degreee. 5000 -> 826, 6500 -> 1074, 7000 -> 1157, 8000 -> 1322
             */
#  if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
            uint16_t tNewMillimeterPer256Degree = (tMillisPer360Degree * 256)
                    / (uint16_t) (36 * RobotCar.rightCarMotor.MillisPerCentimeter);
#  else
            uint16_t tNewMillimeterPer256Degree = (tMillisPer360Degree * 256)
                    / (uint16_t) (72 * RobotCar.rightCarMotor.MillisPerCentimeter);
#  endif
#endif // defined(USE_ENCODER_MOTOR_CONTROL)

            if (aTurnDirection == TURN_IN_PLACE) {
                RobotCar.MillimeterPer256DegreeInPlace = tNewMillimeterPer256Degree;
            } else {
                RobotCar.MillimeterPer256Degree = tNewMillimeterPer256Degree;
            }
#if defined(ROBOT_CAR_BLUE_DISPLAY)
            BlueDisplay1.debug("mm/256deg=", tNewMillimeterPer256Degree);
#else
            Serial.print(F("MillimeterPer256Degreee="));
            Serial.println(tNewMillimeterPer256Degree);
#endif

            RobotCar.stop();
#if !defined(ROBOT_CAR_BLUE_DISPLAY)
            IRDispatcher.requestToStopReceived = false; // to end this loop we receive a stop command
#endif
            return false;
        }
#if defined(ROBOT_CAR_BLUE_DISPLAY)
        checkAndHandleEvents();
#endif
    }
    return true;
}
#endif // #if (defined(USE_IR_REMOTE) || defined(ROBOT_CAR_BLUE_DISPLAY)) && !defined(USE_MPU6050_IMU) && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))

/*
 * Check VIN every 2 seconds (PRINT_VOLTAGE_PERIOD_MILLIS) and print if changed
 * Resolution is 10 mV
 * TODO implement check for low voltage
 */
void checkVinPeriodicallyAndPrintIfChanged() {
#if defined(MONITOR_VIN_VOLTAGE)
#  if defined(ESP32)
    // On ESP32 currently not supported
#  else

    if (sVINVoltageDividerIsAttached) { // sVINVoltageDividerIsAttached is constant true if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
        uint32_t tMillis = millis();

        if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
            sMillisOfLastVCCInfo = tMillis;
            /*
             * Check if voltage has changed (44 bytes)
             */
            if (readVINVoltage()) {
#    if !defined(NO_APPLICATON_INFO) // requires 1504 bytes program space
                Serial.print(F("VIN="));
                Serial.print(sVINVoltage);
                Serial.println(F("V"));
#    endif
            }
        }
    }
#  endif // defined(ESP32)
#endif // defined(MONITOR_VIN_VOLTAGE)
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _ROBOT_CAR_UTILS_HPP
