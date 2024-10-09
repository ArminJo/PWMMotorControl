/*
 * RobotCarUtils.hpp
 *
 *  Contains miscellaneous convenience utility functions for the robot cars.
 *
 *  Copyright (C) 2022-2024  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _ROBOT_CAR_UTILS_HPP
#define _ROBOT_CAR_UTILS_HPP

#include "RobotCarUtils.h"
#include "CarPWMMotorControl.h"
#include "digitalWriteFast.h"

/*
 * This file uses or prints the following macro definitions
 * DEBUG
 * USE_BLUE_DISPLAY_GUI
 * PRINT_VOLTAGE_PERIOD_MILLIS
 * VIN_ATTENUATED_INPUT_PIN
 * VOLTAGE_DIVIDER_DIVISOR
 * CAR_HAS_VIN_VOLTAGE_DIVIDER
 * FULL_BRIDGE_INPUT_MILLIVOLT
 * BASIC_CONFIG_NAME
 * CONFIG_NAME
 * USE_IR_REMOTE
 * IR_REMOTE_NAME
 * IR_RECEIVE_PIN
 * ENABLE_RTTTL_FOR_CAR
 * VIN_VOLTAGE_CORRECTION
 * ADC_INTERNAL_REFERENCE_MILLIVOLT
 * TIMEOUT_BEFORE_DEMO_MODE_STARTS_MILLIS
 * FOLLOWER_DISTANCE_MINIMUM_CENTIMETER
 * FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER
 * US_DISTANCE_SENSOR_ENABLE_PIN
 * DISTANCE_TONE_FEEDBACK_ENABLE_PIN
 * DISTANCE_FEEDBACK_MODE
 * USE_ADAFRUIT_MOTOR_SHIELD
 * CAR_HAS_4_MECANUM_WHEELS
 * FRONT_RIGHT_MOTOR_FORWARD_PIN
 *
 *
 */

#if !defined(DELAY_AND_RETURN_IF_STOP) // Is defined in IRCommandDispatcher.h or eventHandler.h as "if (delayMillisAndCheckForStop(aDurationMillis)) return"
#define DELAY_AND_RETURN_IF_STOP(aDurationMillis)   delay(aDurationMillis)
#endif
#if !defined(IS_STOP_REQUESTED)
#define IS_STOP_REQUESTED               false
#endif

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

#if defined(USE_BLUE_DISPLAY_GUI)
#include "RobotCarGui.h"
bool doCalibration = false;
#endif

#if !defined(PRINT_VOLTAGE_PERIOD_MILLIS)
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000
#endif

#if defined(VIN_ATTENUATED_INPUT_PIN)
#  if !defined(VOLTAGE_DIVIDER_DIVISOR)
#define VOLTAGE_DIVIDER_DIVISOR   11.0  // VIN/11 by 1MOhm to VIN and 100kOhm to ground.
#  endif
#include "ADCUtils.hpp"
uint16_t sLastVINRawSum; // Sum of NUMBER_OF_VIN_SAMPLES raw readings of ADC, used to determine if voltage has changed and must be displayed.
uint32_t sMillisOfLastVCCInfo;
#endif // defined(VIN_ATTENUATED_INPUT_PIN)
float sVINVoltage = FULL_BRIDGE_INPUT_MILLIVOLT / 1000; // Set default value for later use. Is used a parameter for getVoltageAdjustedSpeedPWM

//uint32_t sMillisOfLastAttention = 0;                            // millis() of last doAttention() or doWave()

void printConfigInfo(Print *aSerial) {
#if defined(BASIC_CONFIG_NAME)
    aSerial->print(F("Car configuration is: " BASIC_CONFIG_NAME));
#endif
#if defined(CONFIG_NAME)
    aSerial->print(F(CONFIG_NAME));
#endif
    aSerial->println();
}

void printConfigPinInfo(Print *aSerial, uint8_t aConfigPinNumber, const __FlashStringHelper *aConfigPinDescription) {
    aSerial->print(F("Pin "));
    aSerial->print(aConfigPinNumber);
    aSerial->print(F(" is"));
    bool tIsEnabled = digitalRead(aConfigPinNumber) == LOW;
    if (!tIsEnabled) {
        aSerial->print(F(" not"));
    }
    aSerial->print(F(" connected to ground, "));
    aSerial->print(aConfigPinDescription);
    aSerial->print(F(" is "));
    if (tIsEnabled) {
        aSerial->println(F("enabled"));
    } else {
        aSerial->println(F("disabled"));
    }
}

void printProgramOptions(Print *aSerial) {
    aSerial->println();
    aSerial->println(F("Settings:"));

#if !defined(USE_BLUE_DISPLAY_GUI)
    aSerial->print(F("DO_NOT_USE_IR_REMOTE:"));
#  if !defined(DO_NOT_USE_IR_REMOTE)
    aSerial->print(reinterpret_cast<const __FlashStringHelper*>(StringNot));
#  endif
    aSerial->println(reinterpret_cast<const __FlashStringHelper*>(StringDefined));
#endif

    aSerial->print(F("ENABLE_RTTTL_FOR_CAR:"));
#if !defined(ENABLE_RTTTL_FOR_CAR)
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

#if defined(USE_BLUE_DISPLAY_GUI)
#  if defined(ADC_UTILS_ARE_AVAILABLE)
    aSerial->print(
            F("If not powered by USB, run follower demo after " STR(TIMEOUT_BEFORE_DEMO_MODE_STARTS_MILLIS) " ms. USBpowered="));
    aSerial->println(isVCCUSBPowered());
#  endif
#endif

    aSerial->println(
            F(
                    "Keep distance between " STR(FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) " and " STR(FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) " cm. Scan for target up to " STR(FOLLOWER_TARGET_DISTANCE_TIMEOUT_CENTIMETER) " cm"));

#if defined(US_DISTANCE_SENSOR_ENABLE_PIN) // If this pin is connected to ground, use the US distance sensor instead of the IR distance sensor
    pinMode(US_DISTANCE_SENSOR_ENABLE_PIN, INPUT_PULLUP);
    printConfigPinInfo(aSerial, US_DISTANCE_SENSOR_ENABLE_PIN, F("US instead of IR distance input"));
#endif
#if defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && defined(DISTANCE_FEEDBACK_MODE) // If this pin is connected to ground, enable distance feedback
    pinMode(DISTANCE_TONE_FEEDBACK_ENABLE_PIN, INPUT_PULLUP);
    printConfigPinInfo(aSerial, DISTANCE_TONE_FEEDBACK_ENABLE_PIN, F("distance feedback"));
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
}

#if defined(VIN_ATTENUATED_INPUT_PIN)
/************************************
 * Functions to monitor VIN voltage
 ************************************/
/*
 * @return true, if voltage divider attached and VIN > 4.6 V
 */
bool isVINProvided() {
    pinModeFast(VIN_ATTENUATED_INPUT_PIN, OUTPUT);
    digitalWriteFast(VIN_ATTENUATED_INPUT_PIN, LOW); // discharge any charge at pin
    pinModeFast(VIN_ATTENUATED_INPUT_PIN, INPUT);
    readVINVoltage();
    bool tVINProvided = sVINVoltage > 4.6; // with USB, we have around 4.5 volt at VIN
#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.print(F("VIN voltage "));
    if (!tVINProvided) {
        Serial.print(F("not "));
    }
    Serial.println(F("provided"));
#endif
    return tVINProvided;
}

#if (MOTOR_PWM_PIN == 5) || (MOTOR_PWM_PIN == 6)
#define NUMBER_OF_VIN_SAMPLES   10 // Get 10 samples lasting 1030 us, which is almost the PWM period of 1024 us for Uno/Nano pin 5 and 6.
#else
#define NUMBER_OF_VIN_SAMPLES   20 // Get 20 samples lasting 2060 us, which is almost the PWM period of 2048 us.
#endif

/*
 * Read 10 samples covering a complete PWM period
 * @return true if voltage changed
 */
bool readVINVoltage() {
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
    uint16_t tVINRawSum = readADCChannelMultiSamplesWithReference(VIN_ATTENUATED_INPUT_CHANNEL, INTERNAL, NUMBER_OF_VIN_SAMPLES); // 10 samples
#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    checkAndWaitForReferenceAndChannelToSwitch(tOldADMUX & MASK_FOR_ADC_CHANNELS, tOldADMUX >> SHIFT_VALUE_FOR_REFERENCE);
#endif

#if defined(TRACE)
#  if defined(USE_BLUE_DISPLAY_GUI)
    BlueDisplay1.debug("VINRawSum=", tVINRawSum);
#  else
    Serial.print(F("VINRawSum="));
    Serial.println(tVINRawSum);
#  endif
#endif

// assume resistor network of 1MOhm / 100kOhm (divider by 11)
// tVIN * 0,01182795
#if defined(VIN_VOLTAGE_CORRECTION)
    // we have a diode (requires 0.8 to 0.9 volt) between LiIon and VIN
        sVINVoltage = (tVINRawSum * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / (1000.0 * NUMBER_OF_VIN_SAMPLES))) / 1023)) + VIN_VOLTAGE_CORRECTION;
#else
    /*
     * Here voltage correction is 0 volt.
     * tVINRawSum * 0.000591
     */
    sVINVoltage = tVINRawSum
            * ((VOLTAGE_DIVIDER_DIVISOR * (ADC_INTERNAL_REFERENCE_MILLIVOLT / 1000.0)) / (1023.0 * NUMBER_OF_VIN_SAMPLES));
#endif
    // resolution is about 5 mV and we display in a 10 mV resolution -> compare with (2 * NUMBER_OF_VIN_SAMPLES)
    if (abs(sLastVINRawSum - tVINRawSum) > (2 * NUMBER_OF_VIN_SAMPLES)) {
        sLastVINRawSum = tVINRawSum;
        return true;
    }
#endif // defined(ESP32)
    return false;
}

/*
 * Read multiple samples covering a complete PWM period, adjust DriveSpeedPWMFor2Volt and print old and new value
 */
void readVINVoltageAndAdjustDriveSpeedAndPrint() {
#if defined(ESP32)
    // On ESP32 currently not supported
#else
    readVINVoltage();
    /*
     * Adjust DriveSpeedPWMFor2Volt according to voltage
     */
#  if defined(USE_BLUE_DISPLAY_GUI)
    uint8_t tOldDriveSpeedPWM = RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt;
    RobotCar.setDriveSpeedPWMFor2Volt(sVINVoltage);
    PWMDcMotor::MotorPWMHasChanged = true; // to force a new display of motor voltage

    sprintf_P(sBDStringBuffer, PSTR("2 volt PWM %3d -> %3d"), tOldDriveSpeedPWM, RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
    BlueDisplay1.debug(sBDStringBuffer);
#  else
    Serial.print(F("2 volt PWM: "));
    Serial.print(RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
    Serial.print(F(" -> "));
    RobotCar.setDriveSpeedPWMFor2Volt(sVINVoltage);
    Serial.println(RobotCar.rightCarMotor.DriveSpeedPWMFor2Volt);
#  endif
#endif // defined(ESP32)
}

/*
 * Check VIN every 2 seconds (PRINT_VOLTAGE_PERIOD_MILLIS) and print if changed
 * Resolution is 10 mV
 * TODO implement check for low voltage
 */
void checkVinPeriodicallyAndPrintIfChanged() {
#if defined(ESP32)
    // On ESP32 currently not supported
#else

    uint32_t tMillis = millis();

    if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
        sMillisOfLastVCCInfo = tMillis;
        /*
         * Check if voltage has changed (44 bytes)
         */
        if (readVINVoltage()) {
#  if defined(ENABLE_SERIAL_OUTPUT) // BlueDisplay - requires 1504 bytes program space
            Serial.print(F("VIN="));
            Serial.print(sVINVoltage);
            Serial.println(F("V"));
#  endif
        }
    }
#endif // defined(ESP32)
}

/*
 * Start motors with DEFAULT_DRIVE_SPEED_PWM turning in place, get voltage after 400 ms and call setDriveSpeedPWMFor2Volt()
 */
void calibrateDriveSpeedPWMAndPrint() {
    // Turn right to get VIN value under load
#if defined(CAR_HAS_4_MECANUM_WHEELS)
    RobotCar.startRotate(42, TURN_IN_PLACE); // 42 since we just turn for 400 ms
#else
    RobotCar.setSpeedPWM(DEFAULT_DRIVE_SPEED_PWM, -DEFAULT_DRIVE_SPEED_PWM);
#endif
    delay(400); // cannot be terminated here, since car was not stopped
#if defined(ADC_UTILS_ARE_AVAILABLE)
    readVINVoltageAndAdjustDriveSpeedAndPrint();
#endif
    RobotCar.stop();

    DELAY_AND_RETURN_IF_STOP(400); // can be terminated here
    // Now turn back left
#if defined(CAR_HAS_4_MECANUM_WHEELS)
    RobotCar.startRotate(-42, TURN_IN_PLACE);
#else
    RobotCar.setSpeedPWM(-DEFAULT_DRIVE_SPEED_PWM, DEFAULT_DRIVE_SPEED_PWM);
#endif
    delay(400);
    RobotCar.stop();

#if defined(USE_BLUE_DISPLAY_GUI)
    isPWMCalibrated = true;
#endif
}
#endif // #if defined(VIN_ATTENUATED_INPUT_PIN)

/*
 * Not for 4WD cars with IMU or 2WD car with encoder motor.
 * IR dispatcher or BT control must be provided
 */
#if !defined(USE_MPU6050_IMU) && (defined(_IR_COMMAND_DISPATCHER_HPP) || defined(USE_BLUE_DISPLAY_GUI)) \
    && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))
/*
 * Start a 2 * 360 degree turn (4 * 360 for 2 wheel cars, which turn faster) to be sure to reach 360 degree.
 * The user should press the stop button when 360 degree was reached.
 * Then compute the internal MillimeterPer256Degreee value used for rotations.
 *
 * If a IR or BT stop command is received in the first 2.5 seconds after start of rotation, it is taken as abort command.
 * If no IR command is received, it is also taken as abort. This enable an easy check, i.e if calibration is correct, the 360 / 720 degree are reached.
 *
 * @return true if aborted or timeout
 */
bool calibrateRotation(turn_direction_t aTurnDirection) {
#if defined(USE_BLUE_DISPLAY_GUI)
#    if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
    BlueDisplay1.debug(F("Press stop button at 360 deg.")); // Message must be less than 32 bytes
#    else
    BlueDisplay1.debug(F("Press stop button at 720 deg.")); // Message must be less than 32 bytes
#    endif
    TouchButtonRobotCarStartStop.setValueAndDraw(BUTTON_AUTO_RED_GREEN_VALUE_FOR_GREEN);
#else // defined(USE_BLUE_DISPLAY_GUI)
#  if defined(ENABLE_SERIAL_OUTPUT) || defined(LOCAL_DEBUG) // BlueDisplay
// requires 1504 bytes program space
#    if defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS)
    Serial.println(F("Press stop button at 360 degree"));
#    else
    Serial.println(F("Press stop button at 720 degree"));
#    endif
#  endif
#endif // defined(USE_BLUE_DISPLAY_GUI)
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
    while (RobotCar.updateMotors()) { // wait for 360 / 720 degree, then updateMotors() will return false
#if defined(USE_BLUE_DISPLAY_GUI)
        if (!IS_STOP_REQUESTED) {
#else
        if (IRDispatcher.requestToStopReceived) {
#endif
            /*
             * RequestToStopReceived is set by ISR on receiving another blocking command, for example Stop or Reset.
             * Here we received a stop request from user. The stop command doStop() was NOT executed before, it is scheduled.
             */
#if defined(USE_ENCODER_MOTOR_CONTROL)
            uint16_t tNewMillimeterPer256Degree = RobotCar.rightCarMotor.getDistanceMillimeter();
#else
            unsigned long tMillisPer360Degree = millis() - tStartMillis;
#  if defined(LOCAL_DEBUG) && !defined(USE_BLUE_DISPLAY_GUI)
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
#if defined(USE_BLUE_DISPLAY_GUI)
            BlueDisplay1.debug("mm/256 deg=", tNewMillimeterPer256Degree);
#else
            Serial.print(F("MillimeterPer256Degreee="));
            Serial.println(tNewMillimeterPer256Degree);
#endif

            RobotCar.stop();
            return false;  // no abort or timeout
        }
#if defined(USE_BLUE_DISPLAY_GUI)
        checkAndHandleEvents();
#endif
    }
    return true; // timeout
}
#endif

///*
// * Move Servo if available, otherwise do a beep
// */
//void doAttention() {
//#if defined(INFO)
//    Serial.println(F("Start attention"));
//#endif
////    sCurrentlyRunningAction = ACTION_TYPE_ATTENTION;

//}

/*
 * Drive the car 2 times forward and and 2 times backward, each for a full wheel turn, to check the current values set for driving distance.
 */
void testDriveTwoTurnsBothDirections() {
#define NUMBER_OF_TEST_DRIVES       2
#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.print(F("Move the wheels 2x a full turn i.e. " STR(DEFAULT_CIRCUMFERENCE_MILLIMETER) " mm, both directions"));
#endif
    for (int i = 0; i < NUMBER_OF_TEST_DRIVES; ++i) {
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    DELAY_AND_RETURN_IF_STOP(2000);

    for (int i = 0; i < NUMBER_OF_TEST_DRIVES; ++i) {
        RobotCar.goDistanceMillimeter(-DEFAULT_CIRCUMFERENCE_MILLIMETER);
        DELAY_AND_RETURN_IF_STOP(500);
    }
}

/*
 * Drive the car for 2 times 1/8, wheel turn, then 1/ and 1/2 wheel turn, then a complete turn. First forward, then backward.
 * If distance driving formula and values are correct, this results in 4 full wheel turns ending at the start position.
 */
void testDriveTwoTurnsIn5PartsBothDirections() {
    uint8_t tDirection = DIRECTION_FORWARD;
#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.print(F("Move the wheels 2x 1/8 + 1/4 + 1/2 + 1 turn i.e. " STR(2 * DEFAULT_CIRCUMFERENCE_MILLIMETER)" mm, both directions"));
#endif
    for (int i = 0; i < 2; ++i) {
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 8, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 8, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 4, tDirection);
        DELAY_AND_RETURN_IF_STOP(2000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER / 2, tDirection);
        DELAY_AND_RETURN_IF_STOP(4000);
        RobotCar.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, tDirection);

        DELAY_AND_RETURN_IF_STOP(2000);
        tDirection = DIRECTION_BACKWARD;
    }
}

/*
 * Check the current EEPROM stored values for rotation.
 * - Rotate left forward by 9 times 10 degree -> 90 degree.
 * - Rotate right forward by 90 degree -> car has its initial direction but moved left forward.
 * - Do the same the other direction i.e. first right, then left.
 * - Do the same again but rotate in place.
 */
void testRotation() {
#define DEGREE_OF_TEST_ROTATION    10
#define NUMBER_OF_TEST_ROTATIONS    9 // to have 90 degree at 9 times 10 degree rotation
#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.println(F("Rotate forward 9 times for 10 degree"));
#endif
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(DEGREE_OF_TEST_ROTATION, TURN_FORWARD);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.println(F("Rotate back for 90 degree"));
#endif
    RobotCar.rotate(-(DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_FORWARD);
    DELAY_AND_RETURN_IF_STOP(3000);

#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.println(F("Rotate backwards"));
#endif
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(-DEGREE_OF_TEST_ROTATION, TURN_FORWARD);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    RobotCar.rotate((DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_FORWARD);
    DELAY_AND_RETURN_IF_STOP(2000);

#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.println(F("Rotate in place"));
#endif
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(DEGREE_OF_TEST_ROTATION, TURN_IN_PLACE);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    RobotCar.rotate(-(DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_IN_PLACE);
    DELAY_AND_RETURN_IF_STOP(2000);

#if defined(ENABLE_SERIAL_OUTPUT) // requires 1504 bytes program space
    Serial.println(F("Rotate in place backwards"));
#endif
    for (int i = 0; i < NUMBER_OF_TEST_ROTATIONS; ++i) {
        RobotCar.rotate(-DEGREE_OF_TEST_ROTATION, TURN_IN_PLACE);
        DELAY_AND_RETURN_IF_STOP(500);
    }
    // rotate back
    DELAY_AND_RETURN_IF_STOP(1000);
    RobotCar.rotate((DEGREE_OF_TEST_ROTATION * NUMBER_OF_TEST_ROTATIONS), TURN_IN_PLACE);
    DELAY_AND_RETURN_IF_STOP(2000);
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _ROBOT_CAR_UTILS_HPP
