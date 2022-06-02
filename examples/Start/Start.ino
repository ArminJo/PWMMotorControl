/*
 *  Start.cpp
 *  Example for controlling 2 motors using the basic PWMDcMotor class
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "PWMDcMotor.hpp"
 */
//#define USE_L298_BRIDGE                  // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define VIN_2_LI_ION                     // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                     // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT 6000 // Default. For 4 x AA batteries (6 volt).
//#define FULL_BRIDGE_INPUT_MILLIVOLT 4800 // For 4 x AA NIMH rechargeable batteries (4.8 volt).
//#define DEFAULT_MILLIMETER_PER_SECOND 220 // At DEFAULT_DRIVE_MILLIVOLT (2.0 V) motor supply.
//#define USE_ADAFRUIT_MOTOR_SHIELD        // Use Adafruit Motor Shield v2 with 2xTB6612 connected by I2C instead of external TB6612 or L298 breakout board.
#include "RobotCarPinDefinitionsAndMore.h"

#include "PWMDcMotor.hpp"

PWMDcMotor rightMotor;
PWMDcMotor leftMotor;

void setup() {
    Serial.begin(115200);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    // For Adafruit Motor Shield v2
    leftMotor.init(1);
    rightMotor.init(2);
#else
    leftMotor.init(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
    rightMotor.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN);
#endif

    Serial.print(F("Default drive speed PWM="));
    Serial.print(rightMotor.DriveSpeedPWM);
    Serial.println();

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);

    delay(2000);
}

void loop() {
    static uint8_t sMotorDirection = DIRECTION_FORWARD;

    /*
     * Try start speed (from PWMDCMotor.h), at which the motor starts to move.
     */
    Serial.print(F("Run right motor "));
    PWMDcMotor::printDirectionString(&Serial, sMotorDirection);
    Serial.print(F(" with PWM="));
    Serial.print(DEFAULT_START_SPEED_PWM);
    Serial.print(F(" -> "));
    Serial.print(PWMDcMotor::getMotorVoltageforPWMAndMillivolt(DEFAULT_START_SPEED_PWM, FULL_BRIDGE_INPUT_MILLIVOLT));
    Serial.print(F(" V at "));
    Serial.print(FULL_BRIDGE_INPUT_MILLIVOLT / 1000.0);
    Serial.println(F(" V power supply"));
    rightMotor.setSpeedPWMAndDirection(DEFAULT_START_SPEED_PWM, sMotorDirection);
    delay(1000);
    rightMotor.stop();
    delay(1000);

    /*
     * Now set speed to the default drive speed (from PWMDCMotor.h), at which the motor moves for fixed distance driving.
     */
    Serial.print(F("Run right motor "));
    PWMDcMotor::printDirectionString(&Serial, sMotorDirection);
    Serial.print(F(" with PWM="));
    Serial.print(DEFAULT_DRIVE_SPEED_PWM);
    Serial.print(F(" -> "));
    Serial.print(PWMDcMotor::getMotorVoltageforPWMAndMillivolt(DEFAULT_DRIVE_SPEED_PWM, FULL_BRIDGE_INPUT_MILLIVOLT));
    Serial.print(F(" V at "));
    Serial.print(FULL_BRIDGE_INPUT_MILLIVOLT / 1000.0);
    Serial.println(F(" V power supply"));
    rightMotor.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, sMotorDirection);
    delay(1000);
    rightMotor.stop();
    delay(1000);

    /*
     * Try to move the wheel whole turn (22.0 cm for my wheels)
     */
    Serial.print(F("Try to move the right wheel a full turn i.e. "));
    Serial.print(DEFAULT_CIRCUMFERENCE_MILLIMETER);
    Serial.print(F(" mm. Factor is "));
    Serial.print(rightMotor.MillisPerCentimeter);
    Serial.print(F(" milliseconds per cm or "));
    Serial.print(MILLIS_IN_ONE_SECOND * MILLIMETER_IN_ONE_CENTIMETER / rightMotor.MillisPerCentimeter);
    Serial.print(F(" millimeter per second -> "));
    unsigned long tMillis = millis();
    rightMotor.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, sMotorDirection);
    Serial.print(rightMotor.computedMillisOfMotorStopForDistance - tMillis);
    Serial.print(F(" ms including time for 1. cm of "));
    Serial.print(DEFAULT_MILLIS_FOR_FIRST_CENTIMETER);
    Serial.println(F(" ms."));
    delay(4000);

    /*
     * Run left motor
     */
    Serial.println(F("Now run the same with left motor"));
    leftMotor.setSpeedPWMAndDirection(DEFAULT_START_SPEED_PWM, sMotorDirection);
    delay(1000);
    leftMotor.stop();
    delay(1000);
    leftMotor.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, sMotorDirection);
    delay(1000);
    leftMotor.stop();
    delay(1000);
    leftMotor.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, sMotorDirection);

    /*
     * switch direction
     */
    Serial.println(F("Switch direction and wait"));
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
    delay(8000);
}
