/*
 *  BasicMecanum.cpp
 *
 *  Implements basic mecanum car movements.
 *
 *  Copyright (C) 2023  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
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
 *
 */

#include <Arduino.h>

/*
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
#define MECANUM_BASIC_CONFIGURATION                   // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels
//#define MECANUM_US_DISTANCE_CONFIGURATION             // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
#include "RobotCarConfigurations.h" // sets e.g. CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"
#include "PWMDcMotor.hpp"

PWMDcMotor frontRightCarMotor;
PWMDcMotor frontLeftCarMotor;
PWMDcMotor backRightCarMotor;
PWMDcMotor backLeftCarMotor;

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Specify the pins to use for PWM and direction
    frontRightCarMotor.init(FRONT_RIGHT_MOTOR_FORWARD_PIN, FRONT_RIGHT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN);
    frontLeftCarMotor.init(FRONT_LEFT_MOTOR_FORWARD_PIN, FRONT_LEFT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN);
    backRightCarMotor.init(BACK_RIGHT_MOTOR_FORWARD_PIN, BACK_RIGHT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN);
    backLeftCarMotor.init(BACK_LEFT_MOTOR_FORWARD_PIN, BACK_LEFT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN);


    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);
    delay(1000); // Initial wait

    // Forward for 300 ms
    frontRightCarMotor.setSpeedPWMAndDirection(100);
    frontLeftCarMotor.setSpeedPWMAndDirection(100);
    backRightCarMotor.setSpeedPWMAndDirection(100);
    backLeftCarMotor.setSpeedPWMAndDirection(100);
    delay(300);

    // Borward for 300 ms
    frontRightCarMotor.setSpeedPWMAndDirection(-100);
    frontLeftCarMotor.setSpeedPWMAndDirection(-100);
    backRightCarMotor.setSpeedPWMAndDirection(-100);
    backLeftCarMotor.setSpeedPWMAndDirection(-100);
    delay(300);

    // Do not forget to stop motors :-)
    frontRightCarMotor.stop();
    frontLeftCarMotor.stop();
    backRightCarMotor.stop();
    backLeftCarMotor.stop();
}

void loop() {
    // Just to be sure that motors will stop after initial movement
    frontRightCarMotor.stop();
    frontLeftCarMotor.stop();
    backRightCarMotor.stop();
    backLeftCarMotor.stop();
}
