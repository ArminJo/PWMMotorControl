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

void stopMotors();

#define DURATION_OF_ONE_MOVE_MILLIS    1500
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
    tone(BUZZER_PIN, 2200, 100);
    delay(3000); // Initial wait

    // Forward
    frontRightCarMotor.setDirection(DIRECTION_FORWARD);
    frontLeftCarMotor.setDirection(DIRECTION_FORWARD);
    backRightCarMotor.setDirection(DIRECTION_FORWARD);
    backLeftCarMotor.setDirection(DIRECTION_FORWARD);
    frontRightCarMotor.setSpeedPWM(100); // Required only for one motor, since the speed pin is the same for all motors
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Left
    frontRightCarMotor.setDirection(DIRECTION_FORWARD);
    frontLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    backRightCarMotor.setDirection(DIRECTION_BACKWARD);
    backLeftCarMotor.setDirection(DIRECTION_FORWARD);
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Backward
    frontRightCarMotor.setDirection(DIRECTION_BACKWARD);
    frontLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    backRightCarMotor.setDirection(DIRECTION_BACKWARD);
    backLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Right
    frontRightCarMotor.setDirection(DIRECTION_BACKWARD);
    frontLeftCarMotor.setDirection(DIRECTION_FORWARD);
    backRightCarMotor.setDirection(DIRECTION_FORWARD);
    backLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    stopMotors();
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Diagonal right forward
    frontRightCarMotor.setDirection(DIRECTION_STOP);
    frontLeftCarMotor.setDirection(DIRECTION_FORWARD);
    backRightCarMotor.setDirection(DIRECTION_FORWARD);
    backLeftCarMotor.setDirection(DIRECTION_STOP);
    frontRightCarMotor.setSpeedPWM(100);
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Diagonal left backward
    frontRightCarMotor.setDirection(DIRECTION_STOP);
    frontLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    backRightCarMotor.setDirection(DIRECTION_BACKWARD);
    backLeftCarMotor.setDirection(DIRECTION_STOP);
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Do not forget to stop motors :-)
    stopMotors();
    delay(DURATION_OF_ONE_MOVE_MILLIS);
}

void loop() {

    // Turn right
    frontRightCarMotor.setDirection(DIRECTION_FORWARD);
    frontLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    backRightCarMotor.setDirection(DIRECTION_FORWARD);
    backLeftCarMotor.setDirection(DIRECTION_BACKWARD);
    frontRightCarMotor.setSpeedPWM(100); // Required only for one motor, since the speed pin is the same for all motors
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    frontRightCarMotor.stop();
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    // Turn left
    frontRightCarMotor.setDirection(DIRECTION_BACKWARD);
    frontLeftCarMotor.setDirection(DIRECTION_FORWARD);
    backRightCarMotor.setDirection(DIRECTION_BACKWARD);
    backLeftCarMotor.setDirection(DIRECTION_FORWARD);
    frontRightCarMotor.setSpeedPWM(100); // Required only for one motor, since the speed pin is the same for all motors
    delay(DURATION_OF_ONE_MOVE_MILLIS);

    frontRightCarMotor.stop();
    delay(30000); // wait 1/2 minute
}

void stopMotors() {
    frontRightCarMotor.stop();
}
