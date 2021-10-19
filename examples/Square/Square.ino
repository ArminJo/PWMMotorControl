/*
 *  Square.cpp
 *  Example for driving a 50 cm square using CarPWMMotorControl class
 *
 *  Created on: 19.09.2020
 *  Copyright (C) 2020  Armin Joachimsmeyer
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

//#define USE_ADAFRUIT_MOTOR_SHIELD
#include "CarPWMMotorControl.hpp"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD)
/*
 * Pins for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 * 2 + 3 are reserved for encoder input
 */
#define PIN_RIGHT_MOTOR_FORWARD     4 // IN4 <- Label on the L298N board
#define PIN_RIGHT_MOTOR_BACKWARD    7 // IN3
#define PIN_RIGHT_MOTOR_PWM         5 // ENB - Must be PWM capable

#define PIN_LEFT_MOTOR_FORWARD      9 // IN1
#define PIN_LEFT_MOTOR_BACKWARD     8 // IN2
#define PIN_LEFT_MOTOR_PWM          6 // ENA - Must be PWM capable
#endif

CarPWMMotorControl RobotCarPWMMotorControl;
#define SIZE_OF_SQUARE_MILLIMETER  400

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // For Adafruit Motor Shield v2
    RobotCarPWMMotorControl.init();
#else
    RobotCarPWMMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);
#endif

    /*
     * You will need to change these values according to your motor, wheels and motor supply voltage.
     */
    RobotCarPWMMotorControl.setDriveSpeedAndSpeedCompensationPWM(DEFAULT_DRIVE_SPEED_PWM, SPEED_PWM_COMPENSATION_RIGHT); // Set compensation
#if defined(CAR_HAS_4_WHEELS)
    RobotCarPWMMotorControl.setFactorDegreeToMillimeter(FACTOR_DEGREE_TO_MILLIMETER_4WD_CAR_DEFAULT);
#else
    RobotCarPWMMotorControl.setFactorDegreeToMillimeter(FACTOR_DEGREE_TO_MILLIMETER_2WD_CAR_DEFAULT);
#endif
    // Print info
    PWMDcMotor::printSettings(&Serial);

    delay(5000);
}

void loop() {
    static uint8_t sMotorDirection = DIRECTION_FORWARD;

    for (int i = 0; i < 4; ++i) {
        /*
         * Try to go 40 cm with speed DEFAULT_DRIVE_SPEED.
         * You can adjust the speed as well as the distance to time factor above, to get better results.
         * If you have have slot type photo interrupters assembled, you require no factor if defining USE_ENCODER_MOTOR_CONTROL in PWMDCMotor.h
         */
        RobotCarPWMMotorControl.goDistanceMillimeter(SIZE_OF_SQUARE_MILLIMETER, sMotorDirection);
        delay(400);
        /*
         * Try to turn by 90 degree.
         */
        RobotCarPWMMotorControl.rotate(90, sMotorDirection, NULL, true);
        delay(400);
    }

    /*
     * Turn car around and switch direction
     */
    RobotCarPWMMotorControl.rotate(180, TURN_IN_PLACE, NULL, true);
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
    delay(2000);
}
