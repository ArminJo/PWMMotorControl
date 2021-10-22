/*
 *  Start.cpp
 *  Example for controlling 2 motors without using the basic PWMDcMotor class
 *
 *  Copyright (C) 2020-2021  Armin Joachimsmeyer
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

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "CarPWMMotorControl.hpp"
 */
//#define USE_ENCODER_MOTOR_CONTROL  // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_MPU6050_IMU  // Activate this if you use GY-521 MPU6050 breakout board for precise turning and speed / distance movement. Connectors point to the rear.
//#define USE_ADAFRUIT_MOTOR_SHIELD  // Activate this if you use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
//#define USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD  // Activate this to force using of Adafruit library. Requires 694 bytes program memory.
#define VIN_2_LIPO                 // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO                 // Or if you use a Mosfet bridge, 1 LIPO (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define MOSFET_BRIDGE_USED  // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
#define DO_NOT_SUPPORT_RAMP  // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program space.
#include "PWMDcMotor.hpp"

#include "PinDefinitionsAndMore.h"

PWMDcMotor rightMotor;
PWMDcMotor leftMotor;

void setup() {
// initialize the digital pin as an output.
//    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // For Adafruit Motor Shield v2
    leftMotor.init(1);
    rightMotor.init(2);
#else
    leftMotor.init(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
    rightMotor.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN);
#endif

    Serial.print(F("Drive speed="));
    Serial.print(rightMotor.DriveSpeedPWM);
    Serial.println();

    delay(2000);
}

void loop() {
    static uint8_t sMotorDirection = DIRECTION_FORWARD;

    /*
     * Try start speed (from PWMDCMotor.h), at which the motor starts to move.
     */
    rightMotor.setSpeedPWM(DEFAULT_START_SPEED_PWM, sMotorDirection);
    delay(1000);               // wait for a second
    /*
     * Now set speed to the default drive speed (from PWMDCMotor.h), at which the motor moves for fixed distance driving.
     */
    rightMotor.setSpeedPWM(DEFAULT_DRIVE_SPEED_PWM, sMotorDirection);
    delay(1000);               // wait for a second
    /*
     * Stop motor
     */
    rightMotor.stop();
    delay(1000);               // wait for a second
    /*
     * Try to go a whole turn (22.0 cm for my wheels)
     */
    rightMotor.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, sMotorDirection);
    delay(2000);

    /*
     * Run left motor
     */
    leftMotor.setSpeedPWM(DEFAULT_START_SPEED_PWM, sMotorDirection);
    delay(1000);
    leftMotor.setSpeedPWM(DEFAULT_DRIVE_SPEED_PWM, sMotorDirection);
    delay(1000);
    leftMotor.stop();
    delay(1000);
    leftMotor.goDistanceMillimeter(DEFAULT_CIRCUMFERENCE_MILLIMETER, sMotorDirection);

    /*
     * switch direction
     */
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
    delay(3000);
}
