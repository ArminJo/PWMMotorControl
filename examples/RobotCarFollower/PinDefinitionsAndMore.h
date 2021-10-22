/*
 *  PinDefinitionsAndMore.h
 *
 *  Contains motor pin definitions for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 *  Used for PWMMotorControl examples for various platforms.
 *
 *  Copyright (C) 2021  Armin Joachimsmeyer
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
 *  but WITHOUT ANY WARRANTY without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 * Pin mapping table for different platforms
 *
 * Platform           Left Motor                 Right Motor          Encoder
 *            Forward  Backward  PWM     Forward  Backward  PWM     Left  Right
 * ----------------------------------------------------------------------------
 * (Mega)AVR    9         8       6         4         7      5        3     2
 * ESP32-CAM   14        15      13
 * Label for motor control connections on the L298N board
 *            IN1       IN2     ENA       IN4       IN3    ENB
 * Label for motor control connections on the TB6612 breakout board
 *           AIN1      AIN2    PWMA      BIN1      BIN2   PWMB
 */

#if defined(__AVR__)
//2 + 3 are reserved for encoder input
#define RIGHT_MOTOR_FORWARD_PIN     4 // IN4 <- Label on the L298N board
#define RIGHT_MOTOR_BACKWARD_PIN    7 // IN3
#define RIGHT_MOTOR_PWM_PIN         5 // ENB - Must be PWM capable

#define LEFT_MOTOR_FORWARD_PIN      9 // IN1
#define LEFT_MOTOR_BACKWARD_PIN     8 // IN2
#define LEFT_MOTOR_PWM_PIN          6 // ENA - Must be PWM capable

#define RIGHT_MOTOR_INTERRUPT    INT0 // Pin 2
#define LEFT_MOTOR_INTERRUPT     INT1 // Pin 3

// For HCSR04 ultrasonic distance sensor
#define PIN_TRIGGER_OUT            A0 // Connections on the Arduino Sensor Shield
#define PIN_ECHO_IN                A1

#define PIN_DISTANCE_SERVO         10 // Servo Nr. 2 on Adafruit Motor Shield
#define PIN_BUZZER                 12

#elif defined(ESP32)
#define RIGHT_MOTOR_FORWARD_PIN    17 // IN4 <- Label on the L298N board
#define RIGHT_MOTOR_BACKWARD_PIN   18 // IN3
#define RIGHT_MOTOR_PWM_PIN        16 // ENB - Must be PWM capable

// Suited for ESP32-CAM
#define LEFT_MOTOR_FORWARD_PIN     14 // IN1
#define LEFT_MOTOR_BACKWARD_PIN    15 // IN2
#define LEFT_MOTOR_PWM_PIN         13 // ENA - Must be PWM capable

// Not tested :-(
#define RIGHT_MOTOR_INTERRUPT      12
#define LEFT_MOTOR_INTERRUPT        2

#define PIN_TRIGGER_OUT            25
#define PIN_ECHO_IN                26
#define PIN_DISTANCE_SERVO         27 // Servo Nr. 2 on Adafruit Motor Shield
#define PIN_BUZZER                 23
#endif

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN 2
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif
