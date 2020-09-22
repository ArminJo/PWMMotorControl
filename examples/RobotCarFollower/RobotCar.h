/*
 * RobotCarMotorControl.h
 *
 *  Created on: 29.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef SRC_ROBOTCAR_H_
#define SRC_ROBOTCAR_H_

#include <Arduino.h>
#include <Servo.h>

//#define CAR_HAS_4_WHEELS

//#define USE_LAYOUT_FOR_NANO

// Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger.
//#define USE_US_SENSOR_1_PIN_MODE // Comment it out, if you use modified HC-SR04 modules or HY-SRF05 ones.

//#define CAR_HAS_IR_DISTANCE_SENSOR

//#define CAR_HAS_TOF_DISTANCE_SENSOR

//#define CAR_HAS_CAMERA

//#define CAR_HAS_LASER

/*
 * For pan tilt we have 2 servos in total
 */
//#define CAR_HAS_PAN_SERVO
//#define CAR_HAS_TILT_SERVO
//
/*
 * Plays melody after initial timeout has reached
 * Enables the Play Melody button
 */
// #define ENABLE_RTTTL
//
#include "CarMotorControl.h"
extern CarMotorControl RobotCarMotorControl;

/*
 * Pin usage
 * First the function of the nano board variant. For this variant the PWM is generated with analogWrite().
 */
/*
 * PIN  I/O Function
 *   2  I   Right motor encoder interrupt input
 *   3  I   Left motor encoder interrupt input
 *   4  O   Left motor fwd / NC for UNO board
 *   5  O   Left motor PWM / NC for UNO board
 *   6  O   Right motor PWM / NC for UNO board
 *   7  O   Left motor back / NC for UNO board
 *   8  O   Right motor fwd / NC for UNO board
 *   9  O   Servo US distance - Servo Nr. 2 on Adafruit Motor Shield
 *   10 O   Servo laser pan   - Servo Nr. 1 on Adafruit Motor Shield
 *   11 O   Servo laser tilt / Speaker for UNO board
 *   12 O   Right motor back / NC for UNO board
 *   13 O   Laser power
 *
 *   A0 O   US trigger (and echo in 1 pin US sensor mode)
 *   A1 I   IR distance (needs 1 pin US sensor mode) / US echo
 *   A2 I   VIN/11, 1MOhm to VIN, 100kOhm to ground.
 *   A3 IP  NC
 *   A4 SDA NC for Nano / I2C for UNO board motor shield
 *   A5 SCL NC for Nano / I2C for UNO board motor shield
 *   A6 O   Speaker for Nano board / not available on UNO board
 *   A7 O   Camera supply control
 */

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) // enable it in PWMDCMotor.h
/*
 * Pins for direct motor control with PWM and full bridge
 * Pins 9 + 10 are reserved for Servo
 * 2 + 3 are reserved for encoder input
 */
#define PIN_LEFT_MOTOR_FORWARD     12 // Pin 9 is already reserved for distance servo
#define PIN_LEFT_MOTOR_BACKWARD     8
#define PIN_LEFT_MOTOR_PWM          6 // Must be PWM capable

#define PIN_RIGHT_MOTOR_FORWARD     4
#define PIN_RIGHT_MOTOR_BACKWARD    7
#define PIN_RIGHT_MOTOR_PWM         5 // Must be PWM capable
#endif


/*
 * Servo pins
 */
#define PIN_DISTANCE_SERVO       9 // Servo Nr. 2 on Adafruit Motor Shield
#ifdef CAR_HAS_PAN_SERVO
#define PIN_PAN_SERVO           10 // Servo Nr. 1 on Adafruit Motor Shield
#endif
#ifdef CAR_HAS_TILT_SERVO
#define PIN_TILT_SERVO          11
#endif

#if defined(MONITOR_LIPO_VOLTAGE)
// Pin A0 for VCC monitoring - ADC channel 2
// Assume an attached resistor network of 100k / 10k from VCC to ground (divider by 11)
#define VIN_11TH_IN_CHANNEL      2 // = A2
#endif

/*
 * Pins for US HC-SR04 distance sensor
 */
#define PIN_TRIGGER_OUT         A0 // Connections on the Arduino Sensor Shield
#ifdef USE_US_SENSOR_1_PIN_MODE
#define PIN_IR_DISTANCE_SENSOR  A1 // Otherwise available as US echo pin
#else
#define PIN_ECHO_IN             A1 // used by Sharp IR distance sensor
#endif

#ifdef CAR_HAS_LASER
#define PIN_LASER_OUT           LED_BUILTIN
#endif

/*
 * Different pin layout for UNO with Adafruit motor shield and Nano (Nano hash full bridge) boards
 */
#ifdef USE_LAYOUT_FOR_NANO
/*
 * Nano Layout
 */
#  ifdef USE_ADAFRUIT_MOTOR_SHIELD
#error "Adafruit motor shield makes no sense for a Nano board!"
#  endif
#  ifdef CAR_HAS_CAMERA
#define PIN_CAMERA_SUPPLY_CONTROL A7 // Not available on UNO board
#  endif
#define PIN_SPEAKER               A6 // Not available on UNO board

#else
/*
 * UNO Layout
 */
#  ifdef CAR_HAS_CAMERA
#define PIN_CAMERA_SUPPLY_CONTROL  4
#  endif
#define PIN_SPEAKER               11
#endif

/**************************
 * End of pin definitions
 **************************/

/*
 * Timeouts for demo mode and inactivity remainder
 */
#define TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS 240000L // move Servo after 4 Minutes of inactivity
#define TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS 10000 // Start demo mode 10 seconds after boot up

//#define MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS 500
#define MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS 100

/*
 * Servo timing correction.
 * Values are for my SG90 servo. Servo is mounted head down, so values must be swapped!
 */
#define DISTANCE_SERVO_MIN_PULSE_WIDTH    (MIN_PULSE_WIDTH - 40) // Value for 180 degree
#define DISTANCE_SERVO_MAX_PULSE_WIDTH    (MAX_PULSE_WIDTH - 40) // Value for 0 degree, since servo is mounted head down.
#ifdef CAR_HAS_PAN_SERVO
extern Servo PanServo;
#endif
#ifdef CAR_HAS_TILT_SERVO
#define TILT_SERVO_MIN_VALUE     7 // since lower values will make an insane sound at my pan tilt device
extern Servo TiltServo;
#endif

#define DISTANCE_TIMEOUT_CM 100 // do not measure and process distances greater than 100 cm

/************************************************************************************
 * Definitions and declarations only used for GUI in RobotCarBlueDisplay.cpp example
 ************************************************************************************/
#define DISTANCE_TIMEOUT_COLOR COLOR_CYAN
#define DISTANCE_DISPLAY_PERIOD_MILLIS 500

#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

#if defined(MONITOR_LIPO_VOLTAGE)
#include "ADCUtils.h"

extern float sVINVoltage;
#if defined(MONITOR_LIPO_VOLTAGE)
#define VOLTAGE_LOW_THRESHOLD 6.9 // Formula: 2 * 3.5 volt - voltage loss: 25 mV GND + 45 mV VIN + 35 mV Battery holder internal
#define VOLTAGE_USB_THRESHOLD 5.5
#else
#endif
#define VOLTAGE_TOO_LOW_DELAY_ONLINE 3000 // display VIN every 500 ms for 4 seconds
#define VOLTAGE_TOO_LOW_DELAY_OFFLINE 1000 // wait for 1 seconds after double beep

void readVINVoltage();
#endif

void resetServos();
int doUserCollisionDetection();

#endif /* SRC_ROBOTCAR_H_ */

#pragma once
