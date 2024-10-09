/*
 *  RobotCarPinDefinitionsAndMore.h
 *
 *  Contains motor pin definitions for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 *  Used for PWMMotorControl examples for various platforms.
 *
 *  Copyright (C) 2021-2024  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *  This file is part of PWMMotorControl https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  PWMMotorControl and Arduino-RobotCar are free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef ROBOT_CAR_PIN_DEFINITIONS_AND_MORE_H
#define ROBOT_CAR_PIN_DEFINITIONS_AND_MORE_H

/*
 * Pin mapping table for different platforms
 *
 * Platform           Left Motor                 Right Motor          Encoder
 *            Forward  Backward  PWM     Forward  Backward  PWM     Left  Right
 * ----------------------------------------------------------------------------
 * AVR (UNO)    9         8       6         4         7      5        3     2
 * Motor shield %         %       %         %         %      %        3     2
 * ESP32-CAM   14        15      13
 * Label for motor control connections on the L298N board
 *            IN1       IN2     ENA       IN4       IN3    ENB
 * Label for motor control connections on the TB6612 breakout board
 *           AIN1      AIN2    PWMA      BIN1      BIN2   PWMB
 *
 * Motor Control
 * PIN  I/O Function
 *   2  I   Right motor encoder interrupt input | Force use of US distance sensor if IR distance sensor is available | Line follower sensor left
 *   3  I   Left motor encoder interrupt input  | Distance tone feedback enable pin | Line follower sensor middle
 *   4  O   Right motor fwd     | Line follower sensor left
 *   5  O   Right motor PWM     | Line follower sensor middle
 *   6  O   Left motor PWM      | Line follower sensor right
 *   7  O   Right motor back    | Force use of US distance sensor enable pin
 *   8  O   Left motor fwd      | Distance tone feedback enable pin
 *   9  O/I Left motor back     | IR remote control signal in - on Adafruit Motor Shield marked as Servo Nr. 2
 *
 * PIN  I/O Function
 *  10  O   Servo for distance sensor - on Adafruit Motor Shield marked as Servo Nr. 1 | Line follower sensor right
 *  11  I/O IR remote control signal in | Servo for laser pan | Line follower sensor right
 *  12  O   Buzzer for Uno board | Servo for laser tilt
 *  13  O   Laser power
 *
 * PIN  I/O Function
 *  A0  O   US trigger (and echo in 1 pin US sensor mode) "URF 01 +" connector on the Arduino Sensor Shield
 *  A1  I   US echo on "URF 01 +" connector | IR distance if motor shield; requires no or 1 pin ultrasonic sensor if motor shield
 *  A2  I   VIN/11, 1MOhm to VIN, 100kOhm to ground - required for readVINVoltage(), camera supply control on NANO, IR in on Mecanum
 *  A3  I   IR distance | Buzzer on NANO
 *  A4  SDA I2C for motor shield | VL35L1X TOF sensor | MPU6050 accelerator and gyroscope
 *  A5  SCL I2C for motor shield | VL35L1X TOF sensor | MPU6050 accelerator and gyroscope
 *  A6  O   Only on NANO - IR distance
 *  A7  O   Only on NANO - VIN/11, 1MOhm to VIN, 100kOhm to ground
 */

#if defined(CAR_HAS_ENCODERS)
// This is the default and only required for direct use of EncoderMotor class
#define RIGHT_MOTOR_INTERRUPT       INT0 // on pin 2
#define LEFT_MOTOR_INTERRUPT        INT1 // on pin 3
#else
#  if !defined(US_DISTANCE_SENSOR_ENABLE_PIN)
#    if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)) && !defined(US_DISTANCE_SENSOR_ENABLE_PIN)
#define US_DISTANCE_SENSOR_ENABLE_PIN       2 // If this pin is connected to ground, use the US distance sensor instead of the IR distance sensor
#    endif
#    if !defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && !defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN)
#define DISTANCE_TONE_FEEDBACK_ENABLE_PIN   3 // If this pin is connected to ground, enable distance feedback
#   endif
# endif// !defined(US_DISTANCE_SENSOR_ENABLE_PIN)
#endif

#if !defined(CAR_HAS_4_MECANUM_WHEELS) && !defined(CAR_IS_ESP32_CAM_BASED)

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
// here pin 4 to 9 are available
#  if !defined(LINE_FOLLOWER_LEFT_SENSOR_PIN)
#define LINE_FOLLOWER_LEFT_SENSOR_PIN   4
#define LINE_FOLLOWER_MID_SENSOR_PIN    5
#define LINE_FOLLOWER_RIGHT_SENSOR_PIN  6
#  endif

#  if defined(CAR_HAS_ENCODERS)             // pin 2 and 3 are already occupied by encoder interrupts
#    if (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)) && !defined(US_DISTANCE_SENSOR_ENABLE_PIN)
#define US_DISTANCE_SENSOR_ENABLE_PIN   7   // If this pin is connected to ground, use the US distance sensor instead of the IR distance sensor
#    endif
#    if !defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && !defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN)
#define DISTANCE_TONE_FEEDBACK_ENABLE_PIN 8 // If this pin is connected to ground, enable distance feedback
#    endif
#  endif
#  if !defined(IR_RECEIVE_PIN)
#define IR_RECEIVE_PIN                    9   // on Adafruit Motor Shield marked as Servo Nr. 2
#  endif
#else // defined(USE_ADAFRUIT_MOTOR_SHIELD)
//2 + 3 are normally reserved for encoder input
#  if !defined(LINE_FOLLOWER_LEFT_SENSOR_PIN)
#define LINE_FOLLOWER_LEFT_SENSOR_PIN   2
#define LINE_FOLLOWER_MID_SENSOR_PIN    3
#define LINE_FOLLOWER_RIGHT_SENSOR_PIN 11
#endif

#define RIGHT_MOTOR_FORWARD_PIN     4 // IN4 <- Label on the L298N board
#define RIGHT_MOTOR_BACKWARD_PIN    7 // IN3
#  if !defined(LEFT_MOTOR_PWM_PIN)
#define RIGHT_MOTOR_PWM_PIN         5 // ENB - Must be PWM capable
#  endif

#define LEFT_MOTOR_FORWARD_PIN      9 // IN1
#define LEFT_MOTOR_BACKWARD_PIN     8 // IN2
#  if !defined(LEFT_MOTOR_PWM_PIN)
#define LEFT_MOTOR_PWM_PIN          6 // ENA - Must be PWM capable
#  endif

#  if !defined(IR_RECEIVE_PIN)
#define IR_RECEIVE_PIN             11
#  endif
#endif // defined(USE_ADAFRUIT_MOTOR_SHIELD)

//Servo pins
#define DISTANCE_SERVO_PIN         10 // Servo Nr. 2 on Adafruit Motor Shield - pin 10 can be controlled by Distance.hpp and LightweightServo library
#if defined(CAR_HAS_PAN_SERVO) && !defined(PAN_SERVO_PIN)
#define PAN_SERVO_PIN              11
#endif
#if defined(CAR_HAS_TILT_SERVO) && !defined(TILT_SERVO_PIN)
#define TILT_SERVO_PIN             12
#endif

// For HCSR04 ultrasonic distance sensor
#if !defined(TRIGGER_OUT_PIN)
#define TRIGGER_OUT_PIN            A0 // "URF 01 +" Connector on the Arduino Sensor Shield
#endif
#if !defined(US_SENSOR_SUPPORTS_1_PIN_MODE) && !defined(ECHO_IN_PIN)
#define ECHO_IN_PIN                A1
#endif

#if defined(CAR_HAS_LASER) && !defined(LASER_OUT_PIN)
#define LASER_OUT_PIN               LED_BUILTIN
#endif

#endif // !defined(CAR_HAS_4_MECANUM_WHEELS) && !defined(CAR_IS_ESP32_CAM_BASED)

#if defined(CAR_HAS_4_MECANUM_WHEELS)
//2 + 3 are reserved for encoder input
#define MOTOR_PWM_PIN                   5 // PWMB + PWMA <- Label on the TB6612 board

#define BACK_RIGHT_MOTOR_FORWARD_PIN    4 // BIN1 <- Label on the TB6612 board
#define BACK_RIGHT_MOTOR_BACKWARD_PIN   6 // BIN2
#define BACK_LEFT_MOTOR_FORWARD_PIN     7 // AIN1
#define BACK_LEFT_MOTOR_BACKWARD_PIN    8 // AIN2

#define FRONT_RIGHT_MOTOR_FORWARD_PIN   9 // BIN1 <- Label on the TB6612 board
#define FRONT_RIGHT_MOTOR_BACKWARD_PIN 10 // BIN2
#define FRONT_LEFT_MOTOR_FORWARD_PIN   11 // AIN1
#define FRONT_LEFT_MOTOR_BACKWARD_PIN  12 // AIN2

#if defined(CAR_HAS_PAN_SERVO) && !defined(PAN_SERVO_PIN)
#undef CAR_HAS_PAN_SERVO                  // pin 11 is already in use
#endif
#if defined(CAR_HAS_TILT_SERVO) && !defined(TILT_SERVO_PIN)
#undef CAR_HAS_TILT_SERVO                 // pin 12 is already in use
#endif

#if !defined(TRIGGER_OUT_PIN)
#define TRIGGER_OUT_PIN                A0 // can we see the trigger signal?
#endif
#if !defined(ECHO_IN_PIN)
#define ECHO_IN_PIN                    A1
#endif

#if !defined(IR_RECEIVE_PIN)
#define IR_RECEIVE_PIN                 A2
#endif

#define DISTANCE_SERVO_PIN             13
#if defined(CAR_HAS_LASER) && !defined(LASER_OUT_PIN)
#undef CAR_HAS_LASER                      // pin 13 is used by distance servo
#endif

// Temporarily definition for convenience
#define CAR_IS_NANO_BASED               // We have an Arduino Nano instead of an Uno resulting in a different pin layout.
#endif // defined(CAR_HAS_4_MECANUM_WHEELS)

#if defined(CAR_IS_NANO_BASED)
#if !defined(BUZZER_PIN)
#define BUZZER_PIN                     A3
#endif
#define IR_DISTANCE_SENSOR_PIN         A6 // Sharp IR distance sensor

// Pin A0 for VCC monitoring - ADC channel 7
// Assume an attached resistor network of 100k / 10k from VCC to ground (divider by 11)
#define VIN_ATTENUATED_INPUT_CHANNEL    7 // = A7
#define VIN_ATTENUATED_INPUT_PIN       A7

#  if defined(CAR_HAS_CAMERA)
#define CAMERA_SUPPLY_CONTROL_PIN      A2
#  endif

#elif defined(CAR_IS_ESP32_CAM_BASED)
#define RIGHT_MOTOR_FORWARD_PIN        17 // IN4 <- Label on the L298N board
#define RIGHT_MOTOR_BACKWARD_PIN       18 // IN3
#define RIGHT_MOTOR_PWM_PIN            16 // ENB - Must be PWM capable

// Suited for ESP32-CAM
#define LEFT_MOTOR_FORWARD_PIN         14 // IN1
#define LEFT_MOTOR_BACKWARD_PIN        15 // IN2
#define LEFT_MOTOR_PWM_PIN             13 // ENA - Must be PWM capable
#define ESP32_LEDC_MOTOR_CHANNEL        4 // leave first 4 channel for other purposes e.g. Servo and Light (channel 2)

// Not tested :-(
#define RIGHT_MOTOR_INTERRUPT          12
#define LEFT_MOTOR_INTERRUPT            2

#define TRIGGER_OUT_PIN                25
#define ECHO_IN_PIN                    26
#define DISTANCE_SERVO_PIN             27
#if !defined(BUZZER_PIN)
#define BUZZER_PIN                     23
#endif

#else // NANO_BASED
// Uno based
// Pin A0 for VCC monitoring - ADC channel 2
// Assume an attached resistor network of 100k / 10k from VCC to ground (divider by 11)
#define VIN_ATTENUATED_INPUT_CHANNEL    2 // = A2
#define VIN_ATTENUATED_INPUT_PIN       A2

#if !defined(BUZZER_PIN)
#define BUZZER_PIN                     12
#endif
#define IR_DISTANCE_SENSOR_PIN         A3 // Sharp IR distance sensor
#endif // CAR_IS_NANO_BASED

#endif /* ROBOT_CAR_PIN_DEFINITIONS_AND_MORE_H */
