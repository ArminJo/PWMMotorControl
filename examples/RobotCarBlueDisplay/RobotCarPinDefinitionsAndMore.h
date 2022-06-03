/*
 *  RobotCarPinDefinitionsAndMore.h
 *
 *  Contains motor pin definitions for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 *  Used for PWMMotorControl examples for various platforms.
 *
 *  Copyright (C) 2021-2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
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
 *   2  I   Right motor encoder interrupt input
 *   3  I   Left motor encoder interrupt input / fallback to US distance sensor if IR distance sensor available
 *   4  O   Right motor fwd
 *   5  O   Right motor PWM
 *   6  O   Left motor PWM
 *   7  O   Right motor back
 *   8  O   Left motor fwd
 *   9  O/I Left motor back / IR remote control signal in - on Adafruit Motor Shield marked as Servo Nr. 2
 *
 * PIN  I/O Function
 *  10  O   Servo US distance - on Adafruit Motor Shield marked as Servo Nr. 1
 *  11  I/O IR remote control signal in / Servo laser pan
 *  12  O   Buzzer for UNO board / Servo laser tilt
 *  13  O   Laser power
 *
 * PIN  I/O Function
 *  A0  O   US trigger (and echo in 1 pin US sensor mode) "URF 01 +" connector on the Arduino Sensor Shield
 *  A1  I   US echo on "URF 01 +" connector / IR distance if motor shield; requires no or 1 pin ultrasonic sensor if motor shield
 *  A2  I   VIN/11, 1MOhm to VIN, 100kOhm to ground - required for MONITOR_VIN_VOLTAGE, camera supply control on NANO
 *  A3  I   IR distance, Buzzer on NANO
 *  A4  SDA I2C for motor shield / VL35L1X TOF sensor / MPU6050 accelerator and gyroscope
 *  A5  SCL I2C for motor shield / VL35L1X TOF sensor / MPU6050 accelerator and gyroscope
 *  A6  O   Only on NANO - IR distance
 *  A7  O   Only on NANO - VIN/11, 1MOhm to VIN, 100kOhm to ground
 */

#if defined(USE_ENCODER_MOTOR_CONTROL)
// This is the default and only required for direct use of EncoderMotor class
#define RIGHT_MOTOR_INTERRUPT       INT0 // Pin 2
#define LEFT_MOTOR_INTERRUPT        INT1 // Pin 3
#else
#  if defined(CAR_HAS_US_DISTANCE_SENSOR) && defined(CAR_HAS_IR_DISTANCE_SENSOR)
#define US_DISTANCE_SENSOR_ENABLE_PIN   3 // If this pin is connected to ground, use the US distance sensor instead of the IR distance sensor
#  else
#define DISTANCE_TONE_FEEDBACK_ENABLE_PIN   3 // If this pin is connected to ground, enable distance feedback
#  endif
#endif // defined(USE_ENCODER_MOTOR_CONTROL)

#if !defined(CAR_HAS_4_MECANUM_WHEELS) && !defined(CAR_IS_ESP32_CAM_BASED)

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
#define IR_INPUT_PIN                9 // on Adafruit Motor Shield marked as Servo Nr. 2
#else
//2 + 3 are reserved for encoder input
#define RIGHT_MOTOR_FORWARD_PIN     4 // IN4 <- Label on the L298N board
#define RIGHT_MOTOR_BACKWARD_PIN    7 // IN3
#define RIGHT_MOTOR_PWM_PIN         5 // ENB - Must be PWM capable

#define LEFT_MOTOR_FORWARD_PIN      9 // IN1
#define LEFT_MOTOR_BACKWARD_PIN     8 // IN2
#define LEFT_MOTOR_PWM_PIN          6 // ENA - Must be PWM capable

#define IR_INPUT_PIN               11
#endif // defined(USE_ADAFRUIT_MOTOR_SHIELD)

//Servo pins
#define PIN_DISTANCE_SERVO         10 // Servo Nr. 2 on Adafruit Motor Shield - if pin 10 can be controlled by Distance.hpp and LightweightServo library
#if defined(CAR_HAS_PAN_SERVO)
#define PIN_PAN_SERVO              11
#endif
#if defined(CAR_HAS_TILT_SERVO)
#define PIN_TILT_SERVO             12
#endif

// For HCSR04 ultrasonic distance sensor
#define PIN_TRIGGER_OUT            A0 // "URF 01 +" Connector on the Arduino Sensor Shield
#if !defined(US_SENSOR_SUPPORTS_1_PIN_MODE)
#define PIN_ECHO_IN                A1
#endif

#if defined(CAR_HAS_LASER)
#define PIN_LASER_OUT               LED_BUILTIN
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

#if defined(CAR_HAS_PAN_SERVO)
#undef CAR_HAS_PAN_SERVO                  // pin 11 is already in use
#endif
#if defined(CAR_HAS_TILT_SERVO)
#undef CAR_HAS_TILT_SERVO                 // pin 12 is already in use
#endif

#define PIN_TRIGGER_OUT                A0 // can we see the trigger signal?
#define PIN_ECHO_IN                    A1

#define IR_INPUT_PIN                   A2

#define PIN_DISTANCE_SERVO             13
#if defined(CAR_HAS_LASER)
#undef CAR_HAS_LASER                      // pin 13 is used by distance servo
#endif

// Temporarily definition for convenience
#define CAR_IS_NANO_BASED               // We have an Arduino NANO instead of an UNO resulting in a different pin layout.
#endif // defined(CAR_HAS_4_MECANUM_WHEELS)

#if defined(CAR_IS_NANO_BASED)
#define PIN_BUZZER                     A3
#define PIN_IR_DISTANCE_SENSOR         A6 // Sharp IR distance sensor

// Pin A0 for VCC monitoring - ADC channel 7
// Assume an attached resistor network of 100k / 10k from VCC to ground (divider by 11)
#define VIN_ATTENUATED_INPUT_CHANNEL    7 // = A7
#define PIN_VIN_ATTENUATED_INPUT       A7

#  if defined(CAR_HAS_CAMERA)
#define PIN_CAMERA_SUPPLY_CONTROL      A2
#  endif

#elif defined(CAR_IS_ESP32_CAM_BASED)
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
#define PIN_DISTANCE_SERVO         27
#define PIN_BUZZER                 23

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN 2
#  if !defined(LED_BUILTIN) && !defined(CAR_IS_ESP32_CAM_BASED)
#define LED_BUILTIN PB1
#  endif
#define TONE_LEDC_CHANNEL           1  // Using channel 1 makes tone() independent of receiving timer -> No need to stop receiving timer.
void tone(uint8_t _pin, unsigned int frequency){
    ledcAttachPin(_pin, TONE_LEDC_CHANNEL);
    ledcWriteTone(TONE_LEDC_CHANNEL, frequency);
}
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration){
    ledcAttachPin(_pin, TONE_LEDC_CHANNEL);
    ledcWriteTone(TONE_LEDC_CHANNEL, frequency);
    delay(duration);
    ledcWriteTone(TONE_LEDC_CHANNEL, 0);
}
void noTone(uint8_t _pin){
    ledcWriteTone(TONE_LEDC_CHANNEL, 0);
}
#else // NANO_BASED
// UNO based
// Pin A0 for VCC monitoring - ADC channel 2
// Assume an attached resistor network of 100k / 10k from VCC to ground (divider by 11)
#define VIN_ATTENUATED_INPUT_CHANNEL    2 // = A2
#define PIN_VIN_ATTENUATED_INPUT       A2

#define PIN_BUZZER                     12
#define PIN_IR_DISTANCE_SENSOR         A3 // Sharp IR distance sensor
#endif // CAR_IS_NANO_BASED

#endif /* ROBOT_CAR_PIN_DEFINITIONS_AND_MORE_H */
