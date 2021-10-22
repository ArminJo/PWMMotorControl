/*
 *  RobotCarFollowerSimple.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino and a dual full bridge (e.g. TB6612 or L298) for motor control.
 *
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
 *
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
//#define USE_ADAFRUIT_MOTOR_SHIELD  // Activate this if you use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
//#define USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD  // Activate this to force using of Adafruit library. Requires 694 bytes program memory.
#define VIN_2_LIPO                 // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO                 // Or if you use a Mosfet bridge, 1 LIPO (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define MOSFET_BRIDGE_USED  // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP  // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program space.
//#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN
#ifdef DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN
// Assume you switched to 2 LIPO batteries as motor supply if you also took the effort and mounted the servo head down
#define VIN_2_LIPO
#else
//#define VIN_2_LIPO // Activate it to use speed values for 7.4 Volt
#endif
#include "CarPWMMotorControl.hpp"

#include "PinDefinitionsAndMore.h"

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "HCSR04.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

#define DISTANCE_MINIMUM_CENTIMETER 20 // If measured distance is less than this value, go backwards
#define DISTANCE_MAXIMUM_CENTIMETER 30 // If measured distance is greater than this value, go forward

#define MAX_SPEED_PWM_FOLLOWER              (DEFAULT_DRIVE_SPEED_PWM * 2) // Max speed PWM value used for follower.

CarPWMMotorControl RobotCarPWMMotorControl;

Servo DistanceServo;

unsigned int getDistanceAndPlayTone();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    RobotCarPWMMotorControl.init();
#else
#  ifdef USE_ENCODER_MOTOR_CONTROL
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_INTERRUPT);
#  else
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#  endif
#endif

    RobotCarPWMMotorControl.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set compensation

    /*
     * Set US servo to forward position
     */
    DistanceServo.attach(PIN_DISTANCE_SERVO);
    DistanceServo.write(90);

    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    /*
     * Do not start immediately with driving
     */
    delay(5000);

    /*
     * Tone feedback for start of driving
     */
    tone(PIN_BUZZER, 2200, 100);
    delay(200);
    tone(PIN_BUZZER, 2200, 100);
}

void loop() {

    unsigned int tCentimeter = getDistanceAndPlayTone();
    unsigned int tSpeedPWM;

    if (tCentimeter >= DISTANCE_MAXIMUM_CENTIMETER) {
        /*
         * Target too far -> drive forward with speed proportional to the gap
         */
        tSpeedPWM = DEFAULT_START_SPEED_PWM + (tCentimeter - DISTANCE_MAXIMUM_CENTIMETER) * 2;
        if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
            tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
        }
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_FORWARD) {
            Serial.println(F("Go forward"));
        }
        Serial.print(F("SpeedPWM="));
        Serial.println(tSpeedPWM);

        RobotCarPWMMotorControl.setSpeedPWM(tSpeedPWM, DIRECTION_FORWARD);

    } else if (tCentimeter < DISTANCE_MINIMUM_CENTIMETER) {
        /*
         * Target too close -> drive backwards
         */
        tSpeedPWM = DEFAULT_START_SPEED_PWM + (DISTANCE_MINIMUM_CENTIMETER - tCentimeter) * 4;
        if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
            tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
        }
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_BACKWARD) {
            Serial.println(F("Go backward"));
        }
        Serial.print(F("SpeedPWM="));
        Serial.println(tSpeedPWM);

        RobotCarPWMMotorControl.setSpeedPWM(tSpeedPWM, DIRECTION_BACKWARD);

    } else {
        /*
         * Target is in the right distance -> stop once
         */
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
            Serial.println(F("Stop"));
            RobotCarPWMMotorControl.stop(MOTOR_RELEASE);
        }
    }

    delay(100);
}

unsigned int getDistanceAndPlayTone() {
    /*
     * Get distance
     */
    unsigned int tCentimeter = getUSDistanceAsCentiMeter();
    if (tCentimeter == 0) {
        noTone(PIN_BUZZER);
        Serial.print("Distance timeout ");
        tCentimeter = US_DISTANCE_DEFAULT_TIMEOUT_CENTIMETER;
    } else {
        Serial.print("Distance=");
        Serial.print(tCentimeter);
        Serial.print("cm. ");
        /*
         * Play tone
         */
        int tFrequency = map(tCentimeter, 0, 100, 110, 1760); // 4 octaves per meter
        tone(PIN_BUZZER, tFrequency);
    }
    return tCentimeter;
}
