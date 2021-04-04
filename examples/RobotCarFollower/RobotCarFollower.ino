/*
 *  RobotCarFollower.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino.
 *  To find the target to follow, a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo scans the area on demand. (not yet implemented)
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "CarPWMMotorControl.h"
#include "Servo.h"
#include "HCSR04.h"
#include "pitches.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT            0

#define DISTANCE_MINIMUM_CENTIMETER         20 // If measured distance is less than this value, go backwards
#define DISTANCE_MAXIMUM_CENTIMETER         30 // If measured distance is greater than this value, go forward
#define DISTANCE_DELTA_CENTIMETER           (DISTANCE_MAXIMUM_CENTIMETER - DISTANCE_MINIMUM_CENTIMETER)
#define DISTANCE_TARGET_SCAN_CENTIMETER     70 // search if target moved to side

#ifdef DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN
// Assume you switched to 2 LIPO batteries as motor supply if you also took the effort and mounted the servo head down
#define VIN_2_LIPO
#else
//#define VIN_2_LIPO // Activate it to use speed values for 7.4 Volt
#endif

#if defined(VIN_2_LIPO)
// values for 2xLIPO / 7.4 volt
#define START_SPEED_PWM                 55 // Speed PWM value at which car starts to move.
#define DRIVE_SPEED_PWM                 90 // Speed PWM value used for going fixed distance.
#define MAX_SPEED_PWM_FOLLOWER         135 // Max speed PWM value used for follower.
#else
// Values for 4xAA / 6.0 volt
#define START_SPEED_PWM                140 // Speed PWM value at which car starts to move.
#define DRIVE_SPEED_PWM                220 // Speed PWM value used for going fixed distance.
#define MAX_SPEED_PWM_FOLLOWER         255 // Max speed PWM value used for follower.
#endif

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) // enable it in PWMDCMotor.h
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

#ifdef USE_ENCODER_MOTOR_CONTROL
#define RIGHT_MOTOR_INTERRUPT    INT0 // Pin 2
#define LEFT_MOTOR_INTERRUPT     INT1 // Pin 3
#endif

#define PIN_DISTANCE_SERVO         10 // Servo Nr. 2 on Adafruit Motor Shield

#define PIN_BUZZER                 12

#define PIN_TRIGGER_OUT            A0 // Connections on the Arduino Sensor Shield
#define PIN_ECHO_IN                A1

//#define PLOTTER_OUTPUT // Activate this, if you want to see the result of the US distance sensor and resulting speed in Arduino plotter

//Car Control
CarPWMMotorControl RobotCarPWMMotorControl;
Servo DistanceServo;

unsigned int getDistanceAndPlayTone();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
#ifdef PLOTTER_OUTPUT
    Serial.println(F("Distance[cm] Speed"));
#else
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
#endif

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    RobotCarPWMMotorControl.init();
#else
#  ifdef USE_ENCODER_MOTOR_CONTROL
    RobotCarPWMMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, RIGHT_MOTOR_INTERRUPT, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM, LEFT_MOTOR_INTERRUPT);
#  else
    RobotCarPWMMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);
#  endif
#endif

    /*
     * You will need to change these values according to your motor, wheels and motor supply voltage.
     */
    RobotCarPWMMotorControl.setValuesForFixedDistanceDriving(DEFAULT_START_SPEED_PWM, DEFAULT_DRIVE_SPEED_PWM, SPEED_PWM_COMPENSATION_RIGHT); // Set compensation
#if ! defined(USE_ENCODER_MOTOR_CONTROL)
    // set factor for converting distance to drive time
    RobotCarPWMMotorControl.setMillimeterPerSecondForFixedDistanceDriving(DEFAULT_MILLIMETER_PER_SECOND);
#endif

    DistanceServo.attach(PIN_DISTANCE_SERVO);
    DistanceServo.write(90);
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    tone(PIN_BUZZER, 2200, 100);
    delay(200);
    tone(PIN_BUZZER, 2200, 100);

    /*
     * Do not start immediately with driving
     */
#ifdef USE_MPU6050_IMU
    /*
     * Wait after pressing the reset button, or attaching the power
     * and then take offset values for 1/2 second
     */
    delay(1000);
    tone(PIN_BUZZER, 2200, 50);
    delay(100);
    RobotCarPWMMotorControl.initIMU();
    RobotCarPWMMotorControl.printIMUOffsets(&Serial);
    tone(PIN_BUZZER, 2200, 50);
#endif
    delay(4000);

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

    if (tCentimeter == 0 || tCentimeter > DISTANCE_TARGET_SCAN_CENTIMETER) {
        /*
         * Distance too high or timeout / target not found -> search for target at different directions and turn if found
         */
        noTone(PIN_BUZZER);
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
#ifndef PLOTTER_OUTPUT
            Serial.print(F("Stop and search"));
#endif
            RobotCarPWMMotorControl.stop(MOTOR_RELEASE);
        }

        // check additionally at 70 and 110 degree for vanished target
        for (uint8_t i = 70; i <= 110; i += 40) {
            DistanceServo.write(i);
            delay(200); // To let the servo reach its position
            unsigned int tCentimeterCheck = getUSDistanceAsCentiMeter();
            if (tCentimeterCheck != 0 && tCentimeterCheck < DISTANCE_TARGET_SCAN_CENTIMETER) {
                /*
                 * Target found -> turn and proceed
                 */
#ifdef DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN
                RobotCarPWMMotorControl.rotate(90 - i);
#else
                RobotCarPWMMotorControl.rotate(i - 90);
#endif
                DistanceServo.write(90);
                break;
            }
        }
        // reset servo position
        DistanceServo.write(90);
        delay(100); // Additional delay to let the servo reach its position

    } else if (tCentimeter > DISTANCE_MAXIMUM_CENTIMETER) {
        /*
         * Target too far -> drive forward with speed proportional to the gap
         */
        tSpeedPWM = START_SPEED_PWM + (tCentimeter - DISTANCE_MAXIMUM_CENTIMETER) * 2;
        if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
            tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
        }
#ifdef PLOTTER_OUTPUT
        Serial.print(tSpeedPWM);
#else
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_FORWARD) {
            Serial.println(F("Go forward"));
        }
        Serial.print(F("SpeedPWM="));
        Serial.print(tSpeedPWM);
#endif
#ifdef USE_ENCODER_MOTOR_CONTROL
        RobotCarPWMMotorControl.startGoDistanceCentimeter(tSpeedPWM, (tCentimeter - DISTANCE_MAXIMUM_CENTIMETER) + DISTANCE_DELTA_CENTIMETER / 2,
                DIRECTION_FORWARD);
#else
        RobotCarPWMMotorControl.setSpeedPWMCompensated(tSpeedPWM, DIRECTION_FORWARD);
#endif

    } else if (tCentimeter < DISTANCE_MINIMUM_CENTIMETER) {
        /*
         * Target too close -> drive backwards
         */
        tSpeedPWM = START_SPEED_PWM + (DISTANCE_MINIMUM_CENTIMETER - tCentimeter) * 4;
        if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
            tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
        }
#ifdef PLOTTER_OUTPUT
        Serial.print(tSpeedPWM);
#else
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_BACKWARD) {
            Serial.println(F("Go backward"));
        }
        Serial.print(F("SpeedPWM="));
        Serial.print(tSpeedPWM);
#endif
#ifdef USE_ENCODER_MOTOR_CONTROL
        RobotCarPWMMotorControl.startGoDistanceCentimeter(tSpeedPWM, (DISTANCE_MINIMUM_CENTIMETER - tCentimeter) + DISTANCE_DELTA_CENTIMETER / 2,
                DIRECTION_BACKWARD);
#else
        RobotCarPWMMotorControl.setSpeedPWMCompensated(tSpeedPWM, DIRECTION_BACKWARD);
#endif
    } else {
        /*
         * Target is in the right distance -> stop once
         */
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
#ifndef PLOTTER_OUTPUT
            Serial.print(F("Stop"));
#endif
            RobotCarPWMMotorControl.stop(MOTOR_RELEASE);
        }
    }

    Serial.println();
#ifdef USE_ENCODER_MOTOR_CONTROL
    RobotCarPWMMotorControl.delayAndUpdateMotors(1000);
#else
    delay(100);
#endif
}

unsigned int getDistanceAndPlayTone() {
    /*
     * Get distance
     */
    unsigned int tCentimeter = getUSDistanceAsCentiMeter();
    if (tCentimeter == 0) {
        // Timeout, just try again, it may be a dropout
        delay(20);
        tCentimeter = getUSDistanceAsCentiMeter();
    }
#ifdef PLOTTER_OUTPUT
    Serial.print(tCentimeter);
    Serial.print(' ');
#else
    Serial.print("Distance=");
    Serial.print(tCentimeter);
    Serial.print("cm. ");
#endif
    /*
     * Play tone
     */
    if (tCentimeter > 0) {
        uint8_t tIndex = map(tCentimeter, 0, 100, 0, ARRAY_SIZE_NOTE_C5_TO_C7_PENTATONIC - 1);
        tone(PIN_BUZZER, NoteC5ToC7Pentatonic[tIndex]);
    } else {
        noTone(PIN_BUZZER);
    }
    return tCentimeter;
}
