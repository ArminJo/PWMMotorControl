/*
 *  RobotCarFollowerSimple.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino and a dual full bridge (e.g. TB6612 or L298) for motor control.
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
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
// Assume you switched to 2 LIPO batteries as motor supply if you took the effort and mounted the servo head down
#define VIN_2_LIPO
#endif
#include "CarPWMMotorControl.h"
#include "CarPWMMotorControl.hpp"

#define CCAR_HAS_IR_DISTANCE_SENSOR
#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
#define US_DISTANCE_SENSOR_ENABLE_PIN   3 // If this pin is connected to ground, use the US distance sensor instead if the IR distance sensor
#endif
#include "RobotCarPinDefinitionsAndMore.h"

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "HCSR04.h"

//#define ONLY_TEST_DISTANCE_SENSOR

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

#define DISTANCE_MINIMUM_CENTIMETER 40 // If measured distance is less than this value, go backwards
#define DISTANCE_MAXIMUM_CENTIMETER 60 // If measured distance is greater than this value, go forward

#define MAX_SPEED_PWM_FOLLOWER              (DEFAULT_DRIVE_SPEED_PWM * 2) // Max speed PWM value used for follower.

CarPWMMotorControl RobotCarPWMMotorControl;

Servo DistanceServo;

unsigned int getDistanceAndPlayTone();
unsigned int getIRDistanceAsCentimeter();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);
#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    pinMode(US_DISTANCE_SENSOR_ENABLE_PIN, INPUT_PULLUP);
#endif

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
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);

    /*
     * Do not start immediately with driving
     */
    delay(5000);

    /*
     * Servo feedback for start of loop
     */
    DistanceServo.write(135);
    delay(500);
    DistanceServo.write(45);
    delay(500);
    DistanceServo.write(90);
}

void loop() {

    unsigned int tCentimeter = getDistanceAndPlayTone();

#if defined(ONLY_TEST_DISTANCE_SENSOR)
    Serial.println();
#else
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
            Serial.println(F(" -> go forward")); // print only once at direction change
        }
        Serial.print(F("SpeedPWM="));
        Serial.println(tSpeedPWM);

        RobotCarPWMMotorControl.setSpeedPWM(tSpeedPWM, DIRECTION_FORWARD);

    } else if (tCentimeter < DISTANCE_MINIMUM_CENTIMETER) {
        /*
         * Target too close -> drive backwards with speed proportional to the gap
         */
        tSpeedPWM = DEFAULT_START_SPEED_PWM + (DISTANCE_MINIMUM_CENTIMETER - tCentimeter) * 4;
        if (tSpeedPWM > MAX_SPEED_PWM_FOLLOWER) {
            tSpeedPWM = MAX_SPEED_PWM_FOLLOWER;
        }
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != DIRECTION_BACKWARD) {
            Serial.println(F(" -> go backward")); // print only once at direction change
        }
        Serial.print(F("SpeedPWM="));
        Serial.println(tSpeedPWM);

        RobotCarPWMMotorControl.setSpeedPWM(tSpeedPWM, DIRECTION_BACKWARD);

    } else {
        /*
         * Target is in the right distance -> stop
         */
        if (RobotCarPWMMotorControl.getCarDirectionOrBrakeMode() != MOTOR_RELEASE) {
            Serial.println(F("Now stop")); // stop only once
            RobotCarPWMMotorControl.stop(MOTOR_RELEASE);
        }
    }
#endif
    delay(100);
}

unsigned int getDistanceAndPlayTone() {
    /*
     * Get distance
     */
    unsigned int tCentimeter;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR)
    if (digitalRead(US_DISTANCE_SENSOR_ENABLE_PIN) == HIGH) {
        tCentimeter = getUSDistanceAsCentimeter();
    } else {
        tCentimeter = getIRDistanceAsCentimeter();
    }
#else
    tCentimeter = getUSDistanceAsCentimeter();
#endif
    if (tCentimeter == 0) {
        noTone(PIN_BUZZER);
        Serial.print("Distance timeout ");
        tCentimeter = US_DISTANCE_DEFAULT_TIMEOUT_CENTIMETER;
    } else {
        Serial.print("Distance=");
        Serial.print(tCentimeter);
        Serial.print("cm. ");
        /*
         * Play feedback tone proportional to measured distance
         */
        int tFrequency = map(tCentimeter, 0, 100, 110, 1760); // 4 octaves per meter
        tone(PIN_BUZZER, tFrequency);
    }
    return tCentimeter;
}

#if !defined(IR_SENSOR_TYPE_100550) && !defined(IR_SENSOR_TYPE_20150) && !defined(IR_SENSOR_TYPE_1080) && !defined(IR_SENSOR_TYPE_430)
#define IR_SENSOR_TYPE_1080
#endif
/*
 * The Sharp 1080 takes 39 ms for each measurement cycle
 */
unsigned int getIRDistanceAsCentimeter() {
    float tVolt = analogRead(PIN_IR_DISTANCE_SENSOR); // 100 us
    // tVolt * 0.004887585 = 5(V) for tVolt == 1023
#if defined(IR_SENSOR_TYPE_430) // 4 to 30 cm, 18 ms, GP2YA41SK0F
    return (12.08 * pow(tVolt * 0.004887585, -1.058)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
#elif defined(IR_SENSOR_TYPE_1080) // 10 to 80 cm, GP2Y0A21YK0F
    return (29.988 * pow(tVolt * 0.004887585, -1.173)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
//    return 4800/(analogRead(PIN_IR_DISTANCE_SENSOR)-20);    // see https://github.com/qub1750ul/Arduino_SharpIR/blob/master/src/SharpIR.cpp
#elif defined(IR_SENSOR_TYPE_20150) // 20 to 150 cm, 18 ms, GP2Y0A02YK0F
    // Model 20150 - Do not forget to add at least 100uF capacitor between the Vcc and GND connections on the sensor
    return (60.374 * pow(tVolt * 0.004887585, -1.16)) + 0.5; // see https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
#elif defined(IR_SENSOR_TYPE_100550) // GP2Y0A02YK0F // 100 to 550 cm, 18 ms, GP2Y0A710K0F
    return 1.0 / (((tVolt * 0.004887585 - 1.1250)) / 137.5);
#endif
}
