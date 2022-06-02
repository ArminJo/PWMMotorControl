/*
 *  SmartCarFollowerSimple.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino and a dual full bridge (e.g. TB6612 or L298) for motor control.
 *  The car tries to hold a distance between 20 and 30 cm to an obstacle. Only forward and back movement, no turn!
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

/*
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 4AA.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION   // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION    // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define L298_2WD_4AA_BASIC_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_4WD_4AA_BASIC_CONFIGURATION          // China set with L298 + 4AA.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_VIN_IR_DISTANCE_CONFIGURATION    // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_VIN_IR_IMU_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
//#define MECANUM_DISTANCE_CONFIGURATION            // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
//#define TRACE
#include "RobotCarConfigurations.h" // sets e.g. USE_ENCODER_MOTOR_CONTROL, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

/*
 * Values to configure the behavior of the follower
 */
#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER 20 // If measured distance is less than this value, go backwards
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER 30 // If measured distance is greater than this value, go forward

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "HCSR04.h"
#include "CarPWMMotorControl.hpp"

Servo DistanceServo;

void printConfigInfo();
bool printDistanceIfChanged(unsigned int aCentimeter);
void initRobotCarPWMMotorControl();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
    printConfigInfo();

    initRobotCarPWMMotorControl();
    RobotCar.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set left/right speed compensation

    /*
     * Set US servo to forward position and set US distance sensor pins
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
     * Servo feedback for start of loop / driving
     */
    DistanceServo.write(135);
    delay(500);
    DistanceServo.write(45);
    delay(500);
    DistanceServo.write(90);
}

void loop() {
    /*
     * Get and print distance
     */
    unsigned int tCentimeter = getUSDistanceAsCentimeter(9000); // 9000 is timeout for 1.5 meter
    if (printDistanceIfChanged(tCentimeter)) {
        /*
         * Distance has changed here
         */
        if (tCentimeter >= FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
            /*
             * Target too far -> drive forward
             */
            if (RobotCar.getCarDirection() != DIRECTION_FORWARD) {
                Serial.print(F(" -> go forward")); // print only once at direction change
            }
            RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_FORWARD);
            tone(PIN_BUZZER, 1500);

            // tCentimeter == 0 is timeout
        } else if (tCentimeter > 0 && tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
            /*
             * Target too close -> drive backwards
             */
            if (RobotCar.getCarDirection() != DIRECTION_BACKWARD) {
                Serial.print(F(" -> go backward")); // print only once at direction change
            }
            RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_BACKWARD);
            tone(PIN_BUZZER, 666);

        } else {
            /*
             * Target is in the right distance -> stop
             */
            if (!RobotCar.isStopped()) {
                Serial.print(F(" -> now stop")); // stop only once
                RobotCar.stop(STOP_MODE_RELEASE);
            }
            if (tCentimeter == 0) {
                // distance timeout here
                noTone(PIN_BUZZER);
            } else {
                tone(PIN_BUZZER, 1000);
            }
        }
        Serial.println();
    }

    delay(100);
}

/*
 * Call RobotCar.init() with different sets of parameters
 */
void initRobotCarPWMMotorControl() {
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    RobotCar.init();
#elif defined(CAR_HAS_4_MECANUM_WHEELS)
    RobotCar.init(FRONT_RIGHT_MOTOR_FORWARD_PIN, FRONT_RIGHT_MOTOR_BACKWARD_PIN, MOTOR_PWM_PIN,
    FRONT_LEFT_MOTOR_FORWARD_PIN, FRONT_LEFT_MOTOR_BACKWARD_PIN, BACK_RIGHT_MOTOR_FORWARD_PIN, BACK_RIGHT_MOTOR_BACKWARD_PIN,
    BACK_LEFT_MOTOR_FORWARD_PIN, BACK_LEFT_MOTOR_BACKWARD_PIN);
#else
    RobotCar.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#endif
}

void printConfigInfo() {
#if defined(BASIC_CONFIG_NAME)
    Serial.print(F("Car configuration is: " BASIC_CONFIG_NAME));
#endif
#if defined(CONFIG_NAME)
    Serial.print(F(CONFIG_NAME));
#endif
    Serial.println();
}

/*
 * Print without a newline
 * @return true if distance has changed and was printed
 */
bool printDistanceIfChanged(unsigned int aCentimeter) {
    static unsigned int sLastPrintedDistanceCentimeter;

    if (sLastPrintedDistanceCentimeter != aCentimeter) {
        sLastPrintedDistanceCentimeter = aCentimeter;
        if (aCentimeter == 0) {
            Serial.print("Distance timeout ");
        } else {
            Serial.print("Distance=");
            Serial.print(aCentimeter);
            Serial.print("cm");
        }
        return true;
    }
    return false;
}
