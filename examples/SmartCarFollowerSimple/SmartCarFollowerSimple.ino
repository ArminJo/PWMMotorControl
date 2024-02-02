/*
 *  SmartCarFollowerSimple.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino and a dual full bridge (e.g. TB6612 or L298) for motor control.
 *  The car tries to hold a distance between 20 and 30 cm to a target. Only forward and back movement, no turn!
 *
 *  Copyright (C) 2020-2024  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
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
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION           // China set with TB6612 mosfet bridge + 4 AA.
//#define TBB6612_4WD_4AA_VIN_CONFIGURATION             // China set with TB6612 mosfet bridge + 4 AA + VIN voltage divider.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION            // China set with TB6612 mosfet bridge + 4 AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_4NIMH_BASIC_CONFIGURATION         // China set with TB6612 mosfet bridge + 4 NiMh.
//#define TBB6612_4WD_4NIMH_VIN_CONFIGURATION           // China set with TB6612 mosfet bridge + 4 NiMh + VIN voltage divider.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define L298_2WD_4AA_BASIC_CONFIGURATION              // China 2WD set with L298 bridge and Uno board with series diode for VIN + 4 AA batteries. DEFAULT.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION          // China 2WD set with L298 bridge and Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_2LI_ION_VIN_IR_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_2LI_ION_VIN_IR_IMU_CONFIGURATION     // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
//#define MECANUM_US_DISTANCE_CONFIGURATION             // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
//#define TRACE
//#define DEBUG
//#define INFO
#include "RobotCarConfigurations.h" // sets e.g. CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD
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
#include "HCSR04.hpp"
#include "ADCUtils.hpp"
#include "CarPWMMotorControl.hpp"
#include "RobotCarUtils.hpp"

Servo DistanceServo;

bool hasDistanceChanged(unsigned int aCentimeter);
void printDistance(unsigned int aCentimeter);

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
    printConfigInfo(&Serial);

    initRobotCarPWMMotorControl();
    RobotCar.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set left/right speed compensation

    /*
     * Set US servo to forward position and set US distance sensor pins
     */
    DistanceServo.attach(DISTANCE_SERVO_PIN);
    DistanceServo.write(90);
    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);

#if defined(ADC_UTILS_ARE_AVAILABLE)
    if (isVCCUSBPowered()) {
        /*
         * Avoid starting motors, if powered by USB
         * Signal USB powering by a double beep every 10 seconds
         */
        while (true) {
            tone(BUZZER_PIN, 2200, 100);
            delay(200);
            tone(BUZZER_PIN, 2200, 100);
            delay(10000); // wait
        }
    }
#endif

    /*
     * Tone feedback for end of boot
     */
    tone(BUZZER_PIN, 2200, 100);

    /*
     * Do not start immediately with driving
     */
    delay(2000);

    /*
     * Servo feedback for start of loop / driving
     */
    DistanceServo.write(135);
    delay(500);
    DistanceServo.write(45);
    delay(500);
    DistanceServo.write(90);
    delay(1000);

    Serial.println(F("Start loop"));
}

void loop() {
    /*
     * Get and print distance
     */
    unsigned int tCentimeter = getUSDistanceAsCentimeter(9000); // 9000 is timeout for 1.5 meter
    if (hasDistanceChanged(tCentimeter)) {
        /*
         * Distance has changed here
         */
        printDistance(tCentimeter);
        if (tCentimeter >= FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) {
            /*
             * Target too far -> drive forward
             */
            if (RobotCar.getCarDirection() != DIRECTION_FORWARD) {
                Serial.print(F(" -> go forward")); // print only once at direction change
            }
            RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_FORWARD);

            // tCentimeter == 0 is timeout
        } else if (tCentimeter > 0 && tCentimeter < FOLLOWER_DISTANCE_MINIMUM_CENTIMETER) {
            /*
             * Target too close -> drive backwards
             */
            if (RobotCar.getCarDirection() != DIRECTION_BACKWARD) {
                Serial.print(F(" -> go backward")); // print only once at direction change
            }
            RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_BACKWARD);

        } else {
            /*
             * Target is in the right distance -> stop
             */
            if (!RobotCar.isStopped()) {
                Serial.print(F(" -> stop car")); // stop only once
                RobotCar.stop(STOP_MODE_RELEASE);
            }
        }
        Serial.println();
    }

    delay(100); // Delay, to avoid receiving the US echo of last distance scan. 20 ms delay corresponds to an US echo from 3.43 m.
}

/*
 * @return true if distance has changed
 */
bool hasDistanceChanged(unsigned int aCentimeter) {
    static unsigned int sLastDistanceCentimeter;

    if (sLastDistanceCentimeter != aCentimeter) {
        sLastDistanceCentimeter = aCentimeter;
        return true;
    }
    return false;
}

/*
 * Print without a newline
 */
void printDistance(unsigned int aCentimeter) {
    if (aCentimeter == 0) {
        Serial.print("Distance timeout ");
    } else {
        Serial.print("Distance=");
        Serial.print(aCentimeter);
        Serial.print("cm");
    }
}
