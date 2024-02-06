/*
 *  LineFollower.cpp
 *
 *  Implements line follower driving for a 2 or 4 wheel car.
 *  A TCRT 5000 3-channel sensor is used.
 *
 * According to the 8 different states of the 3 sensor inputs, we perform the following actions:
 * 0 - All sensors are dark or not connected -> stop or go forward after sharp turn
 * 1 - Mid and right sensors are dark -> sharp right
 * 2 - Left and right sensors are dark -> panic stop, because this is unexpected
 * 3 - Only right sensor is dark -> right
 * 4 - Mid and left sensors are dark -> sharp left
 * 5 - Only mid sensor is dark -> forward
 * 6 - Only left sensor is dark -> left
 * 7 - All sensors are not dark -> stop or go backward after turn
 *
 *
 *  Copyright (C) 2022-2024  Armin Joachimsmeyer
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
//
//#define TRACE
//#define DEBUG
//#define INFO
//
// Here you can override the default settings for the sensor connections
//#define LINE_FOLLOWER_LEFT_SENSOR_PIN   4
//#define LINE_FOLLOWER_MID_SENSOR_PIN    5
//#define LINE_FOLLOWER_RIGHT_SENSOR_PIN  6

#include "RobotCarConfigurations.h" // sets CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD etc.
#include "RobotCarPinDefinitionsAndMore.h"

#if !defined(LINE_FOLLOWER_LEFT_SENSOR_PIN)
#error The LineFollower program requires that LINE_FOLLOWER_[LEFT,MID,RIGHT]_SENSOR_PIN are defined
#endif

/*
 * Speed compensation to enable driving straight ahead.
 * Is is usually not needed for a 4 wheel car.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

/*
 * Enable LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR, if the sensors are mounted at back of your car.
 * In this case, the directions must be switched, e.g. we must specify DIRECTION_BACKWARD to go forward.
 */
//#define LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR
#include "CarPWMMotorControl.hpp"
#include "RobotCarUtils.hpp"

#if defined(LINE_FOLLOWER_SENSORS_ARE_MOUNTED_AT_BACK_OF_CAR)
// Here, the sensors are mounted at the back, so we must switch forward and backward directions
#define LINE_FOLLOWER_DIRECTION_FORWARD     DIRECTION_BACKWARD
#define LINE_FOLLOWER_DIRECTION_BACKWARD    DIRECTION_FORWARD
#define LINE_FOLLOWER_TURN_FORWARD          TURN_BACKWARD
#define LINE_FOLLOWER_TURN_BACKWARD         TURN_FORWARD
#else
#define LINE_FOLLOWER_DIRECTION_FORWARD     DIRECTION_FORWARD
#define LINE_FOLLOWER_DIRECTION_BACKWARD    DIRECTION_BACKWARD
#define LINE_FOLLOWER_TURN_FORWARD          TURN_FORWARD
#define LINE_FOLLOWER_TURN_BACKWARD         TURN_BACKWARD
#endif

uint8_t sOldSensorState; // Used to detect changes of sensor state

/*
 * Start of line follower control program
 */
void setup() {
    Serial.begin(115200);

    // use INPUT_PULLUP, to avoid moving if sensor is not connected
    pinMode(LINE_FOLLOWER_LEFT_SENSOR_PIN, INPUT_PULLUP);
    pinMode(LINE_FOLLOWER_MID_SENSOR_PIN, INPUT_PULLUP);
    pinMode(LINE_FOLLOWER_RIGHT_SENSOR_PIN, INPUT_PULLUP);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Print useful information
    printConfigInfo(&Serial);
    Serial.println(F("Left sensor pin=" STR(LINE_FOLLOWER_LEFT_SENSOR_PIN)));
    Serial.println(F("Middle sensor pin=" STR(LINE_FOLLOWER_MID_SENSOR_PIN)));
    Serial.println(F("Right sensor pin=" STR(LINE_FOLLOWER_RIGHT_SENSOR_PIN)));

    /*
     * Init car motor control
     */
    initRobotCarPWMMotorControl();
    RobotCar.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set left/right speed compensation
    RobotCar.setStopMode(STOP_MODE_RELEASE); // to enable more smooth braking

    /*
     * Tone feedback for end of boot
     */
    tone(BUZZER_PIN, 2200, 100);

    /*
     * Do not start immediately with driving
     */
    delay(3000);
}

void loop() {
    /*
     * Get values from the TCRT 5000 3-channel sensor
     * input is high, if sensor sees dark
     */
    uint8_t tNewSensorState = 0;
    if (digitalRead(LINE_FOLLOWER_LEFT_SENSOR_PIN) == LOW) tNewSensorState = 1;   // set bit 0 if input is LOW
    if (digitalRead(LINE_FOLLOWER_MID_SENSOR_PIN) == LOW) tNewSensorState |= 2;   // set bit 1 if input is LOW
    if (digitalRead(LINE_FOLLOWER_RIGHT_SENSOR_PIN) == LOW) tNewSensorState |= 4; // set bit 2 if input is LOW

    /*
     * According to the 8 different states of the 3 sensor inputs, we perform the following actions:
     * 0 - All sensors are dark or not connected -> stop or go forward after sharp turn
     * 1 - Mid and right sensors are dark -> sharp right
     * 2 - Left and right sensors are dark -> panic stop, because this is unexpected
     * 3 - Only right sensor is dark -> right
     * 4 - Mid and left sensors are dark -> sharp left
     * 5 - Only mid sensor is dark -> forward
     * 6 - Only left sensor is dark -> left
     * 7 - All sensors are not dark -> stop or go backward after turn
     */

    /*
     * If sensor input does not change, we do not need to change movement!
     */
    if (sOldSensorState != tNewSensorState) {

        Serial.print(F("SensorState="));
        Serial.print(tNewSensorState);
        Serial.print(F(" -> "));

        switch (tNewSensorState) {
        case 0:
            // All sensors are dark or not connected -> stop or go forward after sharp turn
            if (sOldSensorState == 1 || sOldSensorState == 4) { // test for sharp turns
                RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, LINE_FOLLOWER_DIRECTION_FORWARD);
                Serial.println(F("Forward during sharp turn"));
            } else {
                RobotCar.stop();
                Serial.println(F("Stop"));
            }
            break;
        case 1:
            // Mid and right sensors are dark -> sharp right
            RobotCar.startRotate(-180, TURN_IN_PLACE);
            Serial.println(F("Turn sharp right"));
            break;
        case 2:
            // Left and right sensors are dark -> panic stop, because this is unexpected
            RobotCar.stop();
            Serial.println(F("panic stop"));
            break;
        case 3:
            // Only right sensor is dark -> right
            RobotCar.startRotate(-180, LINE_FOLLOWER_TURN_FORWARD);
            Serial.println(F("Turn right"));
            break;
        case 4:
            // Mid and left sensors are dark -> sharp left
            RobotCar.startRotate(180, TURN_IN_PLACE);
            Serial.println(F("Turn sharp left"));
            break;
        case 5:
            // Only mid sensor is dark -> forward
            RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, LINE_FOLLOWER_DIRECTION_FORWARD);
            Serial.println(F("Forward"));
            break;
        case 6:
            // Only left sensor is dark -> left
            RobotCar.startRotate(180, LINE_FOLLOWER_TURN_FORWARD);
            Serial.println(F("Turn left"));
            break;
        case 7:
            // All sensors are not dark -> stop or go backward after turn
            if (sOldSensorState == 6 || sOldSensorState == 3) {
                RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, LINE_FOLLOWER_DIRECTION_BACKWARD);
                Serial.println(F("Forward after turn"));
            } else {
                RobotCar.stop();
                Serial.println(F("Stop"));
            }
            break;
        }
        sOldSensorState = tNewSensorState;
    }
}
