/*
 *  BasicDistance.cpp
 *
 *  Implements basic car movements controlled by HCSR04 ultrasonic distance measurement.
 *  The car tries to hold a distance between 20 and 30 cm to the target to follow.
 *
 *  Copyright (C) 2023-2024  Armin Joachimsmeyer
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
#define TBB6612_4WD_4AA_BASIC_CONFIGURATION           // China set with TB6612 mosfet bridge + 4 AA.
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
#include "RobotCarConfigurations.h" // sets e.g. CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"
#include "PWMDcMotor.hpp"
#include "HCSR04.hpp"
#include "ADCUtils.hpp" // to check if USB powered

PWMDcMotor rightCarMotor;
PWMDcMotor leftCarMotor;

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Specify the pins to use for PWM and direction
    rightCarMotor.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN);
    leftCarMotor.init(LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);

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
    delay(3000); // Initial wait
    tone(BUZZER_PIN, 2200, 100);
}

void loop() {
    unsigned int tDistanceCentimeter = getUSDistanceAsCentimeter();
    Serial.print(F("Distance = "));
    Serial.print(tDistanceCentimeter);
    Serial.println(F(" cm"));

    if (tDistanceCentimeter > 30) {
        // distance too high -> go forward (follow)
        rightCarMotor.setSpeedPWMAndDirection(100);
        leftCarMotor.setSpeedPWMAndDirection(100);
    } else if (tDistanceCentimeter < 20) {
        // distance too low -> go backward
        rightCarMotor.setSpeedPWMAndDirection(-100);
        leftCarMotor.setSpeedPWMAndDirection(-100);
    } else {
        // distance is acceptable -> stop and wait
        rightCarMotor.stop();
        leftCarMotor.stop();
    }

    /*
     * Minimal delay is around 10 to 20 ms to avoid receiving our ultrasonic signal sent in last loop! 10 ms is equivalent to a distance measurement of 1.71 m.
     */
    delay(200); // To avoid testing to fast
}
