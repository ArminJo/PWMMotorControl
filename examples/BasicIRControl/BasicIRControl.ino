/*
 *  BasicIRControl.cpp
 *
 *  Implements basic car control, like move and turn by an IR remote.
 *  Mapping between keys of any IR remote sending NEC protocol (all the cheap china ones) and car commands are done with a big switch.
 *  To support mapping, the received IR code is printed at the serial output.
 *
 *  Copyright (C) 2023  Armin Joachimsmeyer
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
 * If IR commands are received, you will see output like this:
 * A=0x0 C=0x1D - Received IR data
 */

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
#include "RobotCarConfigurations.h" // sets e.g. CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"
#include "PWMDcMotor.hpp"

#include "TinyIRReceiver.hpp"

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

    /*
     * Tone feedback for end of boot
     */
    tone(BUZZER_PIN, 2200, 100);

    initPCIInterruptForTinyReceiver(); // Enables the interrupt generation on change of IR input signal
    Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_RECEIVE_PIN)));
}

void loop() {
    /*
     * Check for IR commands and execute them.
     */
    if (TinyIRReceiverData.justWritten) {
        TinyIRReceiverData.justWritten = false;
        printTinyReceiverResultMinimal(&Serial);

        switch (TinyIRReceiverData.Command) {
        case 0x46:
            // Forward for 300 ms
            rightCarMotor.setSpeedPWMAndDirection(100);
            leftCarMotor.setSpeedPWMAndDirection(100);
            delay(300);
            break;
        case 0x47:
            // Backward for 300 ms
            rightCarMotor.setSpeedPWMAndDirection(-100);
            leftCarMotor.setSpeedPWMAndDirection(-100);
            delay(300);
            break;
        default:
            Serial.print(F("Unknown command 0x"));
            Serial.println(TinyIRReceiverData.Command, HEX);
            break;
        }
    }
    // Stop car after executing IR command
    rightCarMotor.stop();
    leftCarMotor.stop();
}
