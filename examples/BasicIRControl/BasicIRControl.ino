/*
 *  BasicIRControl.cpp
 *
 *  Implements basic car control  like move and turn by an IR remote.
 *  Mapping between keys of any IR remote sending NEC protocol (all the cheap china ones) and car commands can be done in IRCommandMapping.h.
 *  To support mapping, the received IR code is printed at the serial output if `INFO` is defined.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
//#define DEBUG
//#define INFO

/*
 * defining INFO enables output like this:
 * A=0x0 C=0x1D - Received IR data
 * Run non blocking command: default speed - Called car command
 * 5 CompensatedSpeedPWM=0 DriveSpeedPWM=106 SpeedPWMCompensation=0 CurrentDirection=S - Output of car command
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
#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
#define USE_SOFT_I2C_MASTER             // Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire

#include "RobotCarConfigurations.h" // sets e.g. USE_ENCODER_MOTOR_CONTROL, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"

#include "CarPWMMotorControl.hpp"

/*
 * Choose remote
 * For available IR commands see RobotCarIRCommands.hpp and for the mapping to remote buttons see RobotCarIRCommandMapping.h
 * https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/RobotCarIRCommands.hpp
 * https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/RobotCarIRCommandMapping.h
 */
//#define USE_KEYES_REMOTE_CLONE // With number pad above direction control, will be taken as default
//#define USE_KEYES_REMOTE
//#define USE_DVBT_STICK_REMOTE
#define USE_TINY_IR_RECEIVER // Supports only NEC protocol. Must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#define INFO // Enable info just for IR dispatcher
#include "RobotCarIRCommands.hpp" // requires #include "Distance.hpp"
#include "RobotCarIRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"

#include "RobotCarUtils.hpp" // Requires IR_REMOTE_NAME from IRCommandMappingRobotCar.h. For printConfigInfo(), initRobotCarPWMMotorControl()

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
    printConfigInfo(&Serial);

    initRobotCarPWMMotorControl();
    RobotCar.setSpeedPWMCompensation(0); // If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);

    // For available IR commands see IRCommandMapping.h https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
    IRDispatcher.init();
    Serial.print(F("Listening to IR remote of type "));
    Serial.print(IR_REMOTE_NAME);
    Serial.println(F(" at pin " STR(IR_INPUT_PIN)));
}

void loop() {
    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested action
     */
    IRDispatcher.checkAndRunSuspendedBlockingCommands();
    RobotCar.updateMotors();
}
