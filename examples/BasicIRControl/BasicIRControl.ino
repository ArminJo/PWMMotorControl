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
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "PWMDcMotor.hpp"
 */
//#define USE_L298_BRIDGE                  // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define VIN_2_LI_ION                     // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                     // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define CAR_HAS_4_WHEELS
//#define FULL_BRIDGE_INPUT_MILLIVOLT 6000 // Default. For 4 x AA batteries (6 volt).
#define FULL_BRIDGE_INPUT_MILLIVOLT 4800 // For 4 x AA NIMH rechargeable batteries (4.8 volt).

#include "RobotCarPinDefinitionsAndMore.h"

#include "CarPWMMotorControl.h"
#include "CarPWMMotorControl.hpp"

/*
 * Choose remote
 * For available IR commands see RobotCarIRCommands.hpp and for the mapping to remote buttons see IRCommandMapping.h
 * https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/RobotCarIRCommands.hpp
 * https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
 */
//#define USE_KEYES_REMOTE_CLONE // With number pad above direction control, will be taken as default
//#define USE_KEYES_REMOTE
//#define USE_DVBT_STICK_REMOTE
#define USE_TINY_IR_RECEIVER // Supports only NEC protocol. Must be specified before including IRCommandDispatcher.hpp to define which IR library to use
#define INFO // Enable info just for IR dispatcher
#include "IRCommandDispatcher.h" // for RETURN_IF_STOP
#include "RobotCarIRCommands.hpp" // requires #include "Distance.hpp"
#include "IRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#include "IRCommandDispatcher.hpp"

#include "RobotCarUtils.hpp" // for printConfigInfo(), initRobotCarPWMMotorControl()

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
