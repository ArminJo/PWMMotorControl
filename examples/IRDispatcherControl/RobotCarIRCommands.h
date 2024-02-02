/*
 *  RobotCarIRCommands.h
 *
 *  Contains all commands required for IRCommandMapping.h / accessible by IR remote.
 *
 *  Copyright (C) 2024  Armin Joachimsmeyer
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

#ifndef _ROBOT_CAR_IR_COMMANDS_H
#define _ROBOT_CAR_IR_COMMANDS_H

/*
 * Basic IR functions
 */
void doStop();
void doReset();
void goForward();
void goBackward();
void turnLeft();
void turnRight();
void doDefaultSpeed();
#define SPEED_PWM_CHANGE_VALUE  ((MAX_SPEED_PWM + 1) / 16) // 16
void doIncreaseSpeed();
void doDecreaseSpeed();

void doCalibrate();

void doTestDrive();
void doTestCommand();
void doTestRotation();
void doAdditionalBeepFeedback(bool aDoBeep);

/*
 * IR functions, which require code for distance measurement from Distance.hpp
 */
#if defined(_ROBOT_CAR_DISTANCE_HPP)
void doKeepDistance();
void doFollower();
void stepDistanceFeedbackMode();
void stepDistanceSourceMode();
void toggleDistanceScanSpeed();
extern bool sEnableFollower; // Follower mode activated
extern bool sEnableScanAndTurn; // Follower mode with scan and turn
#endif

#endif // _ROBOT_CAR_IR_COMMANDS_H
