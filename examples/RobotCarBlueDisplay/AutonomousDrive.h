/*
 * AutonomousDrive.h
 *
 *  Copyright (C) 2016-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#if defined(ENABLE_AUTONOMOUS_DRIVE)
#ifndef _AUTONOMOUS_DRIVE_H
#define _AUTONOMOUS_DRIVE_H

// These are the default values as defined in Distances.h
#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER        22 // If measured distance is less than this value, go backwards
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER        30 // If measured distance is greater than this value, go forward
#define FOLLOWER_DISTANCE_TIMEOUT_CENTIMETER        70 // Do not accept target with distance greater than this value

/*
 * Different autonomous driving modes
 */
#define MODE_MANUAL_DRIVE               0
#define MODE_COLLISION_AVOIDING_BUILTIN 1
#define MODE_COLLISION_AVOIDING_USER    2 // like MODE_COLLISION_AVOIDING_BUILTIN but use doUserCollisionAvoiding()
#define MODE_FOLLOWER                   3
extern uint8_t sDriveMode;

/*
 * Step modes for MODE_AUTONOMOUS_DRIVE
 */
#define MODE_CONTINUOUS         0
#define MODE_STEP_TO_NEXT_TURN  1 // stop before a turn
#define MODE_SINGLE_STEP        2 // stop after CENTIMETER_PER_RIDE_2
extern uint8_t sStepMode;
extern bool sDoStep;

/*
 * Used for adaptive collision detection
 */
#if defined(USE_ENCODER_MOTOR_CONTROL)
extern uint8_t sCentimetersDrivenPerScan; // Encoder counts per US scan in autonomous mode
#else
extern const uint8_t sCentimetersDrivenPerScan; // 20 cm
#endif

int postProcessAndCollisionAvoidingAndDraw();
void driveAutonomousOneStep();
void startStopAutomomousDrive(bool aDoStart, uint8_t aDriveMode = MODE_MANUAL_DRIVE);
void driveCollisonAvoidingOneStep();
void driveFollowerModeOneStep();

#endif // _AUTONOMOUS_DRIVE_H
#endif // defined(ENABLE_AUTONOMOUS_DRIVE)
