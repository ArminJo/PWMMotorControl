/*
 * IRCommandMapping.h
 *
 * IR remote button codes, strings, and functions to call for quadruped IR control
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 * Mapping for controlling a smart car
 * Supported IR remote are KEYES (the original mePed remote) and the remote from a DVBT stick
 * Select the one you have below.
 */

#ifndef _IR_COMMAND_MAPPING_H
#define _IR_COMMAND_MAPPING_H

#include <Arduino.h>
#include "IRCommandDispatcher.h" // for IR_COMMAND_FLAG_BLOCKING etc.
//#include "Commands.h" // include for all the commands used in the mapping arrays below

/*
 * !!! Choose your remote !!!
 */
//#define USE_KEYES_REMOTE_CLONE // With number pad above direction control, will be taken as default
//#define USE_KEYES_REMOTE       // The mePed 2 Standard remote with number pad below direction control. Another name printed on the remote is Lafvin
//#define USE_DVBT_STICK_REMOTE
#if !defined(USE_KEYES_REMOTE) && !defined(USE_KEYES_REMOTE_CLONE) && !defined(USE_DVBT_STICK_REMOTE)
#define USE_KEYES_REMOTE_CLONE // The one you can buy at aliexpress
#endif

#if defined(USE_KEYES_REMOTE_CLONE)
#define IR_REMOTE_NAME "KEYES_CLONE"
// Codes for the KEYES CLONE remote control with 17 keys with keypad above direction control
#define IR_ADDRESS 0x00

#define IR_UP    0x18
#define IR_DOWN  0x52
#define IR_RIGHT 0x5A
#define IR_LEFT  0x08
#define IR_OK    0x1C

#define IR_1    0x45
#define IR_2    0x46
#define IR_3    0x47
#define IR_4    0x44
#define IR_5    0x40
#define IR_6    0x43
#define IR_7    0x07
#define IR_8    0x15
#define IR_9    0x09
#define IR_0    0x19

#define IR_STAR 0x16
#define IR_HASH 0x0D
/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT
#define COMMAND_STOP        IR_OK

#define COMMAND_DISTANCE_FEEDBACK   IR_STAR
#define COMMAND_RESET               IR_0
#define COMMAND_DISTANCE_SOURCE     IR_HASH

#define COMMAND_DECREASE_SPEED  IR_1
#define COMMAND_DEFAULT_SPEED   IR_2
#define COMMAND_INCREASE_SPEED  IR_3

#define COMMAND_DISTANCE        IR_4
#define COMMAND_FOLLOWER        IR_5
#define COMMAND_SCAN_SPEED      IR_6

#define COMMAND_TEST_ROTATION   IR_7
#define COMMAND_TEST_DRIVE      IR_8
#define COMMAND_TEST            IR_9

#endif // defined(USE_KEYES_REMOTE_CLONE)

#if defined(USE_KEYES_REMOTE)
#  if defined(IR_REMOTE_NAME)
#error "Please choose only one remote for compile"
#  else
#define IR_REMOTE_NAME "KEYES"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the KEYES remote control with 17 keys
#define IR_ADDRESS 0x00

#define IR_UP    0x46
#define IR_DOWN  0x15
#define IR_RIGHT 0x43
#define IR_LEFT  0x44
#define IR_OK    0x40

#define IR_1    0x16
#define IR_2    0x19
#define IR_3    0x0D
#define IR_4    0x0C
#define IR_5    0x18
#define IR_6    0x5E
#define IR_7    0x08
#define IR_8    0x1C
#define IR_9    0x5A
#define IR_0    0x52

#define IR_STAR 0x42
#define IR_HASH 0x4A

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_FORWARD     IR_UP
#define COMMAND_BACKWARD    IR_DOWN
#define COMMAND_RIGHT       IR_RIGHT
#define COMMAND_LEFT        IR_LEFT
#define COMMAND_STOP        IR_OK

#define COMMAND_DISTANCE_FEEDBACK   IR_STAR
#define COMMAND_RESET               IR_0
#define COMMAND_DISTANCE_SOURCE     IR_HASH

#define COMMAND_DECREASE_SPEED  IR_1
#define COMMAND_DEFAULT_SPEED   IR_2
#define COMMAND_INCREASE_SPEED  IR_3

#define COMMAND_DISTANCE        IR_4
#define COMMAND_FOLLOWER        IR_5
#define COMMAND_SCAN_SPEED      IR_6

#define COMMAND_TEST_ROTATION   IR_7
#define COMMAND_TEST_DRIVE      IR_8
#define COMMAND_TEST            IR_9

#  endif // defined(IR_REMOTE_NAME)
#endif

#if defined(USE_DVBT_STICK_REMOTE)
#  if defined(IR_REMOTE_NAME)
#error "Please choose only one remote for compile"
#  else
#define IR_REMOTE_NAME "DVB-T"
/*
 * FIRST:
 * IR code to button mapping for better reading. IR codes should only referenced here.
 */
// Codes for the silver China DVB-T Stick remote control with 3x7 (21) keys
#define IR_ADDRESS 0x00

#define IR_ON_OFF 0x4D
#define IR_SOURCE 0x54
#define IR_MUTE   0x16

#define IR_RECORD     0x4C
#define IR_TIMESHIFT  0x0C

#define IR_CH_PLUS    0x05
#define IR_CH_MINUS   0x02

#define IR_VOL_MINUS  0xA
#define IR_VOL_PLUS   0x1E

#define IR_FULLSCREEN 0x40
#define IR_RECALL     0x1C

#define IR_0    0x12
#define IR_1    0x09
#define IR_2    0x1D
#define IR_3    0x1F
#define IR_4    0x0D
#define IR_5    0x19
#define IR_6    0x1B
#define IR_7    0x11
#define IR_8    0x15
#define IR_9    0x17

/*
 * SECOND:
 * IR button to command mapping for better reading. IR buttons should only referenced here.
 */
#define COMMAND_RESET               IR_ON_OFF
#define COMMAND_DISTANCE_SOURCE     IR_SOURCE
#define COMMAND_DISTANCE_FEEDBACK   IR_MUTE

#define COMMAND_SCAN_SPEED      IR_RECORD

#define COMMAND_FORWARD         IR_CH_PLUS
#define COMMAND_BACKWARD        IR_CH_MINUS
#define COMMAND_RIGHT           IR_VOL_PLUS
#define COMMAND_LEFT            IR_VOL_MINUS
#define COMMAND_STOP            IR_FULLSCREEN

#define COMMAND_DISTANCE        IR_RECALL
#define COMMAND_FOLLOWER        IR_0

#define COMMAND_DECREASE_SPEED  IR_1
#define COMMAND_DEFAULT_SPEED   IR_2
#define COMMAND_INCREASE_SPEED  IR_3

#define COMMAND_TEST_ROTATION   IR_4
#define COMMAND_TEST_DRIVE      IR_5
#define COMMAND_TEST            IR_6

// IR Layout mirrored
// for attaching at the back of the remote
//
// Sound. |  Dist.   | Reset
// mode   |  source  |
//-----------------------------
//  Scan  | Forward  |
//  speed |          |
//-----------------------------
// Right  |   Stop   | Left
//  <-    |          |  ->
//-----------------------------
// Dist.  | Backward | Follower
//        |          |
//-----------------------------
// Speed  |  Speed   | Speed
//   +    | default  |   -
//-----------------------------
//  Test  |  Test    | Test
//        |  drive   | rotation
//-----------------------------

#  endif // defined(IR_REMOTE_NAME)
#endif // defined(USE_DVBT_STICK_REMOTE)

/*
 * THIRD:
 * Main mapping of commands to C functions
 */

// IR strings of functions for output in alphabetical order
static const char backward[] PROGMEM ="backward";
static const char beep[] PROGMEM ="beep";
static const char center[] PROGMEM ="center";
static const char distance[] PROGMEM ="distance";
static const char drive[] PROGMEM ="drive";
static const char enter[] PROGMEM ="enter";
static const char follower[] PROGMEM ="follower";
static const char forward[] PROGMEM ="forward";
static const char left[] PROGMEM ="left";
static const char reset[] PROGMEM ="reset";
static const char right[] PROGMEM ="right";
static const char rotate[] PROGMEM ="rotate";
static const char speedIncrease[] PROGMEM ="increase speed";
static const char speedDecrease[] PROGMEM ="decrease speed";
static const char speedDefault[] PROGMEM ="default speed";
static const char stop[] PROGMEM ="stop";
static const char test[] PROGMEM ="test";
static const char toggleScanSpeed[] PROGMEM ="toggle scan speed";
static const char stepFeedback[] PROGMEM ="step feedback";
static const char stepDistanceSource[] PROGMEM ="step distance source";
static const char unknown[] PROGMEM ="unknown";

// IR Layout mirrored
// for attaching at the back of the remote
//
//          Forward
//
// Right     Stop      Left
//  <-                  ->
//          Backward
//
//-----------------------------
// Speed  |  Speed   | Speed
//   +    | default  |   -
//-----------------------------
//        | Follower | keep
//        |          | dist.
//-----------------------------
//  Scan  |  Test    | Test
//  speed |  drive   | rotation
//-----------------------------
// Sound. |  Reset   | Dist.
// mode   |          | source

/*
 * Main mapping array of commands to C functions and command strings
 */
const struct IRToCommandMappingStruct IRMapping[] = {
/*
 * Commands, which must run exclusively and therefore must first stop other commands running.
 */
#if defined(_ROBOT_CAR_DISTANCE_HPP)
        { COMMAND_DISTANCE, IR_COMMAND_FLAG_BLOCKING, &doKeepDistance, distance }, /**/
        { COMMAND_FOLLOWER, IR_COMMAND_FLAG_BLOCKING, &doFollower, follower }, /**/
#endif
        { COMMAND_TEST_ROTATION, IR_COMMAND_FLAG_BLOCKING, &testRotation, rotate }, /**/
        { COMMAND_TEST_DRIVE, IR_COMMAND_FLAG_BLOCKING, &testDrive, drive }, /**/
        { COMMAND_TEST, IR_COMMAND_FLAG_BLOCKING, &testCommand, test }, /**/

        /*
         * Commands, which can be executed always, since the are short. Like set mode etc.
         */
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
        { COMMAND_DISTANCE_SOURCE, IR_COMMAND_FLAG_NON_BLOCKING, &stepDistanceSourceMode, stepDistanceSource }, /**/
#endif
#if defined(_ROBOT_CAR_DISTANCE_HPP)
        { COMMAND_DISTANCE_FEEDBACK, IR_COMMAND_FLAG_NON_BLOCKING, &stepDistanceFeedbackMode, stepFeedback }, /**/
        { COMMAND_SCAN_SPEED, IR_COMMAND_FLAG_NON_BLOCKING, &toggleDistanceScanSpeed, toggleScanSpeed }, /**/
#endif
        { COMMAND_INCREASE_SPEED, IR_COMMAND_FLAG_NON_BLOCKING, &doIncreaseSpeed, speedIncrease }, /**/
        { COMMAND_DECREASE_SPEED, IR_COMMAND_FLAG_NON_BLOCKING, &doDecreaseSpeed, speedDecrease }, /**/

        /*
         * Repeatable short commands
         */
        { COMMAND_FORWARD, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &goForward, forward }, /**/
        { COMMAND_BACKWARD, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &goBackward, backward }, /**/
        { COMMAND_RIGHT, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &turnRight, right }, /**/
        { COMMAND_LEFT, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &turnLeft, left }, /**/
        { COMMAND_DEFAULT_SPEED, IR_COMMAND_FLAG_REPEATABLE_NON_BLOCKING, &doDefaultSpeed, speedDefault }, /**/
        { COMMAND_STOP, IR_COMMAND_FLAG_IS_STOP_COMMAND, &doStop, stop }, /**/
        { COMMAND_RESET, IR_COMMAND_FLAG_IS_STOP_COMMAND, &doReset, reset }

};

#endif // _IR_COMMAND_MAPPING_H
