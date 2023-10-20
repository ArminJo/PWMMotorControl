/*
 *  SmartCarFollower.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel smart car with an Arduino.
 *  If the target vanishes, the distance sensor scans to get the vanished (or a new) target.
 *
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
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
 * Values to configure the behavior of the follower
 */
#define FOLLOWER_DISTANCE_MINIMUM_CENTIMETER            22 // If measured distance is less than this value, go backwards
#define FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER            30 // If measured distance is greater than this value, go forward
#define FOLLOWER_TARGET_DISTANCE_TIMEOUT_CENTIMETER     60 // Do not accept target with distance greater than this value
#define FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER   120 // Do not measure and give tone feedback for distances greater than this

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
//#define TBB6612_4WD_2LI_ION_DIRECT_BASIC_CONFIGURATION // TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION but power supply is connected direct to VIN and not to Uno power jack
//#define TBB6612_4WD_2LI_ION_DIRECT_VIN_CONFIGURATION  // China set with TB6612 mosfet bridge + 2 Li-ion direct + VIN voltage divider
//#define L298_2WD_4AA_BASIC_CONFIGURATION              // China 2WD set with L298 bridge and Uno board with series diode for VIN + 4 AA batteries. DEFAULT.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION          // China 2WD set with L298 bridge and Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_2LI_ION_VIN_IR_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_2LI_ION_VIN_IR_IMU_CONFIGURATION     // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
//#define MOTOR_SHIELD_4WD_BASIC_CONFIGURATION          // Adafruit Motor Shield using TB6612 mosfet bridge. 2 Li-ion + VIN voltage divider + servo head down
//#define MOTOR_SHIELD_4WD_FULL_CONFIGURATION           // Motor Shield + encoder + 2 Li-ion + servo head down + IR distance
//#define MECANUM_US_DISTANCE_CONFIGURATION             // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
#define DO_NOT_SUPPORT_RAMP         // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
#define DO_NOT_SUPPORT_AVERAGE_SPEED // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.
#define USE_SOFT_I2C_MASTER             // Saves 2110 bytes program memory and 200 bytes RAM for I2C communication to Adafruit motor shield and MPU6050 IMU compared with Arduino Wire

//#define TRACE
//#define DEBUG
//#define INFO
#include "RobotCarConfigurations.h" // sets e.g. CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"

/*
 * Enable functionality of this program
 */
//#define USE_IR_REMOTE // enables control by an IR remote, see below
// if CAR_HAS_VIN_VOLTAGE_DIVIDER is NOT specified, existence of voltage divider will be determined dynamically if MONITOR_VIN_VOLTAGE is enabled.
#define MONITOR_VIN_VOLTAGE         // Enable monitoring of VIN voltage for exact movements, if available
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000

/*
 * Enabling program features dependent on car configuration
 * If IR or TOF distance sensors are available, they take precedence over the US sensor.
 */
#define SMART_CAR_FOLLOWER_PROGRAM          // The program name, used by RobotCarUtils.hpp

#if !defined(CAR_HAS_DISTANCE_SERVO)
#error This program requires a distance servo mounted indicated by a #define CAR_HAS_DISTANCE_SERVO
#define CAR_HAS_DISTANCE_SERVO // avoid subsequent errors
#endif
//#define DISTANCE_SERVO_TRIM_DEGREE (-10) // Value is added to all degrees in DistanceServoWriteAndWaitForStop()

#if defined(CAR_HAS_MPU6050_IMU)
#define USE_MPU6050_IMU             // Enable it by default, if available
#endif

#include "CarPWMMotorControl.hpp"
#include "Distance.hpp"  // provides DistanceServo definition and uses FOLLOWER_DISTANCE_MINIMUM_CENTIMETER definition

#if defined(USE_IR_REMOTE)
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
#include "RobotCarIRCommands.hpp" // requires #include "Distance.hpp"
#include "RobotCarIRCommandMapping.h" // must be included before IRCommandDispatcher.hpp to define IR_ADDRESS and IRMapping and string "unknown".
#define LOCAL_INFO // Enable info just for IRCommandDispatcher
#include "IRCommandDispatcher.hpp"
#else
// Tone only if no external control (here by IR) is enabled
#define DISTANCE_FEEDBACK_MODE   DISTANCE_FEEDBACK_PENTATONIC // one of DISTANCE_FEEDBACK_CONTINUOUSLY or DISTANCE_FEEDBACK_PENTATONIC
#endif

#include "RobotCarUtils.hpp" // must be after #define MONITOR_VIN_VOLTAGE

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT                0

void getDistanceModesFromPins();
void doFollowerOneStep();
distance_range_t getDistanceRange(uint8_t aCentimeter);

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#if defined(USE_IR_REMOTE)
    // For available IR commands see IRCommandMapping.h https://github.com/ArminJo/PWMMotorControl/blob/master/examples/SmartCarFollower/IRCommandMapping.h
    IRDispatcher.init();
#endif

    printConfigInfo(&Serial);
    printProgramOptions(&Serial);
    PWMDcMotor::printCompileOptions(&Serial);

    initRobotCarPWMMotorControl();
    RobotCar.printCalibrationValues(&Serial);

    getDistanceModesFromPins();

    RobotCar.setSpeedPWMCompensation(SPEED_PWM_COMPENSATION_RIGHT); // Set left/right speed compensation

    /*
     * Initialize US servo and set to forward position
     */
    initDistance();
    DistanceServoWriteAndWaitForStop(90);

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);
    delay(200);
    tone(PIN_BUZZER, 2200, 100);

#if defined(US_DISTANCE_SENSOR_ENABLE_PIN) // If this pin is connected to ground, use the US distance sensor instead of the IR distance sensor
    pinMode(US_DISTANCE_SENSOR_ENABLE_PIN, INPUT_PULLUP);
#endif

#if defined(USE_MPU6050_IMU)
    /*
     * Wait after pressing the reset button, or attaching the power
     * and then take offset values for 1/2 second
     */
    delay(1000);
    tone(PIN_BUZZER, 2200, 50);
    delay(100);
    RobotCar.calculateAndPrintIMUOffsets(&Serial);
    tone(PIN_BUZZER, 2200, 50);
#endif

    /*
     * Do not start immediately with driving
     */
    delay(2000);

    /*
     * Servo feedback for start of loop
     */
    DistanceServoWriteAndWaitForStop(135);
    delay(500);
    DistanceServoWriteAndWaitForStop(45);
    delay(500);
    DistanceServoWriteAndWaitForStop(90);
    delay(1500);

    Serial.println(F("Start loop"));
    Serial.println();
//    sDoSlowScan = true;
}

void loop() {
#if defined(USE_IR_REMOTE)
    RobotCar.updateMotors(); // enables going fixed distance

    /*
     * Check for IR commands and execute them.
     * Returns only AFTER finishing of requested action
     */
    IRDispatcher.checkAndRunSuspendedBlockingCommands();

    // we can enable / disable follower / distance (no turn) mode by IR
    if (sEnableKeepDistance || sEnableFollower) {
        doFollowerOneStep();
    } else {
        getDistanceAsCentimeter(FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER, true); // timeout at 150 cm
        playDistanceFeedbackTone(sEffectiveDistanceCentimeter);
        printDistanceIfChanged(&Serial); // prints "Distance=XXcm" with or "Distance timeout" without a newline{
        if (sEffectiveDistanceJustChanged) {
            Serial.println(); // Terminate line "Distance=57cm ->  SpeedPWM=255" etc.
        }
    }
#else
    getDistanceModesFromPins();
    doFollowerOneStep();
#endif

    checkVinPeriodicallyAndPrintIfChanged();

#if defined(USE_IR_REMOTE)
#else
    // No delay, we have a delay by scanning distances
#  if defined(USE_ENCODER_MOTOR_CONTROL)
    RobotCar.updateMotors();
#  endif
#endif
}

/*
 * Evaluates the US_DISTANCE_SENSOR_ENABLE_PIN switching between IR and US sensor.
 * Evaluates the DISTANCE_TONE_FEEDBACK_ENABLE_PIN to suppress tone feedback.
 */
void getDistanceModesFromPins() {
#  if defined(US_DISTANCE_SENSOR_ENABLE_PIN) && (defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR))
// If US_DISTANCE_SENSOR_ENABLE_PIN is connected to ground we use the US distance as fallback.
// Useful for testing the difference between both sensors.
    if (digitalRead(US_DISTANCE_SENSOR_ENABLE_PIN) == LOW) {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_US;
    } else {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_IR_OR_TOF;
    }
#  endif
#  if defined(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) && defined(DISTANCE_FEEDBACK_MODE) // If this pin is connected to ground, enable distance feedback
    if (digitalRead(DISTANCE_TONE_FEEDBACK_ENABLE_PIN) == LOW) {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_MODE;
    } else {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
        noTone(PIN_BUZZER);
    }
#  endif
}

void doFollowerOneStep() {
    unsigned int tForwardCentimeter;
    int8_t tRotationDegree = 0;

#if defined(USE_IR_REMOTE)
    if (sEnableFollower) { // Scan for target only if FollowerMode enabled, and not if KeepDistance enabled
#endif
    /*
     * Scan target at 70, 90 and 110 degree
     */
    tRotationDegree = scanTarget(FOLLOWER_TARGET_DISTANCE_TIMEOUT_CENTIMETER);
    printForwardDistanceInfo(&Serial);
    tForwardCentimeter = sRawForwardDistancesArray[INDEX_TARGET_FORWARD]; // Values between 1 and FOLLOWER_TARGET_DISTANCE_MAXIMUM_CENTIMETER

#if defined(USE_IR_REMOTE)
    } else { // if (sEnableFollower)
        /*
         * Just delay and then get one value, do not scan.
         */
        delay(50);
        tForwardCentimeter = getDistanceAsCentimeter(FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER, true);
        if (tForwardCentimeter == DISTANCE_TIMEOUT_RESULT) {
            tForwardCentimeter = FOLLOWER_DISPLAY_DISTANCE_TIMEOUT_CENTIMETER;
        }
    }
#endif

    distance_range_t tRange = getDistanceRange(tForwardCentimeter);
    printDistanceRangeCharacter(tRange, &Serial);
    Serial.print(' ');

    /*
     * Rotate if requested
     */
    if (tRotationDegree != 0) {
        // Do a cast, since the values of tRange and rotation match!
        RobotCar.rotate(tRotationDegree, static_cast<turn_direction_t>(tRange));
        sRawForwardDistancesArray[INDEX_TARGET_FORWARD] = 0; // Force sEffectiveDistanceJustChanged to be true at next loop, since we have stopped here.
        // Make a fresh scan, before moving

    } else if (sEffectiveDistanceJustChanged) {
        /*
         * No turn, compute new speed depending on the forward distance
         */
        unsigned int tNewSpeedPWM = 0;
        uint8_t tDirection = DIRECTION_STOP;

        sEffectiveDistanceCentimeter = tForwardCentimeter; // Required for printDistanceIfChanged()
        printDistanceIfChanged(&Serial); // prints "Distance=XXcm" or "Distance timeout" without a newline

        playDistanceFeedbackTone(tForwardCentimeter);
        // we are after printForwardDistanceInfo() and printDistanceRangeCharacter() here
        Serial.print(F("-> "));

        if (tRange == DISTANCE_TO_GREAT) {
            /*
             * FORWARD  - Distance (31 to 60) too great
             * Drive FORWARD with speed proportional to the gap. We start with DEFAULT_START_SPEED_PWM.
             * Maximum difference between current and target distance (tCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER) is 30.
             */
            uint8_t tDifferenceCentimeter = tForwardCentimeter - FOLLOWER_DISTANCE_MAXIMUM_CENTIMETER;
            tNewSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_START_SPEED_PWM, sVINVoltage) + tDifferenceCentimeter * 4; // maximum is + 120 here
            tDirection = DIRECTION_FORWARD;

        } else if (tRange == DISTANCE_TO_SMALL) {
            /*
             * BACKWARD  - Distance (1 to 19) too close
             * Drive with speed proportional to the gap. We start with DEFAULT_DRIVE_SPEED_PWM, to be a bit more agile here.
             * Maximum difference between current and target distance is FOLLOWER_DISTANCE_MINIMUM_CENTIMETER / 20.
             */
            uint16_t tDifferenceCentimeter = FOLLOWER_DISTANCE_MINIMUM_CENTIMETER - tForwardCentimeter;
            tNewSpeedPWM = PWMDcMotor::getVoltageAdjustedSpeedPWM(DEFAULT_DRIVE_SPEED_PWM, sVINVoltage)
//                        + tDifferenceCentimeter * 8; // maximum is + 320 here
                    + tDifferenceCentimeter * 4; // maximum is + 80 here
            tDirection = DIRECTION_BACKWARD;
        }

        /*
         * Print direction changes
         */
        if (RobotCar.getCarDirection() != tDirection) {
            // print only once at direction change
            Serial.print(F("go from "));
            PWMDcMotor::printDirectionString(&Serial, RobotCar.getCarDirection());
            Serial.print(F(" to "));
            PWMDcMotor::printDirectionString(&Serial, tDirection);
            Serial.print(' ');
        }

        /*
         * Process new speed
         */
        if (tNewSpeedPWM != 0) {
            /*
             * Clip speed, since we have a delay introduced by scanning,
             * and we do not want that the car is moving from minimum to maximum or back during this delay.
             * And additionally, it seems that to much speed generates high frequency noise, which disturbs the US sensor.
             */
            uint8_t tMaxSpeed = DEFAULT_DRIVE_SPEED_PWM + (DEFAULT_DRIVE_SPEED_PWM / 2); // Corresponds to 3 volt
            if (sDoSlowScan) {
                tMaxSpeed = DEFAULT_DRIVE_SPEED_PWM; // reduce max speed further to 2 volt
            }
            if (tNewSpeedPWM > tMaxSpeed) {
                tNewSpeedPWM = tMaxSpeed;
            }
            Serial.print(F("SpeedPWM="));
            Serial.print(tNewSpeedPWM);
            /*
             * Set speed and direction
             */
            RobotCar.setSpeedPWMAndDirection(tNewSpeedPWM, tDirection);

        } else {
            /*
             * STOP - Target is in the right distance, or we have a timeout
             */
            if (RobotCar.getCarDirection() != DIRECTION_STOP) {
                RobotCar.stop(STOP_MODE_RELEASE); // stop only once
                Serial.print(F("stop car"));
            } else {
                if (tRange == DISTANCE_OK) {
                    Serial.print(F("ok"));
                } else {
                    // tRange == DISTANCE_TIMEOUT here
                    Serial.print(F("searching"));
                }
            }
        }
    }
    Serial.println(); // Terminate line "Distance=57cm ->  SpeedPWM=255" etc.
}

