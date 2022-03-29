/*
 *  RobotCarConfigurations.h
 *
 *  Contains a few predefined set of car configurations
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
 *  but WITHOUT ANY WARRANTY without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_CAR_CONFIGURATIONS_H
#define _ROBOT_CAR_CONFIGURATIONS_H

/*
 * This is the available set of predefined configurations
 * All configurations include a HC-SR04 ultrasonic distance sensor mounted on a pan servo.
 */
//////////////////////////////////////////////////////
//#define L298_BASIC_2WD_4AA_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_BASIC_2WD_2LI_ION_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion's.
//#define L298_VIN_IR_DISTANCE_CONFIGURATION        // L298_Basic_2WD + VIN voltage divider + IR distance
//#define L298_VIN_IR_IMU_CONFIGURATION             // L298_Basic_2WD + VIN voltage divider + IR distance + MPU6050
//#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION      // Adafruit Motor Shield using TB6612 mosfet bridge. 2 LiPo + servo head down
//#define MOTOR_SHIELD_TOF_CONFIGURATION            // Basic_2WD + VL53L1X TimeOfFlight sensor
//#define MOTOR_SHIELD_ENCODER_TOF_CONFIGURATION    // Basic_2WD + encoder + VL53L1X TimeOfFlight sensor
//#define MOTOR_SHIELD_ENCODER_4WD_IR_CONFIGURATION // Basic + encoder + 4 Wheels + IR distance
//#define MOTOR_SHIELD_2WD_FULL_CONFIGURATION       // Basic + encoder + VL53L1X TimeOfFlight sensor + MPU6050
//#define BREADBOARD_FULL_CONFIGURATION             // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge, pan/tilt servo, MPU6050, camera and laser
//#define MECANUM_BASIC_CONFIGURATION               // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge
//#define CAR_IS_ESP32_CAM_BASED                    // Car controlled by an ESP32-CAM like https://github.com/ArminJo/ESP32-Cam-Sewer-inspection-car
//////////////////////////////////////////////////////
//
/*
 * Distinct parameters for car control boards, sensors and extensions
 */
//#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
//#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
//#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
//#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
//#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
//#define CAR_HAS_IR_DISTANCE_SENSOR      // A Sharp GP2Y0A21YK / 1080 IR distance sensor is mounted
//#define CAR_HAS_TOF_DISTANCE_SENSOR     // A VL53L1X TimeOfFlight distance sensor is mounted
// Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin.
//#define US_SENSOR_SUPPORTS_1_PIN_MODE   // Activate it, if you use modified HC-SR04 modules or HY-SRF05 ones.
//#define CAR_HAS_4_WHEELS
//#define CAR_HAS_4_MECANUM_WHEELS
//
// For pan tilt we have 2 servos in total
//#define CAR_HAS_PAN_SERVO
//#define CAR_HAS_TILT_SERVO
//#define CAR_HAS_CAMERA
//#define CAR_HAS_LASER
/*
 *
 */
//#define CAR_IS_NANO_BASED           // We have an Arduino NANO instead of an UNO. This implies MOSFET_BRIDGE_USED and VIN_VOLTAGE_CORRECTION.
/*
 * Parameters for PWMMotorControl
 */
//#define USE_ENCODER_MOTOR_CONTROL       // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 with 2xTB6612 connected by I2C instead of external TB6612 or L298 breakout board.
//#define USE_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LIPO                      // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO                      // Or if you use a Mosfet bridge (TB6612), 1 LIPO (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT 6000// Default. For 4 x AA batteries (6 volt).
//#define MOSFET_BRIDGE_USED              // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
//#define DEFAULT_DRIVE_MILLIVOLT   2000  // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
//#define DO_NOT_SUPPORT_AVERAGE_SPEED    // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.
/*
 * L298 basic + IR distance + MPU6050 + VIN monitoring
 */
#if defined(L298_VIN_IR_IMU_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define USE_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define L298_IR_DISTANCE_CONFIGURATION
#define CONFIG_NAME         "L298 basic + IR distance" // BASIC_CONFIG_NAME and CONFIG_NAME is printed by printConfigInfo()
#endif

/*
 * L298 basic + IR distance + VIN voltage divider
 * https://github.com/ArminJo/PWMMotorControl/blob/master/pictures/L298Car_TopView_small.jpg
 */
#if defined(L298_VIN_IR_DISTANCE_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define L298_BASIC_2WD_2LI_ION_CONFIGURATION
#define CONFIG_NAME         " + IR distance + VIN divider"
#endif

/*
 * Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion's.
 * https://de.aliexpress.com/item/32816490316.html
 */
#if defined(L298_BASIC_2WD_2LI_ION_CONFIGURATION)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define VIN_2_LIPO                      // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define BASIC_CONFIG_NAME   "L298 + 2 Li-ion"
#endif

/*
 * Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
 * https://de.aliexpress.com/item/32816490316.html
 */
#if defined(L298_BASIC_2WD_4AA_CONFIGURATION)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define BASIC_CONFIG_NAME   "L298 + 4 AA"
#endif

/*
 * Basic + VL53L1X TimeOfFlight sensor
 */
#if defined(MOTOR_SHIELD_TOF_CONFIGURATION)
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + TOF distance"
#endif

/*
 * Basic + encoder + VL53L1X TimeOfFlight sensor
 */
#if defined(MOTOR_SHIELD_ENCODER_TOF_CONFIGURATION)
#define USE_ENCODER_MOTOR_CONTROL       // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + encoder + TOF distance"
#endif

/*
 * The maximum layout of my smart 2wd robot car
 * Basic + VL53L1X TimeOfFlight + encoder + MPU6050
 */
#if defined(MOTOR_SHIELD_2WD_FULL_CONFIGURATION)
#define USE_ENCODER_MOTOR_CONTROL       // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define USE_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + encoder + TOF distance + MPU6050"
#endif

/*
 * The basic layout of my smart 2wd robot car with 2 LiPo's instead of 4 AA
 * Shield + 2 LiPo's + VIN voltage divider + servo head down
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg
 */
#if defined(MOTOR_SHIELD_2WD_BASIC_CONFIGURATION)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#define VIN_2_LIPO                      // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#define BASIC_CONFIG_NAME   "Motor shield,TB6612  + 2 Li-ion + VIN divider + servo head down"
#endif

/*
 * Basic + 4 Wheels + IR distance + encoder
 */
#if defined(MOTOR_SHIELD_ENCODER_4WD_IR_CONFIGURATION)
#define CAR_HAS_4_WHEELS
#define USE_ENCODER_MOTOR_CONTROL       // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + encoder + 4WD + IR distance"
#endif

/*
 * Nano Breadboard version with Arduino NANO, without shield and with pan/tilt servo and MPU camera and laser
 */
#if defined(BREADBOARD_FULL_CONFIGURATION)
#define CAR_IS_NANO_BASED
#define CAR_HAS_4_WHEELS
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define CAR_HAS_PAN_SERVO
#define CAR_HAS_TILT_SERVO
#define CAR_HAS_CAMERA
#define CAR_HAS_LASER
#define VIN_2_LIPO                      // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define USE_ENCODER_MOTOR_CONTROL       // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define USE_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#define BASIC_CONFIG_NAME   "Breadboard TB6612  + 2 Li-ion + VIN divider + servo head down + MPU6050"
#endif

/*
 * Nano Breadboard version with Arduino NANO, without shield and 4 mecanum wheels + US distance + servo
 */
#if defined(MECANUM_DISTANCE_CONFIGURATION)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define MECANUM_BASIC_CONFIGURATION
#define CONFIG_NAME   " US distance + servo head down"
#endif

/*
 * Nano Breadboard version with Arduino NANO, without shield and 4 mecanum wheels
 */
#if defined(MECANUM_BASIC_CONFIGURATION)
#define CAR_IS_NANO_BASED
#define CAR_HAS_4_MECANUM_WHEELS
#define VIN_2_LIPO                      // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define BASIC_CONFIG_NAME   "Breadboard TB6612  + 2 Li-ion + VIN divider + 4 mecanum wheels"
#endif

/*
 * Nano Breadboard version with Arduino NANO, without shield and 4 mecanum wheels
 */
#if defined(CAR_IS_ESP32_CAM_BASED)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define BASIC_CONFIG_NAME   "ESP32-Cam based"
#endif

/*
 * Some rules
 */
#if defined(CAR_IS_NANO_BASED)
#define MOSFET_BRIDGE_USED              // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
#  if !defined(VIN_VOLTAGE_CORRECTION)
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#  endif
#endif

#if defined(CAR_HAS_US_DISTANCE_SENSOR) || defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#define CAR_HAS_DISTANCE_SENSOR         // At least one distance sensor mounted on a pan servo is available
#endif


// Default case
#if !defined(BASIC_CONFIG_NAME)         // use L298_BASIC_2WD_4AA_CONFIGURATION as default
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define BASIC_CONFIG_NAME   "L298 + 4 AA"
#endif

#endif // _ROBOT_CAR_CONFIGURATIONS_H
#pragma once