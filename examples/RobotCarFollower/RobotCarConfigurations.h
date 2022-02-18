/*
 *  RobotCarConfigurations.h
 *
 *  Contains car configuration information for different named configuration names
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

#ifndef ROBOT_CAR_CONFIGURATIONS_H
#define ROBOT_CAR_CONFIGURATIONS_H

/*
 * This is the available set of predefined configurations
 */
//#define LAVWIN_BASIC_2LIPO_CONFIGURATION              // Lafvin 2WD model with 2 LiPos, diode and IR receiver mounted
//#define LAVWIN_IR_DISTANCE_CONFIGURATION        // Basic + IR distance
//#define LAVWIN_IR_IMU_CONFIGURATION             // Basic + IR distance + MPU6050
//#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION    // Shield + 2 LiPo + Servo head down
//#define MOTOR_SHIELD_2WD_IR_TOF_CONFIGURATION   // Basic + TOF sensor
//#define ENCODER_MOTOR_SHIELD_2WD_IR_TOF_CONFIGURATION // Basic + shield + encoder + TOF sensor
//#define MOTOR_SHIELD_2WD_FULL_CONFIGURATION
//#define MOTOR_SHIELD_4WD_IR_CONFIGURATION        // Basic + 4 Wheels + IR distance + encoder
//#define BREADBOARD_FULL_CONFIGURATION
/*
 * Parameters for car control boards, sensors and extensions
 */
//#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
//#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
//#define CAR_HAS_IR_DISTANCE_SENSOR      // Use a Sharp GP2Y0A21YK / 1080 IR distance sensor
//#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
// Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin.
//#define US_SENSOR_SUPPORTS_1_PIN_MODE   // Activate it, if you use modified HC-SR04 modules or HY-SRF05 ones.
//#define CAR_HAS_4_WHEELS
//
// For pan tilt we have 2 servos in total
//#define CAR_HAS_PAN_SERVO
//#define CAR_HAS_TILT_SERVO
//#define CAR_HAS_CAMERA
//#define LASER_MOUNTED
/*
 * Parameters for PWMMotorControl
 */
//#define USE_ENCODER_MOTOR_CONTROL   // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD   // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
//#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LIPO                  // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO                  // Or if you use a Mosfet bridge, 1 LIPO (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define MOSFET_BRIDGE_USED          // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP         // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program space.
//#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.

/*
 * Basic + IR distance + MPU6050
 */
#if defined(LAVWIN_IR_IMU_CONFIGURATION)
#define CAR_HAS_IR_DISTANCE_SENSOR  // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define LAVWIN_IR_DISTANCE_CONFIGURATION
#endif

/*
 * Basic + IR distance
 */
// https://github.com/ArminJo/PWMMotorControl/blob/master/pictures/L298Car_TopView_small.jpg
#if defined(LAVWIN_IR_DISTANCE_CONFIGURATION)
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define LAVWIN_BASIC_2LIPO_CONFIGURATION
#endif

/*
 * Lafvin 2WD model with 2 LiPos and IR receiver mounted
 * https://de.aliexpress.com/item/32816490316.html
 */
#if defined(LAVWIN_BASIC_2LIPO_CONFIGURATION)
#define VIN_2_LIPO                  // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81 // Correction for the series SI-diode in the VIN line of the UNO board
#endif

/*
 * Lafvin 2WD model with 4 AA batteries and IR receiver mounted
 * https://de.aliexpress.com/item/32816490316.html
 */
#if defined(BASIC_2WD_4AA_CONFIGURATION)
#define VIN_VOLTAGE_CORRECTION 0.81 // Correction for the series SI-diode in the VIN line of the UNO board
#endif


/*
 * The basic layout of my smart 2wd robot car with 2 LiPo's instead of 4 AA
 * Shield + 2 LiPo's + Servo head down
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg
 */
#if defined(MOTOR_SHIELD_2WD_BASIC_CONFIGURATION)
#define USE_ADAFRUIT_MOTOR_SHIELD
#define VIN_2_LIPO                  // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#endif

/*
 * Basic + TOF sensor
 */
#if defined(MOTOR_SHIELD_2WD_IR_TOF_CONFIGURATION)
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#endif

/*
 * Basic + TOF sensor + encoder
 */
#if defined(ENCODER_MOTOR_SHIELD_2WD_IR_TOF_CONFIGURATION)
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define USE_ENCODER_MOTOR_CONTROL   // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#endif

/*
 * The maximum layout of my smart 2wd robot car
 * Basic + TOF + encoder + MPU6050
 */
#if defined(MOTOR_SHIELD_2WD_FULL_CONFIGURATION)
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define USE_ENCODER_MOTOR_CONTROL   // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#endif

/*
 * Basic + 4 Wheels + IR distance + encoder
 */
#if defined(MOTOR_SHIELD_4WD_IR_CONFIGURATION)
#define CAR_HAS_4_WHEELS
#define USE_ENCODER_MOTOR_CONTROL   // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#endif

/*
 * Nano Breadboard version without shield and with pan/tilt servo and MPU camera and laser
 */
#if defined(BREADBOARD_FULL_CONFIGURATION)
#define CAR_HAS_4_WHEELS
#define CAR_HAS_PAN_SERVO
#define CAR_HAS_TILT_SERVO
#define CAR_HAS_CAMERA
#define LASER_MOUNTED
#define MOSFET_BRIDGE_USED          // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
#define VIN_2_LIPO                  // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81 // Correction for the series SI-diode in the VIN line of the UNO board
#define USE_ENCODER_MOTOR_CONTROL   // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#endif

#endif /* ROBOT_CAR_CONFIGURATIONS_H */
#pragma once
