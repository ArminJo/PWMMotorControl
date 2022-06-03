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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _ROBOT_CAR_CONFIGURATIONS_H
#define _ROBOT_CAR_CONFIGURATIONS_H

/*
 * This is the available set of predefined configurations of RobotCarConfigurations.h
 * All configurations (except MECANUM_BASIC_CONFIGURATION) include a HC-SR04 ultrasonic distance sensor mounted on a pan servo.
 */
//////////////////////////////////////////////////////
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 4AA.
//#define TBB6612_4WD_4AA_VIN_CONFIGURATION         // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION   // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION    // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define L298_2WD_4AA_BASIC_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_4WD_4AA_BASIC_CONFIGURATION          // China set with L298 + 4AA.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_VIN_IR_DISTANCE_CONFIGURATION    // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_VIN_IR_IMU_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
//#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION      // Adafruit Motor Shield using TB6612 mosfet bridge. 2 Li-ion + VIN voltage divider + servo head down
//#define MOTOR_SHIELD_2WD_TOF_CONFIGURATION        // Basic_2WD + VL53L1X TimeOfFlight sensor
//#define MOTOR_SHIELD_2WD_ENCODER_TOF_CONFIGURATION // Basic_2WD + encoder + VL53L1X TimeOfFlight sensor
//#define MOTOR_SHIELD_2WD_FULL_CONFIGURATION       // Basic + encoder + VL53L1X TimeOfFlight sensor + MPU6050
//#define MOTOR_SHIELD_4WD_BASIC_CONFIGURATION      // Adafruit Motor Shield using TB6612 mosfet bridge. 2 Li-ion + VIN voltage divider + servo head down
//#define MOTOR_SHIELD_4WD_FULL_CONFIGURATION       // Motor Shield + encoder + 2 Li-ion + servo head down + IR distance
//#define BREADBOARD_4WD_FULL_CONFIGURATION         // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge, pan/tilt servo, MPU6050, camera and laser
//#define MECANUM_BASIC_CONFIGURATION               // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels
//#define MECANUM_DISTANCE_CONFIGURATION            // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
//#define CAR_IS_ESP32_CAM_BASED                    // Car controlled by an ESP32-CAM like https://github.com/ArminJo/ESP32-Cam-Sewer-inspection-car
//////////////////////////////////////////////////////
//
/*
 * Distinct parameters for car control boards, sensors and extensions
 */
//#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage
//#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
//#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles
//#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
//#define US_SENSOR_SUPPORTS_1_PIN_MODE   // Activate it, if you use modified HC-SR04 modules or HY-SRF05 ones
//                                           Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin
//#define CAR_HAS_IR_DISTANCE_SENSOR      // A Sharp GP2Y0A21YK / 1080 IR distance sensor is mounted
//#define CAR_HAS_TOF_DISTANCE_SENSOR     // A VL53L1X TimeOfFlight distance sensor is mounted
//#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
//#define CAR_HAS_ENCODERS                // 2 slot type slot-type photo interrupter are mounted and connected with pin 2 and 3
//#define CAR_HAS_MPU6050_IMU             // A MPU6050 breakout board like GY-521 is mounted and connected by I2C
//
// For pan tilt we have 2 servos in total
//#define CAR_HAS_PAN_SERVO
//#define CAR_HAS_TILT_SERVO
//#define CAR_HAS_CAMERA
//#define CAR_HAS_LASER
//
//#define USE_LIGHTWEIGHT_SERVO_LIBRARY   // LightweightServo library can control servos at pin 9 and 10. Saves up to 710 bytes program memory.
//#define CAR_IS_NANO_BASED               // We have an Arduino NANO instead of an UNO resulting in a different pin layout.
/*
 * Parameters for PWMMotorControl
 */
//#define USE_ENCODER_MOTOR_CONTROL       // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 with 2xTB6612 connected by I2C instead of external TB6612 or L298 breakout board.
//#define USE_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                    // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT 6000// Default. For 4 x AA batteries (6 volt).
//#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define DEFAULT_DRIVE_MILLIVOLT   2000  // Drive voltage / motors default speed. Default value is 2.0 volt.
//#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
//#define DO_NOT_SUPPORT_AVERAGE_SPEED    // Disables the encoder function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.
/*
 * Parameters for CarPWMMotorControl
 */
//#define CAR_HAS_4_WHEELS
//#define CAR_HAS_4_MECANUM_WHEELS
/*
 * Disable features of RobotCarBlueDisplay to save program memory
 */
//#define NO_PATH_INFO_PAGE               // Saves up to 1400 bytes
//#define NO_APPLICATON_INFO              // Saves up to 1504 bytes
//#define NO_RTTTL_FOR_CAR                // Saves up to 3654 bytes
/**************************
 * START OF CONFIGURATIONS
 **************************/
/*
 * L298 basic + IR distance + MPU6050 + VIN monitoring
 */
#if defined(L298_2WD_VIN_IR_IMU_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define CAR_HAS_MPU6050_IMU                 // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define L298_IR_DISTANCE_CONFIGURATION
#define CONFIG_NAME         "2WD + L298 basic + IR distance" // BASIC_CONFIG_NAME and CONFIG_NAME is printed by printConfigInfo()
#endif

/*
 * L298 basic + IR distance + VIN voltage divider
 * https://github.com/ArminJo/PWMMotorControl/blob/master/pictures/L298Car_TopView_small.jpg
 */
#if defined(L298_2WD_VIN_IR_DISTANCE_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define L298_2WD_2LI_ION_BASIC_CONFIGURATION
#define CONFIG_NAME         " + IR distance + VIN divider"
#endif

/*
 * BASIC CONFIGURATION
 * Lafvin 2WD model using L298 bridge. Uno board has series diode at power jack connector. + 2 Li-ion.
 * https://de.aliexpress.com/item/32816490316.html
 */
#if defined(L298_2WD_2LI_ION_BASIC_CONFIGURATION)
#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define BASIC_CONFIG_NAME   "2WD + L298 + 2 Li-ion"
#endif

/*
 * BASIC CONFIGURATION
 * Lafvin 2WD model using L298 bridge. Uno board has series diode at power jack connector + 4 AA batteries.
 * https://de.aliexpress.com/item/32816490316.html
 */
#if defined(L298_4WD_4AA_BASIC_CONFIGURATION)
#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define BASIC_CONFIG_NAME   "2WD + L298 + 4 AA"
#endif

/*
 * BASIC CONFIGURATION
 * China ZK-4WD 4WD model using L298 bridge. Uno board has series diode at power jack connector + 4 AA batteries.
 * https://de.aliexpress.com/item/33015596159.html (ZK-4WD / Four rounds kit / 4WD KIT1)
 */
#if defined(L298_4WD_BASIC_CONFIGURATION)
#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
#define CAR_HAS_4_WHEELS
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define BASIC_CONFIG_NAME   "4WD + L298 + 4 AA"
#endif

#if defined(TBB6612_4WD_4AA_VIN_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define TBB6612_4WD_4AA_BASIC_CONFIGURATION
#define CONFIG_NAME         " + VIN divider"
#endif

#if defined(TBB6612_4WD_4AA_FULL_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define NO_APPLICATON_INFO              // Saves up to 1504 bytes
#define TBB6612_4WD_4AA_BASIC_CONFIGURATION
#define CONFIG_NAME         " + VIN divider + MPU6050"
#endif

/*
 * BASIC CONFIGURATION
 * China ZK-4WD model with L298 bridge replaced by TB6612 breakout board (https://de.aliexpress.com/item/32657848225.html).
 * Uno board with series diode at VIN connector + 4 AA batteries.
 * https://de.aliexpress.com/item/33015596159.html (Four rounds kit / 4WD KIT1)
 */
#if defined(TBB6612_4WD_4AA_BASIC_CONFIGURATION)
#define CAR_HAS_4_WHEELS
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define BASIC_CONFIG_NAME   "4WD + TB6612 + 4 AA"
#endif

#if defined(TBB6612_4WD_2LI_ION_FULL_CONFIGURATION)
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION
#define CONFIG_NAME         " + VIN divider"
#endif

/*
 * TBB6612_4WD_4AA_BASIC_CONFIGURATION with 2 Li-ion instead of 4AA. Connect to UNO power jack!
 * Uno board has series diode at power jack connector.
 */
#if defined(TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION)
#define CAR_HAS_4_WHEELS
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define BASIC_CONFIG_NAME   "4WD + TB6612 + 2 Li-ion"
#endif
/*
 * Basic 2WD + VL53L1X TimeOfFlight sensor
 */
#if defined(MOTOR_SHIELD_2WD_TOF_CONFIGURATION)
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + TOF distance"
#endif

/*
 * Basic 2WD + encoder + VL53L1X TimeOfFlight sensor
 */
#if defined(MOTOR_SHIELD_2WD_ENCODER_TOF_CONFIGURATION)
#define CAR_HAS_ENCODERS                // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + encoder + TOF distance"
#endif

/*
 * The maximum layout of my smart 2WD robot car
 * Basic + VL53L1X TimeOfFlight + encoder + MPU6050
 */
#if defined(MOTOR_SHIELD_2WD_FULL_CONFIGURATION)
#define CAR_HAS_ENCODERS                // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_TOF_DISTANCE_SENSOR     // Use a VL53L1X TimeOfFlight distance sensor
#define CAR_HAS_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define USE_LIGHTWEIGHT_SERVO_LIBRARY   // LightweightServo library can control servos at pin 9 and 10. Saves up to 710 bytes program memory.
#define MOTOR_SHIELD_2WD_BASIC_CONFIGURATION
#define NO_APPLICATON_INFO              // Saves up to 1504 bytes
#define NO_RTTTL_FOR_CAR                // Saves up to 3654 bytes
#define CONFIG_NAME         " + encoder + TOF distance + MPU6050"
#endif

/*
 * BASIC CONFIGURATION
 * The basic layout of my smart 2WD robot car with 2 LiPo's instead of 4 AA
 * Shield + 2 LiPo's + VIN voltage divider + servo head down
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg
 */
#if defined(MOTOR_SHIELD_2WD_BASIC_CONFIGURATION)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#define BASIC_CONFIG_NAME   "2WD + Motor shield,TB6612  + 2 Li-ion + VIN divider + servo head down"
#endif

/*
 * The maximum layout of my red smart 4WD robot car
 * Basic + IR distance + encoder
 */
#if defined(MOTOR_SHIELD_4WD_FULL_CONFIGURATION)
#define CAR_HAS_ENCODERS                // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_IR_DISTANCE_SENSOR      // Activate this if your car has an Sharp GP2Y0A21YK / 1080 IR distance sensor mounted
#define MOTOR_SHIELD_4WD_BASIC_CONFIGURATION
#define CONFIG_NAME         " + encoder + IR distance"
#endif

/*
 * BASIC CONFIGURATION
 * The basic layout of my red smart 4WD robot car with 2 LiPo's instead of 4 AA
 * Shield + 2 LiPo's + VIN voltage divider + servo head down
 */
#if defined(MOTOR_SHIELD_4WD_BASIC_CONFIGURATION)
#define CAR_HAS_4_WHEELS
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#define BASIC_CONFIG_NAME   "4WD + Motor shield,TB6612  + 2 Li-ion + VIN divider + servo head down"
#endif

/*
 * BASIC CONFIGURATION
 * Nano Breadboard version with Arduino NANO, without shield and with pan/tilt servo and MPU camera and laser
 */
#if defined(BREADBOARD_4WD_FULL_CONFIGURATION)
#define CAR_IS_NANO_BASED               // We have an Arduino NANO instead of an UNO resulting in a different pin layout.
#define CAR_HAS_4_WHEELS
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define CAR_HAS_PAN_SERVO
#define CAR_HAS_TILT_SERVO
#define CAR_HAS_CAMERA
#define CAR_HAS_LASER
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define CAR_HAS_ENCODERS                // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
#define CAR_HAS_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
//#define NO_PATH_INFO_PAGE               // Saves up to 1400 bytes
//#define NO_APPLICATON_INFO              // Saves up to 1504 bytes
#define NO_RTTTL_FOR_CAR                // Saves up to 3654 bytes
#define BASIC_CONFIG_NAME   "4WD + Breadboard TB6612  + 2 Li-ion + VIN divider + servo head down + MPU6050"
#endif

/*
 * Nano Breadboard version with Arduino NANO, without shield and 4 mecanum wheels + US distance + servo
 */
#if defined(MECANUM_DISTANCE_CONFIGURATION)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN // Activate this, if the distance servo is mounted head down to detect small obstacles.
#define MECANUM_BASIC_CONFIGURATION
#define CONFIG_NAME   " + US distance + servo head down"
#endif

/*
 * BASIC CONFIGURATION
 * Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels
 */
#if defined(MECANUM_BASIC_CONFIGURATION)
#define CAR_IS_NANO_BASED               // We have an Arduino NANO instead of an UNO resulting in a different pin layout.
#define CAR_HAS_4_MECANUM_WHEELS
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage.
#define BASIC_CONFIG_NAME   "Breadboard TB6612  + 2 Li-ion + VIN divider + 4 mecanum wheels"
#endif

/*
 * BASIC CONFIGURATION
 * Car controlled by an ESP32-Cam module
 */
#if defined(CAR_IS_ESP32_CAM_BASED)
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define BASIC_CONFIG_NAME   "ESP32-Cam based"
#endif

/*
 * Default BASIC CONFIGURATION
 */
#if !defined(BASIC_CONFIG_NAME)         // use L298_2WD_4AA_BASIC_CONFIGURATION as default
#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
#define VIN_VOLTAGE_CORRECTION 0.81     // Correction for the series SI-diode in the VIN line of the UNO board
#define CAR_HAS_US_DISTANCE_SENSOR      // A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars)
#define CAR_HAS_DISTANCE_SERVO          // Distance sensor is mounted on a pan servo (default for most China smart cars)
#define BASIC_CONFIG_NAME   "2WD + L298 + 4 AA"
#endif

/*
 * Some rules
 */
#if defined(CAR_HAS_US_DISTANCE_SENSOR) || defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
#define CAR_HAS_DISTANCE_SENSOR         // At least one distance sensor mounted on a pan servo is available
#endif

#if defined(CAR_HAS_DISTANCE_SERVO) || defined(CAR_HAS_PAN_SERVO) || defined(CAR_HAS_TILT_SERVO)
#define CAR_HAS_SERVO                   // At least one servo is mounted on the car
#endif

#endif // _ROBOT_CAR_CONFIGURATIONS_H
