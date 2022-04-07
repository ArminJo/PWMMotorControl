/*
 *  TestMotorWithIMU.cpp
 *  Tests Prints PWM, distance and speed diagram of the right (encoder) motor of a car.
 *  Encoder and IMU data are printed simultaneously, to compare and to detect slipping
 *
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "RobotCarPWMMotorControl.hpp"
 */
#define USE_ENCODER_MOTOR_CONTROL   // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD   // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LIPO                  // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LIPO                  // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define USE_L298_BRIDGE            // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP         // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
//#define DO_NOT_SUPPORT_AVERAGE_SPEED // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.

#include "CarPWMMotorControl.hpp"

#include "IMUCarData.hpp"

#include "RobotCarPinDefinitionsAndMore.h"

//#define ENABLE_EXTRA_NON_PLOTTER_OUTPUT // Generate verbose output for SerialMonitor but this not compatible with Arduino Plotter
#define PRINTS_PER_SECOND 50

unsigned long LastPrintMillis;

void delayAndPrintData(uint8_t aDataSetsToPrint, uint16_t aPeriodMillis, bool aUseRamp);

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Print info
    PWMDcMotor::printCompileOptions(&Serial);
#endif

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    // For Adafruit Motor Shield v2
    RobotCarPWMMotorControl.init();
#else
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#endif

    RobotCarPWMMotorControl.IMUData.delayAndReadIMUCarDataData(3000);
}

void loop() {
    static uint8_t sDirection = DIRECTION_FORWARD;

#if defined(USE_ENCODER_MOTOR_CONTROL)
    RobotCarPWMMotorControl.rightCarMotor.printEncoderDataCaption(&Serial);
#endif
    RobotCarPWMMotorControl.IMUData.printIMUCarDataCaption(&Serial);
#if ! defined(USE_ENCODER_MOTOR_CONTROL)
    Serial.print(F("PWM[2] "));
#endif
    Serial.print(
            F(
                    STR(PRINTS_PER_SECOND) "_values/s_at_" STR(SAMPLE_RATE) "_samples/s_offset=" STR(RAMP_VALUE_OFFSET_MILLIVOLT) "_mV_delta="));
    Serial.print(RAMP_VALUE_DELTA);
    Serial.print(F("_decel="));
    Serial.print(RAMP_DECELERATION_TIMES_2 / 2);
    Serial.println(F("mm/s^2"));

    /*
     * Set to drive speed without ramp
     */
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    Serial.print(F("Set speed to DEFAULT_DRIVE_SPEED_PWM="));
    Serial.println(DEFAULT_DRIVE_SPEED_PWM);
#endif
    uint8_t sSpeedPWM = DEFAULT_DRIVE_SPEED_PWM;

    uint8_t tLoopIndex = 0;

    /*
     * Start and stop car first with and second without ramp
     * Start with DEFAULT_DRIVE_SPEED_PWM and double speed for next turn until MAX_SPEED_PWM
     */
    while (true) {
        bool tUseRamp = true;
        for (uint8_t i = 0; i < 2; ++i) {

            RobotCarPWMMotorControl.IMUData.resetOffsetDataAndWait();
#if defined(USE_ENCODER_MOTOR_CONTROL)
            RobotCarPWMMotorControl.rightCarMotor.resetEncoderControlValues();
#endif

            if (tUseRamp) {
                /*
                 * Go distance - implies ramp
                 */
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
                Serial.print(F("Go distance[mm]="));
                Serial.println((tLoopIndex + 1) * 200); // 200, 400, 600
#endif
                RobotCarPWMMotorControl.startGoDistanceMillimeter(sSpeedPWM, (tLoopIndex + 1) * 200, sDirection);
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
                Serial.print(F("Go distance[mm]="));
                Serial.println(RobotCarPWMMotorControl.CarRequestedDistanceMillimeter);
#endif
                // print 20 data sets after stopping
                delayAndPrintData(40, 1000 / PRINTS_PER_SECOND, tUseRamp);
            } else {
                /*
                 * Set speed, wait and stop - no ramp :-)
                 */
                RobotCarPWMMotorControl.setSpeedPWMAndDirection(sSpeedPWM, sDirection);
                delayAndPrintData(40, 1000 / PRINTS_PER_SECOND, tUseRamp);
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
                Serial.println(F("Stop motors"));
#endif
                RobotCarPWMMotorControl.setStopMode(STOP_MODE_BRAKE); // just to be sure
                RobotCarPWMMotorControl.setSpeedPWMAndDirection(0);
                delayAndPrintData(20, 1000 / PRINTS_PER_SECOND, tUseRamp);
            }

            tUseRamp = false;
            RobotCarPWMMotorControl.IMUData.delayAndReadIMUCarDataData(1000);

        }
        if (sSpeedPWM == MAX_SPEED_PWM) {
            break; // after last loop
        }
        // double speed for next turn
        if (sSpeedPWM <= MAX_SPEED_PWM / 2) {
            sSpeedPWM *= 2;
        } else {
            // last loop with MAX_SPEED
            sSpeedPWM = MAX_SPEED_PWM;
        }
        tLoopIndex++;

#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
        Serial.print(F("Set speed to:"));
        Serial.println(sSpeedPWM);
#endif
        RobotCarPWMMotorControl.IMUData.delayAndReadIMUCarDataData(1000);

    }
    RobotCarPWMMotorControl.IMUData.delayAndReadIMUCarDataData(5000);
    /*
     * switch direction
     */
    sDirection = oppositeDIRECTION(sDirection);
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    Serial.print(F("Switch direction to:"));
    Serial.println(sDirection);
#endif
}

/*
 * Prints values, if a new value is available
 * @param aDataSetsToPrint if aUseRamp is true, number of data sets AFTER stop of car
 * @return true if printed.
 */
void delayAndPrintData(uint8_t aDataSetsToPrint, uint16_t aPeriodMillis, bool aUseRamp) {

    for (uint_fast8_t i = 0; i < aDataSetsToPrint;) {
#if defined(USE_ENCODER_MOTOR_CONTROL)
        if (RobotCarPWMMotorControl.rightCarMotor.printEncoderDataPeriodically(&Serial, aPeriodMillis)) {
            RobotCarPWMMotorControl.IMUData.readCarDataFromMPU6050Fifo();
            RobotCarPWMMotorControl.IMUData.printIMUCarData(&Serial);
            Serial.println();
            i++;
        }
#else
        if (RobotCarPWMMotorControl.IMUData.printIMUCarDataDataPeriodically(&Serial, aPeriodMillis)) {
            Serial.println(RobotCarPWMMotorControl.rightCarMotor.RequestedSpeedPWM / 2); // = PWM, scale it for plotter
            i++;
        }
#endif
        if (aUseRamp && RobotCarPWMMotorControl.updateMotors()) {
            // reset count as long as car is driving
            i = 0;
        }
    }
}
