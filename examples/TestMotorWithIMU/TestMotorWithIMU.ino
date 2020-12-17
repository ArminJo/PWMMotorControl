/*
 *  TestMotorDSettings.cpp
 *  Tests Prints PWM, distance and speed diagram of an encoder motor.
 *
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include "CarMotorControl.h"
#include "IMUCarData.h"

#define ONLY_ARDUINO_PLOTTER_OUTPUT
#define PRINTS_PER_SECOND 50

#if !defined(USE_MPU6050_IMU)
#error For this example to run, USE_MPU6050_IMU must be commented out / defined in CarMotorControl.h line 36
#endif

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) // enable / disable it in PWMDCMotor.h
/*
 * Pins for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 * 2 + 3 are reserved for encoder input
 */
#define PIN_RIGHT_MOTOR_FORWARD     4 // IN4 <- Label on the L298N board
#define PIN_RIGHT_MOTOR_BACKWARD    7 // IN3
#define PIN_RIGHT_MOTOR_PWM         5 // ENB - Must be PWM capable

#define PIN_LEFT_MOTOR_FORWARD      9 // IN1
#define PIN_LEFT_MOTOR_BACKWARD     8 // IN2
#define PIN_LEFT_MOTOR_PWM          6 // ENA - Must be PWM capable
#endif

#define LEFT_MOTOR_INTERRUPT     INT1 // Pin 3

CarMotorControl CarMotorControl;

unsigned long LastPrintMillis;

void printData(uint8_t aDataSetsToPrint, uint16_t aPeriodMillis, bool aUseRamp);

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)
    delay(2000); // To be able to connect Serial monitor after reset and before first printout
#endif
    // Just to know which program is running on my Arduino
#ifndef ONLY_ARDUINO_PLOTTER_OUTPUT
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Print info
    PWMDcMotor::printSettings(&Serial);
#endif

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // For Adafruit Motor Shield v2
    CarMotorControl.init();
#else
    CarMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);
#endif

    CarMotorControl.IMUData.delayAndReadIMUCarDataData(3000);
}

void loop() {
    static uint8_t sDirection = DIRECTION_FORWARD;

#if defined(USE_ENCODER_MOTOR_CONTROL)
    CarMotorControl.rightCarMotor.printEncoderDataCaption(&Serial);
#endif
    CarMotorControl.IMUData.printIMUCarDataCaption(&Serial);
#if ! defined(USE_ENCODER_MOTOR_CONTROL)
    Serial.print(F("PWM[2] "));
#endif
    Serial.print(
            F(
                    STR(PRINTS_PER_SECOND) "_values/s_at_" STR(SAMPLE_RATE) "_samples/s_offset=" STR(RAMP_VALUE_UP_OFFSET_MILLIVOLT) "_/_" STR(RAMP_VALUE_DOWN_OFFSET_MILLIVOLT) "_mV_up="));
    Serial.print(RAMP_UP_VALUE_DELTA);
    Serial.print(F("_down="));
    Serial.print(RAMP_DOWN_VALUE_DELTA);
    Serial.print(F("_decel="));
    Serial.print(RAMP_DECELERATION_TIMES_2 / 2);
    Serial.println(F("mm/s^2"));

    /*
     * Set to drive speed without ramp
     */
#ifndef ONLY_ARDUINO_PLOTTER_OUTPUT
    Serial.print(F("Set speed to DEFAULT_DRIVE_SPEED="));
    Serial.println(DEFAULT_DRIVE_SPEED);
#endif
    uint8_t sSpeedPWM = DEFAULT_DRIVE_SPEED_PWM;

    uint8_t tLoopIndex = 0;

    while (true) {
        bool tUseRamp = true;
        for (uint8_t i = 0; i < 2; ++i) {

            CarMotorControl.IMUData.resetOffsetDataAndWait();
#if defined(USE_ENCODER_MOTOR_CONTROL)
            CarMotorControl.rightCarMotor.resetEncoderControlValues();
#endif

            if (tUseRamp) {
#ifndef ONLY_ARDUINO_PLOTTER_OUTPUT
                Serial.print(F("Go distance[mm]="));
                Serial.println((tLoopIndex + 1) * 200); // 200, 400, 600
#endif
                CarMotorControl.startGoDistanceMillimeter(sSpeedPWM, (tLoopIndex + 1) * 200, sDirection);
                Serial.print(F("Go distance[mm]="));
                Serial.println(CarMotorControl.CarRequestedDistanceMillimeter);
                // print 20 data sets after stopping
                printData(40, 1000 / PRINTS_PER_SECOND, tUseRamp);
            } else {
                CarMotorControl.setSpeedPWM(sSpeedPWM, sDirection);
                printData(40, 1000 / PRINTS_PER_SECOND, tUseRamp);
#ifndef ONLY_ARDUINO_PLOTTER_OUTPUT
                Serial.println(F("Stop motors"));
#endif
                CarMotorControl.setStopMode(MOTOR_BRAKE); // just to be sure
                CarMotorControl.setSpeedPWM(0);
                printData(20, 1000 / PRINTS_PER_SECOND, tUseRamp);
            }

            tUseRamp = false;
            CarMotorControl.IMUData.delayAndReadIMUCarDataData(1000);

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

#ifndef ONLY_ARDUINO_PLOTTER_OUTPUT
        Serial.print(F("Set speed to:"));
        Serial.println(sSpeedPWM);
#endif
        CarMotorControl.IMUData.delayAndReadIMUCarDataData(1000);

    }
    CarMotorControl.IMUData.delayAndReadIMUCarDataData(5000);
    /*
     * switch direction
     */
    sDirection = oppositeDIRECTION(sDirection);
#ifndef ONLY_ARDUINO_PLOTTER_OUTPUT
    Serial.print(F("Switch direction to:"));
    Serial.println(sDirection);
#endif
}

//uint8_t sSpeedHistoryArray[AVERAGE_SPEED_BUFFER_SIZE];
/*
 * Prints values, if a new value is available
 * @return true if printed.
 */
void printData(uint8_t aDataSetsToPrint, uint16_t aPeriodMillis, bool aUseRamp) {

    for (uint8_t i = 0; i < aDataSetsToPrint;) {
        if (aUseRamp) {
            if (CarMotorControl.updateMotors()) {
                // do not count as long as car is driving
                i = 0;
            }
        }
#if defined(USE_ENCODER_MOTOR_CONTROL)

        if (CarMotorControl.rightCarMotor.printEncoderDataPeriodically(&Serial, aPeriodMillis)) {
            CarMotorControl.IMUData.readCarDataFromMPU6050Fifo();
            CarMotorControl.IMUData.printIMUCarData(&Serial);
            Serial.println();
            i++;
        }
#else
        if (CarMotorControl.IMUData.printIMUCarDataDataPeriodically(&Serial, aPeriodMillis)) {
            Serial.println(CarMotorControl.rightCarMotor.CurrentSpeedPWM / 2); // = PWM, scale it for plotter
            i++;
        }
#endif
    }
}
