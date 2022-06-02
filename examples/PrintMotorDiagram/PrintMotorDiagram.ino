/*
 *  PrintMotorDiagram.cpp
 *  Prints PWM, distance and speed diagram of an encoder motor during ramp up and ramp down for Arduino Plotter.
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

//#define USE_CAR_PWM_CONTROL_INSTEAD_OF_ENCODER_MOTOR // runs motor without encoder

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "CarPWMMotorControl.hpp" or "EncoderMotor.hpp".
 */
#define USE_ENCODER_MOTOR_CONTROL   // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD   // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
//#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LI_ION                // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define USE_L298_BRIDGE            // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP         // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
//#define DO_NOT_SUPPORT_AVERAGE_SPEED // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.

#if defined (USE_CAR_PWM_CONTROL_INSTEAD_OF_ENCODER_MOTOR)
#include "CarPWMMotorControl.hpp"
#define MotorUnderTest RobotCar.rightCarMotor
#else
#include "EncoderMotor.hpp"
EncoderMotor MotorUnderTest;
#endif

#include "RobotCarPinDefinitionsAndMore.h"

//#define ENABLE_EXTRA_NON_PLOTTER_OUTPUT // Generate verbose output for SerialMonitor but this not compatible with Arduino Plotter

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));
#endif

    // Print caption
    Serial.println(F("PWM Speed[cm/s] Average[cm/s] EncoderCount"));

#if defined(USE_CAR_PWM_CONTROL_INSTEAD_OF_ENCODER_MOTOR)
#  if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    RobotCar.init();
#  else
    RobotCar.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#  endif
#else // defined(USE_CAR_PWM_CONTROL_INSTEAD_OF_ENCODER_MOTOR)
#  if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    // For Adafruit Motor Shield v2
    MotorUnderTest.init(2, RIGHT_MOTOR_INTERRUPT);
#  else
    MotorUnderTest.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT);
#  endif
#endif

    delay(2000);
}

#define DELAY_MILLIS_BETWEEN_CHANGE 20
void loop() {

    static uint8_t sMotorDirection = DIRECTION_FORWARD;

    MotorUnderTest.resetEncoderControlValues();
    uint8_t tStartPWM;
    int tLastSpeed;
    int tStartSpeed = 0;
    int tMaxSpeed = 0;
    uint8_t tStopPWM = 0;
    int tStopSpeed = 0;
    /*
     * Print value after each encoder count change and increase PWM every DELAY_MILLIS_BETWEEN_CHANGE ms
     */
//    for (uint_fast8_t tSpeed = 0; tSpeed < 250; ++tSpeed) {
//        MotorUnderTest.setSpeedPWM(tSpeed, sMotorDirection);
//        uint32_t tStartMillis = millis();
//        do {
//            if (MotorUnderTest.SensorValuesHaveChanged) {
//                MotorUnderTest.SensorValuesHaveChanged = false;
//                Serial.print(tSpeed);
//                Serial.print(' ');
//                Serial.print(MotorUnderTest.EncoderCount);
//                Serial.print(' ');
//                Serial.print(MotorUnderTest.getSpeed());
//                Serial.print(' ');
//                Serial.println(MotorUnderTest.getAverageSpeed());
//            }
//        } while (millis() - tStartMillis < DELAY_MILLIS_BETWEEN_CHANGE);
//    }
    /*
     * Increase PWM and print values every DELAY_MILLIS_BETWEEN_CHANGE ms
     */
    for (uint_fast8_t tPWM = 0; tPWM < 249; ++tPWM) {
        MotorUnderTest.setSpeedPWMAndDirection(tPWM, sMotorDirection);
        delay(DELAY_MILLIS_BETWEEN_CHANGE);
        Serial.print(tPWM);

        Serial.print(' ');
        int tSpeed = MotorUnderTest.getSpeed();
        if (tStartSpeed == 0 && tSpeed != 0) {
            tStartSpeed = tSpeed;
            tStartPWM = tPWM;
        }
        Serial.print(tSpeed);

        Serial.print(' ');
        Serial.print(MotorUnderTest.getAverageSpeed());

        Serial.print(' ');
        Serial.print(MotorUnderTest.EncoderCount);
        Serial.println();
    }

    // wait 1 second at max speed and print nothing
    int16_t tMaxEncoderCount = MotorUnderTest.EncoderCount;
    delay(1000);
    tMaxSpeed = MotorUnderTest.getAverageSpeed();
    MotorUnderTest.LastRideEncoderCount = 0;
    // and decrease
    for (int tPWM = 248; tPWM >= 0; tPWM--) {
        MotorUnderTest.setSpeedPWMAndDirection(tPWM, sMotorDirection);
        delay(DELAY_MILLIS_BETWEEN_CHANGE);
        Serial.print(tPWM);

        Serial.print(' ');
        int tSpeed = MotorUnderTest.getSpeed();
        if (tStopSpeed == 0 && tSpeed == 0) {
            tStopSpeed = tLastSpeed;
            tStopPWM = tPWM - 1;
        }
        Serial.print(tSpeed);
        tLastSpeed = tSpeed;

        Serial.print(' ');
        Serial.print(MotorUnderTest.getAverageSpeed());

        Serial.print(' ');
        Serial.print(tMaxEncoderCount - (int16_t) MotorUnderTest.LastRideEncoderCount);
        Serial.println();
    }

    /*
     * Print extended caption
     */
    Serial.print(F("PWM Speed Average EncoderCount "));
    Serial.print(F(" Start_PWM_Speed="));
    Serial.print(tStartPWM);
    Serial.print('_');
    Serial.print(tStartSpeed);
    Serial.print(F("|Max_AvgSpeed="));
    Serial.print(tMaxSpeed);
    Serial.print(F("|Stop="));
    Serial.print(tStopPWM);
    Serial.print('_');
    Serial.print(tStopSpeed);
    Serial.print(F("_Delay="));
    Serial.print(DELAY_MILLIS_BETWEEN_CHANGE);
    Serial.println();

    /*
     * switch direction
     */
    delay(10000);
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
}
