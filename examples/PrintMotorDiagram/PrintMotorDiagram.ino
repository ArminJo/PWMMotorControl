/*
 *  PrintMotorDiagram.cpp
 *  Prints PWM, distance and speed diagram of an encoder motor.
 *
 *
 *  Copyright (C) 2020-2021  Armin Joachimsmeyer
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
/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "CarPWMMotorControl.hpp"
 */
#define USE_ENCODER_MOTOR_CONTROL  // Activate this if you have encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD  // Activate this if you use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
//#define USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD  // Activate this to force using of Adafruit library. Requires 694 bytes program memory.
#define VIN_2_LIPO                 // Activate this, if you use 2 LiPo Cells (around 7.4 volt) as Motor supply.
//#define VIN_1_LIPO                 // Or if you use a Mosfet bridge, 1 LIPO (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define MOSFET_BRIDGE_USED  // Activate this, if you use a (recommended) mosfet bridge instead of a L298 bridge, which has higher losses.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP  // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program space.
#include "EncoderMotor.hpp"

#include "PinDefinitionsAndMore.h"

EncoderMotor MotorUnderTest;

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
//    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Print caption
    Serial.println(F("PWM Speed Average Encoder"));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // For Adafruit Motor Shield v2
    MotorUnderTest.init(2, RIGHT_MOTOR_INTERRUPT);
#else
    MotorUnderTest.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT);
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
//    for (uint8_t tSpeed = 0; tSpeed < 250; ++tSpeed) {
//        MotorUnderTest.setSpeed(tSpeed, sMotorDirection);
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
    for (uint8_t tPWM = 0; tPWM < 249; ++tPWM) {
        MotorUnderTest.setSpeedPWM(tPWM, sMotorDirection);
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
        MotorUnderTest.setSpeedPWM(tPWM, sMotorDirection);
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
    Serial.print(F("PWM Speed Average Encoder "));
    Serial.print(F(" Start_PWM_Speed="));
    Serial.print(tStartPWM);
    Serial.print('_');
    Serial.print(tStartSpeed);
    Serial.print(F("|Max_Speed="));
    Serial.print(tMaxSpeed);
    Serial.print(F("|Stop="));
    Serial.print(tStopPWM);
    Serial.print('_');
    Serial.print(tStopSpeed);
    Serial.println();

    /*
     * switch direction
     */
    delay(10000);
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
}
