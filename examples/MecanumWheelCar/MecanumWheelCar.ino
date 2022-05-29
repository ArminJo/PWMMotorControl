/*
 *  MecanumWheelCar.cpp
 *  Example for showing all possible movements of a mecanum 4 wheel car.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "MecanumWheelCarPWMMotorControl.hpp"
 */
//#define USE_ADAFRUIT_MOTOR_SHIELD       // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#define VIN_2_LI_ION                    // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                    // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define USE_L298_BRIDGE                 // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
//#define DO_NOT_SUPPORT_AVERAGE_SPEED    // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.

#define CAR_HAS_4_MECANUM_WHEELS
#include "RobotCarPinDefinitionsAndMore.h"

//#include "CarPWMMotorControl.hpp"
#include "MecanumWheelCarPWMMotorControl.hpp"

#include "RobotCarUtils.hpp" // for initRobotCarPWMMotorControl

#define DEMO_SPEED                      DEFAULT_DRIVE_SPEED_PWM
#define DELAY_BETWEEN_MOVEMENTS         1000

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    initRobotCarPWMMotorControl();

    /*
     * Tone feedback for end of boot
     */
    tone(PIN_BUZZER, 2200, 100);
    RobotCarPWMMotorControl.setSpeedPWMAndDirection(0);
    delay(2000);

    /*
     * do demo
     */
    RobotCarPWMMotorControl.moveSqare(DEMO_SPEED, 1000);
    delay(4000);
    RobotCarPWMMotorControl.moveStar(DEMO_SPEED, 1000);
    delay(4000);
    RobotCarPWMMotorControl.moveTrapezium(DEMO_SPEED, 1000);
    delay(4000);

    /*
     * Do turns
     */
    Serial.println(F("Turn right"));
    RobotCarPWMMotorControl.rotate(-180, TURN_IN_PLACE);
//    RobotCarPWMMotorControl.setSpeedPWMAndDirectionAndDelay(DEMO_SPEED,
//    DIRECTION_STOP | DIRECTION_RIGHT | DIRECTION_TURN, 1000);
    delay(DELAY_BETWEEN_MOVEMENTS);

    Serial.println(F("Turn left"));
    RobotCarPWMMotorControl.rotate(180, TURN_IN_PLACE);
//    RobotCarPWMMotorControl.setSpeedPWMAndDirectionAndDelay(DEMO_SPEED,
//    DIRECTION_STOP | DIRECTION_LEFT | DIRECTION_TURN, 1000);
    delay(DELAY_BETWEEN_MOVEMENTS);

    Serial.println(F("Turn front right"));
    RobotCarPWMMotorControl.rotate(-90, TURN_FORWARD);
//    RobotCarPWMMotorControl.setSpeedPWMAndDirectionAndDelay(DEMO_SPEED,
//    DIRECTION_FORWARD | DIRECTION_RIGHT | DIRECTION_TURN, 1000);
    delay(DELAY_BETWEEN_MOVEMENTS);

    Serial.println(F("Turn front left"));
    RobotCarPWMMotorControl.rotate(90, TURN_FORWARD);
//    RobotCarPWMMotorControl.setSpeedPWMAndDirectionAndDelay(DEMO_SPEED,
//    DIRECTION_FORWARD | DIRECTION_LEFT | DIRECTION_TURN, 1000);
    delay(DELAY_BETWEEN_MOVEMENTS);

    Serial.println(F("Turn back left"));
    RobotCarPWMMotorControl.rotate(90, TURN_BACKWARD);
//    RobotCarPWMMotorControl.setSpeedPWMAndDirectionAndDelay(DEMO_SPEED,
//    DIRECTION_BACKWARD | DIRECTION_LEFT | DIRECTION_TURN, 1000);
    delay(DELAY_BETWEEN_MOVEMENTS);

    Serial.println(F("Turn back right"));
    RobotCarPWMMotorControl.rotate(-90, TURN_BACKWARD);
//    RobotCarPWMMotorControl.setSpeedPWMAndDirectionAndDelay(DEMO_SPEED,
//    DIRECTION_BACKWARD | DIRECTION_RIGHT | DIRECTION_TURN, 1000);
    delay(2000);

    Serial.println(F("End of setup"));
}

void loop() {
#if defined(MONITOR_VIN_VOLTAGE)
    checkVinPeriodicallyAndPrintIfChanged();
#endif
    delay(2000);
}
