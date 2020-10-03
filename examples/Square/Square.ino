/*
 *  Square.cpp
 *  Example for driving a 50 cm square using CarMotorControl class
 *
 *  Created on: 19.09.2020
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include "CarMotorControl.h"

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) // enable / disable it in PWMDCMotor.h
/*
 * Pins for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 * Pins 9 + 10 are already used for Servo
 * 2 + 3 are already used for encoder input
 */
#define PIN_LEFT_MOTOR_FORWARD      4
#define PIN_LEFT_MOTOR_BACKWARD     7
#define PIN_LEFT_MOTOR_PWM          5 // Must be PWM capable

#define PIN_RIGHT_MOTOR_FORWARD     8
#define PIN_RIGHT_MOTOR_BACKWARD   12 // Pin 9 is already reserved for distance servo
#define PIN_RIGHT_MOTOR_PWM         6 // Must be PWM capable
#endif

CarMotorControl RobotCarMotorControl;
#define SIZE_OF_SQUARE  50

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)
    delay(2000); // To be able to connect Serial monitor after reset and before first printout
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    // For Adafruit Motor Shield v2
    RobotCarMotorControl.init();
#else
    RobotCarMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);
#endif

    /*
     * You will need to change these values according to your motor, wheels and motor supply voltage.
     */
    RobotCarMotorControl.setValuesForFixedDistanceDriving(DEFAULT_START_SPEED, DEFAULT_DRIVE_SPEED, 0); // Set compensation to 0
    // set Factor for 2 LIPOS
    RobotCarMotorControl.setDistanceToTimeFactorForFixedDistanceDriving(DEFAULT_DISTANCE_TO_TIME_FACTOR);
#if defined(CAR_HAS_4_WHEELS)
    RobotCarMotorControl.setFactorDegreeToCount(FACTOR_DEGREE_TO_COUNT_4WD_CAR_DEFAULT);
#else
    RobotCarMotorControl.setFactorDegreeToCount(FACTOR_DEGREE_TO_COUNT_2WD_CAR_DEFAULT);
#endif
    delay(2000);
}

void loop() {
    static uint8_t sMotorDirection = DIRECTION_FORWARD;

    /*
     * Try the default minimum speed (from PWMDCMotor.h), at which the motor starts to move.
     */
    RobotCarMotorControl.goDistanceCentimeter(SIZE_OF_SQUARE, sMotorDirection);
    delay(1000);               // wait for a second
    /*
     * Now set speed to the default maximum speed (from PWMDCMotor.h), at which the motor moves for fixed distance driving.
     */
    RobotCarMotorControl.rotateCar(90, sMotorDirection);

    /*
     * switch direction
     */
    sMotorDirection = oppositeDIRECTION(sMotorDirection);
    delay(5000);
}
