/*
 *  RobotCar.cpp
 *
 *  Template for your RobotCar control.
 *  Currently implemented is: drive until distance too low, then stop, and turn random amount.
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "CarMotorControl.h"
#include "Servo.h"
#include "HCSR04.h"

#define VERSION_EXAMPLE "1.0"

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) // enable it in PWMDCMotor.h
/*
 * Pins for direct motor control with PWM and a dual full bridge e.g. TB6612 or L298.
 * Pins 9 + 10 are reserved for Servo
 * 2 + 3 are reserved for encoder input
 */
#define PIN_RIGHT_MOTOR_FORWARD     4 // IN4 <- Label on the L298N board
#define PIN_RIGHT_MOTOR_BACKWARD    7 // IN3
#define PIN_RIGHT_MOTOR_PWM         5 // ENB - Must be PWM capable

#define PIN_LEFT_MOTOR_FORWARD     12 // IN1 - Pin 9 is already reserved for distance servo
#define PIN_LEFT_MOTOR_BACKWARD     8 // IN2
#define PIN_LEFT_MOTOR_PWM          6 // ENA - Must be PWM capable
#endif

#define PIN_DISTANCE_SERVO          9 // Servo Nr. 2 on Adafruit Motor Shield

#define PIN_BUZZER                 11

#define PIN_TRIGGER_OUT            A0 // Connections on the Arduino Sensor Shield
#define PIN_ECHO_IN                A1

//Car Control
CarMotorControl RobotCarMotorControl;

Servo DistanceServo;

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    RobotCarMotorControl.init();
#else
    RobotCarMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);
#endif

    /*
     * Set US servo to forward position
     */
    DistanceServo.attach(PIN_DISTANCE_SERVO);
    DistanceServo.write(90);

    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    tone(PIN_BUZZER, 2200, 100);
    delay(2000);
    RobotCarMotorControl.setSpeedCompensated(DEFAULT_DRIVE_SPEED, DIRECTION_FORWARD);
}

void loop() {

    /*
     * Drive until distance too low, then stop, go back, turn random amount and drive again.
     */
    unsigned int tCentimeter = getUSDistanceAsCentiMeter();

    if (tCentimeter < 20) {
        RobotCarMotorControl.stopMotors();
        delay(2000);
        RobotCarMotorControl.setSpeedCompensated(DEFAULT_DRIVE_SPEED, DIRECTION_BACKWARD);
        delay(400);
        RobotCarMotorControl.stopMotors();
        delay(2000);
        RobotCarMotorControl.rotateCar(random(180), TURN_IN_PLACE);
        delay(2000);
        RobotCarMotorControl.setSpeedCompensated(DEFAULT_DRIVE_SPEED, DIRECTION_FORWARD);
    }

    delay(50);
}

