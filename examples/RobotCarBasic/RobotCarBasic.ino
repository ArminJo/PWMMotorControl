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

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_COMPENSATION_RIGHT    0

#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD) // enable it in PWMDCMotor.h
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

#define PIN_DISTANCE_SERVO         10 // Servo Nr. 2 on Adafruit Motor Shield

#define PIN_BUZZER                 12

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
     * You will need to change these values according to your motor, wheels and motor supply voltage.
     */
    RobotCarMotorControl.setValuesForFixedDistanceDriving(DEFAULT_START_SPEED, DEFAULT_DRIVE_SPEED, SPEED_COMPENSATION_RIGHT); // Set compensation
    // set factor for converting distance to drive time
    RobotCarMotorControl.setDistanceToTimeFactorForFixedDistanceDriving(DEFAULT_DISTANCE_TO_TIME_FACTOR); // 300

    /*
     * Set US servo to forward position
     */
    DistanceServo.attach(PIN_DISTANCE_SERVO);
    DistanceServo.write(90);

    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    tone(PIN_BUZZER, 2200, 100);
    delay(2000);
    DistanceServo.write(120);
    delay(500);
    DistanceServo.write(60);
    delay(500);
    DistanceServo.write(90);
    delay(2000);

    RobotCarMotorControl.setSpeedCompensated(DEFAULT_DRIVE_SPEED, DIRECTION_FORWARD);
}

void loop() {

    /*
     * Drive until distance too low, then stop, go back, turn random amount and drive again.
     */
    unsigned int tCentimeter = getUSDistanceAsCentiMeter();
    Serial.print("US distance=");
    Serial.print(tCentimeter);
    Serial.println(" cm");

    if (tCentimeter < 20) {
        RobotCarMotorControl.stopMotors();
        delay(1000);
        RobotCarMotorControl.setSpeedCompensated(DEFAULT_DRIVE_SPEED, DIRECTION_BACKWARD);
        delay(200);
        RobotCarMotorControl.stopMotors();
        delay(1000);

        int tTurnValueDegree = random(20, 180);
        Serial.print("Turn ");
        Serial.print(tTurnValueDegree);
        Serial.println(" degree");
        RobotCarMotorControl.rotateCar(tTurnValueDegree, TURN_IN_PLACE);
        delay(1000);
        RobotCarMotorControl.setSpeedCompensated(DEFAULT_DRIVE_SPEED, DIRECTION_FORWARD);
    }

    delay(50);
}

