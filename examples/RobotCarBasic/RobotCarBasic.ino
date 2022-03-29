/*
 *  RobotCar.cpp
 *
 *  Template for your RobotCar control.
 *  Currently implemented is: drive until distance too low, then stop, and turn random amount.
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
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

//#define USE_ADAFRUIT_MOTOR_SHIELD
/*
 * Car pin configuration
 */
#include "RobotCarPinDefinitionsAndMore.h"

#include "CarPWMMotorControl.hpp"

#if defined(ESP32)
#include "ESP32Servo.h"
#else
#include "Servo.h"
#endif
#include "HCSR04.h"

#define VERSION_EXAMPLE "1.0"

/*
 * Speed compensation to enable driving straight ahead.
 * If positive, this value is subtracted from the speed of the right motor -> the car turns slightly right.
 * If negative, -value is subtracted from the left speed -> the car turns slightly left.
 */
#define SPEED_PWM_COMPENSATION_RIGHT    0

Servo DistanceServo;

void simpleObjectAvoidance();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    RobotCarPWMMotorControl.init();
#else
#  if defined(USE_ENCODER_MOTOR_CONTROL)
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT,
    LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_INTERRUPT);
#  else
    RobotCarPWMMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#  endif
#endif

    /*
     * You will need to change these values according to your motor, wheels and motor supply voltage.
     */
    RobotCarPWMMotorControl.setDriveSpeedAndSpeedCompensationPWM(DEFAULT_DRIVE_SPEED_PWM, SPEED_PWM_COMPENSATION_RIGHT); // Set compensation
#if ! defined(USE_ENCODER_MOTOR_CONTROL)
    // set factor for converting distance to drive time
    RobotCarPWMMotorControl.setMillimeterPerSecondForFixedDistanceDriving(DEFAULT_MILLIMETER_PER_SECOND);
#endif

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
    delay(1000);
#if defined(USE_MPU6050_IMU)
    /*
     * Wait after pressing the reset button, or attaching the power
     * and then take offset values for 1/2 second
     */
    tone(PIN_BUZZER, 2200, 50);
    delay(100);
    RobotCarPWMMotorControl.initIMU();
    RobotCarPWMMotorControl.printIMUOffsets(&Serial);
    tone(PIN_BUZZER, 2200, 50);
#endif
    delay(1000);

}

void loop() {

    RobotCarPWMMotorControl.goDistanceMillimeter(200, DIRECTION_FORWARD);
    delay(2000);
    /*
     * Try to turn by 90 degree.
     */
    RobotCarPWMMotorControl.rotate(90, TURN_FORWARD);
    delay(2000);

}

void simpleObjectAvoidance() {
    /*
     * Drive until distance too low, then stop, go back, turn random amount and drive again.
     */
    unsigned int tCentimeter = getUSDistanceAsCentimeter();
    Serial.print("US distance=");
    Serial.print(tCentimeter);
    Serial.println(" cm");

    if (tCentimeter < 20) {
        /*
         * Distance too low here -> Stop and go backwards
         */
        RobotCarPWMMotorControl.stop();
        delay(1000);
        RobotCarPWMMotorControl.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_BACKWARD);
        delay(200);
        RobotCarPWMMotorControl.stop();
        delay(1000);

        /*
         * Turn random amount and drive again
         */
        int tTurnValueDegree = random(20, 180);
        Serial.print("Turn ");
        Serial.print(tTurnValueDegree);
        Serial.println(" degree");
        RobotCarPWMMotorControl.rotate(tTurnValueDegree, TURN_IN_PLACE);
        delay(1000);
        RobotCarPWMMotorControl.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_FORWARD);
    }

    delay(50); // Wait until next distance sample
}

