/*
 *  RobotCar.cpp
 *
 *  Simple motor control for RobotCar. It tries to drive a square.
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

#define VERSION_EXAMPLE "1.0"

/*
 * Pins for direct motor control with PWM and full bridge
 * Pins 9 + 10 are reserved for Servo
 * 2 + 3 are reserved for encoder input
 */
#define PIN_LEFT_MOTOR_FORWARD      4
#define PIN_LEFT_MOTOR_BACKWARD     7
#define PIN_LEFT_MOTOR_PWM          5 // Must be PWM capable

#define PIN_RIGHT_MOTOR_FORWARD     8
#define PIN_RIGHT_MOTOR_BACKWARD   12 // Pin 9 is already reserved for distance servo
#define PIN_RIGHT_MOTOR_PWM         6 // Must be PWM capable

#define PIN_SPEAKER                11

//Car Control
CarMotorControl RobotCarMotorControl;

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
#ifdef PLOTTER_OUTPUT
    Serial.println("Distance[cm]");
#else
    Serial.println("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);
#endif

    RobotCarMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD, PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);

    tone(PIN_SPEAKER, 2200, 100);
    delay(2000);
}

void loop() {

    for (int i = 0; i < 4; ++i) {
        RobotCarMotorControl.goDistanceCentimeter(40);
        RobotCarMotorControl.rotateCar(90, TURN_FORWARD, true);
    }
    delay(5000);

}

