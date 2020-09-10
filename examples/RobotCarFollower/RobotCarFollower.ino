/*
 *  RobotCarFollower.cpp
 *
 *  Enables follower mode driving of a 2 or 4 wheel car with an Arduino and an Adafruit Motor Shield V2.
 *  To find the target to follow, a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo scans the area on demand.
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
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
#endif

#define PIN_DISTANCE_SERVO          9 // Servo Nr. 2 on Adafruit Motor Shield

#define PIN_SPEAKER                11

#define PIN_TRIGGER_OUT            A0 // Connections on the Arduino Sensor Shield
#define PIN_ECHO_IN                A1

//Car Control
CarMotorControl RobotCarMotorControl;
Servo DistanceServo;
//#define PLOTTER_OUTPUT // Comment this out, if you want to ses the result of the US distance sensor in Arduino plotter

unsigned int getDistanceAndPlayTone();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
#ifdef PLOTTER_OUTPUT
    Serial.println(F("Distance[cm]"));
#else
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
#endif

#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    RobotCarMotorControl.init();
#else
    RobotCarMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
    PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM);
#endif
    RobotCarMotorControl.setValuesForFixedDistanceDriving(80, 200, 0);

    DistanceServo.attach(PIN_DISTANCE_SERVO);
    DistanceServo.write(90);
    initUSDistancePins(PIN_TRIGGER_OUT, PIN_ECHO_IN);

    tone(PIN_SPEAKER, 2200, 100);
    delay(200);
    tone(PIN_SPEAKER, 2200, 100);

    delay(5000);
    RobotCarMotorControl.startCarAndWaitForDriveSpeed(DIRECTION_FORWARD);
}

bool sFoundPerson = false;
void loop() {

    unsigned int tCentimeter = getDistanceAndPlayTone();

    if (tCentimeter > 30) {
        /*
         * TODO check if distance too high, then search person in another direction
         */
#ifndef PLOTTER_OUTPUT
        Serial.println(F("Go forward"));
#endif
        RobotCarMotorControl.startCarAndWaitForDriveSpeed(DIRECTION_FORWARD);
    } else if (tCentimeter < 22) {
#ifndef PLOTTER_OUTPUT
        Serial.println(F("Go backward"));
#endif
        RobotCarMotorControl.startCarAndWaitForDriveSpeed(DIRECTION_BACKWARD);
    } else {
        sFoundPerson = true;
#ifndef PLOTTER_OUTPUT
        Serial.println(F("Stop"));
#endif
        RobotCarMotorControl.stopMotors(false);
    }
    delay(40); // the IR sensor takes 39 ms for one measurement
}

unsigned int getDistanceAndPlayTone() {
    /*
     * Get distance
     */
    unsigned int tCentimeter = getUSDistanceAsCentiMeter();
#ifdef PLOTTER_OUTPUT
    Serial.println(tCentimeter);
#else
    Serial.print("Distance=");
    Serial.print(tCentimeter);
    Serial.print("cm. ");
#endif
    /*
     * Play tone
     */
    int tFrequency = map(tCentimeter, 0, 100, 110, 1760); // 4 octaves per meter
    tone(PIN_SPEAKER, tFrequency);
    return tCentimeter;
}
