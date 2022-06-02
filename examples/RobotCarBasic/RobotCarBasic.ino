/*
 *  RobotCarBasic.cpp
 *
 *  Template for your RobotCar control.
 *  Currently implemented is: drive until distance too low, then stop, go backwards and turn random amount.
 *  The measured distance range is converted to a pitch as an acoustic feedback.
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
//#define TRACE
/*
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 4AA.
//#define TBB6612_4WD_4AA_VIN_CONFIGURATION         // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION   // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION    // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define L298_2WD_4AA_BASIC_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_4WD_4AA_BASIC_CONFIGURATION          // China set with L298 + 4AA.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_VIN_IR_DISTANCE_CONFIGURATION    // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_VIN_IR_IMU_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050

#include "RobotCarConfigurations.h" // sets e.g. USE_ENCODER_MOTOR_CONTROL, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"

#include "Servo.h"
#include "Distance.hpp"
#include "HCSR04.h"
#include "CarPWMMotorControl.hpp"
#include "RobotCarUtils.hpp"

#define rightMotor RobotCar.rightCarMotor
#define leftMotor   RobotCar.leftCarMotor

#define VERSION_EXAMPLE "1.0"

void simpleObjectAvoidance();

/*
 * Start of robot car control program
 */
void setup() {
    Serial.begin(115200);

    // Just to know which program is running on my Arduino
    Serial.println("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);

    initRobotCarPWMMotorControl();

    /*
     * Initialize pins for the servo and the HC-SR04 ultrasonic distance measurement
     */
    initDistance();

    /*
     * Move it as signal, that we are booted
     */
    delay(500);
    DistanceServo.write(135);
    delay(500);
    DistanceServo.write(45);
    delay(500);
    DistanceServo.write(90);

    tone(PIN_BUZZER, 2200, 200);
    delay(1000);

    /*
     * Rotate both motors forward, wait a second and then stop
     * Speed can go from 0 to 255 (MAX_SPEED_PWM)
     * DEFAULT_DRIVE_SPEED_PWM corresponds to 2 volt motor supply
     */
    leftMotor.setSpeedPWMAndDirection(120, DIRECTION_FORWARD);
    rightMotor.setSpeedPWMAndDirection(120, DIRECTION_FORWARD);
    delay(1000);
    leftMotor.stop();
    rightMotor.stop();

    // wait before entering the loop
    delay(4000);
}

void loop() {
    simpleObjectAvoidance();
    delay(50); // Wait until next distance sample

}

void simpleObjectAvoidance() {
    /*
     * Drive forward until distance too low, then stop, go back, turn random amount and drive again.
     */
    unsigned int tCentimeter = getUSDistanceAsCentimeter();
    Serial.print("US distance=");
    Serial.print(tCentimeter);
    Serial.println(" cm");

    if (tCentimeter < 20) {
        /*
         * Distance too low here -> Stop and go backwards
         */
        RobotCar.stop();
        delay(1000);
        RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_BACKWARD);
        delay(1000);
        RobotCar.stop();
        delay(1000);

        /*
         * Turn random amount and drive again
         */
        int tTurnValueDegree = random(20, 90);
        Serial.print("Turn ");
        Serial.print(tTurnValueDegree);
        Serial.println(" degree");
        RobotCar.rotate(tTurnValueDegree, TURN_IN_PLACE);
        delay(1000);
        RobotCar.setSpeedPWMAndDirection(DEFAULT_DRIVE_SPEED_PWM, DIRECTION_FORWARD);
    }
}

