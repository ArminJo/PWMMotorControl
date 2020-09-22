/*
 * RobotCarGui.cpp
 *
 *  Contains all common GUI elements for operating and controlling the RobotCarMotorControl.
 *
 *  Calibration: Sets lowest speed for which wheels are moving.
 *  Speed Slider left: Sets speed for manual control which serves also as maximum speed for autonomous drive if "Stored"
 *  Store: Stores calibration info and maximum speed.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCarMotorControl.cpp)
 *  Start Pro: Start elaborated driving algorithm
 *
 *  insertToPath() and DrawPath() to show the path we were driving.
 *
 *  Requires BlueDisplay library.
 *
 *  Created on: 20.09.2016
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include "RobotCar.h"
#include "RobotCarGui.h"
#include "Distance.h"

BDButton TouchButtonTestPage;
BDButton TouchButtonLaser;
#ifdef ENABLE_RTTTL
BDButton TouchButtonMelody;
#endif
#ifdef CAR_HAS_CAMERA
BDButton TouchButtonCameraOnOff;
#endif

#ifdef CAR_HAS_PAN_SERVO
BDSlider SliderPan;
#endif
#ifdef CAR_HAS_TILT_SERVO
BDSlider SliderTilt;
#endif

#pragma GCC diagnostic ignored "-Wunused-parameter"

// Here we get values from 0 to 180 degrees from scaled slider
#ifdef CAR_HAS_PAN_SERVO
void doHorizontalServoPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    PanServo.write(aValue);
}
#endif

#ifdef CAR_HAS_TILT_SERVO
void doVerticalServoPosition(BDSlider * aTheTouchedSlider, uint16_t aValue) {
    TiltServo.write(aValue);
}
#endif

#ifdef CAR_HAS_LASER
void doLaserOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    digitalWrite(PIN_LASER_OUT, aValue);
}
#endif

#ifdef CAR_HAS_CAMERA
void doCameraSupplyOnOff(BDButton * aTheTouchedButton, int16_t aValue) {
    digitalWrite(PIN_CAMERA_SUPPLY_CONTROL, aValue);
}
#endif

#ifdef ENABLE_RTTTL
void doPlayMelody(BDButton * aTheTouchedButton, int16_t aValue) {
    sPlayMelody = aValue;
}
#endif

void initHomePage(void) {

    TouchButtonTestPage.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, F("Test"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_TEST, &GUISwitchPages);

#ifdef CAR_HAS_CAMERA
    TouchButtonCameraOnOff.init(0, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER), BUTTON_WIDTH_3,
    TEXT_SIZE_22_HEIGHT, COLOR_BLACK, F("Camera"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            false, &doCameraSupplyOnOff);
#endif

#ifdef ENABLE_RTTTL
    TouchButtonMelody.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, BUTTON_HEIGHT_8, COLOR_BLACK, F("Melody"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sPlayMelody, &doPlayMelody);
#endif

#ifdef CAR_HAS_LASER
    TouchButtonLaser.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR_BLACK, F("Laser"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doLaserOnOff);
#endif

}

/*
 * Manual control page
 */
void drawHomePage(void) {
    drawCommonGui();
    BlueDisplay1.drawText(HEADER_X + TEXT_SIZE_22_WIDTH, (2 * TEXT_SIZE_22_HEIGHT), F("Control"));

#if defined(CAR_HAS_4_WHEELS)
    char tCarTypeString[] = "4WD";
#else
    char tCarTypeString[] = "2WD";
#endif

    BlueDisplay1.drawText(HEADER_X + (3 * TEXT_SIZE_22_WIDTH), (3 * TEXT_SIZE_22_HEIGHT), tCarTypeString);

    TouchButtonRobotCarStartStop.drawButton();
#ifdef CAR_HAS_CAMERA
    TouchButtonCameraOnOff.drawButton();
#endif
#ifdef ENABLE_RTTTL
    TouchButtonMelody.drawButton();
#endif
#ifdef CAR_HAS_LASER
    TouchButtonLaser.drawButton();
#endif
    TouchButtonTestPage.drawButton();
    TouchButtonNextPage.drawButton();

    TouchButtonDirection.drawButton();
#ifdef USE_ENCODER_MOTOR_CONTROL
    TouchButtonCalibrate.drawButton();
#else
    TouchButtonCompensation.drawButton();
#endif

    SliderUSPosition.setValueAndDrawBar(sLastServoAngleInDegrees);
    SliderUSPosition.drawSlider();

    SliderUSDistance.drawSlider();

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR) && ( ! (defined(CAR_HAS_PAN_SERVO) && defined(CAR_HAS_TILT_SERVO)))
    SliderIRDistance.drawSlider();
#endif

#ifdef CAR_HAS_PAN_SERVO
    SliderPan.drawSlider();
#endif
#ifdef CAR_HAS_TILT_SERVO
    SliderTilt.drawSlider();
#endif

    SliderSpeed.drawSlider();
#ifdef USE_ENCODER_MOTOR_CONTROL
    SliderSpeedRight.drawSlider();
    SliderSpeedLeft.drawSlider();
#endif
    PWMDcMotor::MotorValuesHaveChanged = true; // trigger drawing of values
}

void startHomePage(void) {
    TouchButtonDirection.setPosition(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4);
#ifdef USE_ENCODER_MOTOR_CONTROL
    TouchButtonCalibrate.setPosition(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4);
#else
    TouchButtonCompensation.setPosition(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4);
#endif
    TouchButtonNextPage.setCaption(F("Automatic\nControl"));

    drawHomePage();
}

void loopHomePage(void) {
}

void stopHomePage(void) {
    TouchButtonDirection.setPosition(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_5);
#ifdef USE_ENCODER_MOTOR_CONTROL
    TouchButtonCalibrate.setPosition(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2);
#else
    TouchButtonCompensation.setPosition(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2);
#endif
    startStopRobotCar(false);
}

