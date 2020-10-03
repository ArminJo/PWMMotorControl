/*
 * TestPage.cpp
 *
 *  Contains all GUI elements for test controlling the RobotCarMotorControl.
 *
 *  Requires BlueDisplay library.
 *
 *  Created on: 20.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
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

/*
 * Motor GUI
 */
BDButton TouchButtonReset;

BDButton TouchButtonGetAndStoreSpeed;

BDButton TouchButtonDebug;

BDButton TouchButton5cm;
BDButton TouchButton10cm;
BDButton TouchButton20cm;
BDButton TouchButton40cm;

BDButton TouchButton45DegreeRight;
BDButton TouchButton45DegreeLeft;
BDButton TouchButton90DegreeRight;
BDButton TouchButton90DegreeLeft;
BDButton TouchButton360Degree;

bool sShowDebug = false;

#pragma GCC diagnostic ignored "-Wunused-parameter"
void doDistance(BDButton * aTheTouchedButton, int16_t aValue) {
    RobotCarMotorControl.startGoDistanceCentimeter(aValue, sRobotCarDirection);
}

void doShowDebug(BDButton * aTheTouchedButton, int16_t aValue) {
    sShowDebug = aValue;
}

void doRotation(BDButton * aTheTouchedButton, int16_t aValue) {
    RobotCarMotorControl.startRotateCar(aValue, sRobotCarDirection, true);
}

/*
 * Store user speed input as DriveSpeed
 */
void doStoreSpeed(float aValue) {
    uint16_t tValue = aValue;
    if (tValue > 10 && tValue < 256) {
        // must use value for compensation not compensated value
        RobotCarMotorControl.rightCarMotor.DriveSpeed = tValue;
        RobotCarMotorControl.rightCarMotor.writeMotorvaluesToEeprom();

        // use the same value here !
        RobotCarMotorControl.leftCarMotor.DriveSpeed = tValue;
        RobotCarMotorControl.leftCarMotor.writeMotorvaluesToEeprom();
    }
    printMotorValues();
}

/*
 * Request speed value as number
 */
void doGetSpeedAsNumber(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPrompt(&doStoreSpeed, "Drive speed [11-255]", sLastSpeedSliderValue);
}

/*
 * stop and reset motors
 */
void doReset(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopRobotCar(false);
    RobotCarMotorControl.resetControlValues();
}

void initTestPage(void) {
    /*
     * Control buttons
     */

    TouchButtonReset.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_BLUE, F("Reset"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doReset);

    TouchButtonGetAndStoreSpeed.init(0, BUTTON_HEIGHT_4_LINE_4 - BUTTON_HEIGHT_6 - BUTTON_DEFAULT_SPACING_QUARTER, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_6, COLOR_BLUE, F("Set\nspeed"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doGetSpeedAsNumber);

    /*
     * Test buttons
     * Many calls requires 36 bytes code + sometimes 52 bytes to clean up the stack.
     */
    TouchButton5cm.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("5cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 5, &doDistance);
    TouchButton10cm.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("10cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 10, &doDistance);

    TouchButton20cm.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("20cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 20, &doDistance);
    TouchButton40cm.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE, F("40cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 40, &doDistance);

    TouchButton45DegreeLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 45, &doRotation); // \xB0 is degree character
    TouchButton45DegreeRight.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("-45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -45, &doRotation); // \xB0 is degree character

    TouchButton90DegreeLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 90, &doRotation); // \xB0 is degree character
    TouchButton90DegreeRight.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("-90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -90, &doRotation); // \xB0 is degree character

    TouchButton360Degree.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("360\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 360, &doRotation); // \xB0 is degree character

    TouchButtonDebug.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED, F("dbg"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doShowDebug);
}

void drawTestPage(void) {

    drawCommonGui();

    TouchButtonRobotCarStartStop.drawButton();
    TouchButtonReset.drawButton();
    TouchButtonBack.drawButton();

    TouchButtonDirection.drawButton();
    TouchButtonDebug.drawButton();

    TouchButton5cm.drawButton();
    TouchButton10cm.drawButton();
    TouchButton20cm.drawButton();
    TouchButton40cm.drawButton();

    TouchButton45DegreeLeft.drawButton();
    TouchButton45DegreeRight.drawButton();
    TouchButton90DegreeLeft.drawButton();
    TouchButton90DegreeRight.drawButton();
    TouchButton360Degree.drawButton();

    SliderSpeed.drawSlider();
#ifdef USE_ENCODER_MOTOR_CONTROL
    SliderSpeedRight.drawSlider();
    SliderSpeedLeft.drawSlider();
#endif
    TouchButtonGetAndStoreSpeed.drawButton();

    SliderUSPosition.setValueAndDrawBar(sLastServoAngleInDegrees);
    SliderUSPosition.drawSlider();
    SliderUSDistance.drawSlider();

#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    SliderIRDistance.drawSlider();
#  endif
#ifdef USE_ENCODER_MOTOR_CONTROL
    TouchButtonCalibrate.drawButton();
#else
    TouchButtonCompensation.drawButton();
#endif
    PWMDcMotor::MotorValuesHaveChanged = true; // trigger drawing of values
}

/*
 * Stop motors but enable movement
 */
void startTestPage(void) {
    doReset(NULL, 0);
    drawTestPage();
}

void loopTestPage(void) {
}

void stopTestPage(void) {
}
