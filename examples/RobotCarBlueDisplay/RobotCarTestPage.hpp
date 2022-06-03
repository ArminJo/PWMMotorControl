/*
 * RobotCarTestPage.hpp
 *
 *  Contains all GUI elements for test controlling the RobotCar.
 *
 *  Requires BlueDisplay library.
 *
 *  Copyright (C) 2016-2022  Armin Joachimsmeyer
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */
#ifndef _ROBOT_CAR_TEST_PAGE_HPP
#define _ROBOT_CAR_TEST_PAGE_HPP

/*
 * Motor GUI
 */
BDButton TouchButtonReset;

#if defined(ENABLE_EEPROM_STORAGE)
BDButton TouchButtonGetAndStoreSpeed;
#endif

BDButton TouchButton5cm;
BDButton TouchButton10cm;
BDButton TouchButton20cm;
BDButton TouchButton40cm;
BDButton TouchButtonTest;

BDButton TouchButton45DegreeRight;
BDButton TouchButton45DegreeLeft;
BDButton TouchButton90DegreeRight;
BDButton TouchButton90DegreeLeft;
BDButton TouchButton360Degree;

bool sShowInfo = true;

#pragma GCC diagnostic ignored "-Wunused-parameter"
void doDistance(BDButton *aTheTouchedButton, int16_t aValue) {
    RobotCar.startGoDistanceMillimeter(aValue, sRobotCarDirection);
#if !defined(USE_ENCODER_MOTOR_CONTROL)
    BlueDisplay1.debug("Millis=",
            (uint16_t) (RobotCar.rightCarMotor.computedMillisOfMotorStopForDistance - millis()));
#endif
}

/*
 * stop and reset motors
 * reset IMU data and compute new offsets
 */
void doReset(BDButton *aTheTouchedButton, int16_t aValue) {
    startStopRobotCar(false);
    RobotCar.resetEncoderControlValues();
    sLastSpeedSliderValue = 0;
}

/*
 * For miscellaneous test purposes
 */
void doTest(BDButton *aTheTouchedButton, int16_t aValue) {
    doDistance(NULL, 10);
}

void doRotation(BDButton *aTheTouchedButton, int16_t aValue) {
    if (aValue == 360) {
        // use in place for 360 degree and change turn direction according to sRobotCarDirection
        if (sRobotCarDirection != DIRECTION_FORWARD) {
            aValue = -aValue;
        }
        RobotCar.startRotate(aValue, TURN_IN_PLACE);
    } else {
        RobotCar.startRotate(aValue, (turn_direction_t) sRobotCarDirection);
    }
}

#if defined(ENABLE_EEPROM_STORAGE)
/*
 * Callback handler for user speed input
 * Store user speed input as DriveSpeed
 */
void doStoreSpeed(float aValue) {
    uint16_t tValue = aValue;
    if (tValue > 10 && tValue < 256) {
        // must use value for compensation not compensated value
        RobotCar.rightCarMotor.DriveSpeed = tValue;
        // use the same value here !
        RobotCar.leftCarMotor.DriveSpeed = tValue;
        RobotCar.writeMotorValuesToEeprom();
    }
    printMotorValues();
}


/*
 * Request speed value as number from user
 */
void doGetSpeedAsNumber(BDButton * aTheTouchedButton, int16_t aValue) {
    BlueDisplay1.getNumberWithShortPrompt(&doStoreSpeed, "Drive speed [11-255]", sLastSpeedSliderValue);
}
#endif

/*
 * replacing parameter init with structure init INCREASES code size by 82 bytes
 */
//const struct ButtonInit ButtonReset PROGMEM { BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
//COLOR16_BLUE, TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doReset };
//const struct ButtonInit Button5cm PROGMEM { BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8,
//COLOR16_BLUE, TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 50, &doDistance };
//const struct ButtonInit Button10cm PROGMEM { BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8,
//COLOR16_BLUE, TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 100, &doDistance };
void initTestPage(void) {
    /*
     * Control buttons
     */
    TouchButtonReset.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR16_BLUE, F("Reset"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doReset);

#if defined(ENABLE_EEPROM_STORAGE)
    TouchButtonGetAndStoreSpeed.init(0, BUTTON_HEIGHT_4_LINE_4 - BUTTON_HEIGHT_6 - BUTTON_DEFAULT_SPACING_QUARTER, BUTTON_WIDTH_6,
    BUTTON_HEIGHT_6, COLOR16_BLUE, F("Set\nspeed"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doGetSpeedAsNumber);
#endif
    /*
     * Test buttons
     * Many calls requires 36 bytes code + sometimes 52 bytes to clean up the stack.
     */
    TouchButton5cm.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE, F("5cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 50, &doDistance);
    TouchButton10cm.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE, F("10cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 100, &doDistance);

    TouchButton20cm.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE, F("20cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 200, &doDistance);
    TouchButton40cm.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE, F("40cm"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 400, &doDistance);

//    TouchButtonReset.init(&ButtonReset, F("Reset"));
//    TouchButton5cm.init(&Button5cm, F("5cm"));
//    TouchButton10cm.init(&Button10cm, F("10cm"));
//
//    TouchButton20cm.init(&Button20cm, F("20cm"));
//    TouchButton40cm.init(&Button40cm, F("40cm"));
//    TouchButtonInfo.init(&ButtonDebug, F("dbg"));

    TouchButtonTest.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_RED, F("Test"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doTest);

    TouchButton45DegreeLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE,
            F("45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 45, &doRotation); // \xB0 is degree character
    TouchButton45DegreeRight.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE,
            F("-45\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -45, &doRotation); // \xB0 is degree character
    TouchButton360Degree.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_5, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE,
            F("360\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 360, &doRotation); // \xB0 is degree character

    TouchButton90DegreeLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_6, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE,
            F("90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 90, &doRotation); // \xB0 is degree character
    TouchButton90DegreeRight.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_6, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE,
            F("-90\xB0"), TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, -90, &doRotation); // \xB0 is degree character

//    TouchButton45DegreeLeft.init(&Button45DegreeLeft, F("45\xB0")); // \xB0 is degree character
//    TouchButton45DegreeRight.init(&Button45DegreeRight, F("-45\xB0")); // \xB0 is degree character
//    TouchButton360Degree.init(&Button360Degree, F("360\xB0")); // \xB0 is degree character
//
//    TouchButton90DegreeLeft.init(&Button90DegreeLeft, F("90\xB0")); // \xB0 is degree character
//    TouchButton90DegreeRight.init(&Button90DegreeRight, F("-90\xB0")); // \xB0 is degree character
}

void drawTestPage(void) {

    drawCommonGui();

    TouchButtonRobotCarStartStop.drawButton();
    TouchButtonReset.drawButton();
    TouchButtonBack.drawButton();

    TouchButton5cm.drawButton();
    TouchButton10cm.drawButton();
//#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
    TouchButtonCalibrate.drawButton();
//#endif

    TouchButton20cm.drawButton();
    TouchButton40cm.drawButton();
    TouchButtonTest.drawButton();

    TouchButtonCompensationLeft.drawButton();
    TouchButtonCompensationRight.drawButton();
    TouchButtonInfo.drawButton();

#if defined(ENABLE_EEPROM_STORAGE)
    TouchButtonCompensationStore.drawButton();
#endif

    TouchButton45DegreeLeft.drawButton();
    TouchButton45DegreeRight.drawButton();
    TouchButton360Degree.drawButton();

    TouchButton90DegreeLeft.drawButton();
    TouchButton90DegreeRight.drawButton();
    TouchButtonDirection.drawButton();

    SliderSpeed.drawSlider();
#if defined(USE_ENCODER_MOTOR_CONTROL)
    SliderSpeedRight.drawSlider();
    SliderSpeedLeft.drawSlider();
#endif
#if defined(ENABLE_EEPROM_STORAGE)
    TouchButtonGetAndStoreSpeed.drawButton();
#endif

    SliderDistanceServoPosition.drawSlider();
    SliderUSDistance.drawSlider();

#  if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    SliderIROrTofDistance.drawSlider();
#  endif

    PWMDcMotor::MotorControlValuesHaveChanged = true; // trigger drawing of values
}

/*
 * Stop motors but enable movement
 */
void startTestPage(void) {
    doReset(NULL, 0);
    drawTestPage();
    if (!isCalibrated) {
        calibrateAndPrint();
    }
}

void loopTestPage(void) {
}

void stopTestPage(void) {
}
#endif // _ROBOT_CAR_TEST_PAGE_HPP
