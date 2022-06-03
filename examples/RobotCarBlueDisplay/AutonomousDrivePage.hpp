/*
 * AutonomousDrivePage.hpp
 *
 *  Contains all the GUI elements for autonomous driving.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCar.cpp)
 *  Start Pro: Start elaborated driving algorithm
 *
 *  Requires BlueDisplay library.
 *
 *  Copyright (C) 2019-2022  Armin Joachimsmeyer
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

#if defined(ENABLE_AUTONOMOUS_DRIVE)

#ifndef _ROBOT_CAR_AUTOMOMOUS_DRIVE_PAGE_HPP
#define _ROBOT_CAR_AUTOMOMOUS_DRIVE_PAGE_HPP

BDButton TouchButtonStepMode;
const char sStepModeButtonStringContinuousStepToTurn[] PROGMEM = "Continuous\n->\nStep to turn";
const char sStepModeButtonStringStepToTurnSingleStep[] PROGMEM = "Step to turn\n->\nSingle step";
const char sStepModeButtonStringSingleStepContinuous[] PROGMEM = "Single step\n->\nContinuous";
const char *const sStepModeButtonCaptionStringArray[] PROGMEM = { sStepModeButtonStringContinuousStepToTurn,
        sStepModeButtonStringStepToTurnSingleStep, sStepModeButtonStringSingleStepContinuous };

BDButton TouchButtonDistanceFeedbackMode;
const char sDistanceFeedbackModeNoTone[] PROGMEM = "No tone";
const char sDistanceFeedbackModePentatonic[] PROGMEM = "Pentatonic";
const char sDistanceFeedbackModeContinuous[] PROGMEM = "Continuous";
const char *const sDistanceFeedbackModeButtonCaptionStringArray[] PROGMEM = { sDistanceFeedbackModeNoTone,
        sDistanceFeedbackModePentatonic, sDistanceFeedbackModeContinuous };

BDButton TouchButtonStep;
BDButton TouchButtonSingleScan;
BDButton TouchButtonScanSpeed;
#if defined(ENABLE_PATH_INFO_PAGE)
BDButton TouchButtonPathInfoPage;
#endif

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
BDButton TouchButtonScanMode;
const char sDistanceSourceModeButtonStringMinMax[] PROGMEM = "Min->Max";
const char sDistanceSourceModeButtonStringMaxUS[] PROGMEM = "Max->US";
#  if defined(CAR_HAS_IR_DISTANCE_SENSOR)
const char sDistanceSourceModeButtonStringUSIr[] PROGMEM = "US->IR";
const char sDistanceSourceModeButtonStringIrMin[] PROGMEM = "IR->Min";
const char *const sDistanceSourceModeButtonCaptionStringArray[] PROGMEM = { sDistanceSourceModeButtonStringMinMax,
        sDistanceSourceModeButtonStringMaxUS, sDistanceSourceModeButtonStringUSIr, sDistanceSourceModeButtonStringIrMin };
# else
const char sDistanceSourceModeButtonStringUSTof[] PROGMEM = "US->ToF";
const char sDistanceSourceModeButtonStringTofMin[] PROGMEM = "ToF->Min";

const char * const sDistanceSourceModeButtonCaptionStringArray[] PROGMEM = { sDistanceSourceModeButtonStringMinMax, sDistanceSourceModeButtonStringMaxUS,
        sDistanceSourceModeButtonStringUSTof, sDistanceSourceModeButtonStringTofMin};
#  endif
#endif

BDButton TouchButtonStartStopUserAutonomousDrive;
BDButton TouchButtonStartStopBuiltInAutonomousDrive;
BDButton TouchButtonFollower;

void setDistanceFeedbackModeButtonCaption() {
    TouchButtonDistanceFeedbackMode.setCaptionFromStringArrayPGM(sDistanceFeedbackModeButtonCaptionStringArray, sDistanceFeedbackMode, true);
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
void doNextDistanceFeedbackMode(BDButton *aTheTouchedButton, int16_t aValue) {
    sDistanceFeedbackMode++;
    if (sDistanceFeedbackMode > DISTANCE_FEEDBACK_MAX) {
        sDistanceFeedbackMode = DISTANCE_FEEDBACK_NO_TONE;
        noTone(PIN_BUZZER);
    }
    setDistanceFeedbackModeButtonCaption();
}

void setStepModeButtonCaption() {
    TouchButtonStepMode.setCaptionFromStringArrayPGM(sStepModeButtonCaptionStringArray, sStepMode, (sCurrentPage == PAGE_AUTOMATIC_CONTROL));
}

/*
 * Switches modes MODE_CONTINUOUS -> MODE_STEP_TO_NEXT_TURN -> MODE_SINGLE_STEP
 */
void setStepMode(uint8_t aStepMode) {
    if (aStepMode == MODE_SINGLE_STEP) {
        RobotCar.stopAndWaitForIt();
    } else if (aStepMode > MODE_SINGLE_STEP) {
        aStepMode = MODE_CONTINUOUS;
        sDoStep = true;
    }
    sStepMode = aStepMode;
    setStepModeButtonCaption();
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
void doNextStepMode(BDButton *aTheTouchedButton, int16_t aValue) {
    sStepMode++;
    setStepMode(sStepMode);
}

/*
 * enables next step
 */
void doStep(BDButton *aTheTouchedButton, int16_t aValue) {
    if (sStepMode == MODE_CONTINUOUS) {
        // switch to step mode MODE_SINGLE_STEP
        setStepMode(MODE_SINGLE_STEP);
    }
    sDoStep = true;
    /*
     * Start if not yet done
     */
    if (sDriveMode == MODE_MANUAL_DRIVE) {
        startStopAutomomousDrive(true, MODE_COLLISION_AVOIDING_BUILTIN);
    }
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
void setScanModeButtonCaption() {
    TouchButtonScanMode.setCaptionFromStringArrayPGM(sDistanceSourceModeButtonCaptionStringArray, sDistanceSourceMode);
}

void doDistanceSourceMode(BDButton *aTheTouchedButton, int16_t aValue) {
    sDistanceSourceMode++;
    if (sDistanceSourceMode > DISTANCE_LAST_SOURCE_MODE) {
        sDistanceSourceMode = DISTANCE_SOURCE_MODE_MINIMUM;
    }
    setScanModeButtonCaption();
    TouchButtonScanMode.drawButton();
}

#endif // defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)

void doChangeScanSpeed(BDButton *aTheTouchedButton, int16_t aValue) {
    sDoSlowScan = aValue;
}

void doSingleScan(BDButton *aTheTouchedButton, int16_t aValue) {
    if (sDriveMode == MODE_FOLLOWER) {
        scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
    } else {
        clearPrintedForwardDistancesInfos(true);
        fillAndShowForwardDistancesInfo(true, true);
        postProcessAndCollisionDetection();
    }
}

void doStartStopFollowerMode(BDButton *aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, MODE_FOLLOWER);
}

void doStartStopAutomomousDrive(BDButton *aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, MODE_COLLISION_AVOIDING_BUILTIN);
}

void doStartStopTestUser(BDButton *aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, MODE_COLLISION_AVOIDING_USER);
}

/*
 * set buttons accordingly to sDriveMode
 */
void handleAutomomousDriveRadioButtons() {
    TouchButtonStartStopBuiltInAutonomousDrive.setValue(sDriveMode == MODE_COLLISION_AVOIDING_BUILTIN,
            sCurrentPage == PAGE_AUTOMATIC_CONTROL);
    TouchButtonStartStopUserAutonomousDrive.setValue(sDriveMode == MODE_COLLISION_AVOIDING_USER,
            sCurrentPage == PAGE_AUTOMATIC_CONTROL);
    TouchButtonFollower.setValue(sDriveMode == MODE_FOLLOWER, sCurrentPage == PAGE_AUTOMATIC_CONTROL);
}

void initAutonomousDrivePage(void) {

    // left button column
    TouchButtonStepMode.init(0, 0, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6 + 1, COLOR16_BLUE,
            reinterpret_cast<const __FlashStringHelper*>(sStepModeButtonStringContinuousStepToTurn), TEXT_SIZE_9,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, MODE_CONTINUOUS, &doNextStepMode);

    TouchButtonStep.init(0, BUTTON_HEIGHT_6_LINE_2, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR16_BLUE, F("Step"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStep);

    TouchButtonSingleScan.init(0, BUTTON_HEIGHT_6_LINE_3, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR16_BLUE, F("Scan"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doSingleScan);

    TouchButtonScanSpeed.init(0, BUTTON_HEIGHT_6_LINE_4, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_8, COLOR16_BLACK, F("Scan slow"), TEXT_SIZE_14,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doChangeScanSpeed);
    TouchButtonScanSpeed.setCaptionForValueTrue("Scan fast");

#if defined(ENABLE_PATH_INFO_PAGE)
    TouchButtonPathInfoPage.init(BUTTON_WIDTH_3_POS_3, 0, BUTTON_WIDTH_3, BUTTON_HEIGHT_6, COLOR16_RED, F("Show Path"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_SHOW_PATH, &GUISwitchPages);
#endif

    // small buttons
    // use sDriveMode to support reconnect during demo mode
    TouchButtonStartStopUserAutonomousDrive.init(0, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR16_RED, F("Start User"), TEXT_SIZE_14,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, (sDriveMode == MODE_COLLISION_AVOIDING_USER),
            &doStartStopTestUser);
    TouchButtonStartStopUserAutonomousDrive.setCaptionForValueTrue(F("Stop User"));

    TouchButtonDistanceFeedbackMode.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR16_RED, reinterpret_cast<const __FlashStringHelper*>(sDistanceFeedbackModeNoTone), TEXT_SIZE_14,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, DISTANCE_FEEDBACK_NO_TONE, &doNextDistanceFeedbackMode);

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR16_RED,
            reinterpret_cast<const __FlashStringHelper*>(sDistanceSourceModeButtonStringMinMax),
            TEXT_SIZE_16, FLAG_BUTTON_DO_BEEP_ON_TOUCH, DISTANCE_SOURCE_MODE_MINIMUM, &doDistanceSourceMode);
#endif

    // bottom line
    // use sDriveMode to support reconnect during demo mode
    TouchButtonStartStopBuiltInAutonomousDrive.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR16_RED,
            F("Start\nBuiltin"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            (sDriveMode == MODE_COLLISION_AVOIDING_BUILTIN), &doStartStopAutomomousDrive);
    TouchButtonStartStopBuiltInAutonomousDrive.setCaptionForValueTrue(F("Stop"));

    TouchButtonFollower.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR16_RED, F("Start\nFollow"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            (sDriveMode == MODE_FOLLOWER), &doStartStopFollowerMode);
    TouchButtonFollower.setCaptionForValueTrue(F("Stop\nFollow"));
}

void drawAutonomousDrivePage(void) {
    drawCommonGui();

    // - (TEXT_SIZE_22_WIDTH / 2) since we have one character more
    BlueDisplay1.drawText(HEADER_X - (TEXT_SIZE_22_WIDTH / 2), (2 * TEXT_SIZE_22_HEIGHT), F("Auto drive"));

#if defined(ENABLE_PATH_INFO_PAGE)
    TouchButtonPathInfoPage.drawButton();
#endif

    // left button column
    TouchButtonStepMode.drawButton();
    TouchButtonStep.drawButton();
    TouchButtonSingleScan.drawButton();
    TouchButtonScanSpeed.drawButton();

    // small buttons
    TouchButtonStartStopUserAutonomousDrive.drawButton();
//    TouchButtonDistanceFeedbackMode.drawButton();
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.drawButton();
#endif

    // bottom buttons line
    TouchButtonStartStopBuiltInAutonomousDrive.drawButton();
    TouchButtonFollower.drawButton();
    TouchButtonBack.drawButton();
}

void startAutonomousDrivePage(void) {
    // set correct values of radio buttons
    handleAutomomousDriveRadioButtons();

    // restore last step and scan mode
    setStepModeButtonCaption();
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    setScanModeButtonCaption();
#endif
    setDistanceFeedbackModeButtonCaption();

#if defined(ENABLE_PATH_INFO_PAGE)
#define SLIDER_SHIFTED_Y_POS    BUTTON_HEIGHT_6
#else
#define SLIDER_SHIFTED_Y_POS    SLIDER_TOP_MARGIN
#endif
#if defined(US_DISTANCE_SLIDER_IS_SMALL)
    SliderUSDistance.setPosition(POS_X_DISTANCE_POSITION_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2) - TEXT_SIZE_11_WIDTH, SLIDER_SHIFTED_Y_POS);
#else
    SliderUSDistance.setPosition(POS_X_DISTANCE_POSITION_SLIDER - BUTTON_WIDTH_10, SLIDER_SHIFTED_Y_POS);
#endif
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    SliderIROrTofDistance.setPosition(POS_X_DISTANCE_POSITION_SLIDER - (TEXT_SIZE_11_WIDTH + BUTTON_WIDTH_10), SLIDER_SHIFTED_Y_POS);
#endif
    drawAutonomousDrivePage();
    if(!isCalibrated) {
        calibrateAndPrint();
    }
}

// currently not used
void loopAutonomousDrivePage(void) {
// Autonomous driving is done in main loop by calling driveAutonomousOneStep() (in AutonomousDrive.cpp), since it is independent of the display.
}

void stopAutonomousDrivePage(void) {
#if defined(US_DISTANCE_SLIDER_IS_SMALL)
    SliderUSDistance.setPosition(POS_X_US_DISTANCE_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2), SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8);
#else
    SliderUSDistance.setPosition(POS_X_US_DISTANCE_SLIDER - BUTTON_WIDTH_10, SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8);
#endif
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    SliderIROrTofDistance.setPosition(POS_X_THIRD_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2), SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8);
#endif

}

/*
 * Clear drawing area
 * X from TouchButtonStep to end of display
 * Y from TouchButtonBackSmall to TouchButtonScanSpeed
 */
void clearPrintedForwardDistancesInfos(bool aDoFullClear) {
    if(aDoFullClear) {
        BlueDisplay1.fillRect(BUTTON_WIDTH_3_5 + 1, BUTTON_HEIGHT_4 + 1, LAYOUT_320_WIDTH,
                BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - 1, COLOR16_WHITE);
    } else{
        BlueDisplay1.fillRect(BUTTON_WIDTH_3_5 + 1, BUTTON_HEIGHT_4 + 1, POS_X_US_DISTANCE_SLIDER - BUTTON_WIDTH_8,
                BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - 1, COLOR16_WHITE);
    }
}

/*
 * Draws only if sCurrentPage == PAGE_AUTOMATIC_CONTROL
 */
void drawCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClearVector) {

    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        color16_t tColor = COLOR16_BLUE;
        int tDegreeToDisplay = aDegreeToTurn;

        if (tDegreeToDisplay == 180) {
            tColor = COLOR16_RED;
            tDegreeToDisplay = 0;
        }
        if (aDoClearVector) {
            tColor = COLOR16_WHITE;
        }

        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLengthOfVector, tDegreeToDisplay + 90,
                tColor);
        if (!aDoClearVector) {
            //Print result
            sprintf_P(sStringBuffer, PSTR("wall%4d\xB0 rotation: %3d\xB0 wall%4d\xB0"), sForwardDistancesInfo.WallLeftAngleDegrees,
                    aDegreeToTurn, sForwardDistancesInfo.WallRightAngleDegrees); // \xB0 is degree character
            BlueDisplay1.drawText(BUTTON_WIDTH_3_5_POS_2, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11, sStringBuffer, TEXT_SIZE_11,
            COLOR16_BLACK, COLOR16_WHITE);
        }
    }
}
#endif // _ROBOT_CAR_AUTOMOMOUS_DRIVE_PAGE_HPP
#endif // defined(ENABLE_AUTONOMOUS_DRIVE)
