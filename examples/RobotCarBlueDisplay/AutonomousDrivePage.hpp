/*
 * AutonomousDrivePage.hpp
 *
 *  Contains all the GUI elements for autonomous driving.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCarPWMMotorControl.cpp)
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

#ifndef _ROBOT_CAR_AUTOMOMOUS_DRIVE_PAGE_HPP
#define _ROBOT_CAR_AUTOMOMOUS_DRIVE_PAGE_HPP
#include <Arduino.h>

#include "RobotCarPinDefinitionsAndMore.h"
#include "RobotCarBlueDisplay.h"
#include "RobotCarGui.h"
#include "Distance.h"

BDButton TouchButtonStepMode;
const char sStepModeButtonStringContinuousStepToTurn[] PROGMEM = "Continuous\n->\nStep to turn";
const char sStepModeButtonStringStepToTurnSingleStep[] PROGMEM = "Step to turn\n->\nSingle step";
const char sStepModeButtonStringSingleStepContinuous[] PROGMEM = "Single step\n->\nContinuous";
const char *const sStepModeButtonCaptionStringArray[] PROGMEM = { sStepModeButtonStringContinuousStepToTurn,
        sStepModeButtonStringStepToTurnSingleStep, sStepModeButtonStringSingleStepContinuous };

BDButton TouchButtonStep;
BDButton TouchButtonSingleScan;
BDButton TouchButtonScanSpeed;
#if defined(ENABLE_PATH_INFO_PAGE)
BDButton TouchButtonPathInfoPage;
#endif

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
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

void setStepModeButtonCaption();
/*
 * Switches modes MODE_CONTINUOUS -> MODE_STEP_TO_NEXT_TURN -> MODE_SINGLE_STEP
 */
void setStepMode(uint8_t aStepMode) {
    if (aStepMode == MODE_SINGLE_STEP) {
        RobotCarPWMMotorControl.stopAndWaitForIt();
    } else if (aStepMode > MODE_SINGLE_STEP) {
        aStepMode = MODE_CONTINUOUS;
    }
    sStepMode = aStepMode;
    setStepModeButtonCaption();
    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        TouchButtonStepMode.drawButton();
    }
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
    if (sDriveMode != MODE_MANUAL_DRIVE) {
        startStopAutomomousDrive(true, MODE_COLLISION_AVOIDING_BUILTIN);
    }
}

void setStepModeButtonCaption() {
    TouchButtonStepMode.setCaptionFromStringArrayPGM(sStepModeButtonCaptionStringArray, sStepMode);
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
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

#endif // defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)

void doChangeScanSpeed(BDButton *aTheTouchedButton, int16_t aValue) {
    sDoSlowScan = aValue;
}

void doSingleScan(BDButton *aTheTouchedButton, int16_t aValue) {
    if (sDriveMode == MODE_FOLLOWER) {
        scanForTarget(FOLLOWER_DISTANCE_TARGET_SCAN_CENTIMETER);
    } else {
#if defined(CAR_HAS_DISTANCE_SERVO)
        clearPrintedForwardDistancesInfos();
#endif
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
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doNextStepMode);

    TouchButtonSingleScan.init(0, BUTTON_HEIGHT_6_LINE_2, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR16_BLUE, F("Scan"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doSingleScan);

    TouchButtonStep.init(0, BUTTON_HEIGHT_6_LINE_3, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR16_BLUE, F("Step"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStep);

#if defined(ENABLE_PATH_INFO_PAGE)
    TouchButtonPathInfoPage.init(0, BUTTON_HEIGHT_6_LINE_4, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR16_RED, F("Show\nPath"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_SHOW_PATH, &GUISwitchPages);
#endif

    // small buttons
    // use sDriveMode to support reconnect during demo mode
    TouchButtonStartStopUserAutonomousDrive.init(0, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR16_RED, F("Start User"), TEXT_SIZE_14,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, (sDriveMode == MODE_COLLISION_AVOIDING_USER),
            &doStartStopTestUser);
    TouchButtonStartStopUserAutonomousDrive.setCaptionForValueTrue(F("Stop User"));

    TouchButtonScanSpeed.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR16_BLACK, F("Scan slow"), TEXT_SIZE_16,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doChangeScanSpeed);
    TouchButtonScanSpeed.setCaptionForValueTrue("Scan fast");

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
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

    // left button column
    TouchButtonStepMode.drawButton();
    TouchButtonSingleScan.drawButton();
    TouchButtonStep.drawButton();

    // small buttons
    TouchButtonStartStopUserAutonomousDrive.drawButton();
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.drawButton();
#endif
    TouchButtonScanSpeed.drawButton();

    // bottom buttons line
    TouchButtonStartStopBuiltInAutonomousDrive.drawButton();
    TouchButtonFollower.drawButton();
    TouchButtonBack.drawButton();
#if defined(ENABLE_PATH_INFO_PAGE)
    TouchButtonPathInfoPage.drawButton();
#endif
}

void startAutonomousDrivePage(void) {
    // set correct values of radio buttons
    handleAutomomousDriveRadioButtons();

    // restore last step and scan mode
    setStepModeButtonCaption();
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
    setScanModeButtonCaption();
#endif

    drawAutonomousDrivePage();
}

// currently not used
void loopAutonomousDrivePage(void) {
// Autonomous driving is done in an extra loop in AutonomousDrive.cpp driveAutonomousOneStep(), since it is independent of the display.
}

void stopAutonomousDrivePage(void) {
    startStopRobotCar(false);
}

/*
 * Clear drawing area
 * X from TouchButtonStep to end of display
 * Y from TouchButtonBackSmall to TouchButtonScanSpeed
 */
void clearPrintedForwardDistancesInfos() {
    BlueDisplay1.fillRect(BUTTON_WIDTH_3_5 + 1, BUTTON_HEIGHT_4 + 1, LAYOUT_320_WIDTH,
    BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - 1, COLOR16_WHITE);
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
            sprintf_P(sStringBuffer, PSTR("wall%4d\xB0 rotation: %3d\xB0 wall%4d\xB0"), sForwardDistancesInfo.WallLeftAngleDegrees,
                    aDegreeToTurn, sForwardDistancesInfo.WallRightAngleDegrees); // \xB0 is degree character
            BlueDisplay1.drawText(BUTTON_WIDTH_3_5_POS_2, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11, sStringBuffer, TEXT_SIZE_11,
            COLOR16_BLACK, COLOR16_WHITE);
        }
    }
}
#endif // _ROBOT_CAR_AUTOMOMOUS_DRIVE_PAGE_HPP
#pragma once
