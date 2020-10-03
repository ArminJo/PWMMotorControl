/*
 * AutonomousDrivePage.cpp
 *
 *  Contains all the GUI elements for autonomous driving.
 *  Cont ->Step / Step -> SStep, SStep->Cont: Switches mode from "continuous drive" to "drive until next turn" to "drive CENTIMETER_PER_RIDE_PRO"
 *  Start Simple: Start simple driving algorithm (using the 2 "simple" functions in RobotCarMotorControl.cpp)
 *  Start Pro: Start elaborated driving algorithm
 *
 *  Requires BlueDisplay library.
 *
 *  Copyright (C) 2019-2020  Armin Joachimsmeyer
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

BDButton TouchButtonStepMode;
const char sStepModeButtonStringContinuousStepToTurn[] PROGMEM = "Continuous\n->\nStep to turn";
const char sStepModeButtonStringStepToTurnSingleStep[] PROGMEM = "Step to turn\n->\nSingle step";
const char sStepModeButtonStringSingleStepContinuous[] PROGMEM = "Single step\n->\nContinuous";
const char * const sStepModeButtonCaptionStringArray[] PROGMEM = { sStepModeButtonStringContinuousStepToTurn,
        sStepModeButtonStringStepToTurnSingleStep, sStepModeButtonStringSingleStepContinuous };

BDButton TouchButtonStep;
BDButton TouchButtonSingleScan;
BDButton TouchButtonScanSpeed;

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
BDButton TouchButtonScanMode;
const char sScanModeButtonStringMinMax[] PROGMEM = "Min->Max";
const char sScanModeButtonStringMaxUS[] PROGMEM = "Max->US";
#  ifdef CAR_HAS_IR_DISTANCE_SENSOR
const char sScanModeButtonStringUSIr[] PROGMEM = "US->IR";
const char sScanModeButtonStringIrMin[] PROGMEM = "IR->Min";
const char * const sScanModeButtonCaptionStringArray[] PROGMEM = { sScanModeButtonStringMinMax, sScanModeButtonStringMaxUS,
        sScanModeButtonStringUSIr, sScanModeButtonStringIrMin };
# else
const char sScanModeButtonStringUSTof[] PROGMEM = "US->ToF";
const char sScanModeButtonStringTofMin[] PROGMEM = "ToF->Min";

const char * const sScanModeButtonCaptionStringArray[] PROGMEM = { sScanModeButtonStringMinMax, sScanModeButtonStringMaxUS,
        sScanModeButtonStringUSTof, sScanModeButtonStringTofMin};
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
        RobotCarMotorControl.stopCarAndWaitForIt();
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
void doNextStepMode(BDButton * aTheTouchedButton, int16_t aValue) {
    sStepMode++;
    setStepMode(sStepMode);
}

/*
 * enables next step
 */
void doStep(BDButton * aTheTouchedButton, int16_t aValue) {
    if (sStepMode == MODE_CONTINUOUS) {
        // switch to step mode MODE_SINGLE_STEP
        setStepMode(MODE_SINGLE_STEP);
    }
    sDoStep = true;
    /*
     * Start if not yet done
     */
    if (!sRuningAutonomousDrive) {
        startStopAutomomousDrive(true, MODE_AUTONOMOUS_DRIVE_BUILTIN);
    }
}

void setStepModeButtonCaption() {
    TouchButtonStepMode.setCaptionFromStringArrayPGM(sStepModeButtonCaptionStringArray, sStepMode);
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
void setScanModeButtonCaption() {
    TouchButtonScanMode.setCaptionFromStringArrayPGM(sScanModeButtonCaptionStringArray, sScanMode);
}

void doScanMode(BDButton * aTheTouchedButton, int16_t aValue) {
    sScanMode++;
    if (sScanMode > SCAN_MODE_IR) {
        sScanMode = SCAN_MODE_MINIMUM;
    }
    setScanModeButtonCaption();
    TouchButtonScanMode.drawButton();
}

#endif // defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)

void doChangeScanSpeed(BDButton * aTheTouchedButton, int16_t aValue) {
    sDoSlowScan = aValue;
}

void doSingleScan(BDButton * aTheTouchedButton, int16_t aValue) {
    if (sDriveMode == MODE_FOLLOWER) {
        scanForTarget();
    } else {
        clearPrintedForwardDistancesInfos();
        fillAndShowForwardDistancesInfo(true, true);

        postProcessAndCollisionDetection();
    }
}

void doStartStopFollowerMode(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, MODE_FOLLOWER);
}

void doStartStopAutomomousDrive(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, MODE_AUTONOMOUS_DRIVE_BUILTIN);
}

void doStartStopTestUser(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopAutomomousDrive(aValue, MODE_AUTONOMOUS_DRIVE_USER);
}

/*
 * set buttons accordingly to sDriveMode
 */
void handleAutomomousDriveRadioButtons() {
    TouchButtonStartStopBuiltInAutonomousDrive.setValue(sDriveMode == MODE_AUTONOMOUS_DRIVE_BUILTIN,
            sCurrentPage == PAGE_AUTOMATIC_CONTROL);
    TouchButtonStartStopUserAutonomousDrive.setValue(sDriveMode == MODE_AUTONOMOUS_DRIVE_USER,
            sCurrentPage == PAGE_AUTOMATIC_CONTROL);
    TouchButtonFollower.setValue(sDriveMode == MODE_FOLLOWER, sCurrentPage == PAGE_AUTOMATIC_CONTROL);
}

void initAutonomousDrivePage(void) {
    TouchButtonStepMode.init(0, 0, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6 + 1, COLOR_BLUE,
            reinterpret_cast<const __FlashStringHelper *>(sStepModeButtonStringContinuousStepToTurn), TEXT_SIZE_9,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doNextStepMode);

    TouchButtonSingleScan.init(0, BUTTON_HEIGHT_6_LINE_2, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR_BLUE, F("Scan"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doSingleScan);

    TouchButtonStep.init(0, BUTTON_HEIGHT_6_LINE_3, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR_BLUE, F("Step"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doStep);

    // use sDriveMode for reconnect during demo mode
    TouchButtonFollower.init(0, BUTTON_HEIGHT_6_LINE_4, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR_RED, F("Follow"),
    TEXT_SIZE_16, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, (sDriveMode == MODE_FOLLOWER),
            &doStartStopFollowerMode);

    TouchButtonScanSpeed.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR_BLACK, F("Scan slow"), TEXT_SIZE_16,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doChangeScanSpeed);
    TouchButtonScanSpeed.setCaptionForValueTrue("Scan fast");

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER),
    BUTTON_WIDTH_3, TEXT_SIZE_22_HEIGHT, COLOR_RED, reinterpret_cast<const __FlashStringHelper *>(sScanModeButtonStringMinMax),
    TEXT_SIZE_16, FLAG_BUTTON_DO_BEEP_ON_TOUCH, SCAN_MODE_MINIMUM, &doScanMode);
#endif

    // use sDriveMode for reconnect during demo mode
    TouchButtonStartStopBuiltInAutonomousDrive.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED,
            F("Start\nBuiltin"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN,
            (sDriveMode == MODE_AUTONOMOUS_DRIVE_BUILTIN), &doStartStopAutomomousDrive);
    TouchButtonStartStopBuiltInAutonomousDrive.setCaptionForValueTrue(F("Stop"));

    TouchButtonStartStopUserAutonomousDrive.init(BUTTON_WIDTH_3_POS_2, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4,
    COLOR_RED, F("Start\nUser"), TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false,
            &doStartStopTestUser);
    TouchButtonStartStopUserAutonomousDrive.setCaptionForValueTrue(F("Stop\nUser"));

}

void drawAutonomousDrivePage(void) {
    drawCommonGui();

    BlueDisplay1.drawText(HEADER_X - (TEXT_SIZE_22_WIDTH / 2), (2 * TEXT_SIZE_22_HEIGHT), F("Auto drive"));

    TouchButtonBackSmall.drawButton();

    TouchButtonStepMode.drawButton();
    TouchButtonSingleScan.drawButton();
    TouchButtonStep.drawButton();
    TouchButtonFollower.drawButton();

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    TouchButtonScanMode.drawButton();
#endif
    TouchButtonScanSpeed.drawButton();

    TouchButtonStartStopBuiltInAutonomousDrive.drawButton();
    TouchButtonStartStopUserAutonomousDrive.drawButton();
    TouchButtonNextPage.drawButton(); // Show Path button
}

void startAutonomousDrivePage(void) {
    TouchButtonStartStopUserAutonomousDrive.setValue(sDriveMode == MODE_AUTONOMOUS_DRIVE_USER);
    TouchButtonStartStopBuiltInAutonomousDrive.setValue(sDriveMode == MODE_AUTONOMOUS_DRIVE_BUILTIN);
    setStepModeButtonCaption();
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    setScanModeButtonCaption();
#endif
    TouchButtonNextPage.setCaption(F("Show\nPath"));

    drawAutonomousDrivePage();
}

void loopAutonomousDrivePage(void) {
// Autonomous driving is done in an extra loop in AutonomousDrive.cpp -> doAutonomousDrive(), since it is independent of the display.
}

void stopAutonomousDrivePage(void) {
// no action needed
}

/*
 * Clear drawing area
 * X from TouchButtonStep to end of display
 * Y from TouchButtonBackSmall to TouchButtonScanSpeed
 */
void clearPrintedForwardDistancesInfos() {
    BlueDisplay1.fillRect(BUTTON_WIDTH_3_5 + 1, BUTTON_HEIGHT_4 + 1, LAYOUT_320_WIDTH,
    BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - 1, COLOR_WHITE);
}

/*
 * Draw values of ActualDistancesArray as vectors
 * Not used yet
 */
void drawForwardDistancesInfos() {
    color16_t tColor;
    uint8_t tCurrentDegrees = 0;
    /*
     * Clear drawing area
     */
    clearPrintedForwardDistancesInfos();
    for (int i = 0; i < NUMBER_OF_DISTANCES; ++i) {
        /*
         * Determine color
         */
        uint8_t tDistance = sForwardDistancesInfo.RawDistancesArray[i];
        tColor = COLOR_ORANGE;
        if (tDistance >= DISTANCE_TIMEOUT_CM) {
            tDistance = DISTANCE_TIMEOUT_CM;
            tColor = COLOR_GREEN;
        }
        if (tDistance > sCentimeterPerScanTimesTwo) {
            tColor = COLOR_GREEN;
        } else if (tDistance < sCentimeterPerScan) {
            tColor = COLOR_RED;
        }

        /*
         * Draw line
         */
        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, tDistance, tCurrentDegrees, tColor, 3);
        tCurrentDegrees += DEGREES_PER_STEP;
    }
}

/*
 * Draws only if sCurrentPage == PAGE_AUTOMATIC_CONTROL
 */
void drawCollisionDecision(int aDegreeToTurn, uint8_t aLengthOfVector, bool aDoClearVector) {

    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        color16_t tColor = COLOR_BLUE;
        int tDegreeToDisplay = aDegreeToTurn;

        if (tDegreeToDisplay == 180) {
            tColor = COLOR_RED;
            tDegreeToDisplay = 0;
        }
        if (aDoClearVector) {
            tColor = COLOR_WHITE;
        }

        BlueDisplay1.drawVectorDegrees(US_DISTANCE_MAP_ORIGIN_X, US_DISTANCE_MAP_ORIGIN_Y, aLengthOfVector, tDegreeToDisplay + 90,
                tColor);
        if (!aDoClearVector) {
            sprintf_P(sStringBuffer, PSTR("wall%4d\xB0 rotation: %3d\xB0 wall%4d\xB0"), sForwardDistancesInfo.WallLeftAngleDegrees,
                    aDegreeToTurn, sForwardDistancesInfo.WallRightAngleDegrees); // \xB0 is degree character
            BlueDisplay1.drawText(BUTTON_WIDTH_3_5_POS_2, US_DISTANCE_MAP_ORIGIN_Y + TEXT_SIZE_11, sStringBuffer, TEXT_SIZE_11,
            COLOR_BLACK, COLOR_WHITE);
        }
    }
}

