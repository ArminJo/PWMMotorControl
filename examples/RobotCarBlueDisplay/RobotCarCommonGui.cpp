/*
 * RobotCarCommonGui.cpp
 *
 *  Contains all common GUI elements for operating and controlling the RobotCarMotorControl.
 *
 *  Calibration: Sets lowest speed for which wheels are moving.
 *  Speed Slider left: Sets speed for manual control which serves also as maximum speed for autonomous drive if "Stored"
 *  Store: Stores calibration info and maximum speed.
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

// a string buffer for BD info output
char sStringBuffer[128];

bool sSensorCallbacksEnabled;

uint8_t sCurrentPage;
BDButton TouchButtonBack;
BDButton TouchButtonAutomaticDrivePage;
BDButton TouchButtonCalibrate;
BDButton TouchButtonCompensationRight;
BDButton TouchButtonCompensationLeft;
#ifdef SUPPORT_EEPROM_STORAGE
BDButton TouchButtonCompensationStore;
#endif

BDButton TouchButtonRobotCarStartStop;
//bool sRobotCarMoving; // main start flag. If true motors are running and will be updated by main loop.

BDButton TouchButtonDirection;
uint8_t sRobotCarDirection = DIRECTION_FORWARD; // DIRECTION_FORWARD or DIRECTION_BACKWARD

BDSlider SliderSpeed;
uint16_t sLastSpeedSliderValue = 0; // local storage for requested speed by speed slider

BDSlider SliderSpeedRight;
BDSlider SliderSpeedLeft;

/*
 * UltraSonic control GUI
 */
BDSlider SliderUSPosition;
BDSlider SliderUSDistance;
unsigned int sSliderUSLastCentimeter;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
BDSlider SliderIRDistance;
unsigned int sSliderIRLastCentimeter;
#endif

uint32_t sMillisOfNextVCCInfo = 0;

void setupGUI(void) {
    sCurrentPage = PAGE_HOME;

    // Register callback handler and check for connection
    // This leads to call to initDisplay() and startCurrentPage() after connect
    BlueDisplay1.initCommunication(&initRobotCarDisplay, &startCurrentPage);
}

void delayAndLoopGUI(uint16_t aDelayMillis) {
    uint32_t tStartMillis = millis();
    do {
        loopGUI();
    } while (millis() - tStartMillis <= aDelayMillis);
}

void loopGUI(void) {

    if (BlueDisplay1.isConnectionEstablished()) {
        // do not show anything during motor speed ramps
        if (!RobotCarMotorControl.isState(MOTOR_STATE_RAMP_UP)) {

            /*
             * All functions are empty until now
             * actions are centrally managed below
             */
//            if (sCurrentPage == PAGE_HOME) {
//                loopHomePage();
//            } else if (sCurrentPage == PAGE_TEST) {
//                loopTestPage();
//            } else if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
//                loopAutonomousDrivePage();
//            } else if (sCurrentPage == PAGE_BT_SENSOR_CONTROL) {
//                loopBTSensorDrivePage();
//#ifdef ENABLE_PATH_INFO_PAGE
//            } else if (sCurrentPage == PAGE_SHOW_PATH) {
//                loopPathInfoPage();
//#endif
//            }
#ifdef ENABLE_PATH_INFO_PAGE
            // for all but PathInfo page
            if (sCurrentPage != PAGE_SHOW_PATH) {
#endif
#if defined(MONITOR_LIPO_VOLTAGE)
            readCheckAndPrintVinPeriodically();
#endif
            // For Home, sensorDrive and Test page
            if (sCurrentPage != PAGE_AUTOMATIC_CONTROL) {
                if (sCurrentPage != PAGE_BT_SENSOR_CONTROL) {
                    // For Home and Test page
                    readAndShowDistancePeriodically(DISTANCE_DISPLAY_PERIOD_MILLIS);
                }
                printMotorValues();
#ifdef USE_ENCODER_MOTOR_CONTROL
                displayVelocitySliderValues();
                /*
                 * Print changed tick values
                 */
                if (EncoderMotor::EncoderCountHasChanged) {
                    EncoderMotor::EncoderCountHasChanged = false;
                    printMotorDistanceValues();
                }
#endif
            }
#ifdef ENABLE_PATH_INFO_PAGE
            }
#endif
        }
    }
    /*
     * Check if receive buffer contains an event
     * Do it at end since it may change the condition for calling this function and its printing
     */
    checkAndHandleEvents();
}

/*
 * Handle Start/Stop
 */
void startStopRobotCar(bool aDoStart) {
    uint8_t tSpeedSliderValue = 0;
    if (aDoStart) {
        if (sCurrentPage == PAGE_BT_SENSOR_CONTROL) {
            /*
             * Start for sensor drive
             */
            sSensorChangeCallCountForZeroAdjustment = 0;
            registerSensorChangeCallback(FLAG_SENSOR_TYPE_ACCELEROMETER, FLAG_SENSOR_DELAY_NORMAL, FLAG_SENSOR_NO_FILTER,
                    &doSensorChange);
            // Lock the screen orientation to avoid screen flip while rotating the smartphone
            BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_CURRENT);
            sSensorCallbacksEnabled = true;
        } else if (sLastSpeedSliderValue > 0) {
            /*
             * Start car to last speed slider value
             */
            RobotCarMotorControl.setSpeedCompensated(sLastSpeedSliderValue, sRobotCarDirection);
            tSpeedSliderValue = sLastSpeedSliderValue;
        }
    } else {
        /*
         * Stop car
         */
        if (sRuningAutonomousDrive) {
            startStopAutomomousDrive(false);
        }
        if (sSensorCallbacksEnabled) {
            /*
             * Global stop for sensor drive
             */
            registerSensorChangeCallback(FLAG_SENSOR_TYPE_ACCELEROMETER, FLAG_SENSOR_DELAY_NORMAL, FLAG_SENSOR_NO_FILTER, NULL);
            BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_UNLOCK);
            sSensorCallbacksEnabled = false;
        }
        RobotCarMotorControl.stop();
    }

    bool tShowValues = sCurrentPage == PAGE_HOME || sCurrentPage == PAGE_TEST;
    // set button value always just for the case we are called by another than Stop button callback
    TouchButtonRobotCarStartStop.setValue(!RobotCarMotorControl.isStopped() || sSensorCallbacksEnabled, tShowValues);
    if (tShowValues) {
        SliderSpeed.setValueAndDrawBar(tSpeedSliderValue);
    }

}

#pragma GCC diagnostic ignored "-Wunused-parameter"

void doRobotCarStartStop(BDButton * aTheTouchedButton, int16_t aDoStart) {
    startStopRobotCar(aDoStart);
}

/*
 * stop and reset motors
 */
void doReset(BDButton * aTheTouchedButton, int16_t aValue) {
    startStopRobotCar(false);
    RobotCarMotorControl.resetControlValues();
}

#ifdef USE_ENCODER_MOTOR_CONTROL
void doCalibrate(BDButton * aTheTouchedButton, int16_t aValue) {
    RobotCarMotorControl.calibrate();
    printMotorValues();
#ifdef SUPPORT_EEPROM_STORAGE
    RobotCarMotorControl.writeMotorValuesToEeprom();
#endif
}
#endif

/*
 * Minimum Speed is 30 for USB power and no load, 50 for load
 * Minimum Speed is 20 for 2 Lithium 18650 battery power and no load, 25 for load
 */
void doSpeedSlider(BDSlider *aTheTouchedSlider, uint16_t aValue) {
    if (aValue != sLastSpeedSliderValue) {
        sLastSpeedSliderValue = aValue;

        if (RobotCarMotorControl.isStopped()) {
            // handle GUI
            startStopRobotCar(true);
        } else {
            RobotCarMotorControl.setSpeedCompensated(aValue, sRobotCarDirection);
        }
    }
}

void doUSServoPosition(BDSlider *aTheTouchedSlider, uint16_t aValue) {
    DistanceServoWriteAndDelay(aValue);
}

#ifdef USE_ENCODER_MOTOR_CONTROL
/*
 * Display velocity as slider values
 */
void displayVelocitySliderValues() {
    EncoderMotor *tMotorInfo = &RobotCarMotorControl.leftCarMotor;
    BDSlider *tSliderPtr = &SliderSpeedLeft;
    uint16_t tXPos = 0;
    for (int i = 0; i < 2; ++i) {
        if (EncoderMotor::EncoderCountHasChanged) {
            tSliderPtr->setValueAndDrawBar(tMotorInfo->getSpeed());
        }
        tMotorInfo = &RobotCarMotorControl.rightCarMotor;
        tSliderPtr = &SliderSpeedRight;
        tXPos += BUTTON_WIDTH_16 + 4;
    }
}
#endif

/*
 * Stops car and change direction
 */
void doSetDirection(BDButton * aTheTouchedButton, int16_t aValue) {
    sRobotCarDirection = !aValue; // use inverse value since true is forward BUT 0 is DIRECTION_FORWARD

// Stop fixed directions and turns using RobotCarMotorControl
    if (!RobotCarMotorControl.isStopped()) {
        RobotCarMotorControl.stop();
    }
// Stop direct movement by slider
    startStopRobotCar(false);
}

/*
 * changes speed compensation by +1 or -1
 */
void doSetCompensation(BDButton * aTheTouchedButton, int16_t aRightMotorSpeedCompensation) {
    RobotCarMotorControl.changeSpeedCompensation(aRightMotorSpeedCompensation);
}

#ifdef SUPPORT_EEPROM_STORAGE
void doStoreCompensation(BDButton * aTheTouchedButton, int16_t aRightMotorSpeedCompensation) {
    RobotCarMotorControl.writeMotorValuesToEeprom();
}
#endif

void startCurrentPage() {
    switch (sCurrentPage) {
    case PAGE_HOME:
        startHomePage();
        break;
    case PAGE_AUTOMATIC_CONTROL:
        startAutonomousDrivePage();
        break;
    case PAGE_BT_SENSOR_CONTROL:
        startBTSensorDrivePage();
        break;
    case PAGE_TEST:
        startTestPage();
        break;
#ifdef ENABLE_PATH_INFO_PAGE
    case PAGE_SHOW_PATH:
        startPathInfoPage();
        break;
#endif

    }
}

/*
 * For Next and Back button
 * Stop old page and start new one
 * @param aValue the Page number of the new page number
 */
void GUISwitchPages(BDButton * aTheTouchedButton, int16_t aValue) {
    /*
     * Stop old page
     */
    switch (sCurrentPage) {
    case PAGE_HOME:
        stopHomePage();
        break;
    case PAGE_AUTOMATIC_CONTROL:
        stopAutonomousDrivePage();
        break;
    case PAGE_BT_SENSOR_CONTROL:
        stopBTSensorDrivePage();
        break;
    case PAGE_TEST:
        stopTestPage();
        break;
#ifdef ENABLE_PATH_INFO_PAGE
    case PAGE_SHOW_PATH:
        stopPathInfoPage();
        aValue = PAGE_AUTOMATIC_CONTROL; // only back to autonomous page permitted
        break;
#endif
    }

    /*
     * Start new page
     */
    sCurrentPage = aValue;
    startCurrentPage();
}

void initRobotCarDisplay(void) {

    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_TOUCH_BASIC_DISABLE | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH,
    DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0x87, 0x2227); // mapping for AND - Forward
    BlueDisplay1.setCharacterMapping(0x88, 0x2228); // mapping for OR - Backwards
// Lock to landscape layout
    BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_SENSOR_LANDSCAPE);

    /*
     * Common control buttons
     */
    TouchButtonRobotCarStartStop.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_BLUE, F("Start"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doRobotCarStartStop);
    TouchButtonRobotCarStartStop.setCaptionForValueTrue(F("Stop"));

    TouchButtonBack.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR_RED, F("Back"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_HOME, &GUISwitchPages);

#ifdef USE_ENCODER_MOTOR_CONTROL
    TouchButtonCalibrate.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_RED, F("CAL"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doCalibrate);
#endif

// Direction Button value true is forward, false is backward BUT 0 is DIRECTION_FORWARD!!!
    TouchButtonDirection.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_6, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLACK, F("\x88"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, true, &doSetDirection);
    TouchButtonDirection.setCaptionForValueTrue("\x87");

    TouchButtonCompensationLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4,
    BUTTON_WIDTH_8 + (BUTTON_DEFAULT_SPACING_QUARTER - 1), BUTTON_HEIGHT_8, COLOR_BLUE, F("<-Co"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, -1, &doSetCompensation);
    TouchButtonCompensationRight.init(BUTTON_WIDTH_8_POS_5 - (BUTTON_DEFAULT_SPACING_QUARTER - 1), BUTTON_HEIGHT_8_LINE_4,
    BUTTON_WIDTH_8 + (BUTTON_DEFAULT_SPACING_QUARTER - 1), BUTTON_HEIGHT_8, COLOR_BLUE, F("mp->"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doSetCompensation);
#ifdef SUPPORT_EEPROM_STORAGE
    TouchButtonCompensationStore.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR_BLUE,
            F("Store"), TEXT_SIZE_10, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doStoreCompensation);
#endif
    /*
     * Speed Sliders
     */
    SliderSpeed.init(0, SLIDER_TOP_MARGIN, BUTTON_WIDTH_6, SPEED_SLIDER_SIZE, 200, 0, COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE, &doSpeedSlider);
    SliderSpeed.setScaleFactor(255.0 / SPEED_SLIDER_SIZE); // Slider is virtually 2 times larger than displayed, values were divided by 2

    SliderSpeedLeft.init(BUTTON_WIDTH_6 + 4, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2, SPEED_SLIDER_SIZE / 2 - 1, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderSpeedLeft.setValueFormatString("%3d"); // Since we also send values grater 100

    SliderSpeedRight.init(BUTTON_WIDTH_6 + 4 + BUTTON_WIDTH_16 + 8, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2,
    SPEED_SLIDER_SIZE / 2 - 1, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderSpeedRight.setValueFormatString("%3d");

    /*
     * Right sliders
     * From right to left:
     * 1. Scaled (0 to 180) US position slider
     * 2. Scaled (0 to 200) small US distance slider. If IR distance or pan and tilt servos existent, the it is a small one without cm units.
     * 3. Scaled (0 to 200) small IR distance slider / big US small US distance slider. Not displayed on home page if pan and tilt servos existent.
     * 3. Pan position slider. If tilt servo is existent then at position 3.
     * 4. Tilt position slider.
     *
     * Position X values determine the RIGHT edge + 1. The width of the slider must be subtracted for SliderX parameter.
     */
#define POS_X_US_POSITION_SLIDER    LAYOUT_320_WIDTH // 320
#define POS_X_US_DISTANCE_SLIDER    ((POS_X_US_POSITION_SLIDER - BUTTON_WIDTH_6) - 2) // - (width of US position slider + 2 for gap)
#define POS_X_THIRD_SLIDER          (POS_X_US_DISTANCE_SLIDER - (BUTTON_WIDTH_10 / 2))  // - (width of small US distance slider + 2 for gap)
#if  defined(CAR_HAS_PAN_SERVO) && defined(CAR_HAS_TILT_SERVO)
#define POS_X_PAN_SLIDER            (POS_X_THIRD_SLIDER)  //
#define POS_X_TILT_SLIDER           ((POS_X_THIRD_SLIDER - BUTTON_WIDTH_12) - 4) // - width of pan slider + 4 for gap
#else
#define POS_X_PAN_SLIDER            ((POS_X_US_DISTANCE_SLIDER - BUTTON_WIDTH_10 - 2) // - width of 1 big or 2 small sliders + 2 for gap
#endif

    SliderUSPosition.init(POS_X_US_POSITION_SLIDER - BUTTON_WIDTH_6, SLIDER_TOP_MARGIN, BUTTON_WIDTH_6, US_SLIDER_SIZE, 90, 90,
    COLOR_YELLOW,
    SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doUSServoPosition);
    SliderUSPosition.setBarThresholdColor(COLOR_BLUE);
    SliderUSPosition.setScaleFactor(180.0 / US_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderUSPosition.setValueUnitString("\xB0"); // \xB0 is degree character

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR) || (defined(CAR_HAS_PAN_SERVO) && defined(CAR_HAS_TILT_SERVO))
#define US_DISTANCE_SLIDER_IS_SMALL
#endif
    /*
     * One thin or thick US distance slider
     */
#if defined(US_DISTANCE_SLIDER_IS_SMALL)
    // Small US distance slider with captions and without cm units
    SliderUSDistance.init(POS_X_US_DISTANCE_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2), SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8,
            (BUTTON_WIDTH_10 / 2) - 2,
            DISTANCE_SLIDER_SIZE, DISTANCE_TIMEOUT_CM_FOLLOWER / DISTANCE_SLIDER_SCALE_FACTOR, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderUSDistance.setCaptionProperties(TEXT_SIZE_10, FLAG_SLIDER_CAPTION_ALIGN_LEFT | FLAG_SLIDER_CAPTION_BELOW, 2, COLOR_BLACK,
    COLOR_WHITE);
    SliderUSDistance.setCaption("US");
    // below caption - left aligned
    SliderUSDistance.setPrintValueProperties(11, FLAG_SLIDER_CAPTION_ALIGN_LEFT | FLAG_SLIDER_CAPTION_BELOW,
            4 + TEXT_SIZE_10_HEIGHT, COLOR_BLACK, COLOR_WHITE);
#else
    // Big US distance slider without caption but with cm units POS_X_THIRD_SLIDER because it is the position of the left edge
    SliderUSDistance.init(POS_X_US_DISTANCE_SLIDER - BUTTON_WIDTH_10, SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8, BUTTON_WIDTH_10,
    DISTANCE_SLIDER_SIZE, DISTANCE_TIMEOUT_CM_FOLLOWER / DISTANCE_SLIDER_SCALE_FACTOR, 0, SLIDER_DEFAULT_BACKGROUND_COLOR,
            SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderUSDistance.setValueUnitString("cm");
#endif
    SliderUSDistance.setScaleFactor(DISTANCE_SLIDER_SCALE_FACTOR); // Slider is virtually 2 times larger, values were divided by 2
    SliderUSDistance.setBarThresholdColor(DISTANCE_TIMEOUT_COLOR);

    /*
     * One thin IR distance slider
     */
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
    // Small IR distance slider with captions and without cm units
    SliderIRDistance.init(POS_X_THIRD_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2), SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8,
            (BUTTON_WIDTH_10 / 2) - 2,
            DISTANCE_SLIDER_SIZE, DISTANCE_TIMEOUT_CM_FOLLOWER / DISTANCE_SLIDER_SCALE_FACTOR, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderIRDistance.setScaleFactor(DISTANCE_SLIDER_SCALE_FACTOR); // Slider is virtually 2 times larger, values were divided by 2
    SliderIRDistance.setBarThresholdColor(DISTANCE_TIMEOUT_COLOR);
    // Caption properties
    SliderIRDistance.setCaptionProperties(TEXT_SIZE_10, FLAG_SLIDER_CAPTION_ALIGN_RIGHT | FLAG_SLIDER_CAPTION_BELOW, 2, COLOR_BLACK,
    COLOR_WHITE);
    // Captions
    SliderIRDistance.setCaption("IR");
    // value below caption - right aligned
    SliderIRDistance.setPrintValueProperties(11, FLAG_SLIDER_CAPTION_ALIGN_RIGHT | FLAG_SLIDER_CAPTION_BELOW,
            4 + TEXT_SIZE_10_HEIGHT, COLOR_BLACK, COLOR_WHITE);
#endif

#ifdef CAR_HAS_PAN_SERVO
    // left of SliderUSPosition
    SliderPan.init(POS_X_PAN_SLIDER - BUTTON_WIDTH_12, SLIDER_TOP_MARGIN, BUTTON_WIDTH_12, LASER_SLIDER_SIZE, 90, 90, COLOR_YELLOW,
    SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doHorizontalServoPosition);
    SliderPan.setBarThresholdColor(COLOR_BLUE);
    // scale slider values
    SliderPan.setScaleFactor(180.0 / LASER_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderPan.setValueUnitString("\xB0"); // \xB0 is degree character
#endif

#ifdef CAR_HAS_TILT_SERVO
    SliderTilt.init(POS_X_TILT_SLIDER - BUTTON_WIDTH_12, SLIDER_TOP_MARGIN, BUTTON_WIDTH_12, LASER_SLIDER_SIZE, 90,
    TILT_SERVO_MIN_VALUE, COLOR_YELLOW, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doVerticalServoPosition);
    SliderTilt.setBarThresholdColor(COLOR_BLUE);
    // scale slider values
    SliderTilt.setScaleFactor(180.0 / LASER_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderTilt.setValueUnitString("\xB0"); // \xB0 is degree character
#endif

    initHomePage();
    initTestPage();
    initAutonomousDrivePage();
    initBTSensorDrivePage();
#ifdef ENABLE_PATH_INFO_PAGE
    initPathInfoPage();
#endif
}

void drawCommonGui(void) {
    clearDisplayAndDisableButtonsAndSliders();

    BlueDisplay1.drawText(HEADER_X, TEXT_SIZE_22_HEIGHT, F("Robot Car"), TEXT_SIZE_22, COLOR_BLUE,
    COLOR_NO_BACKGROUND);
}

#if defined(MONITOR_LIPO_VOLTAGE)
/*
 * Print VIN (used as motor supply) periodically
 * returns true if voltage was printed
 */
void readAndPrintVin() {
    char tDataBuffer[18];
    char tVCCString[6];
    readVINVoltage();
    dtostrf(sVINVoltage, 4, 2, tVCCString);
    sprintf_P(tDataBuffer, PSTR("%s volt"), tVCCString);
    uint16_t tPosX;
    uint8_t tPosY;
    if (sCurrentPage == PAGE_HOME) {
        tPosX = BUTTON_WIDTH_8_POS_4;
        tPosY = BUTTON_HEIGHT_8_LINE_6 + TEXT_SIZE_11_DECEND;

    } else if (sCurrentPage == PAGE_AUTOMATIC_CONTROL || sCurrentPage == PAGE_BT_SENSOR_CONTROL) {
        tPosX = BUTTON_WIDTH_3_POS_2 - BUTTON_DEFAULT_SPACING - (8 * TEXT_SIZE_11_WIDTH);
        tPosY = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;

    } else {
        // Test page
        tPosX = BUTTON_WIDTH_3_POS_3 - BUTTON_DEFAULT_SPACING - (8 * TEXT_SIZE_11_WIDTH);
        tPosY = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
    }
    BlueDisplay1.drawText(tPosX, tPosY, tDataBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

}

void checkForLowVoltage() {
    static uint8_t sLowVoltageCount = 0;
    if (sVINVoltage < VOLTAGE_LOW_THRESHOLD && sVINVoltage > VOLTAGE_USB_THRESHOLD) {
        sLowVoltageCount++;
    } else if (sLowVoltageCount > 1) {
        sLowVoltageCount--;
    }
    if (sLowVoltageCount > 2) {
        // Here more than 2 consecutive times (for 6 seconds) low voltage detected
        startStopRobotCar(false);

        if (BlueDisplay1.isConnectionEstablished()) {
            drawCommonGui();
            BlueDisplay1.drawText(10, 50, F("Battery voltage"), TEXT_SIZE_33, COLOR_RED, COLOR_WHITE);
            // print current "too low" voltage
            char tDataBuffer[18];
            char tVCCString[6];
            dtostrf(sVINVoltage, 4, 2, tVCCString);
            sprintf_P(tDataBuffer, PSTR("%s volt"), tVCCString);
            BlueDisplay1.drawText(80, 50 + TEXT_SIZE_33_HEIGHT, tDataBuffer);
            BlueDisplay1.drawText(10 + (4 * TEXT_SIZE_33_WIDTH), 50 + (2 * TEXT_SIZE_33_HEIGHT), F("too low"));
        }

        tone(PIN_BUZZER, 2200, 100);
        delay(200);
        tone(PIN_BUZZER, 2200, 100);

        if (BlueDisplay1.isConnectionEstablished()) {
            uint8_t tLoopCount = VOLTAGE_TOO_LOW_DELAY_ONLINE / 500; // 12
            do {
                readAndPrintVin(); // print current voltage
                delayMillisWithCheckAndHandleEvents(500); // and wait
                tLoopCount--;
                readVINVoltage(); // read new VCC value
            } while (tLoopCount > 0 || (sVINVoltage < VOLTAGE_LOW_THRESHOLD && sVINVoltage > VOLTAGE_USB_THRESHOLD));
            // refresh current page
            GUISwitchPages(NULL, 0);
        } else {
            delay(VOLTAGE_TOO_LOW_DELAY_OFFLINE);
        }
    }
}

void readCheckAndPrintVinPeriodically() {
    uint32_t tMillis = millis();

    if (tMillis >= sMillisOfNextVCCInfo) {
        sMillisOfNextVCCInfo = tMillis + PRINT_VOLTAGE_PERIOD_MILLIS;
        readAndPrintVin();
        checkForLowVoltage();
    }
}
#endif

void printMotorSpeed() {
    if (PWMDcMotor::SpeedOrMotorModeHasChanged) {
        PWMDcMotor::SpeedOrMotorModeHasChanged = false;
// position below caption of speed slider
        uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + TEXT_SIZE_11_HEIGHT;
        sprintf_P(sStringBuffer, PSTR("speed%3d %3d"), RobotCarMotorControl.leftCarMotor.CurrentSpeed,
                RobotCarMotorControl.rightCarMotor.CurrentSpeed);
        BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    }
}

void printMotorValues() {
    printMotorSpeed();
    if (PWMDcMotor::MotorValuesHaveChanged) {
        PWMDcMotor::MotorValuesHaveChanged = false;
// position below speed values
        uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + (2 * TEXT_SIZE_11_HEIGHT);
        sprintf_P(sStringBuffer, PSTR("comp. %2d  %2d"), RobotCarMotorControl.leftCarMotor.SpeedCompensation,
                RobotCarMotorControl.rightCarMotor.SpeedCompensation);
        BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);

        tYPos += TEXT_SIZE_11;
        sprintf_P(sStringBuffer, PSTR("start%3d %3d"), RobotCarMotorControl.leftCarMotor.StartSpeed,
                RobotCarMotorControl.rightCarMotor.StartSpeed);
        BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);

        tYPos += TEXT_SIZE_11;
        sprintf_P(sStringBuffer, PSTR("drv. %3d %3d"), RobotCarMotorControl.leftCarMotor.DriveSpeed,
                RobotCarMotorControl.rightCarMotor.DriveSpeed);
        BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);
#ifdef USE_ENCODER_MOTOR_CONTROL
        if (sShowDebug && sCurrentPage == PAGE_TEST) {
            printMotorDebugValues();
        }
#endif
    }
}

#ifdef USE_ENCODER_MOTOR_CONTROL
/*
 * Is called after printMotorValues, so we can take the Draw parameter from it
 */
void printMotorDebugValues() {
    /*
     * Debug info
     */
    uint16_t tYPos = SPEED_SLIDER_SIZE / 2 + 25 + TEXT_SIZE_11_HEIGHT + (4 * TEXT_SIZE_11);
#ifdef SUPPORT_RAMP_UP
    sprintf_P(sStringBuffer, PSTR("ramp1%3d %3d"), RobotCarMotorControl.leftCarMotor.DistanceCountAfterRampUp,
            RobotCarMotorControl.rightCarMotor.DistanceCountAfterRampUp);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("endSp%3d %3d"), RobotCarMotorControl.leftCarMotor.DebugSpeedAtTargetCountReached,
            RobotCarMotorControl.rightCarMotor.DebugSpeedAtTargetCountReached);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("dcnt %3d %3d"), RobotCarMotorControl.leftCarMotor.DebugCount,
            RobotCarMotorControl.rightCarMotor.DebugCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);
    tYPos += TEXT_SIZE_11;
#endif
    sprintf_P(sStringBuffer, PSTR("debug%3d %3d"), RobotCarMotorControl.leftCarMotor.Debug,
            RobotCarMotorControl.rightCarMotor.Debug);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);

    tYPos += TEXT_SIZE_11;
    sprintf_P(sStringBuffer, PSTR("tcnt %3d %3d"), RobotCarMotorControl.leftCarMotor.LastTargetDistanceCount,
            RobotCarMotorControl.rightCarMotor.LastTargetDistanceCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer);
}

void printMotorDistanceValues() {
    uint16_t tYPos;
    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        tYPos = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
    } else {
        tYPos = SPEED_SLIDER_SIZE / 2 + 25;
    }
    sprintf_P(sStringBuffer, PSTR("cnt.%4d%4d"), RobotCarMotorControl.leftCarMotor.EncoderCount,
            RobotCarMotorControl.rightCarMotor.EncoderCount);
    BlueDisplay1.drawText(BUTTON_WIDTH_6 + 4, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}
#endif

void showUSDistance(unsigned int aCentimeter, bool aForceDraw) {
// feedback as slider length
    if (aForceDraw || aCentimeter != sSliderUSLastCentimeter) {
        sSliderUSLastCentimeter = aCentimeter;
        SliderUSDistance.setValueAndDrawBar(aCentimeter);
    }
}

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
void showIRDistance(unsigned int aCentimeter) {
// feedback as slider length
    if (aCentimeter != sSliderIRLastCentimeter) {
        sSliderIRLastCentimeter = aCentimeter;
        SliderIRDistance.setValueAndDrawBar(aCentimeter);
    }
}
#endif
