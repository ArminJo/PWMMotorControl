/*
 * RobotCarCommonGui.hpp
 *
 *  Contains all common GUI elements for operating and controlling the RobotCarPWMMotorControl.
 *
 *  Calibration: Sets lowest speed for which wheels are moving.
 *  Speed Slider left: Sets speed for manual control which serves also as maximum speed for autonomous drive if "Stored"
 *  Store: Stores calibration info and maximum speed.
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
#ifndef _ROBOT_CAR_COMMON_GUI_HPP
#define _ROBOT_CAR_COMMON_GUI_HPP
#include <Arduino.h>

#include "RobotCarPinDefinitionsAndMore.h"

#include "RobotCarBlueDisplay.h"
#include "RobotCarGui.h"
#if defined(CAR_HAS_DISTANCE_SENSOR)
#include "Distance.h"
#endif
#if defined(USE_MPU6050_IMU)
#include "IMUCarData.h"
#endif

// a string buffer for BD info output
char sStringBuffer[128];

bool sSensorCallbacksEnabled;

uint8_t sCurrentPage;
BDButton TouchButtonBack;
BDButton TouchButtonAutomaticDrivePage;
BDButton TouchButtonCompensationRight;
BDButton TouchButtonCompensationLeft;
#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
BDButton TouchButtonCalibrate;
#endif
#if defined(ENABLE_EEPROM_STORAGE)
BDButton TouchButtonCompensationStore;
#endif

BDButton TouchButtonRobotCarStartStop;
//bool sRobotCarMoving; // main start flag. If true motors are running and will be updated by main loop.

BDButton TouchButtonDirection;
uint8_t sRobotCarDirection = DIRECTION_FORWARD; // DIRECTION_FORWARD or DIRECTION_BACKWARD

BDButton TouchButtonInfo;

BDSlider SliderSpeed;
uint16_t sLastSpeedSliderValue = 0; // local storage for requested speed by speed slider

BDSlider SliderSpeedRight;
BDSlider SliderSpeedLeft;

/*
 * UltraSonic control GUI
 */
BDSlider SliderDistanceServoPosition;
BDSlider SliderUSDistance;
unsigned int sSliderUSLastCentimeter;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
BDSlider SliderIROrTofDistance;
unsigned int sLastSliderIROrTofCentimeter;
#endif

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
    /*
     * Update IMU data first, they may be displayed later on
     */
#if defined(USE_MPU6050_IMU)
    // if we are going a fixed distance or are turning, updateTurnAngle() is yet called by updateMotors(),
    // but updateIMUData has a timer to avoid multiple reading
    RobotCarPWMMotorControl.updateIMUData();
#endif

    if (BlueDisplay1.isConnectionEstablished()) {
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
//#if defined(ENABLE_PATH_INFO_PAGE)
//            } else if (sCurrentPage == PAGE_SHOW_PATH) {
//                loopPathInfoPage();
//#endif
//            }
#if defined(ENABLE_PATH_INFO_PAGE)
            // for all but PathInfo page
        if (sCurrentPage != PAGE_SHOW_PATH) {
#endif
#if defined(MONITOR_VIN_VOLTAGE)
        readCheckAndPrintVinPeriodically();
#endif
        // For Home, sensorDrive and Test page
        if (sCurrentPage != PAGE_AUTOMATIC_CONTROL) {
            printMotorValuesPeriodically();
#if defined(USE_MPU6050_IMU)
            printIMUOffsetValues();
#endif
        }

#if defined(CAR_HAS_DISTANCE_SENSOR)
        if (sCurrentPage == PAGE_HOME || sCurrentPage == PAGE_TEST
                || (sCurrentPage == PAGE_AUTOMATIC_CONTROL && sDriveMode == MODE_FOLLOWER)) {
            readAndShowDistancePeriodically();
        }
#endif

#if defined(ENABLE_PATH_INFO_PAGE)
        }
#endif

    }

    /*
     * Check if receive buffer contains an event
     * Do it at end since it may change the condition for calling this function and its printing
     */
    checkAndHandleEvents();
}

#if defined(CAR_HAS_DISTANCE_SENSOR)
void readAndShowDistancePeriodically() {
    static long sLastDistanceMeasurementMillis;

    // Do not show distances during (time critical) acceleration or deceleration
    if (!RobotCarPWMMotorControl.isStateRamp()) {
        if (millis() - sLastDistanceMeasurementMillis >= DISTANCE_DISPLAY_PERIOD_MILLIS) {
            sLastDistanceMeasurementMillis = millis();
            getDistanceAsCentimeter(DISTANCE_TIMEOUT_CM, false);
#  if defined(USE_BLUE_DISPLAY_GUI)
#    if defined(CAR_HAS_US_DISTANCE_SENSOR)
            showUSDistance();
#    endif
#    if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
            showIROrTofDistance();
#    endif
#  endif
        }
    }
}
#endif

#if defined(CAR_HAS_DISTANCE_SERVO)
void doUSServoPosition(BDSlider *aTheTouchedSlider, uint16_t aValue) {
    (void) aTheTouchedSlider; // for the compiler to be happy
    DistanceServoWriteAndDelay(aValue);
}
#endif

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
        } else {
            if (sLastSpeedSliderValue == 0) {
                // If slider was not touched before
                sLastSpeedSliderValue = RobotCarPWMMotorControl.rightCarMotor.DriveSpeedPWM / 2;
            }
            /*
             * Start car to last speed slider value
             */
            RobotCarPWMMotorControl.setSpeedPWMAndDirection(sLastSpeedSliderValue, sRobotCarDirection);
            tSpeedSliderValue = sLastSpeedSliderValue;
        }
    } else {
        /*
         * Stop car
         */
#if defined(CAR_HAS_DISTANCE_SENSOR)
        startStopAutomomousDrive(false);  // calls RobotCarPWMMotorControl.stop()
#else
        RobotCarPWMMotorControl.stop(STOP_MODE_RELEASE);
        TouchButtonRobotCarStartStop.setValue(aDoStart, false);
#endif
        if (sSensorCallbacksEnabled) {
            /*
             * Global stop for sensor drive
             */
            registerSensorChangeCallback(FLAG_SENSOR_TYPE_ACCELEROMETER, FLAG_SENSOR_DELAY_NORMAL, FLAG_SENSOR_NO_FILTER, NULL);
            BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_UNLOCK);
            sSensorCallbacksEnabled = false;
        }
    }

    bool tShowValues = sCurrentPage == PAGE_HOME || sCurrentPage == PAGE_TEST;
    // set start/stop button value always just in case we are called by another than Stop button callback
    TouchButtonRobotCarStartStop.setValue(!RobotCarPWMMotorControl.isStopped() || sSensorCallbacksEnabled, tShowValues);
    // update speed slider value
    if (tShowValues) {
        SliderSpeed.setValueAndDrawBar(tSpeedSliderValue);
    }

}

#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
 * Called by TouchButtonRobotCarStartStop
 */
void doStartStopRobotCar(BDButton *aTheTouchedButton, int16_t aDoStart) {
    startStopRobotCar(aDoStart);
}

#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
void doCalibrate(BDButton *aTheTouchedButton, int16_t aValue) {
//    TouchButtonRobotCarStartStop.setValueAndDraw(RobotCarPWMMotorControl.isStopped());
//    if (RobotCarPWMMotorControl.isStopped()) {
//        RobotCarPWMMotorControl.calibrate(&loopGUI);
//#if defined(ENABLE_EEPROM_STORAGE)
//        RobotCarPWMMotorControl.writeMotorValuesToEeprom();
//#endif
//    } else {
//        // second / recursive call of doCalibrate()
//        RobotCarPWMMotorControl.stop();
//    }
}
#endif

/*
 * Minimum Speed is 30 for USB power and no load, 50 for load
 * Minimum Speed is 20 for 2 Lithium 18650 battery power and no load, 25 for load
 */
void doSpeedSlider(BDSlider *aTheTouchedSlider, uint16_t aValue) {
    if (aValue != sLastSpeedSliderValue) {
        sLastSpeedSliderValue = aValue;

        if (RobotCarPWMMotorControl.isStopped()) {
            // handle GUI
            startStopRobotCar(true);
        } else {
            RobotCarPWMMotorControl.setSpeedPWMAndDirection(aValue, sRobotCarDirection);
        }
    }
}

/*
 * Stops car and change direction
 */
void doSetDirection(BDButton *aTheTouchedButton, int16_t aValue) {
    if (aValue) {
        sRobotCarDirection = DIRECTION_FORWARD;
    } else {
        sRobotCarDirection = DIRECTION_BACKWARD;
    }

// Stop fixed directions and turns using RobotCarPWMMotorControl
    if (!RobotCarPWMMotorControl.isStopped()) {
        RobotCarPWMMotorControl.stop();
    }
// Stop direct movement by slider
    startStopRobotCar(false);
}

/*
 * changes speed compensation by +1 or -1
 */
void doSetCompensation(BDButton *aTheTouchedButton, int16_t aSpeedPWMCompensationRightDelta) {
    RobotCarPWMMotorControl.changeSpeedPWMCompensation(aSpeedPWMCompensationRightDelta);
}

#if defined(ENABLE_EEPROM_STORAGE)
void doStoreCompensation(BDButton * aTheTouchedButton, int16_t aRightMotorSpeedPWMCompensation) {
    RobotCarPWMMotorControl.writeMotorValuesToEeprom();
}
#endif

void startCurrentPage() {
    switch (sCurrentPage) {
    case PAGE_BT_SENSOR_CONTROL:
        startBTSensorDrivePage();
        break;
    case PAGE_TEST:
        startTestPage();
        break;
#if defined(CAR_HAS_DISTANCE_SENSOR)
    case PAGE_AUTOMATIC_CONTROL:
        startAutonomousDrivePage();
        break;
#endif
#if defined(ENABLE_PATH_INFO_PAGE)
    case PAGE_SHOW_PATH:
        startPathInfoPage();
        break;
#endif
    default:
        startHomePage();
        break;

    }
}

void doShowInfo(BDButton *aTheTouchedButton, int16_t aValue) {
    sShowInfo = aValue;
    if (!sShowInfo) {
        // Clear info area
        BlueDisplay1.fillRect(MOTOR_INFO_START_X, MOTOR_INFO_START_Y - TEXT_SIZE_11_ASCEND, BUTTON_WIDTH_8_POS_4 - 1,
        MOTOR_INFO_START_Y - TEXT_SIZE_11_ASCEND + (5 * TEXT_SIZE_11), COLOR16_WHITE);
    } else {
        PWMDcMotor::MotorPWMHasChanged = true;
        PWMDcMotor::MotorControlValuesHaveChanged = true;
    }
}

/*
 * For Next and Back button
 * Stop old page and start new one
 * @param aValue the Page number of the new page number
 */
void GUISwitchPages(BDButton *aTheTouchedButton, int16_t aValue) {
    /*
     * Stop old page
     */
    switch (sCurrentPage) {
    case PAGE_BT_SENSOR_CONTROL:
        stopBTSensorDrivePage();
        break;
    case PAGE_TEST:
        stopTestPage();
        break;
#if defined(CAR_HAS_DISTANCE_SENSOR)
    case PAGE_AUTOMATIC_CONTROL:
        stopAutonomousDrivePage();
        break;
#endif
#if defined(ENABLE_PATH_INFO_PAGE)
    case PAGE_SHOW_PATH:
        stopPathInfoPage();
        aValue = PAGE_AUTOMATIC_CONTROL; // only back to autonomous page permitted
        break;
#endif
    default:
        stopHomePage();
        break;
    }

#if defined(USE_MPU6050_IMU)
    RobotCarPWMMotorControl.IMUData.resetOffsetData(); // just to have a fresh start
#endif
    /*
     * Start new page
     */
    sCurrentPage = aValue;
    startCurrentPage();
}

void initRobotCarDisplay() {

    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_TOUCH_BASIC_DISABLE | BD_FLAG_USE_MAX_SIZE, DISPLAY_WIDTH,
    DISPLAY_HEIGHT);
    BlueDisplay1.setCharacterMapping(0x87, 0x2227); // mapping for unicode AND used as Forward symbol
    BlueDisplay1.setCharacterMapping(0x88, 0x2228); // mapping for unicode OR used as Backwards symbol
// Lock to landscape layout
    BlueDisplay1.setScreenOrientationLock(FLAG_SCREEN_ORIENTATION_LOCK_SENSOR_LANDSCAPE);
    initCommonGui();
}

void initCommonGui() {
    /*
     * Common control buttons
     */
    TouchButtonRobotCarStartStop.init(0, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR16_BLUE, F("Start"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, false, &doStartStopRobotCar);
    TouchButtonRobotCarStartStop.setCaptionForValueTrue(F("Stop"));

    TouchButtonBack.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_4, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR16_RED, F("Back"),
    TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_HOME, &GUISwitchPages);

#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
    TouchButtonCalibrate.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_RED, F("CAL"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doCalibrate);
#endif

    TouchButtonInfo.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_3, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_RED, F("Info"),
    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sShowInfo, &doShowInfo);

//    TouchButtonReset.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_2, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_RED, F("CAL"),
//    TEXT_SIZE_11, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doCalibrate);

// Direction Button value true is forward, false is backward BUT 0 is DIRECTION_FORWARD!!!
    TouchButtonDirection.init(BUTTON_WIDTH_8_POS_5, BUTTON_HEIGHT_8_LINE_6, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLACK,
            F("\x88"),
            TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, true, &doSetDirection);
    TouchButtonDirection.setCaptionForValueTrue("\x87");

    TouchButtonCompensationLeft.init(BUTTON_WIDTH_8_POS_4, BUTTON_HEIGHT_8_LINE_4,
    BUTTON_WIDTH_8 + (BUTTON_DEFAULT_SPACING_QUARTER - 1), BUTTON_HEIGHT_8, COLOR16_BLUE, F("<-Co"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, -1, &doSetCompensation);
    TouchButtonCompensationRight.init(BUTTON_WIDTH_8_POS_5 - (BUTTON_DEFAULT_SPACING_QUARTER - 1), BUTTON_HEIGHT_8_LINE_4,
    BUTTON_WIDTH_8 + (BUTTON_DEFAULT_SPACING_QUARTER - 1), BUTTON_HEIGHT_8, COLOR16_BLUE, F("mp->"), TEXT_SIZE_11,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doSetCompensation);
#if defined(ENABLE_EEPROM_STORAGE)
    TouchButtonCompensationStore.init(BUTTON_WIDTH_8_POS_6, BUTTON_HEIGHT_8_LINE_4, BUTTON_WIDTH_8, BUTTON_HEIGHT_8, COLOR16_BLUE,
            F("Store"), TEXT_SIZE_10, FLAG_BUTTON_DO_BEEP_ON_TOUCH, 1, &doStoreCompensation);
#endif

    /*
     * Speed Sliders
     */
    SliderSpeed.init(0, SLIDER_TOP_MARGIN, BUTTON_WIDTH_6, SPEED_SLIDER_SIZE, 200, 0, COLOR16_YELLOW, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE, &doSpeedSlider);
    SliderSpeed.setScaleFactor(255.0 / SPEED_SLIDER_SIZE); // Slider is virtually 2 times larger than displayed, values were divided by 2

    SliderSpeedLeft.init(MOTOR_INFO_START_X, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2, SPEED_SLIDER_SIZE / 2 - 1, 0,
    SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderSpeedLeft.setValueFormatString("%3d"); // Since we also send values grater 100

    SliderSpeedRight.init(MOTOR_INFO_START_X + BUTTON_WIDTH_16 + 8, 0, BUTTON_WIDTH_16, SPEED_SLIDER_SIZE / 2,
    SPEED_SLIDER_SIZE / 2 - 1, 0, SLIDER_DEFAULT_BACKGROUND_COLOR, SLIDER_DEFAULT_BAR_COLOR,
            FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderSpeedRight.setValueFormatString("%3d");

#if defined(CAR_HAS_DISTANCE_SENSOR)
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
#if defined(CAR_HAS_DISTANCE_SENSOR)
#define POS_X_DISTANCE_POSITION_SLIDER    LAYOUT_320_WIDTH // 320
#endif
#define POS_X_US_DISTANCE_SLIDER    ((POS_X_DISTANCE_POSITION_SLIDER - BUTTON_WIDTH_6) - 2) // - (width of US position slider + 2 for gap)
#define POS_X_THIRD_SLIDER          (POS_X_US_DISTANCE_SLIDER - (BUTTON_WIDTH_10 / 2))  // - (width of small US distance slider + 2 for gap)
#if defined(CAR_HAS_PAN_SERVO) && defined(CAR_HAS_TILT_SERVO)
#define POS_X_PAN_SLIDER            (POS_X_THIRD_SLIDER)  //
#define POS_X_TILT_SLIDER           ((POS_X_THIRD_SLIDER - BUTTON_WIDTH_12) - 4) // - width of pan slider + 4 for gap
#else
#define POS_X_PAN_SLIDER            ((POS_X_US_DISTANCE_SLIDER - BUTTON_WIDTH_10 - 2) // - width of 1 big or 2 small sliders + 2 for gap
#endif

#if defined(CAR_HAS_DISTANCE_SERVO)
    SliderDistanceServoPosition.init(POS_X_DISTANCE_POSITION_SLIDER - BUTTON_WIDTH_6, SLIDER_TOP_MARGIN, BUTTON_WIDTH_6, US_SLIDER_SIZE, 90, 90,
    COLOR16_YELLOW, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doUSServoPosition);
    SliderDistanceServoPosition.setBarThresholdColor(COLOR16_BLUE);
    SliderDistanceServoPosition.setScaleFactor(180.0 / US_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderDistanceServoPosition.setValueUnitString("\xB0"); // \xB0 is degree character
#endif

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR) || (defined(CAR_HAS_PAN_SERVO) && defined(CAR_HAS_TILT_SERVO))
#define US_DISTANCE_SLIDER_IS_SMALL
#endif
    /*
     * One thin or thick US distance slider
     */
#if defined(US_DISTANCE_SLIDER_IS_SMALL)
    // Small US distance slider with captions and without cm units
    SliderUSDistance.init(POS_X_US_DISTANCE_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2), SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8,
            (BUTTON_WIDTH_10 / 2) - 2,
            DISTANCE_SLIDER_SIZE, DISTANCE_TIMEOUT_CM_FOLLOWER / DISTANCE_SLIDER_SCALE_FACTOR, 0, SLIDER_DEFAULT_BACKGROUND_COLOR,
            SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderUSDistance.setCaptionProperties(TEXT_SIZE_10, FLAG_SLIDER_CAPTION_ALIGN_LEFT | FLAG_SLIDER_CAPTION_BELOW, 2,
    COLOR16_BLACK,
    COLOR16_WHITE);
    SliderUSDistance.setCaption("US");
    // below caption - left aligned
    SliderUSDistance.setPrintValueProperties(11, FLAG_SLIDER_CAPTION_ALIGN_LEFT | FLAG_SLIDER_CAPTION_BELOW,
            4 + TEXT_SIZE_10_HEIGHT, COLOR16_BLACK, COLOR16_WHITE);
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
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
    // Small IR distance slider with captions and without cm units
    SliderIROrTofDistance.init(POS_X_THIRD_SLIDER - ((BUTTON_WIDTH_10 / 2) - 2), SLIDER_TOP_MARGIN + BUTTON_HEIGHT_8,
            (BUTTON_WIDTH_10 / 2) - 2,
            DISTANCE_SLIDER_SIZE, DISTANCE_TIMEOUT_CM_FOLLOWER / DISTANCE_SLIDER_SCALE_FACTOR, 0, SLIDER_DEFAULT_BACKGROUND_COLOR,
            SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE | FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
    SliderIROrTofDistance.setScaleFactor(DISTANCE_SLIDER_SCALE_FACTOR); // Slider is virtually 2 times larger, values were divided by 2
    SliderIROrTofDistance.setBarThresholdColor(DISTANCE_TIMEOUT_COLOR);
    // Caption properties
    SliderIROrTofDistance.setCaptionProperties(TEXT_SIZE_10, FLAG_SLIDER_CAPTION_ALIGN_RIGHT | FLAG_SLIDER_CAPTION_BELOW, 2,
    COLOR16_BLACK,
    COLOR16_WHITE);
    // Captions
    SliderIROrTofDistance.setCaption("IR");
    // value below caption - right aligned
    SliderIROrTofDistance.setPrintValueProperties(11, FLAG_SLIDER_CAPTION_ALIGN_RIGHT | FLAG_SLIDER_CAPTION_BELOW,
            4 + TEXT_SIZE_10_HEIGHT, COLOR16_BLACK, COLOR16_WHITE);
#endif

#if defined(CAR_HAS_PAN_SERVO)
    // left of SliderDistanceServoPosition
    SliderPan.init(POS_X_PAN_SLIDER - BUTTON_WIDTH_12, SLIDER_TOP_MARGIN, BUTTON_WIDTH_12, LASER_SLIDER_SIZE, 90, 90, COLOR16_YELLOW,
    SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doHorizontalServoPosition);
    SliderPan.setBarThresholdColor(COLOR16_BLUE);
    // scale slider values
    SliderPan.setScaleFactor(180.0 / LASER_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderPan.setValueUnitString("\xB0"); // \xB0 is degree character
#endif

#if defined(CAR_HAS_TILT_SERVO)
    SliderTilt.init(POS_X_TILT_SLIDER - BUTTON_WIDTH_12, SLIDER_TOP_MARGIN, BUTTON_WIDTH_12, LASER_SLIDER_SIZE, 90,
    TILT_SERVO_MIN_VALUE, COLOR16_YELLOW, SLIDER_DEFAULT_BAR_COLOR, FLAG_SLIDER_SHOW_VALUE, &doVerticalServoPosition);
    SliderTilt.setBarThresholdColor(COLOR16_BLUE);
    // scale slider values
    SliderTilt.setScaleFactor(180.0 / LASER_SLIDER_SIZE); // Values from 0 to 180 degrees
    SliderTilt.setValueUnitString("\xB0"); // \xB0 is degree character
#endif

    initAutonomousDrivePage();
#endif // defined(CAR_HAS_DISTANCE_SENSOR)

    initHomePage();
    initTestPage();
    initBTSensorDrivePage();
#if defined(ENABLE_PATH_INFO_PAGE)
    initPathInfoPage();
#endif
}

void drawCommonGui(void) {
    clearDisplayAndDisableButtonsAndSliders();
    BlueDisplay1.drawText(HEADER_X, TEXT_SIZE_22_HEIGHT, F("Robot Car"), TEXT_SIZE_22, COLOR16_BLUE, COLOR16_NO_BACKGROUND);
}

#if defined(MONITOR_VIN_VOLTAGE)
/*
 * Print VIN (used as motor supply) periodically
 * returns true if voltage was printed
 */
void readAndPrintVin() {
    char tDataBuffer[18];
    char tVCCString[5];
    readVINVoltage();
    dtostrf(sVINVoltage, 4, 2, tVCCString);
    sprintf_P(tDataBuffer, PSTR("%s volt"), tVCCString);

    uint16_t tPosX = BUTTON_WIDTH_8_POS_4;
    uint8_t tPosY;
    if (sCurrentPage == PAGE_HOME) {
        tPosY = BUTTON_HEIGHT_8_LINE_6 + TEXT_SIZE_11_DECEND;

    } else if (sCurrentPage == PAGE_AUTOMATIC_CONTROL || sCurrentPage == PAGE_BT_SENSOR_CONTROL) {
        tPosX = TEXT_SIZE_11_WIDTH;
        if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
            tPosY = BUTTON_HEIGHT_4_LINE_4 - (TEXT_SIZE_22_HEIGHT + BUTTON_DEFAULT_SPACING_QUARTER) - TEXT_SIZE_11_DECEND;
        } else {
            tPosY = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
        }

    } else {
        // Test page
        tPosY = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
    }
    BlueDisplay1.drawText(tPosX, tPosY, tDataBuffer, TEXT_SIZE_11, COLOR16_BLACK, COLOR16_WHITE);

}

void checkForLowVoltage() {
    static uint8_t sLowVoltageCount = 0;
    if (sVINVoltage < VOLTAGE_LIPO_LOW_THRESHOLD && sVINVoltage > VOLTAGE_USB_THRESHOLD) {
        sLowVoltageCount++;
    } else if (sLowVoltageCount > 1) {
        sLowVoltageCount--;
    }
    if (sLowVoltageCount > 2) {
        // Here more than 2 consecutive times (for 6 seconds) low voltage detected
        startStopRobotCar(false);

        if (BlueDisplay1.isConnectionEstablished()) {
            drawCommonGui();
            BlueDisplay1.drawText(10, 50, F("Battery voltage"), TEXT_SIZE_33, COLOR16_RED, COLOR16_WHITE);
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
            } while (tLoopCount > 0 || (sVINVoltage < VOLTAGE_LIPO_LOW_THRESHOLD && sVINVoltage > VOLTAGE_USB_THRESHOLD));
            // refresh current page
            GUISwitchPages(NULL, 0);
        } else {
            delay(VOLTAGE_TOO_LOW_DELAY_OFFLINE);
        }
    }
}

void readCheckAndPrintVinPeriodically() {
    static uint32_t sMillisOfLastVCCInfo;
    uint32_t tMillis = millis();

    if (tMillis - sMillisOfLastVCCInfo >= PRINT_VOLTAGE_PERIOD_MILLIS) {
        sMillisOfLastVCCInfo = tMillis;
        readAndPrintVin();
        checkForLowVoltage();
//        PWMDcMotor::MotorControlValuesHaveChanged = true; // to signal, that PWM voltages may have changed
    }
}
#endif // defined(MONITOR_VIN_VOLTAGE)

/*
 * Checks MotorControlValuesHaveChanged and print
 * below the speed sliders and IMU / encoder values
 * Print PWM values + PWM voltage + compensation values (negative)
 * Then calls print sensor values, which prints the speed sliders.
 */
void printMotorValuesPeriodically() {
    static long sLastPrintMillis;

    uint32_t tMillis = millis();
    if (tMillis - sLastPrintMillis >= PRINT_MOTOR_INFO_PERIOD_MILLIS) {
        sLastPrintMillis = tMillis;
        if (sShowInfo) {
#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
            if (PWMDcMotor::SensorValuesHaveChanged) {
                printMotorSpeedSensorValues();
            }
#endif
            /*
             * Print speed value
             */
            if (PWMDcMotor::MotorPWMHasChanged) {
                PWMDcMotor::MotorPWMHasChanged = false;
                // position below caption of speed slider
                sprintf_P(sStringBuffer, PSTR("PWM  %3d %3d"), RobotCarPWMMotorControl.leftCarMotor.CompensatedSpeedPWM,
                        RobotCarPWMMotorControl.rightCarMotor.CompensatedSpeedPWM);
                BlueDisplay1.drawText(MOTOR_INFO_START_X, MOTOR_INFO_START_Y + TEXT_SIZE_11, sStringBuffer, TEXT_SIZE_11,
                COLOR16_BLACK, COLOR16_WHITE);

                /*
                 * print voltage for left motor PWM
                 */
                char tPWMVoltageString[6];
#if defined(MONITOR_VIN_VOLTAGE)
            // use current voltage minus bridge loss instead of a constant value
                dtostrf(PWMDcMotor::getMotorVoltageforPWM(RobotCarPWMMotorControl.leftCarMotor.CompensatedSpeedPWM, sVINVoltage) , 4, 2, tPWMVoltageString);
#else
                // we can merely use a constant value here
                dtostrf(PWMDcMotor::getMotorVoltageforPWMAndMillivolt(RobotCarPWMMotorControl.leftCarMotor.CompensatedSpeedPWM, FULL_BRIDGE_INPUT_MILLIVOLT), 4, 2, tPWMVoltageString);
#endif
                tPWMVoltageString[4] = 'V';
                tPWMVoltageString[5] = '\0';
                BlueDisplay1.drawText((MOTOR_INFO_START_X + TEXT_SIZE_11_WIDTH) - 1, MOTOR_INFO_START_Y + (2 * TEXT_SIZE_11),
                        tPWMVoltageString);
#if defined(MONITOR_VIN_VOLTAGE)
                dtostrf(PWMDcMotor::getMotorVoltageforPWM(RobotCarPWMMotorControl.rightCarMotor.CompensatedSpeedPWM, sVINVoltage), 4, 2, tPWMVoltageString);
#else
                // we can merely use a constant value here
                dtostrf(PWMDcMotor::getMotorVoltageforPWMAndMillivolt(RobotCarPWMMotorControl.rightCarMotor.CompensatedSpeedPWM, FULL_BRIDGE_INPUT_MILLIVOLT), 4, 2, tPWMVoltageString);
#endif
                tPWMVoltageString[4] = 'V';
                BlueDisplay1.drawText((MOTOR_INFO_START_X + (7 * TEXT_SIZE_11_WIDTH)) - 3, MOTOR_INFO_START_Y + (2 * TEXT_SIZE_11),
                        tPWMVoltageString);

#if defined(USE_ENCODER_MOTOR_CONTROL)
            sprintf_P(sStringBuffer, PSTR("tcnt %3d %3d"), RobotCarPWMMotorControl.leftCarMotor.LastTargetDistanceMillimeter,
                    RobotCarPWMMotorControl.rightCarMotor.LastTargetDistanceMillimeter);
            BlueDisplay1.drawText(MOTOR_INFO_START_X, MOTOR_INFO_START_Y + (4 * TEXT_SIZE_11), sStringBuffer);
#endif
            }

            /*
             * Print motor compensation values
             */
            if (PWMDcMotor::MotorControlValuesHaveChanged) {
                PWMDcMotor::MotorControlValuesHaveChanged = false;
                sprintf_P(sStringBuffer, PSTR("comp %3d %3d"), -RobotCarPWMMotorControl.leftCarMotor.SpeedPWMCompensation,
                        -RobotCarPWMMotorControl.rightCarMotor.SpeedPWMCompensation);
                BlueDisplay1.drawText(MOTOR_INFO_START_X, MOTOR_INFO_START_Y + (3 * TEXT_SIZE_11), sStringBuffer, TEXT_SIZE_11,
                COLOR16_BLACK, COLOR16_WHITE);
            }
        }

        // draw speed slider independent of sShowInfo
#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
        if (PWMDcMotor::SensorValuesHaveChanged) {
            PWMDcMotor::SensorValuesHaveChanged = false;
            displayMotorSpeedSliderValues();
        }
#endif
    }
}

#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
/*
 * Display speed as slider values
 * Encoder values have precedence over IMU values
 */
void displayMotorSpeedSliderValues() {
#  if defined(USE_ENCODER_MOTOR_CONTROL)
    EncoderMotor *tMotorInfo = &RobotCarPWMMotorControl.leftCarMotor;
    BDSlider *tSliderPtr = &SliderSpeedLeft;
    for (int i = 0; i < 2; ++i) {
        tSliderPtr->setValueAndDrawBar(tMotorInfo->getSpeed());
        tMotorInfo = &RobotCarPWMMotorControl.rightCarMotor;
        tSliderPtr = &SliderSpeedRight;
    }
#  else
    int tCarSpeedCmPerSecondFromIMU = abs(RobotCarPWMMotorControl.CarSpeedCmPerSecondFromIMU);
    SliderSpeedLeft.setValueAndDrawBar(tCarSpeedCmPerSecondFromIMU);
    SliderSpeedRight.setValueAndDrawBar(tCarSpeedCmPerSecondFromIMU);
#  endif
}

/**
 * Print speed slider + speed values
 * In the next line, print encoder count or IMU distance and rotation
 *
 * Encoder speed has precedence over IMU speed
 */
void printMotorSpeedSensorValues() {

#  if defined(USE_ENCODER_MOTOR_CONTROL)
    /*
     * Print encoder counts
     */
    uint16_t tYPos;
    if (sCurrentPage == PAGE_AUTOMATIC_CONTROL) {
        tYPos = BUTTON_HEIGHT_4_LINE_4 - TEXT_SIZE_11_DECEND;
    } else {
        tYPos = MOTOR_INFO_START_Y;
    }
    sprintf_P(sStringBuffer, PSTR("cnt.%4d%4d"), RobotCarPWMMotorControl.leftCarMotor.EncoderCount,
            RobotCarPWMMotorControl.rightCarMotor.EncoderCount);
    BlueDisplay1.drawText(MOTOR_INFO_START_X, tYPos, sStringBuffer, TEXT_SIZE_11, COLOR16_BLACK, COLOR16_WHITE);
#  endif

#  if defined(USE_MPU6050_IMU)
    /*
     * Print distance and rotation from IMU
     */
    sprintf_P(sStringBuffer, PSTR("%5dcm%4d\xB0"), RobotCarPWMMotorControl.IMUData.getDistanceCm(),
            RobotCarPWMMotorControl.CarTurnAngleHalfDegreesFromIMU / 2);
    BlueDisplay1.drawText(MOTOR_INFO_START_X, MOTOR_INFO_START_Y, sStringBuffer);
#  endif
}

#endif // defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)

#if defined(USE_MPU6050_IMU)
void printIMUOffsetValues() {
    if (RobotCarPWMMotorControl.IMUData.OffsetsHaveChanged) {
        RobotCarPWMMotorControl.IMUData.OffsetsHaveChanged = false;
        sprintf_P(sStringBuffer, PSTR("off.%4d%4d"), RobotCarPWMMotorControl.IMUData.AcceleratorForwardOffset,
                RobotCarPWMMotorControl.IMUData.GyroscopePanOffset);
        BlueDisplay1.drawText(MOTOR_INFO_START_X, MOTOR_INFO_START_Y + (5 * TEXT_SIZE_11), sStringBuffer, TEXT_SIZE_11,
        COLOR16_BLACK, COLOR16_WHITE);
    }
}
#endif // defined(USE_MPU6050_IMU)

#if defined(CAR_HAS_US_DISTANCE_SENSOR)
void showUSDistance() {
    if (sUSDistanceCentimeter != sSliderUSLastCentimeter) {
        sSliderUSLastCentimeter = sUSDistanceCentimeter;
        SliderUSDistance.setValueAndDrawBar(sUSDistanceCentimeter);
    }
}
#endif

#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_CAR_HAS_TOF_DISTANCE_SENSOR)
void showIROrTofDistance() {
    if (sIROrTofDistanceCentimeter != sLastSliderIROrTofCentimeter) {
        sLastSliderIROrTofCentimeter = sIROrTofDistanceCentimeter;
        SliderIROrTofDistance.setValueAndDrawBar(sIROrTofDistanceCentimeter);
    }
}
#endif

#endif // _ROBOT_CAR_COMMON_GUI_HPP
#pragma once
