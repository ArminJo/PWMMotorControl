/*
 * RobotCarGui.h
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

#ifndef _ROBOT_CAR_GUI_H
#define _ROBOT_CAR_GUI_H

#define USE_BLUE_DISPLAY_GUI

#include "BlueDisplay.h"
#include "AutonomousDrive.h"

// can be deleted for BlueDisplay library version > 2.1.0
# if not defined(BUTTON_WIDTH_3_5_POS_2)
#define BUTTON_WIDTH_3_5_POS_2 (BUTTON_WIDTH_3_5 + BUTTON_DEFAULT_SPACING)
#define TEXT_SIZE_9   9
#endif

#define PATH_LENGTH_MAX 100

#define PRINT_MOTOR_INFO_PERIOD_MILLIS 200

// a string buffer for BD info output
extern char sStringBuffer[128];

#define DISPLAY_WIDTH           DISPLAY_HALF_VGA_WIDTH   // 320
#define DISPLAY_HEIGHT          DISPLAY_HALF_VGA_HEIGHT // 240

#define HEADER_X                BUTTON_WIDTH_3_5_POS_2 - (TEXT_SIZE_22_WIDTH / 2)

#define SLIDER_TOP_MARGIN       10
#define SPEED_SLIDER_SIZE       BUTTON_HEIGHT_4_LINE_3  // 128
#define US_SLIDER_SIZE          BUTTON_HEIGHT_4_LINE_3     // 128
#define LASER_SLIDER_SIZE       BUTTON_HEIGHT_4_LINE_3  // 128
#define DISTANCE_SLIDER_SIZE    (BUTTON_HEIGHT_4_LINE_3 - BUTTON_HEIGHT_8)  // 104
#define DISTANCE_SLIDER_SCALE_FACTOR    2 // Slider is virtually 2 times larger, values were divided by 2
#define DISTANCE_DISPLAY_PERIOD_MILLIS      500

#define US_DISTANCE_MAP_ORIGIN_X 200
#define US_DISTANCE_MAP_ORIGIN_Y 150

#define MOTOR_INFO_START_X (BUTTON_WIDTH_6 + 4)
#define MOTOR_INFO_START_Y (SPEED_SLIDER_SIZE / 2 + 26)

#define PAGE_HOME               0 // Manual control page
#define PAGE_AUTOMATIC_CONTROL  1
#define PAGE_BT_SENSOR_CONTROL  2
#define PAGE_TEST               3
#define PAGE_SHOW_PATH          4
#define PAGE_LAST_NUMBER        PAGE_SHOW_PATH
extern uint8_t sCurrentPage;

#if defined(CAR_HAS_US_DISTANCE_SENSOR)
void showUSDistance();
#endif
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
void showIROrTofDistance();
#endif

#if defined(ENABLE_PATH_INFO_PAGE)
// from PathInfoPage
void initPathInfoPage(void);
void drawPathInfoPage(void);
void startPathInfoPage(void);
void loopPathInfoPage(void);
void stopPathInfoPage(void);

void DrawPath();
void resetPathData();
void insertToPath(int aLength, int aDegree, bool aAddEntry);
#endif

// from AutonomousDrivePage
extern BDButton TouchButtonStep;
extern BDButton TouchButtonScanSpeed;
extern BDButton TouchButtonDistanceFeedbackMode;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
extern BDButton TouchButtonScanMode;
#endif

void initAutonomousDrivePage(void);
void drawAutonomousDrivePage(void);
void startAutonomousDrivePage(void);
void loopAutonomousDrivePage(void);
void stopAutonomousDrivePage(void);

void handleAutomomousDriveRadioButtons();
void doStartStopFollowerMode(BDButton *aTheTouchedButton, int16_t aValue);
void doStartStopAutomomousDrive(BDButton *aTheTouchedButton, int16_t aValue);
void doStartStopTestUser(BDButton *aTheTouchedButton, int16_t aValue);

void doStartStopAutonomousForPathPage(BDButton *aTheTouchedButton, int16_t aValue);
void setStepMode(uint8_t aStepMode);

// from BTSensorDrivePage
void initBTSensorDrivePage(void);
void drawBTSensorDrivePage(void);
void startBTSensorDrivePage(void);
void loopBTSensorDrivePage(void);
void stopBTSensorDrivePage(void);

extern uint8_t sSensorChangeCallCountForZeroAdjustment;
void doSensorChange(uint8_t aSensorType, struct SensorCallback *aSensorCallbackInfo);

// from TestPage
extern bool sShowInfo;

void initTestPage(void);
void drawTestPage(void);
void startTestPage(void);
void loopTestPage(void);
void stopTestPage(void);

// from HomePage
extern BDButton TouchButtonMelody;
#if defined(ENABLE_RTTTL_FOR_CAR)
extern bool sPlayMelody;
#endif

extern void doHorizontalServoPosition(BDSlider *aTheTouchedSlider, uint16_t aValue);
extern void doVerticalServoPosition(BDSlider *aTheTouchedSlider, uint16_t aValue);

void initHomePage(void);
void drawHomePage(void);
void startHomePage(void);
void loopHomePage(void);
void stopHomePage(void);

/*
 * Page management
 */
extern uint8_t sCurrentPage;
extern BDButton TouchButtonAutomaticDrivePage;
extern BDButton TouchButtonBack;
void GUISwitchPages(BDButton *aTheTouchedButton, int16_t aValue);
void startCurrentPage();

/*
 * Common GUI elements
 */
extern BDButton TouchButtonReset;

extern BDButton TouchButtonRobotCarStartStop;
void setStartStopButtonValue();
void startStopRobotCar(bool aDoStart);
void doStartStopRobotCar(BDButton *aTheTochedButton, int16_t aDoStart);
void doReset(BDButton *aTheTochedButton, int16_t aValue);

extern BDButton TouchButtonDirection;
extern BDButton TouchButtonInfo;

//#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
extern BDButton TouchButtonCalibrate;
extern bool isCalibrated;
void calibrateAndPrint();
void doCalibrate(BDButton *aTheTouchedButton, int16_t aValue);

extern BDButton TouchButtonCompensationRight;
extern BDButton TouchButtonCompensationLeft;
#if defined(ENABLE_EEPROM_STORAGE)
extern BDButton TouchButtonCompensationStore;
#endif

extern BDSlider SliderSpeed;
extern uint16_t sLastSpeedSliderValue;
void showSpeedSliderValue();

extern BDSlider SliderSpeedRight;
extern BDSlider SliderSpeedLeft;

extern BDSlider SliderDistanceServoPosition;
extern BDSlider SliderUSDistance;
#if defined(CAR_HAS_IR_DISTANCE_SENSOR) || defined(CAR_HAS_TOF_DISTANCE_SENSOR)
extern BDSlider SliderIROrTofDistance;
#endif

#if defined(CAR_HAS_PAN_SERVO)
extern BDSlider SliderPan;
#endif
#if defined(CAR_HAS_TILT_SERVO)
extern BDSlider SliderTilt;
#endif

#if defined(USE_ENCODER_MOTOR_CONTROL) || defined(USE_MPU6050_IMU)
void displayMotorSpeedSliderValues();
void printMotorSpeedSensorValues();
#endif
#if defined(USE_MPU6050_IMU)
void printIMUOffsetValues();
#endif

void drawCommonGui(void);

extern char sStringBuffer[128];

void setupGUI(void);
void loopGUI(void);
void initCommonGui(void);

void initRobotCarDisplay(void);
void readAndShowDistancePeriodically();
void rotate(int16_t aRotationDegrees, bool inPlace = true);
void showDistance(int aCentimeter);

void printAndDisplayMotorSpeed();
void printMotorValuesPeriodically();

#if defined(MONITOR_VIN_VOLTAGE)
void readAndPrintVin();
void readCheckAndPrintVinPeriodically();
#endif

void delayAndLoopGUI(uint16_t aDelayMillis);

/*
 * Functions contained in RobotCarGuiOutput.cpp
 */
#if defined(CAR_HAS_DISTANCE_SERVO)
void drawForwardDistancesInfos();
void clearPrintedForwardDistancesInfos(bool aDoFullClear);
void drawCollisionDecision(int aDegreesToTurn, uint8_t aLengthOfVector, bool aDoClearVector);
#endif

extern uint8_t sRobotCarDirection;

#endif // _ROBOT_CAR_GUI_H
