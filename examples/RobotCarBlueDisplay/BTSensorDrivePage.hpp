/*
 * BTSensorDrivePage.hpp
 *
 *  Contains all the GUI elements for driving the car with the smartphone sensors.
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
#ifndef _ROBOT_CAR_BT_SENSOR_DRIVE_PAGE_HPP
#define _ROBOT_CAR_BT_SENSOR_DRIVE_PAGE_HPP

// 4 Sliders for accelerometer
BDSlider SliderForward;     // Y negative
BDSlider SliderBackward;    // Y positive
BDSlider SliderRight;       // X negative
BDSlider SliderLeft;        // X positive

#define SLIDER_BACKGROUND_COLOR     COLOR16_YELLOW
#define SLIDER_BAR_COLOR            COLOR16_GREEN
#define SLIDER_THRESHOLD_COLOR      COLOR16_BLUE

#define SENSOR_SLIDER_WIDTH         (DISPLAY_WIDTH / 16)
#define VERTICAL_SLIDER_LENTGH      ((DISPLAY_HEIGHT / 4) + (DISPLAY_HEIGHT / 10))
#define SLIDER_SPEED_THRESHOLD      DEFAULT_DRIVE_SPEED_PWM
#define SPEED_DEAD_BAND             DEFAULT_STOP_SPEED_PWM

#define HORIZONTAL_SLIDER_LENTGH    (DISPLAY_HEIGHT / 4)
#define SLIDER_LEFT_RIGHT_THRESHOLD (HORIZONTAL_SLIDER_LENTGH / 2)
#define LEFT_RIGHT_SENSOR_DEAD_BAND 10

#define SENSOR_SLIDER_CENTER_X      ((DISPLAY_WIDTH - SENSOR_SLIDER_WIDTH) / 2)
#define SENSOR_SLIDER_CENTER_Y      ((DISPLAY_HEIGHT / 2) + (DISPLAY_HEIGHT / 16))

/*
 * Sensor callback handler
 */
uint8_t sSensorChangeCallCountForZeroAdjustment;
#define CALLS_FOR_ZERO_ADJUSTMENT 8 // The values of the first 8 calls are used as zero value.
float sYZeroValueAdded; // The accumulator for the values of the first 8 calls.
float sYZeroValue = 0;

struct positiveNegativeSlider sAccelerationLeftRightSliders;
struct positiveNegativeSlider sAccelerationForwardBackwardSliders;

#if defined(CAR_HAS_4_MECANUM_WHEELS)
BDButton TouchButtonTurnMode;
bool sTurnModeEnabled = false;
/**
 * Called by TouchButtonRobotCarStartStop
 */
void doTurnMode(BDButton *aTheTouchedButton, int16_t aTurnModeEnabled) {
    sTurnModeEnabled = aTurnModeEnabled;
}
#endif

/*
 * Add dead band value, clip values which are too big because of adding right/left speed
 */
int speedOverflowAndDeadBandHandling(int aSpeedPWM) {
    if (aSpeedPWM > 0) {
        aSpeedPWM += SPEED_DEAD_BAND;
        // overflow handling since analogWrite only accepts byte values
        if (aSpeedPWM > MAX_SPEED_PWM) {
            aSpeedPWM = MAX_SPEED_PWM;
        }
    } else if (aSpeedPWM < 0) {
        aSpeedPWM -= SPEED_DEAD_BAND;
        // overflow handling since analogWrite only accepts byte values
        if (aSpeedPWM < -(MAX_SPEED_PWM)) {
            aSpeedPWM = -(MAX_SPEED_PWM);
        }
    }
    return aSpeedPWM;
}

/*
 * Forward / backward speed
 * Values are in (m/s^2)
 * positive -> backward / bottom down
 * negative -> forward  / top down
 *
 * Left / right
 * positive -> go left / left down
 * negative -> go right / right down
 */
void doSensorChange(uint8_t aSensorType, struct SensorCallback *aSensorCallbackInfo) {
    (void) aSensorType; // to avoid -Wunused-parameter

    if (sSensorChangeCallCountForZeroAdjustment < CALLS_FOR_ZERO_ADJUSTMENT) {
        /*
         * Forward backward zero adjustment
         */
        if (sSensorChangeCallCountForZeroAdjustment == 0) {
            // init values
            sYZeroValueAdded = 0;
        }
        sYZeroValueAdded += aSensorCallbackInfo->ValueY; // Sum for zero adjustment
        sSensorChangeCallCountForZeroAdjustment++;
    } else if (sSensorChangeCallCountForZeroAdjustment == CALLS_FOR_ZERO_ADJUSTMENT) {
        sSensorChangeCallCountForZeroAdjustment++;
        // compute and set zero value
        // compute zero value. Only Y values makes sense.
        sYZeroValue = sYZeroValueAdded / CALLS_FOR_ZERO_ADJUSTMENT;
        BlueDisplay1.playTone(24);        // feedback for zero value acquired
    } else {

        /*
         * Regular operation here
         * Left right handling
         */
#if defined(CAR_HAS_4_WHEELS)
        int tLeftRightValue = aSensorCallbackInfo->ValueX * 16.0;
#else
        int tLeftRightValue = aSensorCallbackInfo->ValueX * 8.0; // Scale value
#endif
#if defined(CAR_HAS_4_MECANUM_WHEELS)
        tLeftRightValue = setPositiveNegativeSliders(&sAccelerationLeftRightSliders, tLeftRightValue, SPEED_DEAD_BAND);
#else
        tLeftRightValue = setPositiveNegativeSliders(&sAccelerationLeftRightSliders, tLeftRightValue, LEFT_RIGHT_SENSOR_DEAD_BAND);
#endif
        /*
         * forward backward handling
         */
        int tSpeedPWMValue = -((aSensorCallbackInfo->ValueY - sYZeroValue) * (MAX_SPEED_PWM / 8)); // Scale value
        tSpeedPWMValue = setPositiveNegativeSliders(&sAccelerationForwardBackwardSliders, tSpeedPWMValue, SPEED_DEAD_BAND);

        /*
         * Print speedPWM as value of bottom slider
         */
        sprintf(sStringBuffer, "%4d", tSpeedPWMValue);
        SliderBackward.printValue(sStringBuffer);

#if defined(CAR_HAS_4_MECANUM_WHEELS)
        /*
         * Get direction
         */
        uint8_t tDirection = DIRECTION_STOP;
        if(tSpeedPWMValue > 0) {
            tDirection = DIRECTION_FORWARD;
        } else if (tSpeedPWMValue < 0) {
            tSpeedPWMValue = -tSpeedPWMValue;
            tDirection = DIRECTION_BACKWARD;
        }
        if(tLeftRightValue > 0) {
            // Left
            tDirection |= DIRECTION_LEFT;
        } else if(tLeftRightValue < 0) {
            tLeftRightValue = - tLeftRightValue;
            tDirection |= DIRECTION_RIGHT;
        }
        if(sTurnModeEnabled) {
            tDirection |= DIRECTION_TURN;
        }
        tSpeedPWMValue = max(tSpeedPWMValue, tLeftRightValue);
        RobotCar.setSpeedPWMAndDirection(speedOverflowAndDeadBandHandling(tSpeedPWMValue), tDirection);
#else
        if(tSpeedPWMValue < 0){
            tLeftRightValue = -tLeftRightValue;
        }
        RobotCar.rightCarMotor.setSpeedPWMAndDirection(
                speedOverflowAndDeadBandHandling(tSpeedPWMValue + tLeftRightValue));
        RobotCar.leftCarMotor.setSpeedPWMAndDirection(
                speedOverflowAndDeadBandHandling(tSpeedPWMValue - tLeftRightValue));
#endif
    }
}

void initBTSensorDrivePage(void) {
#if defined(CAR_HAS_4_MECANUM_WHEELS)
    TouchButtonTurnMode.init(BUTTON_WIDTH_3_POS_3, BUTTON_HEIGHT_4_LINE_2, BUTTON_WIDTH_3, BUTTON_HEIGHT_4, COLOR16_BLUE, F("Turn"),
            TEXT_SIZE_22, FLAG_BUTTON_DO_BEEP_ON_TOUCH | FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN, sTurnModeEnabled, &doTurnMode);
#endif
    /*
     * 4 Sliders
     */
// Position Slider at middle of screen
// Top/forward slider
    SliderForward.init(SENSOR_SLIDER_CENTER_X, (SENSOR_SLIDER_CENTER_Y - VERTICAL_SLIDER_LENTGH), SENSOR_SLIDER_WIDTH,
    VERTICAL_SLIDER_LENTGH, SLIDER_SPEED_THRESHOLD, 0, SLIDER_BACKGROUND_COLOR, SLIDER_BAR_COLOR, FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);

//    SliderForward.setBarThresholdColor(SLIDER_THRESHOLD_COLOR);

    // Bottom/backward slider
    SliderBackward.init(SENSOR_SLIDER_CENTER_X, SENSOR_SLIDER_CENTER_Y, SENSOR_SLIDER_WIDTH, -(VERTICAL_SLIDER_LENTGH),
    SLIDER_SPEED_THRESHOLD, 0, SLIDER_BACKGROUND_COLOR, SLIDER_BAR_COLOR, FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);

//    SliderBackward.setBarThresholdColor(SLIDER_THRESHOLD_COLOR);
    SliderForward.setScaleFactor((float) MAX_SPEED_PWM / (float) VERTICAL_SLIDER_LENTGH);
    SliderBackward.setScaleFactor((float) MAX_SPEED_PWM / (float) VERTICAL_SLIDER_LENTGH); // second expression is optimized by compiler :-)
    sAccelerationForwardBackwardSliders.positiveSliderPtr = &SliderForward;
    sAccelerationForwardBackwardSliders.negativeSliderPtr = &SliderBackward;

// Position slider right from  at middle of screen
    SliderRight.init(SENSOR_SLIDER_CENTER_X + SENSOR_SLIDER_WIDTH, SENSOR_SLIDER_CENTER_Y - (SENSOR_SLIDER_WIDTH / 2),
    SENSOR_SLIDER_WIDTH, HORIZONTAL_SLIDER_LENTGH, SLIDER_LEFT_RIGHT_THRESHOLD, 0, SLIDER_BACKGROUND_COLOR, SLIDER_BAR_COLOR,
            FLAG_SLIDER_IS_HORIZONTAL | FLAG_SLIDER_IS_ONLY_OUTPUT | FLAG_SLIDER_SHOW_VALUE, NULL);
//    SliderRight.setBarThresholdColor(SLIDER_THRESHOLD_COLOR);

// Position inverse slider left from  at middle of screen
    SliderLeft.init(SENSOR_SLIDER_CENTER_X - HORIZONTAL_SLIDER_LENTGH, SENSOR_SLIDER_CENTER_Y - (SENSOR_SLIDER_WIDTH / 2),
    SENSOR_SLIDER_WIDTH, -(HORIZONTAL_SLIDER_LENTGH), SLIDER_LEFT_RIGHT_THRESHOLD, 0, SLIDER_BACKGROUND_COLOR,
    SLIDER_BAR_COLOR, FLAG_SLIDER_IS_HORIZONTAL | FLAG_SLIDER_IS_ONLY_OUTPUT | FLAG_SLIDER_SHOW_VALUE, NULL);
//    SliderLeft.setBarThresholdColor(SLIDER_THRESHOLD_COLOR);
    SliderRight.setScaleFactor(1.2);
    SliderLeft.setScaleFactor(1.2);
    sAccelerationLeftRightSliders.positiveSliderPtr = &SliderLeft;
    sAccelerationLeftRightSliders.negativeSliderPtr = &SliderRight;
}

void drawBTSensorDrivePage(void) {
    drawCommonGui();
    BlueDisplay1.drawText(HEADER_X, TEXT_SIZE_22_HEIGHT + TEXT_SIZE_22_HEIGHT, F("Sensor drive"));

#if defined(CAR_HAS_4_MECANUM_WHEELS)
    TouchButtonTurnMode.drawButton();
#endif
    TouchButtonBack.drawButton();
    TouchButtonRobotCarStartStop.drawButton();
    SliderForward.drawSlider();
    SliderBackward.drawSlider();
    SliderRight.drawSlider();
    SliderLeft.drawSlider();
}

void startBTSensorDrivePage(void) {
    doReset(NULL, 0);
    drawBTSensorDrivePage();
}

void loopBTSensorDrivePage(void) {
}

void stopBTSensorDrivePage(void) {
}
#endif // _ROBOT_CAR_BT_SENSOR_DRIVE_PAGE_HPP
