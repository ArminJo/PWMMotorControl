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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */
#ifndef ROBOT_CAR_BT_SENSOR_DRIVE_PAGE_HPP
#define ROBOT_CAR_BT_SENSOR_DRIVE_PAGE_HPP
#include <Arduino.h>

#include "RobotCarPinDefinitionsAndMore.h"
#include "RobotCarBlueDisplay.h"

#include "RobotCarGui.h"

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
#define SLIDER_SPEED_THRESHOLD      (DEFAULT_START_SPEED_PWM)
#define SPEED_SENSOR_DEAD_BAND      20
#define SPEED_DEAD_BAND             (DEFAULT_DRIVE_SPEED_PWM / 4)

#define HORIZONTAL_SLIDER_LENTGH    (DISPLAY_HEIGHT / 4)
#define SLIDER_LEFT_RIGHT_THRESHOLD (HORIZONTAL_SLIDER_LENTGH / 4)
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

#if (VERSION_BLUE_DISPLAY_MAJOR <= 2) && (VERSION_BLUE_DISPLAY_MINOR <= 1)
// positiveNegativeSlider is available since 2.2
/*
 * To show a signed value on two sliders positioned back to back (one of it is inverse or has a negative length value)
 */
struct positiveNegativeSlider {
    BDSlider *positiveSliderPtr;
    BDSlider *negativeSliderPtr;
    int lastSliderValue;      // positive value with sensor dead band applied
    BDSlider *lastZeroSlider; // to decide if we draw new zero slider
};
/*
 * @return aValue with aSliderDeadBand applied
 */
int setPositiveNegativeSliders(struct positiveNegativeSlider *aSliderStructPtr, int aValue, uint8_t aSliderDeadBand) {
    BDSlider *tValueSlider = aSliderStructPtr->positiveSliderPtr;
    BDSlider *tZeroSlider = aSliderStructPtr->negativeSliderPtr;
    if (aValue < 0) {
        aValue = -aValue;
        tValueSlider = tZeroSlider;
        tZeroSlider = aSliderStructPtr->positiveSliderPtr;
    }

    /*
     * Now we have a positive value for dead band and slider length
     */
    if (aValue > aSliderDeadBand) {
        // dead band subtraction -> resulting values start at 0
        aValue -= aSliderDeadBand;
    } else {
        aValue = 0;
    }

    /*
     * Draw slider value if values changed
     */
    if (aSliderStructPtr->lastSliderValue != aValue) {
        aSliderStructPtr->lastSliderValue = aValue;
        tValueSlider->setValueAndDrawBar(aValue);
        if (aSliderStructPtr->lastZeroSlider != tZeroSlider) {
            aSliderStructPtr->lastZeroSlider = tZeroSlider;
            // the sign has changed, clear old value
            tZeroSlider->setValueAndDrawBar(0);
        }
    }

    /*
     * Restore sign for aValue with dead band applied
     */
    if (tZeroSlider == aSliderStructPtr->positiveSliderPtr) {
        aValue = -aValue;
    }
    return aValue;
}
#endif

struct positiveNegativeSlider sAccelerationLeftRightSliders;
struct positiveNegativeSlider sAccelerationForwardBackwardSliders;

uint8_t speedOverflowAndDeadBandHandling(unsigned int aSpeedPWM) {
    aSpeedPWM += SPEED_DEAD_BAND;
    // overflow handling since analogWrite only accepts byte values
    if (aSpeedPWM > MAX_SPEED_PWM) {
        aSpeedPWM = MAX_SPEED_PWM;
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
 * positive -> left down
 * negative -> right down
 */
void doSensorChange(uint8_t aSensorType, struct SensorCallback *aSensorCallbackInfo) {
    (void) aSensorType; // to avoid -Wunused-parameter

    if (sSensorChangeCallCountForZeroAdjustment < CALLS_FOR_ZERO_ADJUSTMENT) {
        if (sSensorChangeCallCountForZeroAdjustment == 0) {
            // init values
            sYZeroValueAdded = 0;
        }
        // Sum for zero adjustment
        sYZeroValueAdded += aSensorCallbackInfo->ValueY;
        sSensorChangeCallCountForZeroAdjustment++;
    } else if (sSensorChangeCallCountForZeroAdjustment == CALLS_FOR_ZERO_ADJUSTMENT) {
        sSensorChangeCallCountForZeroAdjustment++;
        // compute and set zero value
        // compute zero value. Only Y values makes sense.
        sYZeroValue = sYZeroValueAdded / CALLS_FOR_ZERO_ADJUSTMENT;
        BlueDisplay1.playTone(24); // feedback for zero value acquired
    } else {

        /*
         * regular operation here
         * left right handling
         */
#ifdef CAR_HAS_4_WHEELS
        int tLeftRightValue = aSensorCallbackInfo->ValueX * 12.0;
#else
        int tLeftRightValue = aSensorCallbackInfo->ValueX * 8.0; // Scale value
#endif
        tLeftRightValue = setPositiveNegativeSliders(&sAccelerationLeftRightSliders, tLeftRightValue, LEFT_RIGHT_SENSOR_DEAD_BAND);

        /*
         * forward backward handling
         */
        int tSpeedPWMValue = -((aSensorCallbackInfo->ValueY - sYZeroValue) * (MAX_SPEED_PWM / 10)); // Scale value
        tSpeedPWMValue = setPositiveNegativeSliders(&sAccelerationForwardBackwardSliders, tSpeedPWMValue, SPEED_SENSOR_DEAD_BAND);

        /*
         * Print speed as value of bottom slider
         */
        sprintf(sStringBuffer, "%4d", tSpeedPWMValue);
        SliderBackward.printValue(sStringBuffer);

        /*
         * Get direction
         */
        uint8_t tDirection = DIRECTION_FORWARD;
        if (tSpeedPWMValue < 0) {
            tSpeedPWMValue = -tSpeedPWMValue;
            tDirection = DIRECTION_BACKWARD;
        }

        RobotCarMotorControl.rightCarMotor.setSpeedPWM(speedOverflowAndDeadBandHandling(tSpeedPWMValue + tLeftRightValue),
                tDirection);
        RobotCarMotorControl.leftCarMotor.setSpeedPWM(speedOverflowAndDeadBandHandling(tSpeedPWMValue - tLeftRightValue),
                tDirection);
    }
}

void initBTSensorDrivePage(void) {
    /*
     * 4 Slider
     */
// Position Slider at middle of screen
// Top slider
    SliderForward.init(SENSOR_SLIDER_CENTER_X, SENSOR_SLIDER_CENTER_Y - VERTICAL_SLIDER_LENTGH, SENSOR_SLIDER_WIDTH,
    VERTICAL_SLIDER_LENTGH, SLIDER_SPEED_THRESHOLD, 0, SLIDER_BACKGROUND_COLOR, SLIDER_BAR_COLOR, FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);

//    SliderForward.setBarThresholdColor(SLIDER_THRESHOLD_COLOR);

    // Bottom slider
    SliderBackward.init(SENSOR_SLIDER_CENTER_X, SENSOR_SLIDER_CENTER_Y, SENSOR_SLIDER_WIDTH, -(VERTICAL_SLIDER_LENTGH),
    SLIDER_SPEED_THRESHOLD, 0, SLIDER_BACKGROUND_COLOR, SLIDER_BAR_COLOR, FLAG_SLIDER_IS_ONLY_OUTPUT, NULL);
//    SliderBackward.setBarThresholdColor(SLIDER_THRESHOLD_COLOR);
    SliderForward.setScaleFactor((float) MAX_SPEED_PWM / (float) VERTICAL_SLIDER_LENTGH);
    SliderBackward.setScaleFactor((float) MAX_SPEED_PWM / (float) VERTICAL_SLIDER_LENTGH);
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
#endif // ROBOT_CAR_BT_SENSOR_DRIVE_PAGE_HPP
#pragma once
