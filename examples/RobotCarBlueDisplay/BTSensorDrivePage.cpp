/*
 * BTSensorDrivePage.cpp
 *
 *  Contains all the GUI elements for driving the car with the smartphone sensors.
 *
 *  Requires BlueDisplay library.
 *
 *  Created on: 13.05.2019
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

//#pragma GCC diagnostic ignored "-Wunused-parameter"

// 4 Sliders for accelerometer
BDSlider SliderForward;     // Y negative
BDSlider SliderBackward;    // Y positive
BDSlider SliderRight;       // X negative
BDSlider SliderLeft;        // X positive

#define SLIDER_BACKGROUND_COLOR     COLOR_YELLOW
#define SLIDER_BAR_COLOR            COLOR_GREEN
#define SLIDER_THRESHOLD_COLOR      COLOR_BLUE

#define SENSOR_SLIDER_WIDTH         (DISPLAY_WIDTH / 16)
#define VERTICAL_SLIDER_LENTGH      ((DISPLAY_HEIGHT / 4) + (DISPLAY_HEIGHT / 10))
#define SLIDER_SPEED_THRESHOLD      DEFAULT_START_SPEED
#define SPEED_DEAD_BAND             (DEFAULT_START_SPEED / 2)

#define HORIZONTAL_SLIDER_LENTGH    (DISPLAY_HEIGHT / 4)
#define SLIDER_LEFT_RIGHT_THRESHOLD (HORIZONTAL_SLIDER_LENTGH / 4)
#define LEFT_RIGHT_DEAD_BAND        4

#define SENSOR_SLIDER_CENTER_X      ((DISPLAY_WIDTH - SENSOR_SLIDER_WIDTH) / 2)
#define SENSOR_SLIDER_CENTER_Y      ((DISPLAY_HEIGHT / 2) + (DISPLAY_HEIGHT / 16))

/*
 * Sensor callback handler
 */
uint8_t sSensorChangeCallCountForZeroAdjustment;
#define CALLS_FOR_ZERO_ADJUSTMENT 8 // The values of the first 8 calls are used as zero value.
float sYZeroValueAdded; // The accumulator for the values of the first 8 calls.
float sYZeroValue = 0;

int sLastSpeedValue = 0;
int sLastLeftRightValue = 0;

uint8_t speedOverflowAndDeadBandHandling(unsigned int aSpeed) {
    // overflow handling since analogWrite only accepts byte values
    if (aSpeed > 0xFF) {
        aSpeed = 0xFF;
    }
    if (aSpeed <= SPEED_DEAD_BAND) {
        aSpeed = 0;
    }
    return aSpeed;
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
void doSensorChange(uint8_t aSensorType, struct SensorCallback * aSensorCallbackInfo) {
    if (sSensorChangeCallCountForZeroAdjustment < CALLS_FOR_ZERO_ADJUSTMENT) {
        if (sSensorChangeCallCountForZeroAdjustment == 0) {
            // init values
            sYZeroValueAdded = 0;
        }
        // Add for zero adjustment
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
        // Scale value
        int tLeftRightValue = aSensorCallbackInfo->ValueX * 8.0;

        BDSlider * tValueSlider = &SliderLeft;
        BDSlider * tZeroSlider = &SliderRight;
        if (tLeftRightValue < 0) {
            tLeftRightValue = -tLeftRightValue;
            tValueSlider = &SliderRight;
            tZeroSlider = &SliderLeft;
        }
        if (tLeftRightValue > LEFT_RIGHT_DEAD_BAND) {
            // dead band subtraction
            tLeftRightValue -= LEFT_RIGHT_DEAD_BAND;
        } else {
            tLeftRightValue = 0;
        }

        if (sLastLeftRightValue != tLeftRightValue) {
            sLastLeftRightValue = tLeftRightValue;

            tValueSlider->setValueAndDrawBar(tLeftRightValue);
            tZeroSlider->setValueAndDrawBar(0);
        }

        /*
         * restore sign for speed setting below
         */
        if (tZeroSlider == &SliderLeft) {
            tLeftRightValue = -tLeftRightValue;
        }

        /*
         * forward backward handling
         */
        int tSpeedValue = -((aSensorCallbackInfo->ValueY - sYZeroValue) * (MAX_SPEED / 7));
        if (sLastSpeedValue != tSpeedValue) {
            sLastSpeedValue = tSpeedValue;
            if (tSpeedValue >= 0) {
                // Forward
                tSpeedValue = speedOverflowAndDeadBandHandling(tSpeedValue);

                RobotCarMotorControl.setSpeedCompensated(tSpeedValue, DIRECTION_FORWARD, tLeftRightValue);
                SliderBackward.setValueAndDrawBar(0);
                SliderForward.setValueAndDrawBar(tSpeedValue);

            } else {
                // Backward
                tSpeedValue = speedOverflowAndDeadBandHandling(-tSpeedValue);

                RobotCarMotorControl.setSpeedCompensated(tSpeedValue, DIRECTION_BACKWARD, tLeftRightValue);
                SliderForward.setValueAndDrawBar(0);
                SliderBackward.setValueAndDrawBar(tSpeedValue);
            }
            /*
             * Print speed as value of bottom slider
             */
            sprintf(sStringBuffer, "%3d", tSpeedValue);
            SliderBackward.printValue(sStringBuffer);
        }
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
    SliderForward.setScaleFactor((float) MAX_SPEED / (float) VERTICAL_SLIDER_LENTGH);
    SliderBackward.setScaleFactor((float) MAX_SPEED / (float) VERTICAL_SLIDER_LENTGH);

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
    drawBTSensorDrivePage();
}

void loopBTSensorDrivePage(void) {
}

void stopBTSensorDrivePage(void) {
}
