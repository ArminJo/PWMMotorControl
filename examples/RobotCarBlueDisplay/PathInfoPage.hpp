/*
 * PathInfoPage.hpp
 *
 *  Contains all the GUI elements for managing and showing the path.
 *
 *  insertToPath() and DrawPath() to show the path we were driving.
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */
#ifndef _ROBOT_CAR_PATH_INFO_PAGE_HPP
#define _ROBOT_CAR_PATH_INFO_PAGE_HPP

#if defined(ENABLE_PATH_INFO_PAGE)
#if defined(DEBUG)
#define LOCAL_DEBUG
#else
#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

BDButton TouchButtonResetPath;
BDButton TouchButtonBackSmall;

#pragma GCC diagnostic ignored "-Wunused-parameter"
void doResetPath(BDButton * aTheTouchedButton, int16_t aValue) {
    resetPathData();
    drawPathInfoPage();
}

void initPathInfoPage(void) {
    TouchButtonResetPath.init(0, 0, BUTTON_WIDTH_3_5, BUTTON_HEIGHT_6, COLOR16_RED, F("Clear"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, 0, &doResetPath);

    TouchButtonBackSmall.init(BUTTON_WIDTH_4_POS_4, 0, BUTTON_WIDTH_4, BUTTON_HEIGHT_6, COLOR16_RED, F("Back"), TEXT_SIZE_22,
            FLAG_BUTTON_DO_BEEP_ON_TOUCH, PAGE_AUTOMATIC_CONTROL, &GUISwitchPages);
}

void drawPathInfoPage(void) {
    drawCommonGui();
    BlueDisplay1.drawText(HEADER_X + (2 * TEXT_SIZE_22_WIDTH), TEXT_SIZE_22_HEIGHT + TEXT_SIZE_22_HEIGHT, F("Path"));

    TouchButtonBackSmall.drawButton();
    TouchButtonStep.drawButton();
    TouchButtonResetPath.drawButton();

    DrawPath();
}

void startPathInfoPage(void) {
    drawPathInfoPage();
}

void loopPathInfoPage(void) {

}

void stopPathInfoPage(void) {
}

/*
 * Forward 0 degree is X direction
 */
int xPathDelta[PATH_LENGTH_MAX], yPathDelta[PATH_LENGTH_MAX];
int *sXPathDeltaPtr, *sYPathDeltaPtr;
// exact float values to avoid aggregating rounding errors
float sLastXPathFloat, sLastYPathFloat;
int sLastXPathInt, sLastYPathInt;
// for layout of path data
int sXPathMax, sXPathMin, sYPathMax, sYPathMin;
int sLastPathDirectionDegree;

void resetPathData() {
    sXPathDeltaPtr = &xPathDelta[0];
    sYPathDeltaPtr = &yPathDelta[0];
// set to origin
    sLastXPathFloat = 0.0;
    sLastYPathFloat = 0.0;
    sLastXPathInt = 0;
    sLastYPathInt = 0;
    sXPathMax = sYPathMax = 0;
    // Yes current minimum is zero since it can be negative
    sXPathMin = sYPathMin = 0;

    sLastPathDirectionDegree = 0;
// clear delta arrays
    for (unsigned int i = 0; i < PATH_LENGTH_MAX; ++i) {
        xPathDelta[i] = 0;
        yPathDelta[i] = 0;
    }
    // to start new path in right direction
//    sNextDegreesToTurn = 0;
    sLastDegreesTurned = 0;
}

/*
 * (Over-)writes to current pointer position
 * 0 degree goes in X direction
 * @param aAddEntry if false only values of current entry will be adjusted
 */
void insertToPath(int aLength, int aDegree, bool aAddEntry) {
#if defined(LOCAL_DEBUG)
    BlueDisplay1.debug("Degree=", aDegree);
    BlueDisplay1.debug("Length=", aLength);
#endif

// get new direction
    int tLastPathDirectionDegree = sLastPathDirectionDegree + aDegree;
    if (aAddEntry) {
        sLastPathDirectionDegree = tLastPathDirectionDegree;
    }
#if defined(LOCAL_DEBUG)
    BlueDisplay1.debug("LastDegree=", tLastPathDirectionDegree);
#endif

    float tRadianOfDegree = tLastPathDirectionDegree * (M_PI / 180);
    /*
     * compute X and Y delta and min/max
     */
    float tNewXPathFloat = (cos(tRadianOfDegree) * aLength) + sLastXPathFloat;
    if (aAddEntry) {
        sLastXPathFloat = tNewXPathFloat;
    }
    int tXDelta = int(tNewXPathFloat) - sLastXPathInt;
    *sXPathDeltaPtr = tXDelta;
    int tLastXPathInt = sLastXPathInt + tXDelta;
    if (aAddEntry) {
        sLastXPathInt = tLastXPathInt;
    }

    // X-Min and Max
    if (tLastXPathInt > sXPathMax) {
        sXPathMax = tLastXPathInt;
    } else if (tLastXPathInt < sXPathMin) {
        sXPathMin = tLastXPathInt;
    }

    // Y delta
    float tNewYPathFloat = (sin(tRadianOfDegree) * aLength) + sLastYPathFloat;
    if (aAddEntry) {
        sLastYPathFloat = tNewYPathFloat;
    }
    int tYDelta = int(tNewYPathFloat) - sLastYPathInt;
    *sYPathDeltaPtr = tYDelta;
    int tLastYPathInt = sLastYPathInt + tYDelta;
    if (aAddEntry) {
        sLastYPathInt = tLastYPathInt;
    }

    // Y-Min and Max
    if (tLastYPathInt > sYPathMax) {
        sYPathMax = tLastYPathInt;
    } else if (tLastYPathInt < sYPathMin) {
        sYPathMin = tLastYPathInt;
    }
    if (aAddEntry) {
        if (sXPathDeltaPtr < &xPathDelta[PATH_LENGTH_MAX - 2]) {
            sXPathDeltaPtr++;
            sYPathDeltaPtr++;
        }
    }
}

/*
 * Draw so that (forward) x direction is mapped to display y value since we have landscape layout
 * y+ is left y- is right
 */
void DrawPath() {
    /*
     * compute scale factor
     */
    int tXdelta = sXPathMax - sXPathMin;
    int tYdelta = sYPathMax - sYPathMin;
    uint8_t tScaleShift = 0;
    while (tXdelta > DISPLAY_HEIGHT || tYdelta >= DISPLAY_WIDTH) {
        tScaleShift++;
        tXdelta >>= 1;
        tYdelta >>= 1;
    }
//    BlueDisplay1.debug("ScaleShift=", tScaleShift);

    /*
     * Try to position start point at middle of bottom line
     */
    int tXDisplayPos;
    if (tYdelta < (DISPLAY_WIDTH / 2)) {
        tXDisplayPos = DISPLAY_WIDTH / 2;
    } else {
        // position at left so that sYPathMax (which is known to be > 0) fits on screen
        tXDisplayPos = (sYPathMax >> tScaleShift) + 2; // +2 for left border
    }
    int tYDisplayPos = DISPLAY_HEIGHT + (sXPathMin >> tScaleShift);

    /*
     * Draw Path -> map path x to display y
     */
    int * tXDeltaPtr = &xPathDelta[0];
    int * tYDeltaPtr = &yPathDelta[0];
    while (tXDeltaPtr <= sXPathDeltaPtr) {
        int tYDisplayDelta = (-(*tXDeltaPtr++)) >> tScaleShift;
        int tXDisplayDelta = (-(*tYDeltaPtr++)) >> tScaleShift;
        BlueDisplay1.drawLineRel(tXDisplayPos, tYDisplayPos, tXDisplayDelta, tYDisplayDelta, COLOR16_RED);
        tXDisplayPos += tXDisplayDelta;
        tYDisplayPos += tYDisplayDelta;
    }
}
#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // ENABLE_PATH_INFO_PAGE
#endif // _ROBOT_CAR_PATH_INFO_PAGE_HPP
