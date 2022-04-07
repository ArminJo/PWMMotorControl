/*
 * CarIMUData.h
 *
 *  Functions for getting IMU data from MPU6050 for car control.
 *
 *  Created on: 19.11.2020
 *  Copyright (C) 2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of PWMMotorControl https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _IMU_CAR_DATA_H
#define _IMU_CAR_DATA_H

#include <stdint.h>
#include "MPU6050Defines.h"
#include "LongUnion.h"

/*
 * Activate this if the y axis of the GY-521 MPU6050 breakout board points forward / backward, i.e. connectors are at the right.
 */
// #define USE_ACCELERATOR_Y_FOR_SPEED
/*
 * Activate this if the axis of the GY-521 MPU6050 breakout board points backwards.
 */
// #define USE_NEGATIVE_ACCELERATION_FOR_SPEED
// #define DISABLE_AUTO_OFFSET // default is enabled
//
#define NUMBER_OF_OFFSET_CALIBRATION_SAMPLES 512
#define ACCEL_MOVE_THRESHOLD  128
#define GYRO_MOVE_THRESHOLD    64

#if !defined(SAMPLE_RATE)
#define SAMPLE_RATE          1000
//#define SAMPLE_RATE         500
//#define SAMPLE_RATE         250
//#define SAMPLE_RATE         125
#endif
#if SAMPLE_RATE == 1000
#define RATE_SHIFT                      0
#define DELAY_TO_NEXT_IMU_DATA_MILLIS   1
#elif SAMPLE_RATE == 500
#define RATE_SHIFT                      1
#define DELAY_TO_NEXT_IMU_DATA_MILLIS   2
#elif SAMPLE_RATE == 250
#define RATE_SHIFT                      2
#define DELAY_TO_NEXT_IMU_DATA_MILLIS   4
#elif SAMPLE_RATE == 125
#define RATE_SHIFT                      3
#define DELAY_TO_NEXT_IMU_DATA_MILLIS   8
#else
#error SAMPLE_RATE must be 1000, 500, 250 or 125
#endif

#define ACCEL_RAW_TO_G_FOR_2G_RANGE (4.0/65536.0)
#define GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE (500.0/65536.0) // 0.00762939 or 1/131.072

class IMUCarData {
public:

//    IMUCarData();

    void initWire(); // Is called by initMPU6050()
    void initMPU6050();
    void initMPU6050FifoForCarData();
    unsigned int getMPU6050SampleRate(); // just returns SAMPLE_RATE

    void resetCarData();
    void resetMPU6050FifoAndCarData();
    void reset();
    void resetOffsetData();
    void resetOffsetDataAndWait();

    void readCarDataFromMPU6050();
    bool readCarDataFromMPU6050Fifo();

    void delayAndReadIMUCarDataData(unsigned long aMillisDelay);

    void printIMUCarDataCaption(Print *aSerial);
    bool printIMUCarDataDataPeriodically(Print *aSerial, uint16_t aPeriodMillis);
    void printIMUCarData(Print *aSerial);

    int8_t getAcceleratorForward15MilliG();
    int8_t getAcceleratorForward4MilliG();
    int8_t getAcceleratorForwardLowPass8();
    int8_t getAcceleratorForwardLowPass4();
    int getSpeedCmPerSecond();
    int getSpeedFastWithHigherResolution();
    int getDistanceCm();
    int getDistanceMillimeter();

    int getGyroscopePan2DegreePerSecond();
    int getTurnAngleHalfDegree();
    int getTurnAngleDegree();

    void doAutoOffset();
    void calculateSpeedAndTurnOffsets();

    void printSpeedAndTurnOffsets(Print *aSerial);

    uint8_t MPU6050ReadByte(uint8_t aRegisterNumber);
    uint16_t MPU6050ReadWordSwapped(uint8_t aRegisterNumber);
    void MPU6050WriteByte(uint8_t aRegisterNumber, uint8_t aData);

    /*
     * AcceleratorForwardOffset == 0 means, offsets are uninitialized and values are wrong
     */
    bool OffsetsHaveChanged;
    int16_t AcceleratorForwardOffset; // At 1kHz, offset 1 gives speed 4 per second / 1 per 4 second for 0.625 mm/s / 1cm/s resolution
    WordUnion AcceleratorForward; // Average of every bulk read from the fifo. 16384 LSB per g => 61 ug per LSB, ~16 mg per 256 LSB
    LongUnion AcceleratorForwardLowPass4;
    LongUnion AcceleratorForwardLowPass8;
    /*
     * Speed is the sum of AcceleratorForward.
     * At 1 kHz and 1 g (~10 m/s^2): Per sample (1/1000 second) we have 16384 (0x4000) and 10/1000 m/s (1 cm/s).
     */
    LongUnion Speed;    // shift 14 -> 1 (0.981) cm/s , 12 -> 2.5 mm/s, 8 -> 0.15328 mm/s
    /*
     * Distance is the sum of Speed >> 8.
     * At 1 kHz: with 1 cm/s (shift (14-8)) we have 1 cm (or shift (24-8)) after 1 second (1000 additions)
     */
    LongUnion Distance;

    int16_t GyroscopePanOffset;
    WordUnion GyroscopePan; // Values without offset, +/-250 | 500 degree per second at 16 bit full range -> ~2 dps per 256 LSB
    /*
     * 1000 samples / second
     * => 17 bit LSB per degree at +/-250 dps range
     * The upper word has a resolution of 1/2 degree at 1000 samples per second
     */
    LongUnion TurnAngle;
    uint32_t LastFifoCheckMillis;

    /*
     * Variables for AutoOffset
     */
    int16_t sCountOfUndisturbedFifoChunks = 0; // signed, since it is used in formulas with other signed values
    int32_t sSpeedSnapshot;
    int32_t sTurnSnapshot;
};

#endif // _IMU_CAR_DATA_H
#pragma once
