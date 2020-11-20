/*
 * CarIMUData.h
 *
 *  Functions for getting IMU data from MPU6050 for car control.
 *
 *  Created on: 19.11.2019
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef CAR_IMU_DATA_H_
#define CAR_IMU_DATA_H_

#include <stdint.h>
#include "MPU6050Defines.h"

#define ACCEL_RAW_TO_G_FOR_2G_RANGE (4.0/65536.0)
#define GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE (500.0/65536.0) // 0.00762939 or 1/131.072

/*
 * Sometimes it helps the compiler if you use this unions
 */
union WordUnion {
    struct {
        uint8_t LowByte;
        uint8_t HighByte;
    } UByte;
    struct {
        int8_t LowByte;
        int8_t HighByte;
    } Byte;
    uint8_t UBytes[2];
    int8_t Bytes[2];
    uint16_t UWord;
    int16_t Word;
    uint8_t * BytePointer;
};

union LongUnion {
    struct {
        uint8_t LowByte;
        uint8_t MidLowByte;
        uint8_t MidHighByte;
        uint8_t HighByte;
    } Byte;
    struct {
        uint8_t LowByte;
        WordUnion MidWord;
        uint8_t HighByte;
    } ByteWord;
    struct {
        WordUnion LowWord;
        WordUnion HighWord;
    } Word;
    uint8_t Bytes[4];
    uint16_t Words[2];
    uint32_t ULong;
    int32_t Long;
};

extern int16_t AcceleratorOffset[3];
extern int16_t GyroscopeOffset[3];
extern WordUnion Accelerator[3]; // Values without offset, +/-2 | 4 g for 16 bit full range

extern LongUnion Speed;

extern int16_t AcceleratorForwardOffset;
extern LongUnion Speed;

extern int16_t GyroscopePanOffset;
extern WordUnion GyroscopePan; // Values without offset, +/-250 | 500 degree per second at 16 bit full range
extern LongUnion TurnAngle;

void resetMPU6050Fifo();

void initMPU6050ForAccelFifo();
void readAccelFromMPU6050Fifo();

void initMPU6050ForGyroZFifo();
void readGyroZFromMPU6050Fifo();
int getTurnAngleWithHalfDegreeResolution();

void calculateAcceleratorAllOffsets();
void calculateGyroscopeAllOffsets();
void calculateSpeedAndTurnOffsets();

void printAcceleratorAllOffsets(Print *aSerial);
void printGyroscopeAllOffsets(Print *aSerial);
void printSpeedAndTurnOffsets(Print *aSerial);

uint8_t readByte(uint8_t aRegisterNumber);
uint16_t readWord(uint8_t aRegisterNumber);
uint16_t readWordSwapped(uint8_t aRegisterNumber);
void writeData(uint8_t aRegisterNumber, uint8_t aData);

#endif /* CAR_IMU_DATA_H_ */

#pragma once

