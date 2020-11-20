/*
 * CarIMUData.cpp
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>
#include "Wire.h"
#include "CarIMUData.h"

//#define DEBUG // Only for development

#define NUMBER_OF_OFFSET_CALIBRATION_SAMPLES 500

WordUnion Accelerator[3]; // Values without offset, +/-2 | 4 g for 16 bit full range
int16_t AcceleratorOffset[3];
int16_t GyroscopeOffset[3];

int16_t AcceleratorForwardOffset;
LongUnion Speed;

int16_t GyroscopePanOffset;
WordUnion GyroscopePan; // Values without offset, +/-250 | 500 degree per second at 16 bit full range
LongUnion TurnAngle;

void readGyroZFromMPU6050Fifo() {
    uint16_t tFifoCount = readWordSwapped(MPU6050_RA_FIFO_COUNTH);

    /*
     * 120 us @400kHz overhead for each transfer.
     * 560 us for 20 bytes, 900 us for 32 bytes to read
     */
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(MPU6050_RA_FIFO_R_W);
    Wire.endTransmission(false);
    uint8_t tReceivedCount;
    tFifoCount &= 0xFE; // Make count even
    while (tFifoCount > 0) {
        tReceivedCount = Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) tFifoCount, (uint8_t) true);
#ifdef DEBUG
        if (tReceivedCount & 0x01) {
            Serial.print(tReceivedCount);
            Serial.print(" is not even "); // never seen :-)
        }
#endif
        for (uint8_t i = 0; i < tReceivedCount; i += 2) {
            GyroscopePan.Byte.HighByte = Wire.read();
            GyroscopePan.Byte.LowByte = Wire.read();
            GyroscopePan.Word = GyroscopePan.Word - GyroscopePanOffset;
            // Compute gyro integral
            TurnAngle.Long += GyroscopePan.Word;
//                    NumberOfFifoSamples++;
        }
        tFifoCount -= tReceivedCount;
    }
}

int getTurnAngleWithHalfDegreeResolution() {
    return TurnAngle.Word.HighWord.Word;
}

/*
 * This is impossible with 1 kHz FIFO rate!
 */
void readAccelFromMPU6050Fifo() {

    uint16_t tFifoCount = readWordSwapped(MPU6050_RA_FIFO_COUNTH);
    /*
     * 120 us @400kHz overhead for each transfer.
     * 560 us for 20 bytes, 900 us for 32 bytes to read
     */
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(MPU6050_RA_FIFO_R_W);
    Wire.endTransmission(false);
#ifdef DEBUG
    Serial.print(tFifoCount);
    Serial.print("|");
#endif
    while (tFifoCount > 6) {
        uint8_t tChunkCount = tFifoCount / 6; // accept only 6 bytes chunks
        if (tChunkCount > 0) {
            if (tChunkCount > BUFFER_LENGTH / 6) {
                tChunkCount = BUFFER_LENGTH / 6; // we can get only 5 chunks of data with the 32 byte buffer of the Wire library
            }
            // this reads max 30 bytes 5 chunks
            uint8_t tReceivedCount = Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) (tChunkCount * 6),
                    (uint8_t) true);
#ifdef DEBUG
            if (tReceivedCount & 0x01) {
                Serial.print(tReceivedCount);
                Serial.print(" is not even ");
            }
#endif
            for (uint8_t j = 0; j < tReceivedCount; j += 6) {
                for (uint8_t i = 0; i < 3; i++) {
                    Accelerator[i].Byte.HighByte = Wire.read();
                    Accelerator[i].Byte.LowByte = Wire.read();
                    Accelerator[i].Word = Accelerator[i].Word - AcceleratorOffset[i];
                    if (i == 1) {
                        Speed.Long += Accelerator[i].Word;
                    }
                }
            }
            tFifoCount -= tReceivedCount;
        }
    }
}

void calculateAcceleratorAllOffsets() {
    int32_t tSum[3] = { 0, 0, 0 };

    for (int i = 0; i < NUMBER_OF_OFFSET_CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
        Wire.write(MPU6050_RA_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 6, (uint8_t) true);

        for (uint8_t j = 0; j < 3; j++) {
            int16_t tSample = Wire.read() << 8;
            tSample |= Wire.read();
            tSum[j] += tSample;
        }
        delay(1);
    }
    AcceleratorOffset[0] = tSum[0] / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
    AcceleratorOffset[1] = tSum[1] / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
    AcceleratorOffset[2] = tSum[2] / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
}

void calculateGyroscopeAllOffsets() {
    int32_t tSum[3] = { 0, 0, 0 };

    for (int i = 0; i < NUMBER_OF_OFFSET_CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
        Wire.write(MPU6050_RA_GYRO_YOUT_H);

        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 6, (uint8_t) true);

        for (uint8_t j = 0; j < 3; j++) {
            int16_t tSample = Wire.read() << 8;
            tSample |= Wire.read();
            tSum[j] += tSample;
        }
        delay(1);
    }
    for (uint8_t i = 0; i < 3; i++) {
        GyroscopeOffset[i] = tSum[i] / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
    }
}

/*
 * Read all data  at 1 kHz rate direct from registers, not from FIFI
 * and compute and store only the required values in one turn.
 */
void calculateSpeedAndTurnOffsets() {
    initMPU6050ForGyroZFifo(); // this sets also accelerator part to the right value for offset calculation and fifo does not matter here

    int32_t tSumSpeed = 0;
    int32_t tSumTurn = 0;

    for (int i = 0; i < NUMBER_OF_OFFSET_CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
        Wire.write(MPU6050_RA_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 14, (uint8_t) true);

#ifdef USE_ACCELERATOR_Y_FOR_SPEED
        // skip x value
        Wire.read();
        Wire.read();
#endif
        // read forward value
        int16_t tSample = Wire.read() << 8;
        tSample |= Wire.read();
        tSumSpeed += tSample;

#ifndef USE_ACCELERATOR_Y_FOR_SPEED
        // skip y value
        Wire.read();
        Wire.read();
#endif

        // skip z, temp and 2 gyroscope values value
        for (uint8_t i = 0; i < 8; i++) {
            Wire.read();
        }

        // read pan (Z) value
        tSample = Wire.read() << 8;
        tSample |= Wire.read();
        tSumTurn += tSample;

        delay(1);
    }
    AcceleratorForwardOffset = tSumSpeed / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
    GyroscopePanOffset = tSumTurn / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
}

void printAcceleratorAllOffsets(Print *aSerial) {
    aSerial->print(F("Acc offsets: X="));
    aSerial->print(AcceleratorOffset[0]);
    aSerial->print('|');
    aSerial->print(AcceleratorOffset[0] * ACCEL_RAW_TO_G_FOR_2G_RANGE);
    aSerial->print(F(" Y="));
    aSerial->print(AcceleratorOffset[1]);
    aSerial->print('|');
    aSerial->print(AcceleratorOffset[1] * ACCEL_RAW_TO_G_FOR_2G_RANGE);
    aSerial->print(F(" Z="));
    aSerial->print(AcceleratorOffset[2]);
    aSerial->print('|');
    aSerial->print(AcceleratorOffset[2] * ACCEL_RAW_TO_G_FOR_2G_RANGE);
    aSerial->println();
}

void printGyroscopeAllOffsets(Print *aSerial) {
    aSerial->print(F("Gyro offsets: X="));
    aSerial->print(GyroscopeOffset[0]);
    aSerial->print('|');
    aSerial->print(GyroscopeOffset[0] * GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE);
    aSerial->print(F(" Y="));
    aSerial->print(GyroscopeOffset[1]);
    aSerial->print('|');
    aSerial->print(GyroscopeOffset[1] * GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE);
    aSerial->print(F(" Z="));
    aSerial->print(GyroscopeOffset[2]);
    aSerial->print('|');
    aSerial->print(GyroscopeOffset[2] * GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE);
    aSerial->println();
}

void printSpeedAndTurnOffsets(Print *aSerial) {
    aSerial->print(F("Speed offset="));
    aSerial->print(AcceleratorForwardOffset);
    aSerial->print('|');
    aSerial->print(AcceleratorForwardOffset * ACCEL_RAW_TO_G_FOR_2G_RANGE);
    aSerial->print(F(" turn offset="));
    aSerial->print(GyroscopePanOffset);
    aSerial->print('|');
    aSerial->println(GyroscopePanOffset * GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE);
}

/*
 * This requires 180 bytes more program memory :-( (because it could not be inlined ?)
 */
void calculateOffsets(uint8_t aOutRegisterStartAddress, int16_t * aOffsetArray) {
    int32_t tSum[3] = { 0, 0, 0 };

    for (int i = 0; i < NUMBER_OF_OFFSET_CALIBRATION_SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
        Wire.write(aOutRegisterStartAddress);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 6, (uint8_t) true);

        for (uint8_t j = 0; j < 3; j++) {
            int16_t tSample = Wire.read() << 8;
            tSample |= Wire.read();
            tSum[j] += tSample;
        }
        delay(1);
    }
    for (uint8_t i = 0; i < 3; i++) {
        *aOffsetArray++ = tSum[i] / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
    }
}

void resetMPU6050Fifo() {
    writeData(MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_FIFO_RESET_BIT)); // Reset FIFO
    writeData(MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_FIFO_EN_BIT)); // enable FIFO
}

void initMPU6050ForGyroZFifo() {
    TurnAngle.ULong = 0;
    writeData(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // use recommended gyro reference: PLL with Z axis gyroscope reference
    // Set sample rate to 1 kHz
    writeData(MPU6050_RA_SMPLRT_DIV, 0x00); // Sample rate, divider is minimum (1)
    writeData(MPU6050_RA_CONFIG, 0x01); // ext input disabled, DLPF enabled: accel 184Hz gyro 188Hz @1kHz
    // the next is default and never changed by us
//    writeData(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250 << (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1)); // range +/- 250
    writeData(MPU6050_RA_FIFO_EN, _BV(MPU6050_ZG_FIFO_EN_BIT)); // FIFO: only Gyro Z enabled
    resetMPU6050Fifo();
}

void initMPU6050ForAccelFifo() {
    Speed.ULong = 0;
    writeData(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // use recommended gyro reference: PLL with Z axis gyroscope reference
    // Set sample rate to 1/4 kHz
    writeData(MPU6050_RA_SMPLRT_DIV, 0x03); // Sample rate 1/4 kHz
    writeData(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); // ext input disabled, DLPF enabled: ~50 Hz Sample freq = 1kHz
    // the next is default and never changed by us
//    writeData(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1)); // range +/- 2g
    writeData(MPU6050_RA_FIFO_EN, _BV(MPU6050_ACCEL_FIFO_EN_BIT)); // FIFO: all Accel axes enabled
    resetMPU6050Fifo();
}

void initMPU6050() {
//    writeData(MPU6050_RA_SMPLRT_DIV, 0x07); // Sample rate, divider is 8 since DLPF is disabled und gyro runs at 8MHz
//    writeData(MPU6050_RA_CONFIG, 0x00); // ext input disabled, DLPF disabled: accel 260Hz gyro 260Hz @8kHz
    writeData(MPU6050_RA_SMPLRT_DIV, 0x00); // Sample rate, divider is minimum (1)
    writeData(MPU6050_RA_CONFIG, 0x01); // ext input disabled, DLPF enabled: accel 184Hz gyro 188Hz @1kHz

//    writeData(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); // ext input disabled, DLPF enabled: ~50 Hz Sample freq = 1kHz
//    writeData(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_10); // ext input disabled, DLPF enabled: ~10 Hz Sample freq = 1kHz

    // range select
    writeData(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250 << (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1)); // range +/- 250
    writeData(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1)); // range +/- 2g

    writeData(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // use recommended gyro reference: PLL with Z axis gyroscope reference
}

void writeData(uint8_t aRegisterNumber, uint8_t aData) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.write(aData);
    Wire.endTransmission();
}

uint8_t readByte(uint8_t aRegisterNumber) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.endTransmission(true);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 1, (uint8_t) true);
    return Wire.read();
}

uint16_t readWord(uint8_t aRegisterNumber) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.endTransmission(true);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 2, (uint8_t) true);
    WordUnion tWord;
    tWord.UByte.LowByte = Wire.read();
    tWord.UByte.HighByte = Wire.read();
    return tWord.UWord;
}

/*
 * Read high byte first
 */
uint16_t readWordSwapped(uint8_t aRegisterNumber) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.endTransmission(true);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 2, (uint8_t) true);
    WordUnion tWord;
    tWord.UByte.HighByte = Wire.read();
    tWord.UByte.LowByte = Wire.read();
    return tWord.UWord;
}
