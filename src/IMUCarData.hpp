/*
 * IMUCarData.hpp
 *
 *  Functions for getting IMU data from MPU6050 for car control.
 *
 *  Created on: 19.11.2020
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
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
#ifndef _IMU_CAR_DATA_HPP
#define _IMU_CAR_DATA_HPP

#include <Arduino.h>
#include "Wire.h"
#include "IMUCarData.h"

#define FIFO_NUMBER_OF_ACCEL_VALUES 3
#define FIFO_NUMBER_OF_GYRO_VALUES  1
#define FIFO_CHUNK_SIZE             ((FIFO_NUMBER_OF_ACCEL_VALUES + FIFO_NUMBER_OF_GYRO_VALUES) * 2)

//#define DEBUG_WITHOUT_SERIAL // For debugging without Serial development
#if defined(DEBUG_WITHOUT_SERIAL)
#include "DigitalWriteFast.h"
#endif
//#define DEBUG // Only for development
//#define WARN    // Information that the program may encounter problems, like small Heap/Stack area.
//#define AUTO_OFFSET_DEBUG // Show auto offset values as dummy values for the plotter output
#include "DebugLevel.h" // Include to propagate debug levels

int8_t IMUCarData::getAcceleratorForward15MilliG() {
    return AcceleratorForward.Byte.HighByte;
}

int8_t IMUCarData::getAcceleratorForward4MilliG() {
    return AcceleratorForward.Word >> 6; // /64 -> 256 per g
}

int8_t IMUCarData::getAcceleratorForwardLowPass8() {
//    return AcceleratorForwardLowPass8.Byte.HighByte;
    return AcceleratorForwardLowPass8.Word.HighWord >> 6; // 256 per g
}

int8_t IMUCarData::getAcceleratorForwardLowPass4() {
    return AcceleratorForwardLowPass4.Word.HighWord >> 6; // 256 per g
}

// shift 14 -> 1 (0.981) cm/s , 12 -> 2.5 mm/s, 8 -> 0.15328 mm/s
int IMUCarData::getSpeedCmPerSecond() {
    return Speed.Long >> (14 - RATE_SHIFT);
}
/*
 * 0.15 mm/s resolution for 1kHz sampling rate
 */
int IMUCarData::getSpeedFastWithHigherResolution() {
    return Speed.Long >> 8;
}

/*
 * Distance is the sum of Speed >> 8.
 * At 1 kHz: with 1 cm/s (shift (14-8)) we have 1 cm (or shift (24-8)) after 1 second (1000 additions)
 */
int IMUCarData::getDistanceCm() {
#if SAMPLE_RATE == 1000
    return Distance.Word.HighWord; // 0.958 cm since we shift by 10 and do not divide by 1000
#else
    return Distance.Long >> (16 - (2 * RATE_SHIFT)); // 0.958 cm since we shift by additional 10 and do not divide by 1000
#endif
}

int IMUCarData::getDistanceMillimeter() {
#if SAMPLE_RATE == 1000
    LongUnion tDistance;
    tDistance.Long = Distance.Long * 10; // use full resolution for * 10
    return tDistance.Word.HighWord; // saves 10 bytes
//    return ((Distance.Long * 10 ) >> 16) ; // 0.958 mm
#else
    return (Distance.Long * 10) >> (16 - (2 * RATE_SHIFT)); // 0.958 cm since we shift by additional 10 and do not divide by 1000
#endif
}

/*
 * Gyroscope
 */
int IMUCarData::getGyroscopePan2DegreePerSecond() {
    return GyroscopePan.Byte.HighByte;
}

int IMUCarData::getTurnAngleHalfDegree() {
#if SAMPLE_RATE == 1000
    return TurnAngle.Word.HighWord;
#else
    return TurnAngle.Long >> (16 - RATE_SHIFT);
#endif
}

int IMUCarData::getTurnAngleDegree() {
#if SAMPLE_RATE == 1000
    return TurnAngle.Word.HighWord >> 1;
#elif SAMPLE_RATE == 500
    return TurnAngle.Word.HighWord;
#else
    return TurnAngle.Long >> (16 - (RATE_SHIFT - 1));
#endif
}

/*
 * Print caption for Serial Plotter
 * Space but NO println() at the end, to enable additional information to be printed
 */
void IMUCarData::printIMUCarDataCaption(Print *aSerial) {
//    aSerial->print(F("AccelForward[4mg/LSB] AccelForwardLP8 Speed[cm/s] Distance[cm] GyroZ[2dps/LSB] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
//    aSerial->print(F("AccelForward[4mg/LSB] AccelForwardAverage AccelForwardLP8 Speed[cm/s] Distance[cm] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
//    aSerial->print(F("AccelAvg.[4mg/LSB] AccelLP8 Speed[cm/s] Distance[cm] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
//    aSerial->print(F("AccelLP8 Speed[cm/s] Distance[cm] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
    aSerial->print(F("AccelLP4 Speed[cm/s] Distance[cm] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
}

/*
 * read from FIFO and does auto offset (if enabled)
 */
void IMUCarData::delayAndReadIMUCarDataData(unsigned long aMillisDelay) {
    while (aMillisDelay > 0) {
        readCarDataFromMPU6050Fifo();
        delay(DELAY_TO_NEXT_IMU_DATA_MILLIS);
        aMillisDelay -= DELAY_TO_NEXT_IMU_DATA_MILLIS;
    }
}

bool IMUCarData::printIMUCarDataDataPeriodically(Print *aSerial, uint16_t aPeriodMillis) {
    static unsigned long sLastPrintMillis;

    if ((millis() - sLastPrintMillis) >= aPeriodMillis) { // print data every n ms
        sLastPrintMillis = millis();
        readCarDataFromMPU6050Fifo();
        printIMUCarData(aSerial);
        return true;
    }
    return false;
}

void IMUCarData::printIMUCarData(Print *aSerial) {
//    aSerial->print(getAcceleratorForward4MilliG());
//    aSerial->print(" ");
    // Too noisy
//    aSerial->print(getAcceleratorForward4MilliG());
//    aSerial->print(" ");
    aSerial->print(getAcceleratorForwardLowPass4());
    aSerial->print(" ");
    aSerial->print(abs(getSpeedCmPerSecond()));
    aSerial->print(" ");
    aSerial->print(abs(getDistanceCm()));
    aSerial->print(" ");
//    aSerial->print(getGyroscopePan2DegreePerSecond());
//    aSerial->print(" ");
    aSerial->print(getTurnAngleHalfDegree());
    aSerial->print(" ");
}

unsigned int IMUCarData::getMPU6050SampleRate() {
    return (SAMPLE_RATE);
}

/*
 * Read raw 14 vales. Requires 500 us.
 * Sets AcceleratorForward and GyroscopePan
 */
void IMUCarData::readCarDataFromMPU6050() {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 14, (uint8_t) true);

#if defined(USE_ACCELERATOR_Y_FOR_SPEED)
// skip x value
    Wire.read();
    Wire.read();
#endif
// read forward value
    AcceleratorForward.Byte.HighByte = Wire.read();
    AcceleratorForward.Byte.LowByte = Wire.read();
#if defined(USE_NEGATIVE_ACCELERATION_FOR_SPEED)
    AcceleratorForward.Word = (-AcceleratorForward.Word) - AcceleratorForwardOffset;
#else
    AcceleratorForward.Word = AcceleratorForward.Word - AcceleratorForwardOffset;
#endif

    AcceleratorForwardLowPass8.Long += ((((int32_t) AcceleratorForward.Word) << 16) - AcceleratorForwardLowPass8.Long) >> 8; // Fixed point 2.0 us
    AcceleratorForwardLowPass4.Long += ((((int32_t) AcceleratorForward.Word) << 16) - AcceleratorForwardLowPass4.Long) >> 4; // Fixed point 2.0 us
    Speed.Long += AcceleratorForward.Word;
    Distance.Long += Speed.Long >> 8;

#if !defined(USE_ACCELERATOR_Y_FOR_SPEED)
    // skip y value
    Wire.read();
    Wire.read();
#endif

// skip z, temp and 2 gyroscope values value
    for (uint_fast8_t i = 0; i < 8; i++) {
        Wire.read();
    }

// read pan (Z) value
    GyroscopePan.Byte.HighByte = Wire.read();
    GyroscopePan.Byte.LowByte = Wire.read();
    GyroscopePan.Word -= GyroscopePanOffset;
    TurnAngle.Long += GyroscopePan.Word;
}

/*
 * @return true, if data might have changed
 */
bool IMUCarData::readCarDataFromMPU6050Fifo() {
#if defined(DEBUG_WITHOUT_SERIAL)
    digitalToggleFast(12);
#endif
    if (millis() - LastFifoCheckMillis < DELAY_TO_NEXT_IMU_DATA_MILLIS) {
        return false; // no new data expected
    }

    /*
     * Read FIFO count only once at start of the function
     */
    uint16_t tFifoCount = MPU6050ReadWordSwapped(MPU6050_RA_FIFO_COUNTH);
    LastFifoCheckMillis = millis();
    if (tFifoCount == 1024) {
        resetMPU6050FifoAndCarData();
        return true;
    }
    /*
     * 660 /1200 us @400kHz for 16 / 32 byte transfer.
     * 140 us between each block transfer
     * 1000 us for next 32 bytes transfer => ~3.1 ms for 10 chunks
     */
#if defined(DEBUG)
    Serial.print(tFifoCount);
    Serial.print("|");
#endif
    int32_t tAcceleratorForward = 0;
    int32_t tGyroscopePan = 0;
    int8_t tReadChunckCount = 0;
    while (tFifoCount >= FIFO_CHUNK_SIZE) {
        /*
         * Use only multiple of FIFO_CHUNK_SIZE as read length and clip read length to I2C BUFFER_LENGTH
         */
        uint8_t tChunkCount = tFifoCount / FIFO_CHUNK_SIZE;
        if (tChunkCount > BUFFER_LENGTH / FIFO_CHUNK_SIZE) {
            tChunkCount = BUFFER_LENGTH / FIFO_CHUNK_SIZE; // we can get 4 chunks of data with the 32 byte buffer of the Wire library
        }
        // this reads max 28 bytes or 4 chunks
        Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
        Wire.write(MPU6050_RA_FIFO_R_W);
        Wire.endTransmission(false);
        uint8_t tReceivedCount = Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) (tChunkCount * FIFO_CHUNK_SIZE),
                (uint8_t) true);
#if defined(WARN)
        if (tReceivedCount != (tChunkCount * FIFO_CHUNK_SIZE)) {
            // should never happen
            Serial.print(tReceivedCount);
            Serial.print(F(" received bytes are not equal requested quantity of "));
            Serial.println(tChunkCount * FIFO_CHUNK_SIZE);
        }
#endif
        for (uint_fast8_t j = 0; j < tReceivedCount; j += FIFO_CHUNK_SIZE) {
            // we must read all 3 values
            for (uint_fast8_t i = 0; i < FIFO_NUMBER_OF_ACCEL_VALUES; i++) {
                WordUnion tAcceleratorValue;
                tAcceleratorValue.Byte.HighByte = Wire.read();
                tAcceleratorValue.Byte.LowByte = Wire.read();
                // process only forward value
#if defined(USE_ACCELERATOR_Y_FOR_SPEED)
                if (i == 1) {
#else
                if (i == 0) {
#endif
#if defined(USE_NEGATIVE_ACCELERATION_FOR_SPEED)
                    tAcceleratorValue.Word = (-tAcceleratorValue.Word) - AcceleratorForwardOffset;
#else
                    tAcceleratorValue.Word = tAcceleratorValue.Word - AcceleratorForwardOffset;
#endif
                    tAcceleratorForward += tAcceleratorValue.Word;
                    AcceleratorForwardLowPass8.Long +=
                            ((((int32_t) tAcceleratorValue.Word) << 16) - AcceleratorForwardLowPass8.Long) >> 8; // Fixed point 2.0 us
                    AcceleratorForwardLowPass4.Long +=
                            ((((int32_t) tAcceleratorValue.Word) << 16) - AcceleratorForwardLowPass4.Long) >> 4; // Fixed point 2.0 us

                    Speed.Long += tAcceleratorValue.Word;
                    Distance.Long += Speed.Long >> 8;
                    tReadChunckCount++;
                }
            }
            WordUnion tValue;
            tValue.Byte.HighByte = Wire.read();
            tValue.Byte.LowByte = Wire.read();
            tValue.Word = tValue.Word - GyroscopePanOffset;
            tGyroscopePan += tValue.Word;
            // Compute turn angle
            TurnAngle.Long += tValue.Word;
        }
        tFifoCount -= tReceivedCount;

    } // while tFifoCount >= FIFO_CHUNK_SIZE
    sCountOfUndisturbedFifoChunks += tReadChunckCount;
    // compute average of read values
    AcceleratorForward.Word = tAcceleratorForward / (int32_t) tReadChunckCount;
    GyroscopePan.Word = tGyroscopePan / (int32_t) tReadChunckCount;

    /*
     * Get initial offset values
     */
    if (AcceleratorForwardOffset == 0 && sCountOfUndisturbedFifoChunks >= NUMBER_OF_OFFSET_CALIBRATION_SAMPLES) {
        /*
         * Take the first NUMBER_OF_OFFSET_CALIBRATION_SAMPLES values for auto offset
         * My experience is, that the Gyro offset is changing in the first seconds after power up and needs a recalibration
         */
        AcceleratorForwardOffset = Speed.Long / sCountOfUndisturbedFifoChunks;
        GyroscopePanOffset = TurnAngle.Long / sCountOfUndisturbedFifoChunks;
        OffsetsHaveChanged = true;
        resetCarData(); // reset temporarily used values
#if defined(DEBUG)
        printSpeedAndTurnOffsets(&Serial);
#endif
    }

#if !defined(DISABLE_AUTO_OFFSET)
    doAutoOffset();
#endif
    return true;
}

/*
 * Automatic offset acquisition 100 ms after boot and offset correction if no acceleration takes place
 */
void IMUCarData::doAutoOffset() {
    // do not auto adjust if we have no initial offsets, which are required for decision if sensor has not moved
    if (AcceleratorForwardOffset != 0) {
        /*
         * Adjust Offsets, if sensor has not moved after NUMBER_OF_OFFSET_CALIBRATION_SAMPLES
         * Requires 420 bytes program memory
         */
        int16_t tAccelDifference = abs(AcceleratorForwardLowPass8.Word.HighWord - AcceleratorForward.Word);
        if (tAccelDifference > ACCEL_MOVE_THRESHOLD || GyroscopePan.Word < -(GYRO_MOVE_THRESHOLD)
                || GYRO_MOVE_THRESHOLD < GyroscopePan.Word) { // using abs() costs 6 byte program memory
#if defined(AUTO_OFFSET_DEBUG)
            // just to show the removed deltas in Arduino Plotter
            if (AcceleratorForward.Word < -(ACCEL_MOVE_THRESHOLD) || ACCEL_MOVE_THRESHOLD < AcceleratorForward.Word) {
                AcceleratorForward.Word = 64 * 10 * ((Speed.Long - sSpeedSnapshot) / sCountOfUndisturbedFifoChunks);
            } else {
                GyroscopePan.Word = 10 * 256 * ((TurnAngle.Long - sTurnSnapshot) / sCountOfUndisturbedFifoChunks);
            }
#endif
            sCountOfUndisturbedFifoChunks = 0; // reset count
            sSpeedSnapshot = Speed.Long;
            sTurnSnapshot = TurnAngle.Long;
        } else {
            /*
             * Do we have enough samples for auto offset?
             */
            if (sCountOfUndisturbedFifoChunks >= NUMBER_OF_OFFSET_CALIBRATION_SAMPLES) {
                if (abs(Speed.Long - sSpeedSnapshot) > sCountOfUndisturbedFifoChunks) {
                    // difference is higher than 1 per sample, so adjust accelerator offset
                    AcceleratorForwardOffset += (Speed.Long - sSpeedSnapshot) / sCountOfUndisturbedFifoChunks;
                    OffsetsHaveChanged = true;
#if defined(AUTO_OFFSET_DEBUG)
                    // just to show in Arduino Plotter - 5 for each accelerator offset increment
                    AcceleratorForward.Word = 64 * 5 * ((Speed.Long - sSpeedSnapshot) / sCountOfUndisturbedFifoChunks);
#endif
                    sSpeedSnapshot = 0;
                    Speed.Long = 0;
                    Distance.Long = 0;

                    sCountOfUndisturbedFifoChunks = 0; // reset count
                }

                if (abs(TurnAngle.Long - sTurnSnapshot) > sCountOfUndisturbedFifoChunks) {
                    // adjust gyroscope offset and reset gyroscope to last value
                    GyroscopePanOffset += (TurnAngle.Long - sTurnSnapshot) / sCountOfUndisturbedFifoChunks;
                    OffsetsHaveChanged = true;
#if defined(AUTO_OFFSET_DEBUG)
                    // just to show in Arduino Plotter - 5 for each gyroscope offset increment
                    GyroscopePan.Word = 5 * 256 * ((TurnAngle.Long - sTurnSnapshot) / sCountOfUndisturbedFifoChunks);
#endif
                    TurnAngle.Long = sTurnSnapshot;

                    sCountOfUndisturbedFifoChunks = 0; // reset count
                }
            }
        }
    }
}

/*
 * Read all data  at 1 kHz rate direct from registers, not from FIFO
 * and compute and store only the required values in one turn.
 */
void IMUCarData::calculateSpeedAndTurnOffsets() {

    resetCarData();
    AcceleratorForwardOffset = 0;
    GyroscopePanOffset = 0;

    uint32_t LastDataMillis = millis();
    for (int i = 0; i < NUMBER_OF_OFFSET_CALIBRATION_SAMPLES; i++) {
        // get data every ms
        while (millis() == LastDataMillis) {
            ;
        }
        LastDataMillis = millis();
        readCarDataFromMPU6050();
    }
    AcceleratorForwardOffset = Speed.Long / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;
    GyroscopePanOffset = TurnAngle.Long / NUMBER_OF_OFFSET_CALIBRATION_SAMPLES;

    resetCarData();
}

void IMUCarData::printSpeedAndTurnOffsets(Print *aSerial) {
    aSerial->print(F("Speed offset="));
    aSerial->print(AcceleratorForwardOffset);
    aSerial->print('|');
    aSerial->print(AcceleratorForwardOffset * ACCEL_RAW_TO_G_FOR_2G_RANGE);
    aSerial->print(F(" turn offset="));
    aSerial->print(GyroscopePanOffset);
    aSerial->print('|');
    aSerial->println(GyroscopePanOffset * GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE);
}

void IMUCarData::reset() {
    initMPU6050FifoForCarData(); // resets also CarData
}

void IMUCarData::resetCarData() {
    AcceleratorForwardLowPass8.ULong = 0;
    AcceleratorForwardLowPass4.ULong = 0;
    Speed.ULong = 0;
    Distance.Long = 0;
    TurnAngle.ULong = 0;
}

void IMUCarData::resetOffsetData() {
    /*
     * Reset both offset data, since they are used to comupte the new sums, which in turn gives the new offsets
     */
    AcceleratorForwardOffset = 0;
    GyroscopePanOffset = 0;
    sCountOfUndisturbedFifoChunks = 0;
    resetMPU6050FifoAndCarData(); // flush FIFO content and reset car data. Speed.Long and TurnAngle.Long serve as temporarily accumulator for offset value
}

void IMUCarData::resetOffsetDataAndWait() {
    resetOffsetData();
    while (AcceleratorForwardOffset == 0) {
        readCarDataFromMPU6050Fifo();
        delay(DELAY_TO_NEXT_IMU_DATA_MILLIS);
    }
}

void IMUCarData::resetMPU6050FifoAndCarData() {
    MPU6050WriteByte(MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_FIFO_RESET_BIT)); // Reset FIFO
    MPU6050WriteByte(MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_FIFO_EN_BIT)); // enable FIFO
    resetCarData(); // values might be invalid
}

void IMUCarData::initMPU6050FifoForCarData() {
    initMPU6050();
    MPU6050WriteByte(MPU6050_RA_FIFO_EN, _BV(MPU6050_ACCEL_FIFO_EN_BIT) | _BV(MPU6050_ZG_FIFO_EN_BIT)); // FIFO: all Accel axes + Gyro Z enabled
    resetOffsetData();
}

/*
 * I2C fast mode is supported by MPU6050
 */
void IMUCarData::initWire() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(5000); // Sets timeout to 5 ms. default is 25 ms.
}

void IMUCarData::initMPU6050() {
    initWire();
    MPU6050WriteByte(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // use recommended gyro reference: PLL with Z axis gyroscope reference
#if SAMPLE_RATE == 1000
    MPU6050WriteByte(MPU6050_RA_SMPLRT_DIV, 0x00); // // Set sample rate to 1 kHz, divider is minimum (1)
    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_188); // ext input disabled, DLPF enabled: accel 184Hz gyro 188Hz @1kHz
#elif SAMPLE_RATE == 500
    MPU6050WriteByte(MPU6050_RA_SMPLRT_DIV, 0x01); // // Set sample rate to 1/2 kHz, divider is 2
    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); // ext input disabled, DLPF enabled: ~50 Hz Sample freq = 1kHz
#elif SAMPLE_RATE == 250
    MPU6050WriteByte(MPU6050_RA_SMPLRT_DIV, 0x03); // // Set sample rate to 1/4 kHz, divider is 4
    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_20); // ext input disabled, DLPF enabled: ~20 Hz Sample freq = 1kHz
#else
    MPU6050WriteByte(MPU6050_RA_SMPLRT_DIV, 0x07); // // Set sample rate to 1/8 kHz, divider is 8
    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_10); // ext input disabled, DLPF enabled: ~10 Hz Sample freq = 1kHz
#endif

//    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_10); // ext input disabled, DLPF enabled: ~10 Hz Sample freq = 1kHz
//    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_20); // ext input disabled, DLPF enabled: ~20 Hz Sample freq = 1kHz
//    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); // ext input disabled, DLPF enabled: ~50 Hz Sample freq = 1kHz
//    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_98); // ext input disabled, DLPF enabled: accel 184Hz gyro 188Hz @1kHz
//    MPU6050WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_188); // ext input disabled, DLPF enabled: accel 184Hz gyro 188Hz @1kHz

// the next is default and never changed by us
//    MPU6050WriteByte(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1)); // range +/- 2g
//    MPU6050WriteByte(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250 << (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1)); // range +/- 250 - default
}

void IMUCarData::MPU6050WriteByte(uint8_t aRegisterNumber, uint8_t aData) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.write(aData);
    Wire.endTransmission();
}

/*
 * Read high byte first
 */
uint16_t IMUCarData::MPU6050ReadWordSwapped(uint8_t aRegisterNumber) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 2, (uint8_t) true);
    WordUnion tWord;
    tWord.UByte.HighByte = Wire.read();
    tWord.UByte.LowByte = Wire.read();
    return tWord.UWord;
}
#endif // _IMU_CAR_DATA_HPP
#pragma once
