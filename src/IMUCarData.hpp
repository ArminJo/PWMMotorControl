/*
 * IMUCarData.hpp
 *
 *  Functions for getting IMU data from MPU6050 for car control.
 *  If PRINT_MOVEMENT_DETECTION_MAX_VALUES is enabled, the maximum delta values of the 4 values used for motion detection are printed.
 *
 *  The first NUMBER_OF_OFFSET_CALIBRATION_SAMPLES values are taken for initial offset
 *  All offset dependent values are cleared and offset recalculation values are reset.
 *  Automatic offset recalculation is done, if we do not detect any movement for NUMBER_OF_OFFSET_CALIBRATION_SAMPLES samples.
 *  It can be activated (default) or suspended.
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

//#define USE_ACCELERATOR_Y_FOR_SPEED           // The y axis of the GY-521 MPU6050 breakout board points forward / backward, i.e. connectors are at the right.
//#define USE_NEGATIVE_ACCELERATION_FOR_SPEED   // The x axis of the GY-521 MPU6050 breakout board points backwards.
//#define PRINT_MOVEMENT_DETECTION_MAX_VALUES   // The maximum delta values of the 4 values used for motion detection are printed, when offset recalculation is done.
//#define DEBUG_CHECK_PLOTTER_FLAG              // Check external flag sOnlyPlotterOutput for debug print
#include "IMUCarData.h"

//#define USE_SOFT_I2C_MASTER // Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
#if defined(USE_SOFT_I2C_MASTER)
#include "SoftI2CMasterConfig.h"
#include "SoftI2CMaster.h"
#else
#include "Wire.h"
#endif // defined(USE_SOFT_I2C_MASTER)

//#define WARN    // Information that the program may encounter problems, like small Heap/Stack area.
#include "DebugLevel.h" // Include to propagate debug levels

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

#define FIFO_CHUNK_SIZE_FOR_CAR_DATA       ((NUMBER_OF_ACCEL_VALUES + 1) * 2) // 3 accel and only z gyro requested

//#define AUTO_OFFSET_DEBUG // Show auto offset values as dummy values for the plotter output

/********************************************************************************
 * MPU6050 related functions
 ********************************************************************************/
/*
 * Sets AcceleratorForward, Speed, Distance, GyroscopePan and TurnAngle
 * Sets AcceleratorForwardLowPass8, AcceleratorForwardLowPass6 and AcceleratorForwardLowPass4
 * Read raw 14 vales. Requires 500 us.
 */
void IMUCarData::readCarDataFromMPU6050() {
#if defined(USE_SOFT_I2C_MASTER)
    // Here we have no buffer and can read all chunks in one row
    i2c_start(MPU6050_DEFAULT_ADDRESS << 1);
#  if defined(USE_ACCELERATOR_Y_FOR_SPEED)
    i2c_write(MPU6050_RA_ACCEL_YOUT_H); // skip x value
#  else
    i2c_write(MPU6050_RA_ACCEL_XOUT_H);
#  endif
    i2c_rep_start((MPU6050_DEFAULT_ADDRESS << 1) | I2C_READ); // restart for reading
#else

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
#  if defined(USE_ACCELERATOR_Y_FOR_SPEED)
    Wire.write(MPU6050_RA_ACCEL_YOUT_H); // skip x value
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 12, (uint8_t) true);
#  else
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 14, (uint8_t) true);
#  endif
#endif

// read forward value
#if defined(USE_SOFT_I2C_MASTER)
    AcceleratorForward.Byte.HighByte = i2c_read(false);
    AcceleratorForward.Byte.LowByte = i2c_read(false);
#else
    AcceleratorForward.Byte.HighByte = Wire.read();
    AcceleratorForward.Byte.LowByte = Wire.read();
#endif
#if defined(USE_NEGATIVE_ACCELERATION_FOR_SPEED)
    AcceleratorForward.Word = (-AcceleratorForward.Word) - AcceleratorForwardOffset;
#else
    AcceleratorForward.Word = AcceleratorForward.Word - AcceleratorForwardOffset;
#endif

    AcceleratorForwardLowPass8.Long += ((((int32_t) AcceleratorForward.Word) << 16) - AcceleratorForwardLowPass8.Long) >> 8; // Fixed point 2.0 us
    AcceleratorForwardLowPass6.Long += ((((int32_t) AcceleratorForward.Word) << 16) - AcceleratorForwardLowPass6.Long) >> 6; // Fixed point 2.0 us
    Speed.Long += AcceleratorForward.Word;
    Distance.Long += Speed.Long >> 8;

#if defined(USE_ACCELERATOR_Y_FOR_SPEED)
// skip z, temp and 2 gyroscope values value
    for (uint_fast8_t i = 0; i < 8; i++)
#else
    // skip y, z, temp and 2 gyroscope values value
    for (uint_fast8_t i = 0; i < 10; i++)
#endif
            {
#if defined(USE_SOFT_I2C_MASTER)
        i2c_read(false);
#else
        Wire.read();
#endif
    }

// read pan (Z) value
#if defined(USE_SOFT_I2C_MASTER)
    GyroscopePan.Byte.HighByte = i2c_read(false);
    GyroscopePan.Byte.LowByte = i2c_read(true);
#else
    GyroscopePan.Byte.HighByte = Wire.read();
    GyroscopePan.Byte.LowByte = Wire.read();
#endif
    GyroscopePan.Word -= GyroscopePanOffset;
    TurnAngle.Long += GyroscopePan.Word;
#if defined(USE_SOFT_I2C_MASTER)
    i2c_stop();
#endif
}

/*
 * like delay() but read from FIFO during delay
 */
void IMUCarData::delayAndReadIMUCarDataFromMPU6050FIFO(unsigned long aMillisDelay) {
    while (aMillisDelay > 0) {
        readCarDataFromMPU6050Fifo();
        delay(DELAY_TO_NEXT_IMU_DATA_MILLIS);
        aMillisDelay -= DELAY_TO_NEXT_IMU_DATA_MILLIS;
    }
}

/*
 * Sets LastFifoCheckMillis
 * Sets AcceleratorForward, Speed, Distance, GyroscopePan and TurnAngle
 * AcceleratorForward and GyroscopePan are the average of all values from current FIFO content
 * Sets AcceleratorForwardLowPass8, AcceleratorForwardLowPass6 and AcceleratorForwardLowPass4
 * The first NUMBER_OF_OFFSET_CALIBRATION_SAMPLES values are taken to compute AcceleratorForwardOffset and GyroscopePanOffset, afterwards OffsetsHaveJustChanged is set to true
 * @return true, if data might have changed
 */
bool IMUCarData::readCarDataFromMPU6050Fifo() {
    if (millis() - LastFifoCheckMillis < DELAY_TO_NEXT_IMU_DATA_MILLIS) {
        return false; // no new data expected
    }

    /*
     * Read FIFO count only once at start of the function
     */
    uint16_t tFifoCount = MPU6050ReadWordSwapped(MPU6050_RA_FIFO_COUNTH);  // multiple of 8 -> 160 for 20 ms
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
    if (!sOnlyPlotterOutput) {
#  endif
        Serial.print(F("FiFoCnt="));
        Serial.print(tFifoCount);
        Serial.print(" ");
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
    }
#  endif
#endif

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
    int32_t tAcceleratorForward = 0;
    int32_t tGyroscopePan = 0;
    uint8_t tNumberOfChunks = tFifoCount / FIFO_CHUNK_SIZE_FOR_CAR_DATA;
    if (tNumberOfChunks > 0) {
#if defined(USE_SOFT_I2C_MASTER)
        // Here we have no buffer and can read all chunks in one row
        i2c_start(MPU6050_DEFAULT_ADDRESS << 1);
        i2c_write(MPU6050_RA_FIFO_R_W);
        i2c_rep_start((MPU6050_DEFAULT_ADDRESS << 1) | I2C_READ); // restart for reading
#endif
        for (uint_fast8_t tChunckCount = 0; tChunckCount < tNumberOfChunks; tChunckCount++) {

#if !defined(USE_SOFT_I2C_MASTER)
            Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
            Wire.write(MPU6050_RA_FIFO_R_W);
            Wire.endTransmission(false);
            // read chunk by chunk
            Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) (FIFO_CHUNK_SIZE_FOR_CAR_DATA), (uint8_t) true);
#endif
            /*
             * We must read all 3 accelerator values
             */
            for (uint_fast8_t i = 0; i < NUMBER_OF_ACCEL_VALUES; i++) {
                WordUnion tAcceleratorValue;
#if defined(USE_SOFT_I2C_MASTER)
                tAcceleratorValue.Byte.HighByte = i2c_read(false);
                tAcceleratorValue.Byte.LowByte = i2c_read(false);
#else
                tAcceleratorValue.Byte.HighByte = Wire.read();
                tAcceleratorValue.Byte.LowByte = Wire.read();
#endif
                // process only forward value
#if defined(USE_ACCELERATOR_Y_FOR_SPEED)
                if (i == 1)
#else
                if (i == 0)
#endif
                        {
#if defined(USE_NEGATIVE_ACCELERATION_FOR_SPEED)
                    // this works, because computed offset is also negative :-)
                    tAcceleratorValue.Word = (-tAcceleratorValue.Word) - AcceleratorForwardOffset;
#else
                    tAcceleratorValue.Word = tAcceleratorValue.Word - AcceleratorForwardOffset;
#endif
                    tAcceleratorForward += tAcceleratorValue.Word;
                    AcceleratorForwardLowPass8.Long +=
                            ((((int32_t) tAcceleratorValue.Word) << 16) - AcceleratorForwardLowPass8.Long) >> 8; // Fixed point 2.0 us
                    AcceleratorForwardLowPass6.Long +=
                            ((((int32_t) tAcceleratorValue.Word) << 16) - AcceleratorForwardLowPass6.Long) >> 6; // Fixed point 2.0 us

                    Speed.Long += tAcceleratorValue.Word;
                    Distance.Long += Speed.Long >> 8;
                }
            }
            /*
             * Now the Gyro value
             */
            WordUnion tValue;
#if defined(USE_SOFT_I2C_MASTER)
            tValue.Byte.HighByte = i2c_read(false);
            tValue.Byte.LowByte = i2c_read(tChunckCount == (tNumberOfChunks - 1));
#else
            tValue.Byte.HighByte = Wire.read();
            tValue.Byte.LowByte = Wire.read();
#endif
            tValue.Word = tValue.Word - GyroscopePanOffset;
            tGyroscopePan += tValue.Word;
            GyroscopePanLowPass8.Long += ((((int32_t) tValue.Word) << 16) - GyroscopePanLowPass8.Long) >> 8; // Fixed point 2.0 us
            // Compute turn angle
            TurnAngle.Long += tValue.Word;
        } // for (tChunckCount = 0; tChunckCount < tNumberOfChunks; tChunckCount++)
#if defined(USE_SOFT_I2C_MASTER)
        i2c_stop();
#endif
        CountOfFifoChunksForOffset += tNumberOfChunks;
        // compute average of read values
        AcceleratorForward.Word = tAcceleratorForward / (int32_t) tNumberOfChunks;
        GyroscopePan.Word = tGyroscopePan / (int32_t) tNumberOfChunks;

        /*
         * Automatically get initial offset values once at the beginning
         */
        if (AcceleratorForwardOffset == 0 && CountOfFifoChunksForOffset >= NUMBER_OF_OFFSET_CALIBRATION_SAMPLES) {
            /*
             * The first NUMBER_OF_OFFSET_CALIBRATION_SAMPLES values are taken for initial offset
             * All offset dependent values are cleared and offset recalculation values are reset
             */
            AcceleratorForwardOffset = Speed.Long / CountOfFifoChunksForOffset; // speed is the accelerator sum :-)
            GyroscopePanOffset = TurnAngle.Long / CountOfFifoChunksForOffset;
            resetAllIMUCarOffsetAdjustedValues(); // reset all offset adjusted values to zero
            resetForOffsetRecalculation(); // must be after resetAllIMUCarOffsetAdjustedValues()!
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
            if (!sOnlyPlotterOutput) {
#  endif
                Serial.println();
                printSpeedAndTurnOffsets(&Serial);
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
            }
#  endif
#endif
            OffsetsJustHaveChanged = true; // must be after printSpeedAndTurnOffsets() since this sets it to false
        }

        if (!DisableAutoOffsetRecalculation) {
            doOffsetRecalculation();
        }
    } // if (tNumberOfChunks > 0)
    return true;
}

/*
 * Automatic offset recalculation is done, if we do not detect any movement for NUMBER_OF_OFFSET_CALIBRATION_SAMPLES samples.
 * Initial offset is required to detect movement.
 * Only offset compensated values are stored by readCarDataFromMPU6050Fifo.
 * Therefore we compute a delta value and apply it, if its delta is greater than 1.
 */
void IMUCarData::doOffsetRecalculation() {
    // Do not auto adjust if we have no initial offsets, which are required for decision if sensor has not moved
    if (AcceleratorForwardOffset != 0) {
        /*
         * Adjust Offsets, if sensor has not moved after NUMBER_OF_OFFSET_CALIBRATION_SAMPLES
         * Check for movement
         */
        uint16_t tAcceleratorHighPassValue = abs(AcceleratorForwardLowPass8.Word.HighWord - AcceleratorForward.Word);
        uint16_t tGyroscopeHighPassValue = abs(GyroscopePanLowPass8.Word.HighWord - GyroscopePan.Word);
#if defined(PRINT_MOVEMENT_DETECTION_MAX_VALUES)
        AcceleratorHighPassValueMaximum = max(AcceleratorHighPassValueMaximum, tAcceleratorHighPassValue);
        GyroscopeHighPassValueMaximum = max(GyroscopeHighPassValueMaximum, tGyroscopeHighPassValue);
        AcceleratorForwardeMaximum = max(AcceleratorForwardeMaximum, abs(AcceleratorForward.Word));
        GyroscopePanMaximum = max(GyroscopePanMaximum, abs(GyroscopePan.Word));
#endif
        if (abs(AcceleratorForward.Word) > (ACCEL_MOVE_THRESHOLD * 2)|| tAcceleratorHighPassValue > ACCEL_MOVE_THRESHOLD
        || abs(GyroscopePan.Word) > (GYRO_MOVE_THRESHOLD * 2) || tGyroscopeHighPassValue > GYRO_MOVE_THRESHOLD) {
            /*
             * Movement detected, reset recalculation
             */
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
            if (!sOnlyPlotterOutput) {
#  endif
                // do not print it every time
                if(CountOfFifoChunksForOffset != 0) {
                    Serial.print(F("Movement detected ACDelta="));
                    Serial.print(tAcceleratorHighPassValue);
                    Serial.print(F(" GyroDelta="));
                    Serial.println(tGyroscopeHighPassValue);
                }
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
        }
#  endif
#endif
            resetForOffsetRecalculation();
        } else {
            /*
             * No movement here. Check if we have enough samples for offset recalculation.
             */
            if (CountOfFifoChunksForOffset >= NUMBER_OF_OFFSET_CALIBRATION_SAMPLES) {
#if defined(PRINT_MOVEMENT_DETECTION_MAX_VALUES)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                if (!sOnlyPlotterOutput) {
#  endif
                    Serial.print(F("Maximum values: AcHP="));
                    Serial.print(AcceleratorHighPassValueMaximum);
                    Serial.print(F(" Ac="));
                    Serial.print(AcceleratorForwardeMaximum);
                    Serial.print(F(" GyHP="));
                    Serial.print(GyroscopeHighPassValueMaximum);
                    Serial.print(F(" Gy="));
                    Serial.println(GyroscopePanMaximum);
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                }
#  endif
#endif // defined(PRINT_MOVEMENT_DETECTION_MAX_VALUES)
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                if (!sOnlyPlotterOutput) {
#  endif
                    Serial.print(F("Offset recalculated:"));
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                }
#  endif
#endif
                if (abs(Speed.Long - SpeedSnapshotForOffsetRecalculation) > CountOfFifoChunksForOffset) {
                    // Accelerator difference is higher than 1 per sample, so adjust accelerator offset
                    AcceleratorForwardOffset += (Speed.Long - SpeedSnapshotForOffsetRecalculation) / CountOfFifoChunksForOffset;
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                    if (!sOnlyPlotterOutput) {
#  endif
                        Serial.print(F(" AccelOffset delta="));
                        Serial.print((Speed.Long - SpeedSnapshotForOffsetRecalculation) / CountOfFifoChunksForOffset);
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                }
#  endif
#endif
                    OffsetsJustHaveChanged = true;
#if defined(LOCAL_DEBUG)
                    // just to show in Arduino Plotter - 5 for each accelerator offset increment
                    AcceleratorForward.Word = 64 * 5 * ((Speed.Long - SpeedSnapshotForOffsetRecalculation) / CountOfFifoChunksForOffset);
#else
                    // because we do not move
                    AcceleratorForward.Word = 0;
#endif
                    AcceleratorForwardLowPass8.Word.HighWord = 0;
                    Distance.Long -= (NUMBER_OF_OFFSET_CALIBRATION_SAMPLES * Speed.Long) >> 8; // subtract the speed added since last end of movement
                    Speed.Long = 0; // assume, that we do not have constant speed
                }

                if (abs(TurnAngle.Long - TurnSnapshotForOffsetRecalculation) > CountOfFifoChunksForOffset) {
                    // Gyroscope difference is higher than 1 per sample, so adjust gyroscope offset and reset gyroscope to last value
                    GyroscopePanOffset += (TurnAngle.Long - TurnSnapshotForOffsetRecalculation) / CountOfFifoChunksForOffset;
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                    if (!sOnlyPlotterOutput) {
#  endif
                        Serial.print(F(" GyroOffset delta="));
                        Serial.print((TurnAngle.Long - TurnSnapshotForOffsetRecalculation) / CountOfFifoChunksForOffset);
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                }
#  endif
#endif
                    OffsetsJustHaveChanged = true;
#if defined(LOCAL_DEBUG)
                    // just to show in Arduino Plotter - 5 for each gyroscope offset increment
                    GyroscopePan.Word = 5 * 256 * ((TurnAngle.Long - TurnSnapshotForOffsetRecalculation) / CountOfFifoChunksForOffset);
#else
                    // because we do not move
                    GyroscopePan.Word = 0;
#endif
                    GyroscopePanLowPass8.Word.HighWord = 0;
                    TurnAngle.Long = TurnSnapshotForOffsetRecalculation;
                }
                resetForOffsetRecalculation();
#if defined(LOCAL_DEBUG)
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                if (!sOnlyPlotterOutput) {
#  endif
                    Serial.print(F(" -> "));
                    printSpeedAndTurnOffsets(&Serial);
#  if defined(DEBUG_CHECK_PLOTTER_FLAG)
                }
#  endif
#endif
            }
        }
    }
}

/*
 * Convenience functions
 */
void IMUCarData::enableOffsetRecalculation() {
    DisableAutoOffsetRecalculation = false;
}
void IMUCarData::disableOffsetRecalculation() {
    DisableAutoOffsetRecalculation = true;
}
void IMUCarData::setOffsetRecalculation(bool aDoOffsetRecalculation) {
    DisableAutoOffsetRecalculation = !aDoOffsetRecalculation;
}
/*
 * Read all data at 1 kHz rate direct from registers, not from FIFO
 * and compute and store only the required values in one turn.
 */
void IMUCarData::calculateSpeedAndTurnOffsetsWithoutFIFO() {

    resetAllIMUCarOffsetAdjustedValues();
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

    resetAllIMUCarOffsetAdjustedValues();
}

/********************************************************************************
 * Init and reset functions
 ********************************************************************************/
/*
 * Does a blocking wait for around 1/2 second
 */
bool IMUCarData::initMPU6050CarDataAndCalculateAllOffsetsAndWait() {
    /*
     * Initially set the sample rate to 1 ms and the filter to 184 Hz
     */
    if (!initMPU6050FifoForCarData()) {
        return false;
    }

    //Read until offset was filled
    while (!OffsetsJustHaveChanged) {
        delay(1);
        readCarDataFromMPU6050Fifo();
    }
    return true;
}

/*
 * Refresh the snapshot value and reset the chunk counter to start for a new dataset for offset recalculation
 */
void IMUCarData::resetForOffsetRecalculation() {
    SpeedSnapshotForOffsetRecalculation = Speed.Long;
    TurnSnapshotForOffsetRecalculation = TurnAngle.Long;
    CountOfFifoChunksForOffset = 0;

#if defined(PRINT_MOVEMENT_DETECTION_MAX_VALUES)
    AcceleratorHighPassValueMaximum = 0;
    GyroscopeHighPassValueMaximum = 0;
    AcceleratorForwardeMaximum = 0;
    GyroscopePanMaximum = 0;
#endif
}

/*
 *  Reset all offset adjusted values to zero
 */
void IMUCarData::resetAllIMUCarOffsetAdjustedValues() {
    AcceleratorForward.Word = 0;
    AcceleratorForwardLowPass8.ULong = 0;
    AcceleratorForwardLowPass6.ULong = 0;
    GyroscopePanLowPass8.ULong = 0;
    Speed.ULong = 0;
    Distance.Long = 0;
    GyroscopePan.Word = 0;
    TurnAngle.ULong = 0;
}

void IMUCarData::resetMPU6050FifoAndCarData() {
    resetMPU6050Fifo();
    resetAllIMUCarOffsetAdjustedValues();
}

/*
 * Reset all values depending from offset
 * Reset both offset data, since they are used to compute the new sums, which in turn gives the new offsets
 */
void IMUCarData::resetOffsetFifoAndCarData() {
    AcceleratorForwardOffset = 0;
    GyroscopePanOffset = 0;
    CountOfFifoChunksForOffset = 0;
    resetMPU6050FifoAndCarData(); // flush FIFO content and reset car data. Speed.Long and TurnAngle.Long serve as temporarily accumulator for offset value
}

void IMUCarData::resetOffsetFifoAndCarDataAndWait() {
    resetOffsetFifoAndCarData();
    while (AcceleratorForwardOffset == 0) {
        readCarDataFromMPU6050Fifo();
        delay(DELAY_TO_NEXT_IMU_DATA_MILLIS);
    }
}

bool IMUCarData::initMPU6050FifoForCarData() {
    bool tReturnValue;
#if SAMPLE_RATE == 1000
    tReturnValue = initMPU6050(1, MPU6050_BAND_184_HZ); // Set sample rate to 1 kHz, divider is minimum (1). ext input disabled, DLPF enabled: accel 184Hz gyro 188Hz @1kHz
#elif SAMPLE_RATE == 500
    tReturnValue = initMPU6050(2, MPU6050_BAND_44_HZ); // Set sample rate to 1/2 kHz, divider is 2, ext input disabled, DLPF enabled: ~50 Hz Sample freq = 1kHz
#elif SAMPLE_RATE == 250
    tReturnValue = initMPU6050(4, MPU6050_BAND_21_HZ); // Set sample rate to 1/4 kHz, ext input disabled, DLPF enabled: ~20 Hz Sample freq = 1kHz
#else
    tReturnValue = initMPU6050(8, MPU6050_BAND_10_HZ); // Set sample rate to 1/8 kHz, ext input disabled, DLPF enabled: ~10 Hz Sample freq = 1kHz
#endif
    if (!tReturnValue) {
        return false;
    }
    /*
     * Enable only z axis gyro and all accel data (can only be enabled as set)
     */
    MPU6050WriteByte(MPU6050_RA_FIFO_EN, _BV(MPU6050_ACCEL_FIFO_EN_BIT) | _BV(MPU6050_ZG_FIFO_EN_BIT)); // FIFO: all Accel axes + Gyro Z enabled
    resetOffsetFifoAndCarData();
    return true;
}

void IMUCarData::resetMPU6050Fifo() {
    MPU6050WriteByte(MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_FIFO_RESET_BIT)); // Reset FIFO
    MPU6050WriteByte(MPU6050_RA_USER_CTRL, _BV(MPU6050_USERCTRL_FIFO_EN_BIT)); // enable FIFO
}

/*
 * Resets all variables, initializes the MPU, but not the FIFO and sets LowPassShiftValue
 * @param aSampleRateDivider 1 to 256. Divider of the 1 kHz clock used for FiFo
 * @param aLowPassIndex one of: MPU6050_BAND_260_HZ (LP disabled) MPU6050_BAND_184_HZ (184 Hz), 94, 44, 21, 10 to MPU6050_BAND_5_HZ
 * @return false if i2c_start() was not successful / MPU6050 not attached
 */
bool IMUCarData::initMPU6050(uint8_t aSampleRateDivider, mpu6050_bandwidth_t aLowPassType) {
#if defined(USE_SOFT_I2C_MASTER)
    /*
     * Check if MPU6050 is attached
     */
    if (!i2c_start(MPU6050_DEFAULT_ADDRESS << 1)) {
        i2c_stop();
        return false;
    }
    i2c_stop();
#endif
    MPU6050WriteByte(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // use recommended gyro reference: PLL with Z axis gyroscope reference
    MPU6050WriteByte(MPU6050_RA_SMPLRT_DIV, aSampleRateDivider - 1); // parameter 0 => divider 1, 19 -> divider 20
    MPU6050WriteByte(MPU6050_RA_CONFIG, aLowPassType); // ext input disabled, DLPF enabled

// range select
    // the next is default and never changed by us
//    MPU6050WriteByte(MPU6050_RA_ACCEL_CONFIG,
//    MPU6050_ACCEL_FS_2 << (MPU6050_ACONFIG_AFS_SEL_BIT - MPU6050_ACONFIG_AFS_SEL_LENGTH + 1)); // range +/- 2 g - default

    // the next is default and never changed by us
//    MPU6050WriteByte(MPU6050_RA_GYRO_CONFIG,
//    MPU6050_GYRO_FS_250 << (MPU6050_GCONFIG_FS_SEL_BIT - MPU6050_GCONFIG_FS_SEL_LENGTH + 1)); // range +/- 250 deg/s - default

    return true;
}

/********************************************************************************
 * Functions to get car data values
 ********************************************************************************/
/*
 * Accelerator values
 */
int8_t IMUCarData::getAcceleratorForward15MilliG() {
    return AcceleratorForward.Byte.HighByte;
}

int8_t IMUCarData::getAcceleratorForward4MilliG() {
    /*
     * For GCC we can use shift right for signed values :-)
     * we get a 0 for values 0 to 63, but we get a -1 for values -1 to -63 :-(
     */
    return AcceleratorForward.Word >> 6; // /64 -> 256 per g
}

int8_t IMUCarData::getAcceleratorForwardLowPass8() {
//    return AcceleratorForwardLowPass8.Byte.HighByte;
    return AcceleratorForwardLowPass8.Word.HighWord >> 6; // 256 per g
}

int8_t IMUCarData::getAcceleratorForwardLowPass6() {
//    return AcceleratorForwardLowPass8.Byte.HighByte;
    return AcceleratorForwardLowPass6.Word.HighWord >> 6; // 256 per g
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
 * Gyroscope values
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

/********************************************************************************
 * Print functions
 ********************************************************************************/
bool IMUCarData::printIMUCarFIFODataPeriodically(Print *aSerial, uint16_t aPeriodMillis) {
    static unsigned long sLastPrintMillis;

    if ((millis() - sLastPrintMillis) >= aPeriodMillis) { // read and print data every n ms
        sLastPrintMillis = millis();
        readCarDataFromMPU6050Fifo();
        printIMUCarData(aSerial);
        return true;
    }
    return false;
}

/*
 * Print caption for Serial Plotter
 * Space but NO println() at the end, to enable additional information to be printed
 */
void IMUCarData::printIMUCarDataCaption(Print *aSerial) {
#if defined(LOCAL_DEBUG)
    aSerial->print(F("AccelForward[4mg/LSB] AccelForwardLP8 Speed[cm/s] Distance[cm] GyroZ[2dps/LSB] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
#else
//    aSerial->print(F("AccelForward[4mg/LSB] AccelForwardAverage AccelForwardLP8 Speed[cm/s] Distance[cm] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
//    aSerial->print(F("AccelAvg.[4mg/LSB] AccelLP8 Speed[cm/s] Distance[cm] AngleZ[0.5d/LSB] ")); // +/-250 | 500 degree per second for 16 bit full range
    aSerial->print(F("Accel[4cm/s^2] AccelAvg6 Speed[cm/s] Distance[cm] AngleZ[0.5d] ")); // +/-250 | 500 degree per second for 16 bit full range
#endif
}

/*
 * Do not end with newline, to allow other data to be appended
 */
void IMUCarData::printIMUCarData(Print *aSerial) {
    aSerial->print(getAcceleratorForward4MilliG());
    aSerial->print(" ");
    aSerial->print(getAcceleratorForwardLowPass6());
    aSerial->print(" ");

#if defined(LOCAL_DEBUG)
//    aSerial->print(getAcceleratorForwardLowPass4());
//    aSerial->print(" ");
    aSerial->print(getAcceleratorForwardLowPass8());
    aSerial->print(" ");
#endif
    aSerial->print(getSpeedCmPerSecond());
    aSerial->print(" ");
    aSerial->print(getDistanceCm());
    aSerial->print(" ");
#if defined(LOCAL_DEBUG)
    aSerial->print(getGyroscopePan2DegreePerSecond());
    aSerial->print(" ");
#endif
    aSerial->print(getTurnAngleHalfDegree());
}

unsigned int IMUCarData::getMPU6050SampleRate() {
    return (SAMPLE_RATE);
}

/*
 * resets also the OffsetsJustHaveChanged flag
 */
void IMUCarData::printSpeedAndTurnOffsets(Print *aSerial) {
    aSerial->print(F("Speed offset="));
    aSerial->print(AcceleratorForwardOffset);
    aSerial->print('|');
    aSerial->print(AcceleratorForwardOffset * ACCEL_RAW_TO_G_FOR_2G_RANGE);
    aSerial->print(F("g turn offset="));
    aSerial->print(GyroscopePanOffset);
    aSerial->print('|');
    aSerial->print(GyroscopePanOffset * GYRO_RAW_TO_DEGREE_PER_SECOND_FOR_250DPS_RANGE);
    aSerial->println(F("dps"));
    OffsetsJustHaveChanged = false;
}

/********************************************************************************
 * MPU6050 I2C functions
 ********************************************************************************/

/*
 * I2C fast mode is supported by MPU6050
 * @return false if initialization is not OK
 */
bool initWire() {
#if defined(USE_SOFT_I2C_MASTER)
    return i2c_init(); // Initialize everything and check for bus lockup
#else
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(5000);
    return true;
#endif
}

void IMUCarData::MPU6050WriteByte(uint8_t aRegisterNumber, uint8_t aData) {

#if defined(USE_SOFT_I2C_MASTER)
    i2c_write_byte_to_register((MPU6050_DEFAULT_ADDRESS << 1), aRegisterNumber, aData);
#else
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.write(aData);
    Wire.endTransmission();
#endif
}

/*
 * Read high byte first
 */
uint16_t IMUCarData::MPU6050ReadWordSwapped(uint8_t aRegisterNumber) {

#if defined(USE_SOFT_I2C_MASTER)
    return i2c_read_word_swapped_from_register((MPU6050_DEFAULT_ADDRESS << 1), aRegisterNumber);

#else
    WordUnion tWord; // saves 16 bytes program space and improves speed

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 2, (uint8_t) true);
    tWord.UByte.HighByte = Wire.read();
    tWord.UByte.LowByte = Wire.read();
    return tWord.UWord;
#endif
}

uint16_t IMUCarData::MPU6050ReadWord(uint8_t aRegisterNumber) {

#if defined(USE_SOFT_I2C_MASTER)
    return i2c_read_word_from_register((MPU6050_DEFAULT_ADDRESS << 1), aRegisterNumber);

#else
    WordUnion tWord; // here it saves no program space

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(aRegisterNumber);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t) MPU6050_DEFAULT_ADDRESS, (uint8_t) 2, (uint8_t) true);
    tWord.UByte.LowByte = Wire.read();
    tWord.UByte.HighByte = Wire.read();
    return tWord.UWord;
#endif
}
#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _IMU_CAR_DATA_HPP
