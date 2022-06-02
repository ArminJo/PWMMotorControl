/*
 *  PrintCarValuesWithIMU.cpp
 *
 *  Start and stop car first with and second without ramp.
 *  Start with DEFAULT_DRIVE_SPEED_PWM and double speed for next turn until MAX_SPEED_PWM.
 *  Prints PWM, distance and speed diagram of the right (encoder) motor of a car.
 *  Encoder and IMU data are printed simultaneously, to compare and to detect slipping.
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/PWMMotorControl.
 *
 *  PWMMotorControl is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

/*
 * You will need to change these values according to your motor, H-bridge and motor supply voltage.
 * You must specify this before the include of "RobotCar.hpp"
 */
//#define USE_ENCODER_MOTOR_CONTROL   // Use encoder interrupts attached at pin 2 and 3 and want to use the methods of the EncoderMotor class.
//#define USE_ADAFRUIT_MOTOR_SHIELD   // Use Adafruit Motor Shield v2 connected by I2C instead of TB6612 or L298 breakout board.
#define USE_MPU6050_IMU             // Use GY-521 MPU6050 breakout board connected by I2C for support of precise turning. Connectors point to the rear.
//#define VIN_2_LI_ION                  // Activate this, if you use 2 Li-ion cells (around 7.4 volt) as motor supply.
//#define VIN_1_LI_ION                  // If you use a mosfet bridge (TB6612), 1 Li-ion cell (around 3.7 volt) may be sufficient.
//#define FULL_BRIDGE_INPUT_MILLIVOLT   6000  // Default. For 4 x AA batteries (6 volt).
//#define USE_L298_BRIDGE               // Activate this, if you use a L298 bridge, which has higher losses than a recommended mosfet bridge like TB6612.
//#define DEFAULT_DRIVE_MILLIVOLT       2000 // Drive voltage -motors default speed- is 2.0 volt
//#define DO_NOT_SUPPORT_AVERAGE_SPEED  // Disables the function getAverageSpeed(). Saves 44 bytes RAM per motor and 156 bytes program memory.

#define USE_SOFT_I2C_MASTER           // Saves 1044 bytes program memory and 99 bytes RAM compared with Arduino Wire
#include "RobotCarPinDefinitionsAndMore.h"

#include "CarPWMMotorControl.hpp"

//#define ENABLE_EXTRA_NON_PLOTTER_OUTPUT // Generate verbose output for SerialMonitor but this not compatible with Arduino Plotter
#define PRINTS_PER_SECOND 50

#define CAR_HAS_VIN_VOLTAGE_DIVIDER     // VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage
#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
#define VOLTAGE_DIVIDER_DIVISOR   11.0  // VIN/11 by 1MOhm to VIN and 100kOhm to ground.
#include "ADCUtils.hpp"
#define VIN_11TH_IN_CHANNEL         2 // = A2
uint16_t readVINVoltageMilliVolt();
float sVINVoltage;
#endif

unsigned long LastPrintMillis;

void printCaption();
void delayAndPrintData(uint8_t aDataSetsToPrint, uint16_t aPeriodMillis, bool aWaitForStop);

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_PWMMOTORCONTROL));

    // Print info
    PWMDcMotor::printCompileOptions(&Serial);
#endif

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    // For Adafruit Motor Shield v2
    RobotCar.init();
#else
    RobotCar.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#endif

    tone(PIN_BUZZER, 2200, 200);

    // wait 5 seconds before start moving
    RobotCar.IMUData.delayAndReadIMUCarDataFromMPU6050FIFO(5000);
}

void loop() {
    static uint8_t sDirection = DIRECTION_FORWARD;

    /*
     * Set to drive speed without ramp
     */
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    Serial.print(F("Set speed to DEFAULT_DRIVE_SPEED_PWM="));
    Serial.println(DEFAULT_DRIVE_SPEED_PWM);
#endif
//    uint8_t sSpeedPWM = DEFAULT_DRIVE_SPEED_PWM;
    uint8_t sSpeedPWM = DEFAULT_DRIVE_SPEED_PWM *2;

    uint8_t tLoopIndex = 0;

    /*
     * Start and stop car first with and second without ramp
     * Start with DEFAULT_DRIVE_SPEED_PWM and double speed for next turn until MAX_SPEED_PWM
     */
    while (true) {
        bool tUseRamp = true;
        for (uint8_t i = 0; i < 2; ++i) {
            printCaption();
            tone(PIN_BUZZER, 2200, 100);
            delay(200);
            RobotCar.IMUData.resetOffsetFifoAndCarDataAndWait();
#if defined(USE_ENCODER_MOTOR_CONTROL)
            RobotCar.rightCarMotor.resetEncoderControlValues();
#endif

            if (tUseRamp) {
                /*
                 * Go distance - implies ramp
                 */
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
                Serial.print(F("Go distance[mm]="));
                Serial.println((tLoopIndex + 1) * 200); // 200, 400, 600
#endif
                RobotCar.startGoDistanceMillimeter(sSpeedPWM, (tLoopIndex + 1) * 200, sDirection);
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
                Serial.print(F("Go distance[mm]="));
                Serial.println(RobotCar.CarRequestedDistanceMillimeter);
#endif
                // print 20 data sets after stopping
                delayAndPrintData(20, 1000 / PRINTS_PER_SECOND, tUseRamp);
            } else {
                /*
                 * Set speed, wait and stop - no ramp :-)
                 */
                RobotCar.setSpeedPWMAndDirection(sSpeedPWM, sDirection);
                delayAndPrintData(40, 1000 / PRINTS_PER_SECOND, tUseRamp);
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
                Serial.println(F("Stop motors"));
#endif
                RobotCar.setStopMode(STOP_MODE_BRAKE); // just to be sure to brake and not to release
                RobotCar.setSpeedPWMAndDirection(0);
                delayAndPrintData(20, 1000 / PRINTS_PER_SECOND, tUseRamp);
            }

            tUseRamp = false;
            RobotCar.IMUData.delayAndReadIMUCarDataFromMPU6050FIFO(2000);

        }
        if (sSpeedPWM == MAX_SPEED_PWM) {
            break; // after last loop
        }
        // double speed for next turn
        if (sSpeedPWM <= MAX_SPEED_PWM / 2) {
            sSpeedPWM *= 2;
        } else {
            // last loop with MAX_SPEED
            sSpeedPWM = MAX_SPEED_PWM;
        }
        tLoopIndex++;

#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
        Serial.print(F("Set speed to:"));
        Serial.println(sSpeedPWM);
#endif
        RobotCar.IMUData.delayAndReadIMUCarDataFromMPU6050FIFO(2000);

    }
    RobotCar.IMUData.delayAndReadIMUCarDataFromMPU6050FIFO(5000);
    /*
     * switch direction
     */
    sDirection = oppositeDIRECTION(sDirection);
#if defined(ENABLE_EXTRA_NON_PLOTTER_OUTPUT)
    Serial.print(F("Switch direction to:"));
    Serial.println(sDirection);
#endif
}

void printCaption() {
    Serial.print(F("PWM[2] "));
#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
    Serial.print(F("MotorVoltage[0.1V] "));
#endif

    RobotCar.IMUData.printIMUCarDataCaption(&Serial);

#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
    Serial.print(F("VIN[0.1V] Speed/V[mm/sV] "));
#endif

#if defined(USE_ENCODER_MOTOR_CONTROL)
    RobotCar.rightCarMotor.printEncoderDataCaption(&Serial);
#endif
    Serial.print(
            F(
                    STR(PRINTS_PER_SECOND) "_values/s__ramp_offsets_up|down=" STR(RAMP_UP_VALUE_OFFSET_MILLIVOLT) "|" STR(RAMP_DOWN_VALUE_OFFSET_MILLIVOLT) "_mV__rdelta=" STR(RAMP_UP_VOLTAGE_PER_SECOND) "|" STR(RAMP_DOWN_VOLTAGE_PER_SECOND) "_V/s__decel="));
    Serial.print(RAMP_DECELERATION_TIMES_2 / 20);
    Serial.println(F("cm/s^2"));
}

/*
 * Prints values, if a new value is available
 * @param aDataSetsToPrint if aWaitForStop is true, number of data sets AFTER stop of car
 * @return true if printed.
 */
void delayAndPrintData(uint8_t aDataSetsToPrint, uint16_t aPeriodMillis, bool aWaitForStop) {
    static unsigned long sLastPrintMillis;

    for (uint_fast8_t i = 0; i < aDataSetsToPrint;) {
        if ((millis() - sLastPrintMillis) >= aPeriodMillis) { // read and print data every n ms
            sLastPrintMillis = millis();

            Serial.print(RobotCar.rightCarMotor.RequestedSpeedPWM / 2); // = PWM, scale it for plotter
            Serial.print(" ");

#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
            uint16_t tMillivolt = readVINVoltageMilliVolt();
            uint8_t tMotorPWM = RobotCar.rightCarMotor.CurrentCompensatedSpeedPWM;
            uint16_t tMotorDezivolt = (((uint32_t) tMillivolt * tMotorPWM) + 12800) / 25600L;
            Serial.print(tMotorDezivolt);
            Serial.print(' ');
#endif
            RobotCar.IMUData.readCarDataFromMPU6050Fifo();
            RobotCar.IMUData.printIMUCarData(&Serial);

#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
            Serial.print(' ');
            Serial.print((tMillivolt + 50) / 100); // VIN Dezivolt
            Serial.print(' ');
            /*
             * Compute speed per volt
             */
            uint16_t tSpeedPerVolt;
            int tSpeedPerSecond = RobotCar.IMUData.getSpeedCmPerSecond();
            if (tMotorPWM
                    > 0&& tSpeedPerSecond > 0 && RobotCar.rightCarMotor.MotorRampState != MOTOR_STATE_RAMP_DOWN) {
                tSpeedPerVolt = ((uint32_t) tSpeedPerSecond * 2550000L) / ((uint32_t) tMillivolt * tMotorPWM);
            } else {
                tSpeedPerVolt = 0;
            }
            Serial.print(tSpeedPerVolt);
#endif
#if defined(USE_ENCODER_MOTOR_CONTROL)
            Serial.print(' ');
            RobotCar.rightCarMotor.printEncoderData(&Serial);
#endif
            Serial.println();

            i++;
        } //  if ((millis() - sLastPrintMillis) >= aPeriodMillis)
        if (aWaitForStop && RobotCar.updateMotors()) {
            // reset count as long as car is driving
            i = 0;
        }
    } // for (uint_fast8_t i = 0; i < aDataSetsToPrint;)
}

#if defined(CAR_HAS_VIN_VOLTAGE_DIVIDER)
uint16_t readVINVoltageMilliVolt() {
    uint16_t tVIN = waitAndReadADCChannelWithReferenceAndRestoreADMUXAndReference(VIN_11TH_IN_CHANNEL, INTERNAL);
    return ((ADC_INTERNAL_REFERENCE_MILLIVOLT * 11 * (uint32_t) tVIN) + 512) / 1023;
}
#endif // MONITOR_VIN_VOLTAGE
