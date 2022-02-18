/*
 *  RobotCarBlueDisplay.cpp
 *
 *  Enables Robot car control with the BlueDisplay app.
 *
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *  Just overwrite the 2 functions myOwnFillForwardDistancesInfo() and doUserCollisionDetection() to test your own skill.
 *
 *  If Bluetooth is not connected, after TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS (10 seconds) the car starts demo mode.
 *  After power up it runs in follower mode and after reset it runs in autonomous drive mode.
 *
 *  Program size of GUI is 63 percent of 32kByte. 27% vs. 90%.
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

#include <Arduino.h>

//#define ENABLE_EEPROM_STORAGE       // Activates the GUI buttons to store compensation and drive speed
//#define MONITOR_VIN_VOLTAGE         // Shows VIN voltage and monitors it for undervoltage. VIN/11 at A2, 1MOhm to VIN, 100kOhm to ground
//#define CAR_ENABLE_RTTTL            // Plays melody after initial timeout has reached and enables the "Play Melody" BD-button
//#define ENABLE_PATH_INFO_PAGE       // Saves program space

/*
 * Car configuration
 */
#include "RobotCarPinDefinitionsAndMore.h"

#include "RobotCarBlueDisplay.h"
#include "CarPWMMotorControl.hpp" // include source of library
#if defined(USE_MPU6050_IMU)
#include "IMUCarData.hpp" // include source of library
#endif
#include "RobotCarGui.hpp"
#include "Distance.hpp" // requires definitions from RobotCarGui.h
#ifdef CAR_ENABLE_RTTTL
//#define USE_NO_RTX_EXTENSIONS       // saves program space if defined globally
#include <PlayRtttl.h>
#endif


/*
 * Timeouts for demo mode and inactivity remainder
 */
#define TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS 240000L // move Servo after 4 Minutes of inactivity
#define TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS 10000 // Start demo mode 10 seconds after boot up

//#define DEBUG_TRACE_INIT
//#include "AvrTracing.hpp"

/****************************************************************************
 * Change this if you have reprogrammed the hc05 module for other baud rate
 ***************************************************************************/
#ifndef BLUETOOTH_BAUD_RATE
//#define BLUETOOTH_BAUD_RATE BAUD_115200
#define BLUETOOTH_BAUD_RATE BAUD_9600
#endif

#define VERSION_EXAMPLE "3.0"

float sVINVoltage;

#ifdef CAR_ENABLE_RTTTL
bool sPlayMelody = false;
void playRandomMelody();
#endif

void initServos();

/*************************************************************************************
 * Extend this basic collision detection to test your own skill in autonomous driving
 *
 * Checks distances and returns degree to turn
 * @return 0 -> no turn, >0 -> turn left, <0 -> turn right
 *************************************************************************************/
#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

int doUserCollisionDetection() {
// if left three distances are all less than 21 centimeter then turn right.
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 1] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return -90;
        // check right three distances are all less then 21 centimeter than turn left.
    } else if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 1] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return 90;
        // check front distance is longer then 35 centimeter than do not turn.
    } else if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] >= MINIMUM_DISTANCE_TO_FRONT
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] >= MINIMUM_DISTANCE_TO_FRONT) {
        return 0;
    } else if (sForwardDistancesInfo.MaxDistance >= MINIMUM_DISTANCE_TO_SIDE) {
        /*
         * here front distance is less then 35 centimeter:
         * go to max side distance
         */
        // formula to convert index to degree.
        return DEGREES_PER_STEP * sForwardDistancesInfo.IndexOfMaxDistance + START_DEGREES - 90;
    } else {
        // Turn backwards.
        return 180;
    }
}

/*
 * !!! THIS WORKS ONLY WITH VERSION 8.0 OF THE OPTIBOOT BOOTLOADER !!!
 First, we need a variable to hold the reset cause that can be written before
 early sketch initialization (that might change r2), and won't be reset by the
 various initialization code.
 avr-gcc provides for this via the ".noinit" section.
 */
uint8_t sMCUSR __attribute__ ((section(".noinit")));

/*
 Next, we need to put some code to save reset cause from the bootloader (in r2)
 to the variable.  Again, avr-gcc provides special code sections for this.
 If compiled with link time optimization (-flto), as done by the Arduino
 IDE version 1.6 and higher, we need the "used" attribute to prevent this
 from being omitted.
 */
void resetFlagsInit(void) __attribute__ ((naked))
__attribute__ ((used))
__attribute__ ((section (".init0")));
void resetFlagsInit(void) {
    /*
     save the reset flags passed from the bootloader
     This is a "simple" matter of storing (STS) r2 in the special variable
     that we have created. We use assembler to access the right variable.
     */
    __asm__ __volatile__ ("sts %0, r2\n" : "=m" (sMCUSR) :);
}

bool sBootReasonWasPowerUp = false;
/*
 * Start of robot car control program
 */
void setup() {
    MCUSR = 0;
    if (sMCUSR & (1 << PORF)) {
        sBootReasonWasPowerUp = true; // always true for old Optiboot bootloader
    }

    /*
     * Configure first set of pins
     */
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // on my UNO R3 the LED is on otherwise
#if defined(LASER_MOUNTED) && (PIN_LASER_OUT != LED_BUILTIN)
    pinMode(PIN_LASER_OUT, OUTPUT);
#endif

    tone(PIN_BUZZER, 2200, 50); // Booted

    initSerial(BLUETOOTH_BAUD_RATE);
//    initTrace();
//    printNumberOfPushesForISR();

    // initialize motors, this also stops motors
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    RobotCarMotorControl.init();
#else
#  ifdef USE_ENCODER_MOTOR_CONTROL
    RobotCarMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_INTERRUPT,
    LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_INTERRUPT);
#  else
    RobotCarMotorControl.init(RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_FORWARD_PIN,
    LEFT_MOTOR_BACKWARD_PIN, LEFT_MOTOR_PWM_PIN);
#  endif
#endif

#ifdef ENABLE_EEPROM_STORAGE
    RobotCarMotorControl.readMotorValuesFromEeprom();
#endif

    sRuningAutonomousDrive = false;

    delay(100);
    tone(PIN_BUZZER, 2200, 50); // motor initialized

    sLastServoAngleInDegrees = 90; // is required before setupGUI()

    // Must be after RobotCarMotorControl.init, since it tries to stop motors in connect callback
    setupGUI(); // this enables output by BlueDisplay1 and lasts around 100 milliseconds

    tone(PIN_BUZZER, 2200, 50); // GUI initialized (if connected)

    if (BlueDisplay1.isConnectionEstablished()) {
        // Just to know which program is running on my Arduino
        BlueDisplay1.debug("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);
//        BlueDisplay1.debug("sMCUSR=", sMCUSR);
    } else {
#if !defined(USE_SIMPLE_SERIAL) && !defined(USE_SERIAL1)  // print it now if not printed above
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) || defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from  " __DATE__));
        Serial.print(F("sMCUSR=0x"));
        Serial.println(sMCUSR, HEX);
#endif
    }

#ifdef CAR_HAS_CAMERA
    pinMode(PIN_CAMERA_SUPPLY_CONTROL, OUTPUT);
#endif

    initServos(); // must be after RobotCarMotorControl.init() since it uses is2WDCar set there.

// reset all values
#ifdef ENABLE_PATH_INFO_PAGE
    resetPathData();
#endif
    initDistance();

#if defined(MONITOR_VIN_VOLTAGE)
    readVINVoltage();
    randomSeed(sVINVoltage * 100);
#endif

    delay(100);
    if (sBootReasonWasPowerUp) {
        tone(PIN_BUZZER, 1000, 50); // power up finished
    } else {
        tone(PIN_BUZZER, 1000, 300); // long tone, reset finished - only detectable with Optiboot 8.0
    }
}

void loop() {

    /*
     * check if timeout, no Bluetooth connection and connected to LIPO battery
     */
    if ((!BlueDisplay1.isConnectionEstablished()) && (millis() < (TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS + 1000))
            && (millis() > TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS)
#if defined(MONITOR_VIN_VOLTAGE)
            && (sVINVoltage > VOLTAGE_USB_THRESHOLD)
#endif
            ) {

        /*
         * Timeout just reached, play melody and start autonomous drive
         */
#ifdef CAR_ENABLE_RTTTL
        playRandomMelody();
        delayAndLoopGUI(1000);
#else
        delayAndLoopGUI(6000); // delay needed for millis() check above!
#endif
        // check again, maybe we are connected now
        if (!BlueDisplay1.isConnectionEstablished()) {
            // Set right page for reconnect
            GUISwitchPages(NULL, PAGE_AUTOMATIC_CONTROL);
            if (sBootReasonWasPowerUp) {
                startStopAutomomousDrive(true, MODE_FOLLOWER);
            } else {
                startStopAutomomousDrive(true, MODE_AUTONOMOUS_DRIVE_BUILTIN);
            }
        }
    }

    /*
     * Required, if we use rotation, ramps and fixed distance driving
     */
    RobotCarMotorControl.updateMotors();
    /*
     * check for user input and update display output
     */
    loopGUI();

    /*
     * After 4 minutes of user inactivity, make noise by scanning with US Servo and repeat it every 2. minute
     */
    if (BlueDisplay1.isConnectionEstablished() && sMillisOfLastReceivedBDEvent + TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS < millis()) {
        sMillisOfLastReceivedBDEvent = millis() - (TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS / 2); // adjust sMillisOfLastReceivedBDEvent to have the next scan in 2 minutes
        fillAndShowForwardDistancesInfo(true, true);
#if defined(CAR_HAS_PAN_SERVO) || defined(CAR_HAS_TILT_SERVO)
        DistanceServo.write(90); // set servo back to normal
#else
        write10(90);
#endif
    }

#ifdef CAR_ENABLE_RTTTL
    /*
     * check for playing melody
     */
    if (sPlayMelody) {
        RobotCarMotorControl.stop();
        playRandomMelody();
    }

#endif

    if (sRuningAutonomousDrive) {
        if (sDriveMode == MODE_FOLLOWER) {
            driveFollowerModeOneStep();
        } else {
            driveAutonomousOneStep();
        }
    }
}

#if defined(MONITOR_VIN_VOLTAGE)
void readVINVoltage() {
    uint16_t tVIN = waitAndReadADCChannelWithReferenceAndRestoreADMUX(VIN_11TH_IN_CHANNEL, INTERNAL);

//    BlueDisplay1.debug("VIN Raw=", tVIN);

// assume resistor network of 1MOhm / 100kOhm (divider by 11)
// tVIN * 0,01182795
#  ifdef VIN_VOLTAGE_CORRECTION
    // we have a diode (requires 0.8 volt) between LIPO and VIN
    sVINVoltage = (tVIN * ((11.0 * 1.1) / 1023)) + VIN_VOLTAGE_CORRECTION;
#  else
    sVINVoltage = tVIN * ((11.0 * 1.1) / 1023);
#  endif
}
#endif // MONITOR_VIN_VOLTAGE

#ifdef CAR_ENABLE_RTTTL
#include "digitalWriteFast.h"
/*
 * Prepare for tone, use motor as loudspeaker
 */
void playRandomMelody() {
// this flag may be reseted by checkAndHandleEvents()
    sPlayMelody = true;
//    BlueDisplay1.debug("Play melody");

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    startPlayRandomRtttlFromArrayPGM(PIN_BUZZER, RTTTLMelodiesTiny, ARRAY_SIZE_MELODIES_TINY);
//    startPlayRandomRtttlFromArrayPGM(PIN_BUZZER, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
#else
    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1);            // enable interrupt for inverted pin handling
    startPlayRandomRtttlFromArrayPGM(PIN_LEFT_MOTOR_FORWARD, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
#endif
    while (updatePlayRtttl()) {
#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD)
        // check for pause in melody (i.e. timer disabled) and disable motor for this period
        if (TIMSK2 & _BV(OCIE2A)) {
            // timer enabled
            digitalWriteFast(PIN_LEFT_MOTOR_PWM, HIGH);            // re-enable motor
        } else {
            // timer disabled
            digitalWriteFast(PIN_LEFT_MOTOR_PWM, LOW);            // disable motor for pause in melody
        }
#endif
        checkAndHandleEvents();
        if (!sPlayMelody) {
//            BlueDisplay1.debug("Stop melody");
            break;
        }
    }
#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD)
    digitalWriteFast(PIN_LEFT_MOTOR_PWM, LOW); // disable motor
    bitWrite(TIMSK2, OCIE2B, 0); // disable interrupt
#endif
    TouchButtonMelody.setValue(false, (sCurrentPage == PAGE_HOME));
    sPlayMelody = false;
}

void playTone(unsigned int aFrequency, unsigned long aDuration = 0) {
#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    tone(PIN_BUZZER, aFrequency);
    delay(aDuration);
    noTone(PIN_BUZZER);
#else
    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1); // enable interrupt for inverted pin handling
    tone(PIN_LEFT_MOTOR_FORWARD, aFrequency);
    delay(aDuration);
    noTone(PIN_LEFT_MOTOR_FORWARD);
    digitalWriteFast(PIN_LEFT_MOTOR_PWM, LOW); // disable motor
    bitWrite(TIMSK2, OCIE2B, 0); // disable interrupt
#endif
}

/*
 * set INVERTED_TONE_PIN to inverse value of TONE_PIN to avoid DC current
 */
#if ! defined(USE_ADAFRUIT_MOTOR_SHIELD)
ISR(TIMER2_COMPB_vect) {
    digitalWriteFast(PIN_LEFT_MOTOR_BACKWARD, !digitalReadFast(PIN_LEFT_MOTOR_FORWARD));
}
#endif
#endif // CAR_ENABLE_RTTTL

/*
 * Pn tilt servo stuff
 */
#ifdef CAR_HAS_PAN_SERVO
Servo PanServo;
#endif
#ifdef CAR_HAS_TILT_SERVO
Servo TiltServo;
#endif

void resetServos() {
    sLastServoAngleInDegrees = 0; // to force setting of 90 degree
    DistanceServoWriteAndDelay(90, false);
#ifdef CAR_HAS_PAN_SERVO
    PanServo.write(90);
#endif
#ifdef CAR_HAS_TILT_SERVO
    TiltServo.write(TILT_SERVO_MIN_VALUE); // my servo makes noise at 0 degree.
#endif
}

void initServos() {
#if defined(CAR_HAS_PAN_SERVO) || defined(CAR_HAS_TILT_SERVO)
    DistanceServo.attach(PIN_DISTANCE_SERVO); // Use standard servo library, because we have more servos and cannot use LightweightServo library
#else
    initLightweightServoPin10(); // Use LightweightServo library, because we have only one servo
#endif

#ifdef CAR_HAS_PAN_SERVO
// initialize and set laser pan servo
    PanServo.attach(PIN_PAN_SERVO);
#endif
#ifdef CAR_HAS_TILT_SERVO
    TiltServo.attach(PIN_TILT_SERVO);
#endif
    resetServos();
}

