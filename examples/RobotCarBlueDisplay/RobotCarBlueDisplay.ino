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
 *  Program size of GUI is 63 percent. 27% vs. 90%.
 *
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

#include <Arduino.h>

#include "RobotCar.h"
#include "RobotCarGui.h"
#include "Distance.h"

#include "HCSR04.h"

#ifdef ENABLE_RTTTL
#include <PlayRtttl.h>
#endif
//#include "Trace.cpp.h"

#define VERSION_EXAMPLE "3.0"

/*
 * Car Control
 */
CarMotorControl RobotCarMotorControl;
float sVINVoltage;

#ifdef ENABLE_RTTTL
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
 First, we need a variable to hold the reset cause that can be written before
 early sketch initialization (that might change r2), and won't be reset by the
 various initialization code.
 avr-gcc provides for this via the ".noinit" section.
 */
uint8_t sMCUSR __attribute__ ((section(".noinit")));

/*
 Next, we need to put some code to save reset cause from the bootload (in r2)
 to the variable.  Again, avr-gcc provides special code sections for this.
 If compiled with link time optimization (-flto), as done by the Arduno
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
     that we have created.  We use assembler to access the right variable.
     */
    __asm__ __volatile__ ("sts %0, r2\n" : "=m" (sMCUSR) :);
}

bool sBootReasonWasReset = false;
/*
 * Start of robot car control program
 */
void setup() {
    MCUSR = 0;
    if (sMCUSR & (1 << EXTRF)) {
        sBootReasonWasReset = true;
    }

    /*
     * Configure first set of pins
     */
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // on my UNO R3 the LED is on otherwise
#if defined(CAR_HAS_LASER) && (PIN_LASER_OUT != LED_BUILTIN)
    pinMode(PIN_LASER_OUT, OUTPUT);
#endif

    tone(PIN_SPEAKER, 2200, 50); // Booted

    // initialize motors
#ifdef USE_ADAFRUIT_MOTOR_SHIELD
    RobotCarMotorControl.init(true); // true -> read from EEPROM
#else
    RobotCarMotorControl.init(PIN_RIGHT_MOTOR_FORWARD, PIN_RIGHT_MOTOR_BACKWARD, PIN_RIGHT_MOTOR_PWM, PIN_LEFT_MOTOR_FORWARD,
            PIN_LEFT_MOTOR_BACKWARD, PIN_LEFT_MOTOR_PWM, true); // true -> read from EEPROM
#endif

    delay(100);
    tone(PIN_SPEAKER, 2200, 50); // motor initialized

    sLastServoAngleInDegrees = 90;
    // Must be after RobotCarMotorControl.init, since it tries to stop motors in connect callback
    setupGUI(); // this enables output by BlueDisplay1 and lasts around 100 milliseconds

    tone(PIN_SPEAKER, 2200, 50); // GUI initialized (if connected)

    if (!BlueDisplay1.isConnectionEstablished()) {
#if defined (USE_STANDARD_SERIAL) && !defined(USE_SERIAL1)  // print it now if not printed above
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)
    delay(2000); // To be able to connect Serial monitor after reset and before first printout
#endif
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from  " __DATE__));
        Serial.print(F("sMCUSR=0x"));
        Serial.println(sMCUSR, HEX);
#endif
    } else {
        // Just to know which program is running on my Arduino
        BlueDisplay1.debug("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);
//        BlueDisplay1.debug("sMCUSR=", sMCUSR);
    }

#ifdef CAR_HAS_CAMERA
    pinMode(PIN_CAMERA_SUPPLY_CONTROL, OUTPUT);
#endif

    initServos(); // must be after RobotCarMotorControl.init() since it uses is2WDCar set there.

// reset all values
    resetPathData();
    initDistance();

#if defined(MONITOR_LIPO_VOLTAGE)
    readVINVoltage();
    randomSeed(sVINVoltage * 100);
#endif

//    initTrace();

    delay(100);
    tone(PIN_SPEAKER, 2200, 50); // startup finished
}

void loop() {

    /*
     * check if timeout, no Bluetooth connection and connected to LIPO battery
     */
    if ((!BlueDisplay1.isConnectionEstablished()) && (millis() < (TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS + 1000))
            && (millis() > TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS)
#if defined(MONITOR_LIPO_VOLTAGE)
            && (sVINVoltage > VOLTAGE_USB_THRESHOLD)
#endif
            ) {
        /*
         * Timeout just reached, play melody and start autonomous drive
         */
#ifdef ENABLE_RTTTL
        playRandomMelody();
        delayAndLoopGUI(1000);
#else
        delayAndLoopGUI(6000); // delay needed for millis() check above!
#endif
        if (!BlueDisplay1.isConnectionEstablished()) {
            // Set right page for reconnect
            GUISwitchPages(NULL, PAGE_AUTOMATIC_CONTROL);
            if (sBootReasonWasReset) {
                startStopAutomomousDrive(true, MODE_AUTONOMOUS_DRIVE_BUILTIN);
            } else {
                startStopAutomomousDrive(true, MODE_FOLLOWER);
            }
        }
    }

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
        DistanceServo.write(90); // set servo back to normal
    }

#ifdef ENABLE_RTTTL
    /*
     * check for playing melody
     */
    if (sPlayMelody) {
        RobotCarMotorControl.stopMotors();
        playRandomMelody();
    }

#endif

    if (sCurrentPage == PAGE_HOME || sCurrentPage == PAGE_TEST) {
        /*
         * Direct speed control by GUI
         */

        if (RobotCarMotorControl.updateMotors()) {
#ifdef USE_ENCODER_MOTOR_CONTROL
            // At least one motor is moving here
            RobotCarMotorControl.rightCarMotor.synchronizeMotor(&RobotCarMotorControl.leftCarMotor,
            MOTOR_DEFAULT_SYNCHRONIZE_INTERVAL_MILLIS);
#endif
        }

    }

    if (sRuningAutonomousDrive) {
        if (sDriveMode == MODE_FOLLOWER) {
            driveFollowerModeOneStep();
        } else {
            driveAutonomousOneStep();
        }
    }
}

#if defined(MONITOR_LIPO_VOLTAGE)
void readVINVoltage() {
    uint8_t tOldADMUX = checkAndWaitForReferenceAndChannelToSwitch(VIN_11TH_IN_CHANNEL, INTERNAL);
    uint16_t tVIN = readADCChannelWithReferenceOversample(VIN_11TH_IN_CHANNEL, INTERNAL, 2); // 4 samples
//    BlueDisplay1.debug("VIN Raw=", tVIN);
            // Switch back (to DEFAULT)
    ADMUX = tOldADMUX;

// assume resistor network of 1Mk / 100k (divider by 11)
// tVCC * 0,01181640625
#ifdef VIN_VOLTAGE_CORRECTION
    // we have a diode (requires 0.8 volt) between LIPO and VIN
    sVINVoltage = (tVIN * ((11.0 * 1.07) / 1023)) + VIN_VOLTAGE_CORRECTION;
#else
    sVINVoltage = tVIN * ((11.0 * 1.07) / 1023);
#endif
}
#endif

#ifdef ENABLE_RTTTL
#include "digitalWriteFast.h"
/*
 * Prepare for tone, use motor as loudspeaker
 */
void playRandomMelody() {
// this flag may be reseted by checkAndHandleEvents()
    sPlayMelody = true;
//    BlueDisplay1.debug("Play melody");

#if defined(USE_ADAFRUIT_MOTOR_SHIELD)
    startPlayRandomRtttlFromArrayPGM(PIN_SPEAKER, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
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
    tone(PIN_SPEAKER, aFrequency);
    delay(aDuration);
    noTone(PIN_SPEAKER);
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
#endif // ENABLE_RTTTL

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
    DistanceServoWriteAndDelay(90, false);
#ifdef CAR_HAS_PAN_SERVO
    PanServo.write(90);
#endif
#ifdef CAR_HAS_TILT_SERVO
    TiltServo.write(TILT_SERVO_MIN_VALUE); // my servo makes noise at 0 degree.
#endif
}

void initServos() {
//        DistanceServo.attach(PIN_DISTANCE_SERVO, DISTANCE_SERVO_MIN_PULSE_WIDTH, DISTANCE_SERVO_MAX_PULSE_WIDTH);
    DistanceServo.attach(PIN_DISTANCE_SERVO);
#ifdef CAR_HAS_PAN_SERVO
// initialize and set Laser pan servo
    PanServo.attach(PIN_PAN_SERVO);
#endif
#ifdef CAR_HAS_TILT_SERVO
    TiltServo.attach(PIN_TILT_SERVO);
#endif
    resetServos();
}

