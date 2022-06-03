/*
 *  RobotCarBlueDisplay.cpp
 *
 *  Enables Robot car control with the BlueDisplay app.
 *  Requires the BlueDisplay and PlayRtttl library.
 *
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *  Just overwrite the 2 functions myOwnFillForwardDistancesInfo() and doUserCollisionDetection() to test your own skill.
 *
 *  If Bluetooth is not connected, after TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS (10 seconds) the car starts demo mode.
 *  After power up it runs in follower mode and after reset it runs in autonomous drive mode.
 *
 *  Program size of GUI is 63 percent of 32kByte.
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#define VERSION_EXAMPLE "2.0.0"

//#define DEBUG
//#define TRACE
/*
 * Timeouts for demo mode and inactivity remainder
 */
#define TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS 240000L // move Servo after 4 Minutes of inactivity
#define TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS 10000 // Start demo mode 10 seconds after boot up

/*
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 4AA.
//#define TBB6612_4WD_4AA_VIN_CONFIGURATION         // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 4AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION   // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION    // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define L298_2WD_4AA_BASIC_CONFIGURATION          // Default. Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 4 AA batteries.
//#define L298_4WD_4AA_BASIC_CONFIGURATION          // China set with L298 + 4AA.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION      // Basic = Lafvin 2WD model using L298 bridge. Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_VIN_IR_DISTANCE_CONFIGURATION    // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_VIN_IR_IMU_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
//#define MECANUM_DISTANCE_CONFIGURATION            // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
#define USE_SOFT_I2C_MASTER             // Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
/*
 * For moving exact distances and turns you may modify this values according to your actual car configuration
 */
//#define DEFAULT_CIRCUMFERENCE_MILLIMETER     220  // The circumference of your wheel in millimeter
#include "RobotCarConfigurations.h" // sets e.g. USE_ENCODER_MOTOR_CONTROL, USE_ADAFRUIT_MOTOR_SHIELD
#include "RobotCarPinDefinitionsAndMore.h"

/*
 * Enabling program features dependent on car configuration
 */
#if defined(CAR_HAS_ENCODERS)
#define USE_ENCODER_MOTOR_CONTROL   // Enable if by default, if available
#endif
#if defined(CAR_HAS_MPU6050_IMU)
#define USE_MPU6050_IMU             // Requires up to 2850 bytes program memory
#endif
#if defined(CAR_HAS_DISTANCE_SENSOR) && defined(CAR_HAS_DISTANCE_SERVO)
#define ENABLE_AUTONOMOUS_DRIVE     // Enable if by default, if available
#endif
#if defined(CAR_HAS_PAN_SERVO)
#include <Servo.h>
Servo PanServo;
#endif
#if defined(CAR_HAS_TILT_SERVO)
#include <Servo.h>
Servo TiltServo;
#define TILT_SERVO_MIN_VALUE     7 // since lower values will make an insane sound at my pan tilt device
#endif

/*
 * Enable functionality of this program
 */
#if defined(NO_APPLICATON_INFO)
#define USE_SIMPLE_SERIAL           // saves 1224 bytes
#else
#define APPLICATON_INFO             // Prints configuration info at startup. Requires additional 1504 bytes of program memory
#endif
#define MONITOR_VIN_VOLTAGE         // Enable monitoring of VIN voltage for exact movements, if available. Check at startup.
//#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L    // Value measured at the AREF pin
#define PRINT_VOLTAGE_PERIOD_MILLIS 2000
#define VOLTAGE_LIPO_LOW_THRESHOLD          6.9 // Formula: 2 * 3.5 volt - voltage loss: 25 mV GND + 45 mV VIN + 35 mV Battery holder internal
#define VOLTAGE_USB_THRESHOLD               5.5
#define VOLTAGE_USB_THRESHOLD_MILLIVOLT     5500 // required for preprocessor condition
#define VOLTAGE_TOO_LOW_DELAY_ONLINE        3000 // display VIN every 500 ms for 3 seconds
#define VOLTAGE_TOO_LOW_DELAY_OFFLINE       1000 // wait for 1 seconds after double beep

#if !defined(NO_RTTTL_FOR_CAR)
#define ENABLE_RTTTL_FOR_CAR        // Plays melody after initial timeout has reached and enables the "Play Melody" BD-button - 3730 bytes
#endif
//#define USE_MOTOR_FOR_MELODY        // Generates the tone by using left motor coils as tone generator. Not for motor shield. Requires 68 bytes
#if !defined(NO_PATH_INFO_PAGE) && defined(USE_ENCODER_MOTOR_CONTROL) // We use LastRideEncoderCount for path info entry
#define ENABLE_PATH_INFO_PAGE       // Requires up to 1400 bytes of program memory
#endif
//#define TEST_TIMING

#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT) && (defined(MONITOR_VIN_VOLTAGE) || defined(CAR_HAS_IR_DISTANCE_SENSOR))
// Must be before #include "BlueDisplay.hpp"
#define ADC_INTERNAL_REFERENCE_MILLIVOLT 1100L    // Value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

int doUserCollisionDetection();

#include "CarPWMMotorControl.hpp"   // include source of library
#include "RobotCarUtils.hpp"        // include source of library

/*
 * Settings to configure the BlueDisplay library and to reduce its size
 */
//#define BLUETOOTH_BAUD_RATE BAUD_115200  // Activate this, if you have reprogrammed the HC05 module for 115200, otherwise 9600 is used as baud rate
#define DO_NOT_NEED_BASIC_TOUCH_EVENTS // Disables basic touch events like down, move and up. Saves 620 bytes program memory and 36 bytes RAM
#include "BlueDisplay.hpp"          // include source of library

#if defined(USE_MPU6050_IMU)
#include "IMUCarData.hpp"           // include source of library
#endif
#include "RobotCarGui.hpp"

#if defined(CAR_HAS_DISTANCE_SENSOR)
#include "Distance.hpp"             // requires definitions from RobotCarGui.h
#endif

#if defined(ENABLE_RTTTL_FOR_CAR)
#define USE_NO_RTX_EXTENSIONS       // Disables RTX format definitions `'s'` (style) and `'l'` (loop). Saves up to 332 bytes program memory
#include <PlayRtttl.hpp>
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
#if defined(ENABLE_AUTONOMOUS_DRIVE)
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
#else
    return 0;
#endif
}

/*
 * Start of robot car control program
 */
void setup() {
    /*
     * Configure first set of pins
     */
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // on my UNO R3 the LED is on otherwise
#if defined(CAR_HAS_LASER) && (PIN_LASER_OUT != LED_BUILTIN)
    pinMode(PIN_LASER_OUT, OUTPUT);
#endif

    tone(PIN_BUZZER, 2200, 50); // Booted

    initSerial();
//    initTrace();
//    printNumberOfPushesForISR();

    // initialize motors, this also stops motors
    initRobotCarPWMMotorControl();

#if defined(ENABLE_EEPROM_STORAGE)
    RobotCar.readMotorValuesFromEeprom();
#endif

    delay(100);
    tone(PIN_BUZZER, 2200, 50); // motor initialized

#if defined(CAR_HAS_DISTANCE_SERVO)
    sLastDistanceServoAngleInDegrees = 90; // is required before setupGUI()
#endif

    // Must be after RobotCar.init, since it tries to stop motors in connect callback
    setupGUI(); // this enables output by BlueDisplay1 and lasts around 100 milliseconds

    tone(PIN_BUZZER, 2200, 50); // GUI initialized (if connected)

#if defined(APPLICATON_INFO)
    if (BlueDisplay1.isConnectionEstablished())
#else
     if (true)
#endif
    {
        // Just to know which program is running on my Arduino
        BlueDisplay1.debug("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);
//        BlueDisplay1.debug("sMCUSR=", sMCUSR);
    } else {
#if defined(APPLICATON_INFO) // requires 1504 bytes program space
#  if !defined(USE_SIMPLE_SERIAL) && !defined(USE_SERIAL1)  // print it now if not printed above
#    if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#    endif
        // Just to know which program is running on my Arduino
        Serial.println(
                F(
                        "START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from  " __DATE__ "\r\nUsing PWMMotorControl library version " VERSION_PWMMOTORCONTROL));
        printConfigInfo(&Serial);
        printProgramOptions(&Serial);
        PWMDcMotor::printCompileOptions(&Serial);
#  endif
#endif
    }

#if defined(CAR_HAS_CAMERA)
    pinMode(PIN_CAMERA_SUPPLY_CONTROL, OUTPUT);
#endif

    initServos(); // must be after RobotCar.init() since it uses is2WDCar set there.

// reset all values
#if defined(ENABLE_PATH_INFO_PAGE)
    resetPathData();
#endif
#if defined(CAR_HAS_DISTANCE_SENSOR)
    initDistance();
#endif

#if defined(MONITOR_VIN_VOLTAGE) && defined(ENABLE_RTTTL_FOR_CAR)
    randomSeed(sVINVoltage * 100);
#endif

    delay(100);
    // power up finished
    if (BlueDisplay1.isConnectionEstablished()) {
        tone(PIN_BUZZER, 1000, 300);
    } else {
        tone(PIN_BUZZER, 1000, 50);
    }
}

/*
 * 35us per loop for idle HomePage, TestPage and SensorDrivePage
 * 140 us per loop for idle AutoDrivePage
 */
void loop() {
#if defined(TEST_TIMING)
    digitalToggleFast(PIN_BUZZER);
#endif
    /*
     * Required, if we use rotation, ramps and fixed distance driving
     */
    RobotCar.updateMotors(); // 2 us, if driving

    /*
     * check for user input and update display output
     */
    loopGUI();

#if defined(ENABLE_AUTONOMOUS_DRIVE)
    /*
     * Handle autonomous driving
     */
    driveAutonomousOneStep();
#endif

#if defined(ENABLE_RTTTL_FOR_CAR)
    /*
     * check for playing melody
     */
    if (sPlayMelody) {
        RobotCar.stop();
        playRandomMelody();
    }

#endif

    /*
     * check if timeout, no Bluetooth connection and not connected to USB. For
     */
    if ((!BlueDisplay1.isConnectionEstablished()) && (millis() < (TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS + 1000))
            && (millis() > TIMOUT_BEFORE_DEMO_MODE_STARTS_MILLIS)
#if defined(MONITOR_VIN_VOLTAGE) && (FULL_BRIDGE_INPUT_MILLIVOLT > VOLTAGE_USB_THRESHOLD_MILLIVOLT)
            && (sVINVoltage > VOLTAGE_USB_THRESHOLD_MILLIVOLT)
#endif
            ) {

        /*
         * Timeout just reached, play melody and start autonomous drive
         */
#if defined(ENABLE_RTTTL_FOR_CAR)
        playRandomMelody();
        delayAndLoopGUI(1000);
#else
        delayAndLoopGUI(6000); // delay needed for millis() check above!
#endif
        // check again, maybe we are connected now
        if (!BlueDisplay1.isConnectionEstablished()) {
            // Set right page for reconnect
#if defined(ENABLE_AUTONOMOUS_DRIVE)
            GUISwitchPages(NULL, PAGE_AUTOMATIC_CONTROL);
            startStopAutomomousDrive(true, MODE_FOLLOWER);
#else
            GUISwitchPages(NULL, PAGE_HOME);
#endif
        }
    }

    /*
     * After 4 minutes of user inactivity, make noise by scanning with US Servo and repeat it every 2. minute
     */
    if (BlueDisplay1.isConnectionEstablished() && sMillisOfLastReceivedBDEvent + TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS < millis()) {
        sMillisOfLastReceivedBDEvent = millis() - (TIMOUT_AFTER_LAST_BD_COMMAND_MILLIS / 2); // adjust sMillisOfLastReceivedBDEvent to have the next scan in 2 minutes
#if defined(CAR_HAS_DISTANCE_SERVO)
        fillAndShowForwardDistancesInfo(true, true);
#  if defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
        write10(90);
#  else
        DistanceServo.write(90); // set servo back to normal
#  endif
#endif

    }
}

#if defined(ENABLE_RTTTL_FOR_CAR)
#include "digitalWriteFast.h"
/*
 * Prepare for tone, use motor as loudspeaker
 */
void playRandomMelody() {
// this flag may be reseted by checkAndHandleEvents()
    sPlayMelody = true;
//    BlueDisplay1.debug("Play melody");

#if defined(USE_MOTOR_FOR_MELODY) && defined(LEFT_MOTOR_FORWARD_PIN)
    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1);            // enable interrupt for inverted pin handling
    startPlayRandomRtttlFromArrayPGM(LEFT_MOTOR_FORWARD_PIN, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
#else
#if defined(DEBUG)
    startPlayRandomRtttlFromArrayPGM(PIN_BUZZER, RTTTLMelodiesTiny, ARRAY_SIZE_MELODIES_TINY);
#else
    startPlayRandomRtttlFromArrayPGM(PIN_BUZZER, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
#endif
#endif
    while (updatePlayRtttl()) {
#if defined(USE_MOTOR_FOR_MELODY) && defined(LEFT_MOTOR_FORWARD_PIN)
        // check for pause in melody (i.e. timer disabled) and disable motor for this period
        if (TIMSK2 & _BV(OCIE2A)) {
            // timer enabled
            digitalWriteFast(LEFT_MOTOR_PWM_PIN, HIGH);            // re-enable motor
        } else {
            // timer disabled
            digitalWriteFast(LEFT_MOTOR_PWM_PIN, LOW);            // disable motor for pause in melody
        }
#endif
        checkAndHandleEvents();
        if (!sPlayMelody) {
//            BlueDisplay1.debug("Stop melody");
            break;
        }
    }
#if defined(USE_MOTOR_FOR_MELODY) && defined(LEFT_MOTOR_FORWARD_PIN)
    digitalWriteFast(LEFT_MOTOR_PWM_PIN, LOW); // disable motor
    bitWrite(TIMSK2, OCIE2B, 0); // disable interrupt
#endif
    TouchButtonMelody.setValue(false, (sCurrentPage == PAGE_HOME));
    sPlayMelody = false;
}

void playTone(unsigned int aFrequency, unsigned long aDuration = 0) {
#if defined(USE_MOTOR_FOR_MELODY) && defined(LEFT_MOTOR_FORWARD_PIN)
    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1); // enable interrupt for inverted pin handling
    tone(LEFT_MOTOR_FORWARD_PIN, aFrequency);
    delay(aDuration);
    noTone(LEFT_MOTOR_FORWARD_PIN);
    digitalWriteFast(LEFT_MOTOR_PWM_PIN, LOW); // disable motor
    bitWrite(TIMSK2, OCIE2B, 0); // disable interrupt
#else
    tone(PIN_BUZZER, aFrequency);
    delay(aDuration);
    noTone(PIN_BUZZER);
#endif
}

/*
 * set INVERTED_TONE_PIN to inverse value of TONE_PIN to avoid DC current
 */
#if defined(USE_MOTOR_FOR_MELODY) && defined(LEFT_MOTOR_FORWARD_PIN)
ISR(TIMER2_COMPB_vect) {
    digitalWriteFast(LEFT_MOTOR_BACKWARD_PIN, !digitalReadFast(LEFT_MOTOR_FORWARD_PIN));
}
#endif
#endif // ENABLE_RTTTL_FOR_CAR

void resetServos() {
#if defined(CAR_HAS_DISTANCE_SERVO)
    sLastDistanceServoAngleInDegrees = 0; // to force setting of 90 degree
    DistanceServoWriteAndDelay(90, false);
#endif
#if defined(CAR_HAS_PAN_SERVO)
    PanServo.write(90);
#endif
#if defined(CAR_HAS_TILT_SERVO)
    TiltServo.write(TILT_SERVO_MIN_VALUE); // my servo makes noise at 0 degree.
#endif
}

void initServos() {
#if defined(CAR_HAS_DISTANCE_SERVO)
#  if defined(USE_LIGHTWEIGHT_SERVO_LIBRARY)
    initLightweightServoPin10(); // Use LightweightServo library, because we have only one servo
#  else
    DistanceServo.attach(PIN_DISTANCE_SERVO); // Use standard servo library, because we have more servos and cannot use LightweightServo library
#  endif
#endif

#if defined(CAR_HAS_PAN_SERVO)
// initialize and set laser pan servo
    PanServo.attach(PIN_PAN_SERVO);
#endif
#if defined(CAR_HAS_TILT_SERVO)
    TiltServo.attach(PIN_TILT_SERVO);
#endif
    resetServos();
}

