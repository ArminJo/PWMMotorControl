/*
 *  RobotCarBlueDisplay.cpp
 *
 *  Enables Robot car control with the BlueDisplay app.
 *  Requires the BlueDisplay and PlayRtttl library.
 *
 *  Enables autonomous driving of a 2 or 4 wheel car with an Arduino and a Adafruit Motor Shield V2.
 *  To avoid obstacles a HC-SR04 Ultrasonic sensor mounted on a SG90 Servo continuously scans the area.
 *  Manual control is by a GUI implemented with a Bluetooth HC-05 Module and the BlueDisplay library.
 *
 *  Define ENABLE_USER_PROVIDED_COLLISION_DETECTION and overwrite the 2 functions myOwnFillForwardDistancesInfo()
 *  and doUserCollisionAvoiding() to test your own skill.
 *
 *  If Bluetooth is not connected, after TIMEOUT_BEFORE_DEMO_MODE_STARTS_MILLIS (30 seconds) the car starts demo mode.
 *  After power up it runs in follower mode and after reset it runs in autonomous drive mode.
 *
 *  Program size of GUI is 63 percent of 32kByte.
 *
 *  Copyright (C) 2016-2026  Armin Joachimsmeyer
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#define VERSION_EXAMPLE "2.2.0"

#define INFO // Requires up to 2532 bytes of program memory for L298_2WD_2LI_ION_BASIC_CONFIGURATION
//#define DEBUG
//#define TRACE
/*
 * Timeouts for demo mode and inactivity remainder
 */
#define ATTENTION_AFTER_LAST_BD_COMMAND_MILLIS 240000L // move Servo after 4 Minutes of inactivity
#define TIMEOUT_BEFORE_DEMO_MODE_STARTS_SECONDS    30 // Start demo mode 30 seconds after boot up
#define TIMEOUT_BEFORE_DEMO_MODE_STARTS_MILLIS      (TIMEOUT_BEFORE_DEMO_MODE_STARTS_SECONDS * MILLIS_IN_ONE_SECOND)
#define TIMEOUT_BETWEEN_DEMOS_MILLIS           120000 // Repeat demo every 2 minutes

/*
 * Car configuration
 * For a complete list of available configurations see RobotCarConfigurations.h
 * https://github.com/ArminJo/Arduino-RobotCar/blob/master/src/RobotCarConfigurations.h
 */
//#define TBB6612_4WD_4AA_BASIC_CONFIGURATION           // China set with TB6612 mosfet bridge + 4 AA.
//#define TBB6612_4WD_4AA_VIN_CONFIGURATION             // China set with TB6612 mosfet bridge + 4 AA + VIN voltage divider.
//#define TBB6612_4WD_4AA_FULL_CONFIGURATION            // China set with TB6612 mosfet bridge + 4 AA + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_4NIMH_BASIC_CONFIGURATION         // China set with TB6612 mosfet bridge + 4 NiMh.
//#define TBB6612_4WD_4NIMH_VIN_CONFIGURATION           // China set with TB6612 mosfet bridge + 4 NiMh + VIN voltage divider.
//#define TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION       // China set with TB6612 mosfet bridge + 2 Li-ion.
//#define TBB6612_4WD_2LI_ION_FULL_CONFIGURATION        // China set with TB6612 mosfet bridge + 2 Li-ion + VIN voltage divider + MPU6050.
//#define TBB6612_4WD_2LI_ION_DIRECT_BASIC_CONFIGURATION // TBB6612_4WD_2LI_ION_BASIC_CONFIGURATION but power supply is connected direct to VIN and not to Uno power jack
//#define TBB6612_4WD_2LI_ION_DIRECT_VIN_CONFIGURATION  // China set with TB6612 mosfet bridge + 2 Li-ion direct + VIN voltage divider
//#define L298_2WD_4AA_BASIC_CONFIGURATION              // China 2WD set with L298 bridge and Uno board with series diode for VIN + 4 AA batteries. DEFAULT.
//#define L298_2WD_2LI_ION_BASIC_CONFIGURATION          // China 2WD set with L298 bridge and Uno board with series diode for VIN + 2 Li-ion.
//#define L298_2WD_2LI_ION_VIN_IR_CONFIGURATION         // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance
//#define L298_2WD_2LI_ION_VIN_IR_IMU_CONFIGURATION     // L298_2WD_2LI_ION_BASIC + VIN voltage divider + IR distance + MPU6050
//#define MECANUM_US_DISTANCE_CONFIGURATION             // Nano Breadboard version with Arduino NANO, TB6612 mosfet bridge and 4 mecanum wheels + US distance + servo
#define DO_NOT_SUPPORT_RAMP             // Ramps are anyway not used if drive speed voltage (default 2.0 V) is below 2.3 V. Saves 378 bytes program memory.
#define USE_SOFT_I2C_MASTER             // Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
/*
 * For moving exact distances and turns you may modify this values according to your actual car configuration
 */
//#define DEFAULT_CIRCUMFERENCE_MILLIMETER     220  // The circumference of your wheel in millimeter
#include "RobotCarConfigurations.h" // sets e.g. CAR_HAS_ENCODERS, USE_ADAFRUIT_MOTOR_SHIELD
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
#if !defined(INFO) && !defined(DEBUG) && !defined(TRACE) // No printing with Serial.print or we get "multiple definition of __vector_18".
// This leads to an error "multiple definition of __vector_18" if LOCAL_DEBUG etc. is defined manually in one included hpp file -> deactivate the next line
//#define BD_USE_SIMPLE_SERIAL            // We can use simple serial here, since no Serial.print are active. Saves up to 2480 bytes in BlueDisplay library.
#endif
#if defined(VIN_ATTENUATED_INPUT_PIN)
#define SHOW_VOLTAGE_ON_DISPLAY         // requires  bytes program space
#endif
#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT) && (defined(MONITOR_VIN_VOLTAGE) || defined(CAR_HAS_IR_DISTANCE_SENSOR))
// Here we use the ADC
// Must be defined before #include "BlueDisplay.hpp"
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100UL // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif
#define PRINT_VOLTAGE_PERIOD_MILLIS         500 // we only print if changed
#define VOLTAGE_TOO_LOW_DELAY_ONLINE_MILLIS         3000 // display VIN every 500 ms for 3 seconds
#define VOLTAGE_TOO_LOW_DELAY_OFFLINE_MILLIS       10000 // wait for 10 seconds after double beep

#if !defined(NO_RTTTL_FOR_CAR)
#define ENABLE_RTTTL_FOR_CAR        // Plays melody after initial timeout has reached and enables the "Play Melody" BD-button - 3730 bytes
#endif
//#define USE_MOTOR_FOR_MELODY        // Generates the tone by using left motor coils as tone generator. Not for motor shield. Requires 68 bytes
#if !defined(NO_PATH_INFO_PAGE) && defined(USE_ENCODER_MOTOR_CONTROL) // We use LastRideEncoderCount for path info entry
#define ENABLE_PATH_INFO_PAGE       // Requires up to 1400 bytes of program memory
#endif
//#define TEST_TIMING

/*
 * Values used in distance.hpp
 */
//#define IR_SENSOR_TYPE_430            // 4 to 30 cm, 18 ms, GP2YA41SK0F
//#define IR_SENSOR_TYPE_1080           // 10 to 80 cm, GP2Y0A21YK0F - default
//#define IR_SENSOR_TYPE_20150          // 20 to 150 cm, 18 ms, GP2Y0A02YK0F
//#define IR_SENSOR_TYPE_100550         // 100 to 550 cm, 18 ms, GP2Y0A710K0F
//#define TOF_OFFSET_MILLIMETER      10 // The offset measured manually or by calibrateOffset(). Offset = RealDistance - MeasuredDistance
//#define DISTANCE_SERVO_TRIM_DEGREE  5 // This value is internally added to all servo writes.
#if defined(ENABLE_USER_PROVIDED_COLLISION_DETECTION)
int doUserCollisionAvoiding();
#endif

/*
 * Settings to configure the BlueDisplay library and to reduce its size
 */
//#define BLUETOOTH_BAUD_RATE BAUD_115200  // Activate this, if you have reprogrammed the HC05 module for 115200, otherwise 9600 is used as baud rate
#define DO_NOT_NEED_BASIC_TOUCH_EVENTS // Disables unused basic touch events down, move and up. Saves 620 bytes program memory and 36 bytes RAM
#define DO_NOT_NEED_LONG_TOUCH_DOWN_AND_SWIPE_EVENTS  // Disables LongTouchDown and SwipeEnd events. Saves up to 88 bytes program memory and 4 bytes RAM.
#include "BlueDisplay.hpp"          // include source of library
#if !defined(USE_BLUE_DISPLAY_GUI)
#define USE_BLUE_DISPLAY_GUI        // for Distance.hpp and MecanumWheelCarPWMMotorControl.hpp included by CarPWMMotorControl.hpp
#endif

#include "CarPWMMotorControl.hpp"   // after BlueDisplay.hpp

#define VOLTAGE_USB_POWERED_UPPER_THRESHOLD_MILLIVOLT   4975 // Because Uno boards lack the series diode and have a low voltage drop
#include "RobotCarUtils.hpp"        // after BlueDisplay.hpp

#if defined(USE_MPU6050_IMU)
#include "IMUCarData.hpp"           // include source of library
#endif
#include "RobotCarGui.hpp"

#if defined(CAR_HAS_DISTANCE_SENSOR)
#include "Distance.hpp"             // requires definitions from RobotCarGui.h
#endif

#if defined(ENABLE_RTTTL_FOR_CAR)
#include "digitalWriteFast.h"
#define USE_NO_RTX_EXTENSIONS       // Disables RTX format definitions `'s'` (style) and `'l'` (loop). Saves up to 332 bytes program memory
#include <PlayRtttl.hpp>
bool sPlayMelody = false; // this flag may be reseted by checkAndHandleEvents()
void playRandomMelody();
#endif

#include "LocalDebugLevelStart.h"

#if defined(CAR_HAS_4_MECANUM_WHEELS)
bool sEnableDemo = false; // modified by BD button
#endif

void initServos();
void handleNotConnectedDemo();
void checkForCalibration();

#if defined(ENABLE_USER_PROVIDED_COLLISION_DETECTION)
/*************************************************************************************
 * Extend this basic collision detection to test your own skill in autonomous driving
 *
 * Checks distances and returns degree to turn
 * @return 0 -> no turn, >0 -> turn left, <0 -> turn right
 *************************************************************************************/
#define MINIMUM_DISTANCE_TO_SIDE 21
#define MINIMUM_DISTANCE_TO_FRONT 35

int doUserCollisionAvoiding() {
#if defined(ENABLE_AUTONOMOUS_DRIVE)
    // If left three distances are all less than 21 centimeter, then turn right.
    if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 1] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_LEFT - 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return -20;
        // If right three distances are all less then 21 centimeter than turn left.
    } else if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 1] <= MINIMUM_DISTANCE_TO_SIDE
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_RIGHT + 2] <= MINIMUM_DISTANCE_TO_SIDE) {
        return 20;
        // If front distance is longer then 35 centimeter than do not turn.
    } else if (sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_1] >= MINIMUM_DISTANCE_TO_FRONT
            && sForwardDistancesInfo.ProcessedDistancesArray[INDEX_FORWARD_2] >= MINIMUM_DISTANCE_TO_FRONT) {
        return 0;
    } else if (sForwardDistancesInfo.MaxDistance >= MINIMUM_DISTANCE_TO_SIDE) {
        /*
         * here front distance is less then 35 centimeter:
         * go to max side distance
         */
        return sForwardDistancesInfo.DegreeOfMaxDistance;
    } else {
        // Turn backwards.
        return 180;
    }
#else
    return 0;
#endif
}
#endif

/*
 * Start of robot car control program
 */
void setup() {
    /*
     * Configure first set of pins
     */
    // initialize the digital pin as an output.
#if defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // on my Uno R3 the LED is on otherwise
#endif
#if defined(AUX_PIN)
    pinMode(AUX_PIN, INPUT_PULLUP);
#endif
#if defined(CAR_HAS_LASER) && (LASER_OUT_PIN != LED_BUILTIN)
    pinMode(LASER_OUT_PIN, OUTPUT);
#endif

    tone(BUZZER_PIN, 2200, 50); // Booted

    initSerial();
//    initTrace();
//    printNumberOfPushesForISR();

    // initialize motors, this also stops motors
    initRobotCarPWMMotorControl();

    delay(100);
    tone(BUZZER_PIN, 2200, 50); // motor initialized

#if defined(CAR_HAS_DISTANCE_SERVO)
    sLastDistanceServoAngleInDegrees = 90; // is required before setupGUI()
#endif

    // Must be after RobotCar.init, since it tries to stop motors in connect callback
    setupGUI(); // this enables output by BlueDisplay1 and lasts around 100 milliseconds

    tone(BUZZER_PIN, 2200, 50); // GUI initialized (if connected)

#if defined(LOCAL_INFO)
    if (BlueDisplay1.isConnectionEstablished())
#else
    if (true)
#endif
    {
        // Just to know which program is running on my Arduino
        BlueDisplay1.debug("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__);
//        BlueDisplay1.debug("sMCUSR=", sMCUSR);
//        displayRotationValues();

    } else {
#if defined(LOCAL_INFO) // print it now if not printed above
#  if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#  endif
        // Just to know which program is running on my Arduino
        Serial.println(
                F(
                        "START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from  " __DATE__ "\r\nUsing PWMMotorControl library version " VERSION_PWMMOTORCONTROL));
#endif
    }
#if defined(LOCAL_INFO)
    readVCCOrVINVoltage(); // for isUSBPowered() below
    /*
     * This can be seen in the app log
     */
    RobotCar.printCalibrationValues(&Serial);
    printConfigInfo(&Serial);
    printProgramOptions(&Serial); // calls isUSBPowered()
    PWMDcMotor::printCompileOptions(&Serial);
#endif

#if defined(CAR_HAS_CAMERA)
    pinMode(CAMERA_SUPPLY_CONTROL_PIN, OUTPUT);
#endif

    initServos(); // must be after RobotCar.init() since it uses is2WDCar set there.

// reset all values
#if defined(ENABLE_PATH_INFO_PAGE)
    resetPathData();
#endif
#if defined(CAR_HAS_DISTANCE_SENSOR)
    initDistance();
#endif

#if defined(ENABLE_RTTTL_FOR_CAR)
#  if defined(VIN_ATTENUATED_INPUT_PIN)
    randomSeed(sVINVoltage * 100);
#   else
    randomSeed(sVCCVoltageMillivolt);
#  endif
#endif

    delay(100);
    // power up finished
    if (BlueDisplay1.isConnectionEstablished()) {
        tone(BUZZER_PIN, 1000, 300);
    } else {
        tone(BUZZER_PIN, 1000, 50);
    }
}

/*
 * 35us per loop for idle HomePage, TestPage and SensorDrivePage
 * 140 us per loop for idle AutoDrivePage
 */
void loop() {
#if defined(TEST_TIMING)
    digitalToggleFast(BUZZER_PIN);
#endif
    /*
     * Required, if we use rotation, ramps and fixed distance driving
     */
    RobotCar.updateMotors(); // 2 us, if driving

    /*
     * Check periodically for voltage change, undervoltage, user input and update display output
     */
    loopGUI();

#if defined(ENABLE_AUTONOMOUS_DRIVE)
    driveAutonomousOneStep(); // Handle autonomous driving
#endif

#if defined(ENABLE_RTTTL_FOR_CAR)
    /*
     * Check for playing melody in main loop requested by BD button
     */
    if (sPlayMelody) {
        RobotCar.stop();
        playRandomMelody();
    }
#endif

#if defined(CAR_HAS_4_MECANUM_WHEELS)
    /*
     * Check for mecanum wheel car demo requested by BD button
     */
    if (sEnableDemo) {
        startStopRobotCar(false); // Stop car and reset also button
        RobotCar.doDemo(true); // true = long demo
        TouchButtonDemo.setValueAndDraw(false); // switch demo button back to red
        sEnableDemo = false;
    }
#endif

#if !defined(USE_MPU6050_IMU)
    checkForCalibration();
#endif

    if (!BlueDisplay1.isConnectionEstablished()) {
        /*
         * After 30 seconds of being disconnected, run the demo.
         * Runs forever and returns only if connection is established
         */
        handleNotConnectedDemo();
    }

    /*
     * After 4 minutes of user inactivity, play double tone and make noise by scanning with US Servo and then repeat it every 2. minute
     */
    if (BlueDisplay1.isConnectionEstablished()
            && sMillisOfLastReceivedBDEvent + ATTENTION_AFTER_LAST_BD_COMMAND_MILLIS < millis()) {
        sMillisOfLastReceivedBDEvent = millis() - (ATTENTION_AFTER_LAST_BD_COMMAND_MILLIS / 2); // adjust sMillisOfLastReceivedBDEvent to have the next scan in 2 minutes
        playDoubleTone();
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

#if !defined(USE_MPU6050_IMU)
void checkForCalibration() {
    if (doCalibration) {
        startStopRobotCar(false); // Reset also button

        // first get EEPROM values, in order to not work with the values we accidently set before in a former calibration
        RobotCar.readCarValuesFromEeprom();
        displayRotationValues();
#  if defined(VIN_ATTENUATED_INPUT_PIN)
        calibrateDriveSpeedPWMAndPrint();
#  endif
#  if !defined(USE_MPU6050_IMU) && (defined(CAR_HAS_4_WHEELS) || defined(CAR_HAS_4_MECANUM_WHEELS) || !defined(USE_ENCODER_MOTOR_CONTROL))
        if (!delayMillisAndCheckForStop(3000)) { // time to rearrange car
            calibrateRotation();
        }
#  else
        RobotCar.writeCarValuesToEeprom();  // write only DriveSpeedPWM values
#  endif
        doCalibration = false;
    }
}
#endif

/*
 * After 30 seconds of being disconnected, start autonomous drive or the demo which is in turn repeated after 2 minutes.
 * Do not run it if the car is connected to USB (e.g. for programming or debugging), which can be tested only for a Li-ion supply :-(.
 * If AUX_PIN is LOW, do short demo, otherwise do all demo moves in a row.
 */
void handleNotConnectedDemo() {
    if (millis() == TIMEOUT_BEFORE_DEMO_MODE_STARTS_MILLIS) {
#if defined(ADC_UTILS_ARE_AVAILABLE)
        if (isUSBPowered()) {
            INFO_PRINTLN(F("Timeout and USB powered -> skip demo"));
            playDoubleTone();
            // Here we are for programming or debugging -> enable periodic attention
            return;
        }
#endif

        /*
         * Timeout just reached and not USB powered, play melody and start demo or autonomous drive
         */
#if defined(CAR_HAS_4_MECANUM_WHEELS)
        INFO_PRINTLN(F("Timeout -> run demo and repeat it every 2 minutes"));
#else
        INFO_PRINTLN(F("Timeout -> run follower demo"));
#endif
        /*
         * Start demo once with a melody
         */
#if defined(ENABLE_RTTTL_FOR_CAR)
        playRandomMelody(); // blocking call
        delayAndLoopGUI(1000); // make a short break before starting demo or autonomous drive
#endif
        /*
         * Run and check forever, until we are connected
         */
        while (!BlueDisplay1.isConnectionEstablished()) {
            // Set right page for reconnect
#if defined(CAR_HAS_4_MECANUM_WHEELS)
            // If AUX_PIN is LOW, do short demo, otherwise do all demo moves in a row.
            bool tDoLongDemo = digitalRead(AUX_PIN);
            RobotCar.doDemo(tDoLongDemo);
            delayAndLoopGUI(TIMEOUT_BETWEEN_DEMOS_MILLIS / 2); // wait 1 minute before next short demo and check for connection
            if (tDoLongDemo) {
                delayAndLoopGUI(TIMEOUT_BETWEEN_DEMOS_MILLIS / 2); // wait additional 1 minutes before next long demo and check for connection
            }
#elif defined(ENABLE_AUTONOMOUS_DRIVE)
            GUISwitchPages(nullptr, PAGE_AUTOMATIC_CONTROL); // ??? we are not connected!
            startStopAutomomousDrive(true, MODE_FOLLOWER);
#else
            GUISwitchPages(nullptr, PAGE_HOME); // no demo here
#endif
        }
    }
}

#if defined(ENABLE_RTTTL_FOR_CAR)
/*
 * Play melody blocking
 * Can use motor as loudspeaker if USE_MOTOR_FOR_MELODY is defined
 * Can be stopped by pressing BD melody button
 */
void playRandomMelody() {
    sPlayMelody = true;
#if defined(LOCAL_DEBUG)
    BlueDisplay1.debug("Play melody");
#endif

#if defined(USE_MOTOR_FOR_MELODY) && defined(LEFT_MOTOR_FORWARD_PIN)
    OCR2B = 0;
    bitWrite(TIMSK2, OCIE2B, 1);            // enable interrupt for inverted pin handling
    startPlayRandomRtttlFromArrayPGM(LEFT_MOTOR_FORWARD_PIN, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
#else
#if defined(LOCAL_DEBUG)
    startPlayRandomRtttlFromArrayPGM(BUZZER_PIN, RTTTLMelodiesTiny, ARRAY_SIZE_MELODIES_TINY); // save space for debug statements
#else
    startPlayRandomRtttlFromArrayPGM(BUZZER_PIN, RTTTLMelodiesSmall, ARRAY_SIZE_MELODIES_SMALL);
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
    tone(BUZZER_PIN, aFrequency, aDuration);
    delay(aDuration);
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
    DistanceServoWriteAndWaitForStop(90, false);
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
    DistanceServo.attach(DISTANCE_SERVO_PIN); // Use standard servo library, because we have more servos and cannot use LightweightServo library
#  endif
#endif

#if defined(CAR_HAS_PAN_SERVO)
// initialize and set laser pan servo
    PanServo.attach(PAN_SERVO_PIN);
#endif
#if defined(CAR_HAS_TILT_SERVO)
    TiltServo.attach(TILT_SERVO_PIN);
#endif
    resetServos();
}

