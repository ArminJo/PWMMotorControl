# [PWMMotorControl](https://github.com/ArminJo/PWMMotorControl)
Available as Arduino library "PWMMotorControl"

### [Version 1.0.0](https://github.com/ArminJo/PWMMotorControl/releases)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Installation instructions](https://www.ardu-badge.com/badge/PWMMotorControl.svg?)](https://www.ardu-badge.com/PWMMotorControl)
[![Commits since latest](https://img.shields.io/github/commits-since/ArminJo/PWMMotorControl/latest)](https://github.com/ArminJo/PWMMotorControl/commits/master)
[![Build Status](https://github.com/ArminJo/PWMMotorControl/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/PWMMotorControl/actions)
![Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_PWMMotorControl)

- The PWMDcMotor.cpp controls **brushed DC motors** by PWM using standard full bridge IC's like **L298**, **TB6612** (new low loss dual full bridge IC), or **Adafruit_MotorShield** (using PCA9685 -> 2 x TB6612).
- The EncoderMotor.cpp.cpp controls a DC motor with attached encoder disc and slot-type photo interrupters to enable **driving a specified distance**.
- The CarMotorControl.cpp controls **2 motors simultaneously** like it is required for most **Robot Cars**.

Basic commands are:
- `init(uint8_t aForwardPin, uint8_t aBackwardPin, uint8_t aPWMPin)`
- `setSpeed(uint8_t Unsigned_Speed,uint8_t Direction)` or `setSpeed(int Signed_Speed)`
- `stop()`

To go a specified distance (in 5mm/one encoder tick steps), use:
- `setDefaultsForFixedDistanceDriving()` to set minimal speed and maximal speed. Minimal speed is the PWM value where the motors start to move. It depends of the motor supply voltage.<br/>
Maximal speed is the PWM value to use for driving a fixed distance. For encoder equipped motors the software generates a ramp up from minimal to maximal at the start of the movement and a ramp down to stop.
- `calibrate()` to automatically set minimal speed for encoder motors.
- `initGoDistanceCount(uint8_t Unsigned_DistanceCount,uint8_tDirection)` or `setSpeed(intSigned_DistanceCount)` - for non encoder motors a formula, using distance and the difference between minimal speed and maximal speed, is used to convert counts into motor driving time.
- `updateMotor()` - call this in your loop if you use the start* functions.

# Compile options / macros for this library
To customize the library to different requirements, there are some compile options / macros available.<br/>
Modify it by commenting them out or in, or change the values if applicable. Or define the macro with the -D compiler option for gobal compile (the latter is not possible with the Arduino IDE, so consider to use [sloeber](https://eclipse.baeyens.it).<br/>
Some options which are enabed by default can be disabled by defining a *inhibit* macro like `USE_STANDARD_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD`.

| Macro | Default | File | Description |
|-|-|-|-|
| `USE_ENCODER_MOTOR_CONTROL` | disabled | PWMDCMotor.h | Use slot-type photo interrupter and an attached encoder disc to enable motor distance and speed sensing for closed loop control. |
| `USE_ADAFRUIT_MOTOR_SHIELD` | disabled | PWMDcMotor.h | Use Adafruit Motor Shield v2 connected by I2C instead of simple TB6612 or L298 breakout board.<br/>This disables tone output by using motor as loudspeaker, but requires only 2 I2C/TWI pins in contrast to the 6 pins used for the full bridge.<br/>For full bridge, analogWrite the millis() timer0 is used since we use pin 5 & 6. |
| `USE_OWN_LIBRARY_FOR_`<br/>`ADAFRUIT_MOTOR_SHIELD` | enabled | PWMDcMotor.h | Saves around 694 bytes program memory.<br/>Disable macro=`USE_STANDARD_LIBRARY_`<br/>`FOR_ADAFRUIT_MOTOR_SHIELD` |
| `SUPPORT_RAMP_UP` | enabled | PWMDcMotor.h | Saves around 300 bytes program memory.<br/>Disable macro=`DO_NOT_SUPPORT_RAMP_UP` |

# Default car geometry dependent values used in this library
These values are for a standard 2 WD car as can be seen on the pictures below.
| Macro | Default | File | Description |
|-|-|-|-|
| `DEFAULT_COUNTS_PER_FULL_ROTATION` | 40 | PWMDCMotor.h | This value is compatible with 20 slot encoder discs, giving 20 on and 20 off counts per full rotation. |
| `DEFAULT_MILLIMETER_PER_COUNT` | 5 | PWMDCMotor.h | At a circumference of around 20 cm (21.5 cm actual) this gives 5 mm per count. |
| `FACTOR_CENTIMETER_TO_`<br/>`COUNT_INTEGER_DEFAULT` | 2 | CarMotorControl.h | Exact value is 1.86, but integer saves program space and time. |
| `FACTOR_DEGREE_TO_COUNT_DEFAULT` | 0.4277777 for 2 wheel drive cars, 0.8 for 4 WD cars | CarMotorControl.h | Reflects the geometry of the standard 2 WD car sets. The 4 WD car value is estimated for slip on smooth surfaces. |

# Other default values for this library
These values are used by functions and the first 2 can be overwritten by set* functions.
| Macro | Default | File | Description |
|-|-|-|-|
| `DEFAULT_START_SPEED` | 45/150 for 7.4/6.0 volt supply | PWMDCMotor.h | START_SPEED is the speed PWM value at which car starts to move. |
| `DEFAULT_DRIVE_SPEED` | 80/255(max speed) for 7.4/6.0 volt supply | PWMDCMotor.h | The speed PWM value for going fixed distance. |
| `DEFAULT_DISTANCE_TO_TIME_FACTOR` | 135/300 for 7.4/6.0 volt supply | PWMDCMotor.h | The factor used to convert distance in 5mm steps to motor on time in milliseconds using the formula:<br/>`computedMillisOf`<br/>`MotorStopForDistance = 150 + (10 * ((aRequestedDistanceCount * DistanceToTimeFactor) / DriveSpeed))` |
| `RAMP_UP_UPDATE_INTERVAL_MILLIS` | 16 | PWMDCMotor.h | The smaller the value the steeper the ramp. |
| `RAMP_UP_UPDATE_INTERVAL_STEPS` | 16 | PWMDCMotor.h | Results in a ramp up time of 16 steps * 16 millis = 256 milliseconds. |


# Compile options for RobotCar example
To customize the RobotCar example to cover different extensions, there are some compile options available.
| Option | Default | File | Description |
|-|-|-|-|
| `USE_LAYOUT_FOR_NANO` | disabled | RobotCar.h | Use different pinout for Nano board. It has A6 and A7 available as pins. |
| `CAR_HAS_4_WHEELS` | disabled | RobotCar.h | Use modified formula for turning the car. |
| `USE_US_SENSOR_1_PIN_MODE` | disabled | RobotCar.h | Use modified HC-SR04 modules or HY-SRF05 ones.</br>Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin. |
| `CAR_HAS_IR_DISTANCE_SENSOR` | disabled | RobotCar.h | Use Sharp GP2Y0A21YK / 1080 IR distance sensor. |
| `CAR_HAS_TOF_DISTANCE_SENSOR` | disabled | RobotCar.h | Use VL53L1X TimeOfFlight distance sensor. |
| `DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN` | disabled | Distance.h | The distance servo is mounted head down to detect even small obstacles. |
| `CAR_HAS_CAMERA` | disabled | RobotCar.h | Enables the `Camera` button for the `PIN_CAMERA_SUPPLY_CONTROL` pin. |
| `CAR_HAS_LASER` | disabled | RobotCar.h | Enables the `Laser` button for the `PIN_LASER_OUT` / `LED_BUILTIN` pin. |
| `CAR_HAS_PAN_SERVO` | disabled | RobotCar.h | Enables the pan slider for the `PanServo` at the `PIN_PAN_SERVO` pin. |
| `CAR_HAS_TILT_SERVO` | disabled | RobotCar.h | Enables the tilt slider for the `TiltServo` at the `PIN_TILT_SERVO` pin.. |
| `MONITOR_LIPO_VOLTAGE` | disabled | RobotCar.h | Shows VIN voltage and monitors it for undervoltage. Requires 2 additional resistors at pin A2. |
| `VIN_VOLTAGE_CORRECTION` | undefined | RobotCar.h | Voltage to be subtracted from VIN voltage. E.g. if there is a series diode between LIPO and VIN set it to 0.8. |

#### If you find this library useful, please give it a star.

# Pictures
2 wheel car with encoders, 2 LiPo batteries, Adafruit Motor Shield V2, Bluetooth connection, and servo mounted head down
![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg)
4 wheel car, like 2 WD car before, but with servo mounted head up.
![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WheelDriveCar.jpg)
Encoder slot-type photo interrupter sensor
![Encoder slot-type photo interrupter sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ForkSensor.jpg)
Servo mounted head down
![Servo mounting](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ServoAtTopBack.jpg)
VIN sensing
![VIN sensing](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/SensingVIn.jpg)

# SCREENSHOTS
Start page
![MStart page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/HomePage.png)
Test page
![Manual control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/TestPage.png)
Automatic control page with detected wall at right
![Automatic control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/AutoDrivePage.png)
- Green bars are distances above 1 meter or above double distance of one ride per scan whichever is less.
- Red bars are distanced below the distance of one ride per scan -> collision during next "scan and ride" cycle if obstacle is ahead.
- Orange bars are the values between the 2 thresholds.
- The tiny white bars are the distances computed by the doWallDetection() function. They overlay the green (assumed timeout) values.
- The tiny black bar is the rotation chosen by doCollisionDetection() function.

# Revision History
### Version 1.1.0
- Added and renamed functions.

### Version 1.0.0
Initial Arduino library version.

