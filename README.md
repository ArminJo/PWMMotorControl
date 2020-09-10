# [PWMMotorControl](https://github.com/ArminJo/PWMMotorControl)
Available as Arduino library "PWMMotorControl"

### [Version 1.0.0](https://github.com/ArminJo/PWMMotorControl/releases)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/PWMMotorControl/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/PWMMotorControl/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FPWMMotorControl)](https://github.com/brentvollebregt/hit-counter)

- The PWMDcMotor.cpp controls **brushed DC motors** by PWM using standard full bridge IC's like **L298**, **TB6612** (new low loss dual full bridge IC), or **Adafruit_MotorShield** (using PCA9685 -> 2 x TB6612).
- The EncoderMotor.cpp.cpp controls a DC motor with attached encoder disc and fork light barrier to enable **driving a specified distance**.
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

# Compile options for library
To customize the library to different requirements, there are some compile options available.

| Option | Default | File | Description |
|-|-|-|-|
| `USE_ENCODER_MOTOR_CONTROL` | disabled | PWMDCMotor.h | Use fork light barrier and an attached encoder disc to enable motor distance and speed sensing for closed loop control. |
| `USE_ADAFRUIT_MOTOR_SHIELD` | disabled | PWMDcMotor.h | Use Adafruit Motor Shield v2 connected by I2C instead of simple TB6612 or L298 breakout board.<br/>
This disables tone output by using motor as loudspeaker, but requires only 2 I2C/TWI pins in contrast to the 6 pins used for the full bride.<br/>
For full bride, analogWrite the millis() timer0 is used since we use pin 5 & 6. |
| `USE_OWN_LIBRARY_FOR_ADAFRUIT_MOTOR_SHIELD` | enabled | PWMDcMotor.h | Saves 694 bytes program memory |

# Compile options for examples
To customize the Robot Car example to cover different extensions, there are some compile options available.

| Option | Default | File | Description |
|-|-|-|-|
| `USE_US_SENSOR_1_PIN_MODE` | disabled | RobotCar.h | Use modified HC-SR04 modules or HY-SRF05 ones.</br>Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin. |
| `CAR_HAS_IR_DISTANCE_SENSOR` | disabled | RobotCar.h | Use Sharp GP2Y0A21YK / 1080 IR distance sensor. |
| `CAR_HAS_TOF_DISTANCE_SENSOR` | disabled | RobotCar.h | Use VL53L1X TimeOfFlight distance sensor. |
| `CAR_HAS_CAMERA` | disabled | RobotCar.h | Enables the `Camera` button for the `PIN_CAMERA_SUPPLY_CONTROL` pin. |
| `CAR_HAS_LASER` | disabled | RobotCar.h | Enables the `Laser` button for the `PIN_LASER_OUT` / `LED_BUILTIN` pin. |
| `CAR_HAS_PAN_SERVO` | disabled | RobotCar.h | Enables the pan slider for the `PanServo` at the `PIN_PAN_SERVO` pin. |
| `CAR_HAS_TILT_SERVO` | disabled | RobotCar.h | Enables the tilt slider for the `TiltServo` at the `PIN_TILT_SERVO` pin.. |
| `DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN` | disabled | Distance.h | The distance servo is mounted head down to detect small obstacles. |

#### If you find this library useful, please give it a star.

# Pictures
2 wheel car
![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg)
4 wheel car
![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WheelDriveCar.jpg)
Encoder fork sensor
![Encoder fork sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ForkSensor.jpg)
Servo mounting
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
### Version 1.0.0
Initial Arduino library version

