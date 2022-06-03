# Autonomous driving robot car
### Version 2.0.0
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Arduino-RobotCar/latest)](https://github.com/ArminJo/Arduino-RobotCar/commits/master)
[![Build Status](https://github.com/ArminJo/Arduino-RobotCar/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-RobotCar/actions)
![Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Arduino-RobotCar)

- Enables autonomous driving of a 2 or 4 wheel car controlled by an Arduino.
- To avoid obstacles a HC-SR04 ultrasonic sensor mounted on a SG90 Servo continuously scans the environment.
- To overcome the drawbacks of ultrasonic sensors, an additional IR or TOF (TimeOfFlight) sensor can be mounted to the servo.
- Manual control is implemented by a GUI using a Bluetooth HC-05 Module and the [BlueDisplay library](https://github.com/ArminJo/Arduino-BlueDisplay).

**Just overwrite the function doUserCollisionDetection() to test your own skill**.
You may also overwrite the function fillAndShowForwardDistancesInfo(), if you use your own scanning method.

This example requires the **[PWMMotorControl library](https://github.com/ArminJo/PWMMotorControl)** to control the motors,
the **[BlueDisplay library](https://github.com/ArminJo/BlueDisplay)** for Smartphone control and **[PlayRtttl library](https://github.com/ArminJo/PlayRtttl)** for the melody feature.

# Pictures
| 4WD car with IR receiver and Bluetooth module and 4 AA rechargeable batteries. | Instructable |
|-|-|
| ![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WDInstructable.jpg) | [![Instructable](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/instructables-logo-v2.png)](https://www.instructables.com/Arduino-4WD-Car-Assembly-and-Code-With-Optional-In/) |
| 4 wheel car, like 2 WD car before, but with servo mounted head up. | 2 wheel car with encoders, 2 Li-ion batteries, Adafruit Motor Shield V2, Bluetooth connection, and servo mounted head down.  |
| ![4 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/4WheelDriveCar.jpg) | ![2 wheel car](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/2WheelDriveCar.jpg)  |
| Encoder fork sensor | Servo mounted head down |
| ![Encoder fork sensor](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ForkSensor.jpg) | ![Servo mounting](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/ServoAtTopBack.jpg) |
| VIN sensing |   |
| ![VIN sensing](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/SensingVIn.jpg) |   |

# SCREENSHOTS
| Start page | Test page |
|-|-|
| ![Start page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/HomePage.png) | ![Test page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/TestPage.png) |

Automatic control page with detected wall at right
![Automatic control page](https://github.com/ArminJo/Arduino-RobotCar/blob/master/pictures/AutoDrivePageWallDetected.jpg)
- Cyan bars are distances above 1 meter.
- Green bars are distances above double distance of one ride per scan (default 40 cm).
- Red bars are distanced below the distance of one ride per scan (default 20 cm) -> collision during next "scan and ride" cycle if obstacle is ahead.
- Yellow bars are the values between the 2 thresholds (default 20 cm to 40 cm).
- The tiny white bars are the distances computed by the doWallDetection() function. They overlay the green (assumed timeout) values.
- The tiny black bar is the rotation chosen by doCollisionDetection() function.

# Wall detection
Ultrasonic distance measurement has a problem with walls.
You can **only detect a wall** if the angle of the wall relative to sensor axis is approximately **between 70 and 110 degree**.
For other angels the reflected ultrasonic beam cannot reach the receiver which leads to unrealistic great distances.<br/>
The implemented wall detection function `doWallDetection()` takes samples every 18 degrees and if it gets 2 adjacent short distances below `DISTANCE_MAX_FOR_WALL_DETECTION_CM`, it assumes a wall determined by these 2 samples.
The (invalid) values 18 degrees right and left of these samples are then extrapolated by `computeNeigbourValue()`.

# Compile options / macros for this software
To customize the software to different car configurations, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them, or change the values if applicable.

Compile options for the used **PWMMotorControl library** like `USE_ENCODER_MOTOR_CONTROL` are described [here](https://github.com/ArminJo/PWMMotorControl#compile-options--macros-for-this-library).

| Name | Default value | Description |
|-|-|-|
| `CAR_HAS_VIN_VOLTAGE_DIVIDER` | undefined | VIN/11 at A2, e.g. 1MOhm to VIN, 100kOhm to ground. Required to show and monitor (for undervoltage) VIN voltage. |
| `VIN_VOLTAGE_CORRECTION` | undefined or 0.8 for UNO | Voltage to be subtracted from VIN voltage for voltage monitoring. E.g. if there is a series diode between Li-ion and VIN as on the UNO boards, set it to 0.8. |
| `CAR_HAS_US_DISTANCE_SENSOR` | disabled | A HC-SR04 ultrasonic distance sensor is mounted (default for most China smart cars). |
| `US_SENSOR_SUPPORTS_1_PIN_MODE` | disabled | Use modified HC-SR04 modules or HY-SRF05 ones.</br>Modify HC-SR04 by connecting 10kOhm between echo and trigger and then use only trigger pin. |
| `CAR_HAS_IR_DISTANCE_SENSOR` | disabled | Use Sharp GP2Y0A21YK / 1080 IR distance sensor. |
| `CAR_HAS_TOF_DISTANCE_SENSOR` | disabled | Use VL53L1X TimeOfFlight distance sensor. |
| `CAR_HAS_DISTANCE_SERVO` | disabled | Distance sensor is mounted on a pan servo (default for most China smart cars). |
| `DISTANCE_SERVO_IS_MOUNTED_HEAD_DOWN` | disabled | Distance.h | The distance servo is mounted head down to detect even small obstacles. The Servo direction is reverse then. |
| `CAR_HAS_PAN_SERVO` | disabled | Enables the pan slider for the `PanServo` at the `PIN_PAN_SERVO` pin. |
| `CAR_HAS_TILT_SERVO` | disabled | Enables the tilt slider for the `TiltServo` at the `PIN_TILT_SERVO` pin. |
| `CAR_HAS_CAMERA` | disabled | Enables the `Camera` button for the `PIN_CAMERA_SUPPLY_CONTROL` pin. |
| `CAR_HAS_LASER` | disabled | Enables the `Laser` button for the `PIN_LASER_OUT` / `LED_BUILTIN` pin. |
| `ENABLE_RTTTL_FOR_CAR` | undefined | Plays melody after initial timeout has reached. Enables the Melody button, which plays a random melody. |
| `MONITOR_VIN_VOLTAGE` | disabled | Shows VIN voltage and monitors it for undervoltage. VIN/11 at A2, 1MOhm to VIN, 100kOhm to ground. |
| `ENABLE_EEPROM_STORAGE` | disabled | Activates the buttons to store compensation and drive speed. |

### Changing include (*.h) files with Arduino IDE
First, use *Sketch > Show Sketch Folder (Ctrl+K)*.<br/>
If you have not yet saved the example as your own sketch, then you are instantly in the right library folder.<br/>
Otherwise you have to navigate to the parallel `libraries` folder and select the library you want to access.<br/>
In both cases the library source and include files are located in the libraries `src` directory.<br/>
The modification must be renewed for each new library version!

### Modifying compile options / macros with PlatformIO
If you are using PlatformIO, you can define the macros in the *[platformio.ini](https://docs.platformio.org/en/latest/projectconf/section_env_build.html)* file with `build_flags = -D MACRO_NAME` or `build_flags = -D MACRO_NAME=macroValue`.

### Modifying compile options / macros with Sloeber IDE
If you are using [Sloeber](https://eclipse.baeyens.it) as your IDE, you can easily define global symbols with *Properties > Arduino > CompileOptions*.<br/>
![Sloeber settings](https://github.com/Arduino-IRremote/Arduino-IRremote/blob/master/pictures/SloeberDefineSymbols.png)
