![](../images/banner.png)

# Overview

It may be desireable to use the Arduino IDE to work with Stretch Firmware rather than the command line tool. Here we describe how to set up the IDE.

* Download and unzip the latest version (1.8.12 as of this writing) from the [Arduino site](https://www.arduino.cc/en/main/software)
  * The Ubuntu package does not play well with stretch_firmware. Do not use this one.

```bash
cd Downloads
tar -xvf arduino-1.8.12-linux64.tar.xz
cd arduino-1.8.12
sudo ./install.sh
arduino
```

Set the default Sketchbook location under File/Preferences to 

```
~/repos/stretch_firmware/arduino
```

<img src="../images/arduino-1.png" height="300" />

Now close down and restart the IDE. Now under Tools/Board you should see the three Hello Robot board types available at the bottom of the list:

<img src="../images/arduino-2.png"  height="400" />

Next, install the Arduino SAMD Boards (32-bits Arm Cortex-M0+) package via the Boards Manager (Tools/Board/Boards Manager) . **It is important to install version 1.6.21 and not a newer version.**

<img src="../images/arduino-3.png"  height="400" />

Now test that your install is ready to go. 

* Open up the sketch hello_wacc.ino
* Select the board type Tools/Board/Hello Wacc
* Select Sketch/Verify-Compile. It should compile with no errors



# Updating Stretch Firmware

**Warning: It is possible to 'brick' your Stretch robot if you don't follow these instructions carefully. Firmware upgrade should only be done at the recommendation of Hello Robot support.** 

First, update the stretch_firmware repository to the correct version. Generally this will be done by

```
cd ~/repos/stretch_firmware
git pull
```

Hello Robot support will have specified a firmware version to install. If you are unsure, you can verify the version of firmware for a particular board be looking in the Common.h file for the  Sketch. For example:

```
#define FIRMWARE_VERSION "Pimu.V0.0.1p0"
#define BOARD_VERSION "Pimu.Guthrie.V1"
```

There are 6 boards that you may want to update.

| Board               | USB Device                   | Arduino Sketch    |
| ------------------- | ---------------------------- | ----------------- |
| Left wheel stepper  | /dev/hello-motor-left-wheel  | hello_stepper.ino |
| Right wheel stepper | /dev/hello-motor-right-wheel | hello_stepper.ino |
| Lift stepper        | /dev/hello-motor-lift        | hello_stepper.ino |
| Arm stepper         | /dev/hello-motor-arm         | hello_stepper.ino |
| Pimu                | /dev/hello-pimu              | hello_pimu.ino    |
| Wacc                | /dev/hello-wacc              | hello_wacc.ino    |

The device names listed are sim-links to the actual USB device. For the board to be upgraded, determine the actual port name that will be used by the Arduino IDE. This can be determined by:

```
ls -l /dev/hello*
```

Next

* Launch the Arduino IDE
* Open the appropriate sketch for the board to be updated
* Select the appropriate board type from the menu Tools/Board/hello*
* Select the appropriate port from Tools/Port (eg /dev/ttyACM0 ) that maps to the correct USB device
* Select Sketch/Upload from the menu. The new firmware will load to the board as shown

<img src="../images/arduino-4.png" height="600" />

## Flashing the Stepper Calibration

After updating the firmware to any of the stepper motors the encoder calibration must be rewritten to the flash. If the flash calibration has not been written the motor will not work. 

For each motor (eg, the left wheel):

```
cd ~/repos/stretch_firmware/python
./stepper_calibration_YAML_to_flash.py hello-motor-left-wheel
```



