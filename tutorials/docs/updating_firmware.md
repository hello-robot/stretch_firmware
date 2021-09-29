![](../images/banner.png)

# Updating Stretch Firmware

**Warning: Firmware upgrade should only be done at the recommendation of Hello Robot support.** 

Stretch has 6 Arduino based microcontroller boards that may require a firmware update:

| Board               | USB Device                   | Arduino Sketch    |
| ------------------- | ---------------------------- | ----------------- |
| Left wheel stepper  | /dev/hello-motor-left-wheel  | hello_stepper.ino |
| Right wheel stepper | /dev/hello-motor-right-wheel | hello_stepper.ino |
| Lift stepper        | /dev/hello-motor-lift        | hello_stepper.ino |
| Arm stepper         | /dev/hello-motor-arm         | hello_stepper.ino |
| Pimu                | /dev/hello-pimu              | hello_pimu.ino    |
| Wacc                | /dev/hello-wacc              | hello_wacc.ino    |

Firmware updates are managed through the [RE1_firmware_updater.py](https://github.com/hello-robot/stretch_factory/blob/master/python/bin/RE1_firmware_updater.py) tool, which uses the [Arduino Cli](https://github.com/arduino/arduino-cli) tool to flash the firmware.

Some robots may not have these tools installed by the factory. See the Appendix for details on manual installation.

## Firmware Update Tool

Running  `RE1_firmware_updater.py --status` will report the currently install firmware as well as recommend updates. 

Running  `RE1_firmware_updater.py --update` will prompt you to install the recommended updates.

For example:

```bash
>>$RE1_firmware_updater.py --status
Cloning latest version of Stretch Firmware to /tmp/stretch_firmware_update_20210620191742
############## Currently Installed Configuration ##############
------------ HELLO-WACC ------------
Installed Firmware: Wacc.v0.0.1p0
Stretch Body requires protocol: p0
Protocol match
------------ HELLO-MOTOR-LIFT ------------
Installed Firmware: Stepper.v0.0.4p0
Stretch Body requires protocol: p0
Protocol match
------------ HELLO-PIMU ------------
Installed Firmware: Pimu.v0.0.1p0
Stretch Body requires protocol: p0
Protocol match
------------ HELLO-MOTOR-ARM ------------
Installed Firmware: Stepper.v0.0.4p0
Stretch Body requires protocol: p0
Protocol match
------------ HELLO-MOTOR-LEFT-WHEEL ------------
Installed Firmware: Stepper.v0.0.4p0
Stretch Body requires protocol: p0
Protocol match
------------ HELLO-MOTOR-RIGHT-WHEEL ------------
Installed Firmware: Stepper.v0.0.4p0
Stretch Body requires protocol: p0
Protocol match

######### Currently Available Versions of Stretch Firmware on GitHub ##########
---- HELLO-WACC ----
Wacc.v0.0.1p0
---- HELLO-MOTOR-LIFT ----
Stepper.v0.0.1p0
Stepper.v0.0.2p0
Stepper.v0.0.3p0
Stepper.v0.0.4p0
---- HELLO-PIMU ----
Pimu.v0.0.1p0
---- HELLO-MOTOR-ARM ----
Stepper.v0.0.1p0
Stepper.v0.0.2p0
Stepper.v0.0.3p0
Stepper.v0.0.4p0
---- HELLO-MOTOR-LEFT-WHEEL ----
Stepper.v0.0.1p0
Stepper.v0.0.2p0
Stepper.v0.0.3p0
Stepper.v0.0.4p0
---- HELLO-MOTOR-RIGHT-WHEEL ----
Stepper.v0.0.1p0
Stepper.v0.0.2p0
Stepper.v0.0.3p0
Stepper.v0.0.4p0

############## Recommended Firmware Updates ##############
HELLO-WACC                | At most recent version with Wacc.v0.0.1p0 
HELLO-MOTOR-LIFT          | At most recent version with Stepper.v0.0.4p0 
HELLO-PIMU                | At most recent version with Pimu.v0.0.1p0 
HELLO-MOTOR-ARM           | At most recent version with Stepper.v0.0.4p0 
HELLO-MOTOR-LEFT-WHEEL    | At most recent version with Stepper.v0.0.4p0 
HELLO-MOTOR-RIGHT-WHEEL   | At most recent version with Stepper.v0.0.4p0

```

## Appendix: Software Installation

### Install Arduino CLI

Check if the [Arduino Cli](https://github.com/arduino/arduino-cli) tool is already installed with version 1.6.21 of the SAMD package:

```bash
>>$ arduino-cli core list
ID               Installed Latest Name                                        
arduino:samd     1.6.21    1.6.21 Arduino SAMD Boards (32-bits ARM Cortex-M0+)
```

If not:

```bash
>>$ cd ~/repos
>>$ git clone https://github.com/hello-robot/stretch_install
>>$ cd stretch_install/factory
>>$ ./stretch_install_arduino.sh
```

### Install Firmware Update Tool

Check if the firmware update tool is installed:

```bash
>>$ RE1_firmware_updater.py
RE1_firmware_updater.py --help
usage: RE1_firmware_updater.py [-h] [--status]
                               [--update | --update_to | --update_to_branch | --mgmt]
                               [--pimu] [--wacc] [--arm] [--lift]
                               [--left_wheel] [--right_wheel]

...
```

If not:

```bash
>>$ pip install --upgrade hello-robot-stretch-factory
```

