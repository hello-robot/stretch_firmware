![](./images/banner.png)

# Updating Stretch Firmware


Stretch has 6 Arduino based microcontroller boards that may require a firmware update:

| Board               | USB Device                   | Arduino Sketch    |
| ------------------- | ---------------------------- | ----------------- |
| Left wheel stepper  | /dev/hello-motor-left-wheel  | hello_stepper.ino |
| Right wheel stepper | /dev/hello-motor-right-wheel | hello_stepper.ino |
| Lift stepper        | /dev/hello-motor-lift        | hello_stepper.ino |
| Arm stepper         | /dev/hello-motor-arm         | hello_stepper.ino |
| Pimu                | /dev/hello-pimu              | hello_pimu.ino    |
| Wacc                | /dev/hello-wacc              | hello_wacc.ino    |

Firmware updates are managed through the [REx_firmware_updater.py](https://github.com/hello-robot/stretch_factory/blob/master/python/bin/REx_firmware_updater.py) tool, which uses the [Arduino Cli](https://github.com/arduino/arduino-cli) tool to flash the firmware.

Before doing an upgrade first ensure the latest Stretch Body is installed as well as Stretch Factory

```bash
>>$ pip install  hello-robot-stretch-factory --upgrade --no-cache-dir
>>$ pip install  hello-robot-stretch-body --upgrade --no-cache-dir
```
## Firmware Update Tool

The firmware update tool will automatically recommend and install the latest version of firmware found on GitHub. It will only recommend versions that are compatible with the currently installed Stretch Body.

```bash
>>$ REx_firmware_updater.py --help
usage: REx_firmware_updater.py [-h]
                               [--current | --available | --recommended | --install | --install_version | --install_branch | --install_path INSTALL_PATH | --mgmt]
                               [--pimu] [--wacc] [--arm] [--lift]
                               [--left_wheel] [--right_wheel]

Upload Stretch firmware to microcontrollers

optional arguments:
  -h, --help            show this help message and exit
  --current             Display the currently installed firmware versions
  --available           Display the available firmware versions
  --recommended         Display the recommended firmware
  --install             Install the recommended firmware
  --install_version     Install a specific firmware version
  --install_branch      Install the HEAD of a specific branch
  --install_path        INSTALL_PATH Install the firmware on the provided path (eg ./stretch_firmware/arduino)
  --mgmt                Display overview on firmware management
  --pimu                Upload Pimu firmware
  --wacc                Upload Wacc firmware
  --arm                 Upload Arm Stepper firmware
  --lift                Upload Lift Stepper firmware
  --left_wheel          Upload Left Wheel Stepper firmware
  --right_wheel         Upload Right Wheel Stepper firmware
```
## Updating to the Latest Firmware
To update to the latest version of firmware:

```bash
>>$ REx_firmware_updater.py --install
Collecting information...
######################################## Recommended Firmware Updates ########################################


DEVICE                    | INSTALLED                 | RECOMMENDED               | ACTION                    
--------------------------------------------------------------------------------------------------------------
HELLO-WACC                | Wacc.v0.0.2p1             | Wacc.v0.0.1p0             | Downgrade recommended     
HELLO-MOTOR-LIFT          | Stepper.v0.1.0p1          | Stepper.v0.0.4p0          | Downgrade recommended     
HELLO-PIMU                | Pimu.v0.0.2p1             | Pimu.v0.0.1p0             | Downgrade recommended     
HELLO-MOTOR-ARM           | Stepper.v0.1.0p1          | Stepper.v0.0.4p0          | Downgrade recommended     
HELLO-MOTOR-LEFT-WHEEL    | Stepper.v0.1.0p1          | Stepper.v0.0.4p0          | Downgrade recommended     
HELLO-MOTOR-RIGHT-WHEEL   | Stepper.v0.1.0p1          | Stepper.v0.0.4p0          | Downgrade recommended   
######################################### UPDATING FIRMWARE TO... ###########################################
HELLO-WACC                | Downgrading to Wacc.v0.0.1p0             
HELLO-MOTOR-LEFT-WHEEL    | Downgrading to Stepper.v0.0.4p0          
HELLO-MOTOR-RIGHT-WHEEL   | Downgrading to Stepper.v0.0.4p0          
HELLO-MOTOR-LIFT          | Downgrading to Stepper.v0.0.4p0          
HELLO-PIMU                | Downgrading to Pimu.v0.0.1p0             
HELLO-MOTOR-ARM           | Downgrading to Stepper.v0.0.4p0          
------------------------------------------------
WARNING: (1) Updating robot firmware should only be done by experienced users
WARNING: (2) Do not have other robot processes running during update
WARNING: (3) Leave robot powered on during update
WARNING: (4) Ensure Lift has support clamp in place
WARNING: (5) Lift may make a loud noise during programming. This is normal.
Proceed with update?? [y/N]: 
```
Review the recommendations and warnings before proceeding ('y'). If you prefer to only update one or more boards you can specify it on the command line. For example:
```bash
REx_firmware_updater.py --install --arm --wacc
```
## Other Useful Commands

Running  `REx_firmware_updater.py --current` will report the currently install firmware: 
```bash
>>$REx_firmware_updater.py --current
######################################## Currently Installed Firmware ########################################
------------ HELLO-WACC ------------
Installed Firmware: Wacc.v0.0.2p1
Installed Stretch Body supports protocols: p0 , p1
Installed protocol p1 : VALID
------------ HELLO-MOTOR-LIFT ------------
Installed Firmware: Stepper.v0.1.0p1
Installed Stretch Body supports protocols: p0 , p1
Installed protocol p1 : VALID
------------ HELLO-PIMU ------------
Installed Firmware: Pimu.v0.0.2p1
Installed Stretch Body supports protocols: p0 , p1
Installed protocol p1 : VALID
------------ HELLO-MOTOR-ARM ------------
Installed Firmware: Stepper.v0.1.0p1
Installed Stretch Body supports protocols: p0 , p1
Installed protocol p1 : VALID
------------ HELLO-MOTOR-LEFT-WHEEL ------------
Installed Firmware: Stepper.v0.1.0p1
Installed Stretch Body supports protocols: p0 , p1
Installed protocol p1 : VALID
------------ HELLO-MOTOR-RIGHT-WHEEL ------------
Installed Firmware: Stepper.v0.1.0p1
Installed Stretch Body supports protocols: p0 , p1
Installed protocol p1 : VALID

```

Running  `REx_firmware_updater.py --available` will list the available versions on GitHub:

```bash
>>$ REx_firmware_updater.py --available
Collecting information...
####################### Currently Tagged Versions of Stretch Firmware on Master Branch #######################
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
```



------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks. The Stretch RE1 and RE2 robots are covered by U.S. Patent 11,230,000 and other patents pending.</div>

