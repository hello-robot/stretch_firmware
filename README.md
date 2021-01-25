![](./images/banner.png)

# Overview

The stretch_firmware repository provides the Arduino based firmware for the Stretch robot. The user is not typically expected to modify the firmware. However, it may occasionally be necessary to update the Stretch firmware while doing a Hello Robot recommended software upgrade. 

The repository includes: 

* hello_stepper: firmware for stepper motor controller based on the Mechaduino project
* hello_pimu:  firmware for power and imu board (Pimu) in the base
* hello_wacc: firmware for wrist accelerometer board (Wacc) in the wrist 
* Hello_Serial_Transport: serial communication library
* [Tutorials](tutorials/README.md) on how to integrate custom hardware on to the Stretch Wacc board

Additional 3rd party libraries are provided for convenience.

## License

For details, see the LICENSE.md file in the root directory. All materials within this repository are licensed with the [GNU General Public License v3.0 (GNU GPLv3)](https://www.gnu.org/licenses/gpl-3.0.html) except where other third-party licenses must apply.  

We thank people who have contributed to this work via open-source code and open hardware. We especially thank the [Mechaduino](https://tropical-labs.com/mechaduino/) project and [Tropical Labs](https://tropical-labs.com/). The motor controller firmware and hardware are derived from the excellent [firmware](https://github.com/jcchurch13/Mechaduino-Firmware) and [hardware](https://github.com/jcchurch13/Mechaduino-Hardware) created for the Mechaduino project by Tropical Labs.

# Installation

## Clone the repository

```
cd ~/repos
git clone https://github.com/hello-robot/stretch_firmware
```

## Install the Arduino Command Line Interface

First, test if the Arduino command line tool is already installed. 

```bash
>>$ arduino-cli
Arduino Command Line Interface (arduino-cli).

Usage:
  arduino-cli [command]
...
```

If it isn't installed already, install arduino-cli and configure it:

```bash
>>$ cd ~/
>>$ curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh
>>$ arduino-cli config init
>>$ arduino-cli core install arduino:samd@1.6.21
```

Then modify the file `~/.arduino15/arduino-cli.yaml` and set the `user` field to point to  Stretch Firmware:

```
>>$ emacs ~/.arduino15/arduino-cli.yaml

  user: /home/hello-robot/repos/stretch_firmware/arduino

```

Now test that the install is working:

```bash
>>$arduino-cli compile --fqbn hello-robot:samd:hello_pimu ~/repos/stretch_firmware/arduino/hello_pimu

Sketch uses 40904 bytes (15%) of program storage space. Maximum is 262144 bytes.
```

You may also uses the Arduino IDE to update the Stretch Firmware. Instructions are found [here](./tutorial/Arduino_IDE_install.md).

# Updating Stretch Firmware

**Warning: Firmware upgrade should only be done at the recommendation of Hello Robot support.** 

First, update the stretch_firmware repository to the lastest version.

```bash
>>$ cd ~/repos/stretch_firmware
>>$ git pull
```

Next, update your stretch_body to the latest version if it isn't already.

```
>>$ pip2 install hello-robot-stretch-body
>>$ pip2 install hello-robot-stretch-factory
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

Stretch Firmware includes a command line tool for compiling and uploading firmware updates:

```bash
~/repos/stretch_firmware/python/stretch_firmware_upload.py --help
usage: stretch_firmware_upload.py [-h] [--pimu] [--wacc] [--arm] [--lift] [--left_wheel] [--right_wheel]

Upload Stretch firmware to microcontrollers

optional arguments:
  -h, --help     show this help message and exit
  --pimu         Upload Pimu firmware
  --wacc         Upload Wacc firmware
  --arm          Upload Arm Stepper firmware
  --lift         Upload Lift Stepper firmware
  --left_wheel   Upload Left Wheel Stepper firmware
  --right_wheel  Upload Right Wheel Stepper firmware

```



## Update the Pimu

Compile and then upload the firmware for the Pimu board.

```bash
>>$  ~/repos/stretch_firmware/python/stretch_firmware_upload.py --pimu
Found Pimu at ttyACM3
Uploading firmware to hello_pimu
---------------Compile-------------------------
Sketch uses 40904 bytes (15%) of program storage space. Maximum is 262144 bytes.
---------------Upload-------------------------
Atmel SMART device 0x10010005 found
Device       : ATSAMD21G18A
Chip ID      : 10010005
Version      : v2.0 [Arduino:XYZ] Dec 20 2016 15:36:39
Address      : 8192
Pages        : 3968
Page Size    : 64 bytes
Total Size   : 248KB
Planes       : 1
Lock Regions : 16
Locked       : none
Security     : false
Boot Flash   : true
BOD          : true
BOR          : true
Arduino      : FAST_CHIP_ERASE
Arduino      : FAST_MULTI_PAGE_WRITE
Arduino      : CAN_CHECKSUM_MEMORY_BUFFER
Erase flash
done in 0.785 seconds

Write 41288 bytes to flash (646 pages)
[==============================] 100% (646/646 pages)
done in 0.193 seconds

Verify 41288 bytes of flash with checksum.
Verify successful
done in 0.041 seconds
CPU reset.
---------------Next Steps-------------------------
Test the Pimu using: stretch_pimu_jog.py
```

Next test that the Pimu is working by causing it to beep from the tool menu:

```bash
>>$ stretch_pimu_jog.py 
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.

------ MENU -------
m: menu
i: reset imu
f: toggle fan
b: toggle buzzer
p: beep
s: trigger status sync
t: trigger motor sync
k: disable sync mode
l: enable sync mode
r: reset board
x: reset runstop event
o: trigger runstop event
z: zero clock
y: reset cliff event
-------------------
p

```

## Update the Wacc

Upload the firmware for the Wacc board. 

```bash
>>$ ~/repos/stretch_firmware/python/stretch_firmware_upload.py --wacc
```

Next test that the Wacc is working by checking that the accelerometer reports AX as near 9.8:

```bash
>>$ 
stretch_wacc_jog.py 
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.

------ MENU -------
m: menu
r: reset board
a: set D2 on
b: set D2 off
c: set D3 on
d: set D3 off
s: trigger status sync
k: disable sync mode
l: enable sync mode
z: zero clock
-------------------

------------------------------
Ax (m/s^2) 9.61330890656
...
```

## Update the Steppers

### Arm

Upload the firmware for the Arm stepper board. Afterwards push the encoder calibration data to its flash memory.

```bash
>>$ ~/repos/stretch_firmware/python/stretch_firmware_upload.py --arm
...
>>$ RE1_stepper_calibration_YAML_to_flash.py hello-motor-arm
Reading calibration data from YAML...
Writing calibration data to flash...
Writing encoder calibration...
Sending page 0 of 255
...
Sending page 255 of 255
Successful write of FLASH. Resetting board now.
```

Test that the arm is working by homing it. 

```bash
>>$  stretch_arm_home.py
```

### Lift

**Note: If powered the lift may drop during firmware upload. Apply a clamp to the mast below the shoulder prior to running the scripts.**

**Note: When pushing the calibration data to the lift the motor will make a brief clanking noise. This is normal.**

Upload the firmware for the Lift stepper board. Afterwards push the encoder calibration data to its flash memory.

```bash
>>$ ~/repos/stretch_firmware/python/stretch_firmware_upload.py --lift
>>$ RE1_stepper_calibration_YAML_to_flash.py hello-motor-lift
```

Test that the lift is working by homing it. 

```bash
>>$  stretch_lift_home.py
```

### Wheels

Upload the firmware for the base wheels. Afterwards push the encoder calibration data to their flash memory.

```bash
>>$ ~/repos/stretch_firmware/python/stretch_firmware_upload.py --left_wheel
>>$ ~/repos/stretch_firmware/python/stretch_firmware_upload.py --right_wheel
>>$ RE1_stepper_calibration_YAML_to_flash.py hello-motor-left-wheel
>>$ RE1_stepper_calibration_YAML_to_flash.py hello-motor-right-wheel
```

Test that the base is working by jogging it forward and back using the `f` and `b` commands.

```bash
>>$ stretch_base_jog.py 
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.

--------------
m: menu

1: rate slow
2: rate default
3: rate fast
4: rate max
w: CW/CCW 90 deg
x: forward-> back 0.5m
y: spin at 22.5deg/s

f / b / l / r : small forward / back / left / right
F / B / L / R : large forward / back / left / right
o: freewheel
p: pretty print
q: quit

Input?
```

### 
