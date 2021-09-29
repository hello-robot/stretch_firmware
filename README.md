![](./tutorials/images/banner.png)

# Overview
The [Stretch Firmware](https://github.com/hello-robot/stretch_firmware) repository provides the Arduino based firmware for the Stretch robot. 

Minor version updates to Stretch Body may occasionally require the robot's firmware to also be updated.

The repository includes: 

* hello_stepper: firmware for stepper motor controller based on the Mechaduino project
* hello_wacc: firmware for wrist accelerometer board (Wacc) in the wrist 
* hello_pimu:  firmware for power and imu board (Pimu) in the base
* Hello_Serial_Transport: serial communication library
* [Tutorial](./tutorials/docs/updating_firmware.md) on how to update the robot's firmware
* [Tutorials](./tutorials/docs/README.md) on how to integrate custom hardware on to the Stretch Wacc board

## License
For details, see the LICENSE.md file in the root directory. All materials within this repository are licensed with the [GNU General Public License v3.0 (GNU GPLv3)](https://www.gnu.org/licenses/gpl-3.0.html) except where other third-party licenses must apply.  

We thank people who have contributed to this work via open-source code and open hardware. We especially thank the [Mechaduino](https://tropical-labs.com/mechaduino/) project and [Tropical Labs](https://tropical-labs.com/). The motor controller firmware and hardware are derived from the excellent [firmware](https://github.com/jcchurch13/Mechaduino-Firmware) and [hardware](https://github.com/jcchurch13/Mechaduino-Hardware) created for the Mechaduino project by Tropical Labs.
