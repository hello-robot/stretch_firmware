![](../images/banner.png)

# Integrating Custom Hardware

These tutorials demonstrate how to use the Stretch wrist expansion header to integrate custom hardware into [Stretch Body](https://docs.hello-robot.com/stretch_body_guide/).



**NOTE** It is possible to brick the Wacc board by incorrectly configuring the hardware peripherals of the SAMD uC. Therefore, when integrating your custom hardware into the Wacc we strongly recommend emulating the Wacc board until the functionality is complete. The tutorial [Wacc Emulation](./wacc_emulation.md) describes how to configure an Adafruit Metro M0 Express to behave as a stand-in for a Wacc board.



| Tutorial                                          | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ |
| [Data Transfer](./data_transfer.md)          | How to plumb your custom data from the wrist Arduino through Stretch Body |
| [Wrist Board Emulation](./wacc_emulation.md) | How to emulate the Stretch wrist board using an off the shelf Arduino |
| [SPI Sensor](./spi_sensor.md)                | How to integrate an SPI sensor                               |
| [I2C Sensor](./i2c_sensor.md)                | How to integrate an I2C sensor                               |
| [Serial Sensor](./serial_sensor.md)          | How to integrate a Serial UART sensor                        |

**Note: These tutorials may require the latest version of Stretch Body. If necessary, please update your install:**

```bash
>>$ python -m pip install --upgrade hello-robot-stretch-body
```

