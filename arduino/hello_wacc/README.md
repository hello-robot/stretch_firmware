# Stretch RE1 - Wacc Firmware

The Stretch RE1 Wacc Firmware runs on the Wacc PCBA found inside the robot arm. Wacc stands for the 'Wrist + Accelerometer' board. The Wacc firmware provides

* Monitoring of the wrist accelerometer
* Monitoring the Arduino expansion header in the wrist

The Wacc expansion header is electrically routed to potentially provide

* Analog in (x1)
* Digital in (x1)
* Digital out (x2)
* Serial SPI 
* Serial I2C 
* Serial UART

See the [Stretch RE1 Hardware User Guide](https://hello-robot.github.io/hardware_user_guide/) for pin-out, electrical, and connector information.

## Using the Expansion DIO Header

### Factory Interface

By default the Expansion DIO header is configured in firmware to provide two digital inputs, two digital outputs, and an analog input. This can be seen in the [Wacc_status](https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_wacc/Common.h) structure.

```c
struct __attribute__ ((packed)) Wacc_Status{
  float ax;	//Accelerometer AX
  float ay;	//Accelerometer AY
  float az;	//Accelerometer AZ
  int16_t a0; //expansion header analog in
  uint8_t d0; //expansion header digital in
  uint8_t d1; //expansion header digital in
  uint8_t d2; //expansion header digital out
  uint8_t d3; //expansion header digital out
  uint32_t single_tap_count; //Accelerometer tap count
  uint32_t state;
  uint32_t timestamp; //ms, overflows every 50 days
  uint32_t debug;
};
```

The user can interact with these pins through the [Stretch Body python interface](https://hello-robot.github.io/stretch_body_guide/). 

### Custom Interface

Advanced users may want to create a custom interface to the Expansion DIO header that utilizes SPI, I2C, or UART. For example, interfacing a 3rd party sensor that communicates by SPI requires:

* Create a local version of the firmware that the user can modify.
* Create a new Arduino board 'variant' which maps the SPI hardware to the DIO pins
* Modify the firmware to include the sensors SPI driver, allowing the sensor to be read / written.
* Modify the [Wacc_Status and Wacc_Command]() structures so that the data is piped back to Stretch Body python
* Create a local version of the [stretch_body/wacc.py](stretch_body/wacc.py) that the user can modify.
* Update Status and Command interfaces of wacc.py to accommodate the new firmware data structures
* Expose the sensor data and commands through the wacc.py API

While non-trivial, developing a custom interface is straightforward for an advanced user who is experienced with firmware, Python, and Arduino development. 