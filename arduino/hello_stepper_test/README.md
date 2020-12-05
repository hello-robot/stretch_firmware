# Stretch RE1 - Stepper Firmware

The Stretch RE1 Stepper Firmware runs on the Stepper PCBA found on the back of the Stretch stepper motors. It is a modified version of the open source [Mechaduino project](https://github.com/jcchurch13/Mechaduino-Firmware), which provides closed loop current control of a stepper motor.

The firmware is organized so as to allow switching, via serial command, between 'stock Mechaduino mode' with its menu interface and the Hello Robot firmware with its RPC interface.

The primary hardware modifications are:

* Higher current capacity by use of parallel motor drivers
* Locking JST connectors
* Addition of a 'sync' line for multi-dof synchronization

The primary software modifications are:

* RPC and serialization layer to manage controller parameters, controller commands, etc

* Trapezoidal trajectory generators based on the library [MotionGenerator](https://github.com/EmanuelFeru/MotionGenerator) 

* Handling of runstop and motor synchronization

* Implementation of a guarded move behavior

* Various controller variants, controller and parameter management functions

  

  



