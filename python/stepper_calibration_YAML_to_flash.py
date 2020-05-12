#!/usr/bin/env python
import sys
import stretch_body.stepper as stepper



if len(sys.argv) < 2:
    raise Exception("Provide motor name e.g.: stepper_flash_calibration_from_YAML.py hello-motor1")
motor_name = sys.argv[1]

motor = stepper.Stepper('/dev/'+motor_name)
motor.startup()
print 'Reading calibration data from YAML...'
data=motor.read_encoder_calibration_from_YAML()
print 'Writing calibration data to flash...'
motor.write_encoder_calibration_to_flash(data)
print 'Successful write of FLASH. Resetting board now.'
motor.board_reset()
motor.push_command()

