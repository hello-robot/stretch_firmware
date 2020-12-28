#!/usr/bin/env python
import sys
import argparse
import stretch_body.hello_utils as hu
from subprocess import Popen, PIPE

parser=argparse.ArgumentParser(description='Upload Stretch firmware to microcontrollers')
parser.add_argument("--pimu", help="Upload Pimu firmware",action="store_true")
parser.add_argument("--wacc", help="Upload Wacc firmware",action="store_true")
parser.add_argument("--arm", help="Upload Arm Stepper firmware",action="store_true")
parser.add_argument("--lift", help="Upload Lift Stepper firmware",action="store_true")
parser.add_argument("--left_wheel", help="Upload Left Wheel Stepper firmware",action="store_true")
parser.add_argument("--right_wheel", help="Upload Right Wheel Stepper firmware",action="store_true")
args=parser.parse_args()

port_name=None
sketch_name=None
device_name=None

if args.wacc:
    device_name = 'hello-wacc'
    sketch_name = 'hello_wacc'
    port_name=Popen("ls -l /dev/"+device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    print('Found Wacc at %s'%port_name)
    next_steps="Test the Wacc using: stretch_wacc_jog.py"

if args.pimu:
    device_name = 'hello-pimu'
    sketch_name = 'hello_pimu'
    port_name=Popen("ls -l /dev/"+device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    print('Found Pimu at %s'%port_name)
    next_steps = "Test the Pimu using: stretch_pimu_jog.py"

if args.lift:
    device_name = 'hello-motor-lift'
    sketch_name = 'hello_stepper'
    port_name=Popen("ls -l /dev/"+device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    print('Found Lift at %s'%port_name)
    next_steps = "Write the stepper calibration using: RE1_stepper_calibration_YAML_to_flash.py hello-motor-lift"

if args.arm:
    device_name = 'hello-motor-arm'
    sketch_name = 'hello_stepper'
    port_name=Popen("ls -l /dev/"+device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    print('Found Arm at %s'%port_name)
    next_steps = "Write the stepper calibration using: RE1_stepper_calibration_YAML_to_flash.py hello-motor-arm"

if args.left_wheel:
    device_name = 'hello-motor-left-wheel'
    sketch_name = 'hello_stepper'
    port_name=Popen("ls -l /dev/"+device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    print('Found Left Wheel at %s'%port_name)
    next_steps = "Write the stepper calibration using: RE1_stepper_calibration_YAML_to_flash.py hello-motor-left-wheel"

if args.right_wheel:
    device_name = 'hello-motor-right-wheel'
    sketch_name = 'hello_stepper'
    port_name=Popen("ls -l /dev/"+device_name, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().split()[-1]
    print('Found Right Wheel at %s'%port_name)
    next_steps = "Write the stepper calibration using: RE1_stepper_calibration_YAML_to_flash.py hello-motor-right-wheel"

if port_name is not None:
    print('Uploading firmware to %s'%sketch_name)
    print('---------------Compile-------------------------')
    compile_command = 'arduino-cli compile --fqbn hello-robot:samd:%s ~/repos/stretch_firmware/arduino/%s'%(sketch_name,sketch_name)
    upload_command = 'arduino-cli upload -p /dev/%s --fqbn hello-robot:samd:%s ~/repos/stretch_firmware/arduino/%s'%(port_name,sketch_name,sketch_name)
    c=Popen(compile_command, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip()
    print(c)
    print('---------------Upload-------------------------')
    u = Popen(upload_command, shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip()
    print(u)
    print('---------------Next Steps-------------------------')
    print(next_steps)
else:
    print 'Failed to upload...'