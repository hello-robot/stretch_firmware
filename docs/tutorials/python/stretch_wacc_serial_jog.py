#!/usr/bin/env python
from wacc_serial_ext import WaccSerialExt
w=WaccSerialExt()
w.startup()
try:
    while True:
        print('Hit enter to do TX/RX cycle')
        raw_input()
        w.serial_data_increment()
        w.push_command()
        print('TX to SerialExt', w._command['serial_ext'])
        w.pull_status()
        print('RX from SerialExt', w.status['serial_ext'])
except (KeyboardInterrupt, SystemExit):
    w.stop()
