#!/usr/bin/env python3

print('''# Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
''')

mcu_dict = {
    'SAMD21': {
        'flash_size': 262144,
        'data_size': 0,
        'offset': '0x2000',
        'build_mcu': 'cortex-m0plus',
        'f_cpu': '48000000L',
        'extra_flags': '-DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS',
        'openocdscript': 'scripts/openocd/daplink_samd21.cfg',
    },

    'SAMD51': {
        'flash_size': 507904, # SAMD51P20A and SAMD51J20A has 1032192
        'data_size': 0,
        'offset': '0x4000',
        'build_mcu': 'cortex-m4',
        'f_cpu': '120000000L',
        'extra_flags': '-D__SAMD51__ -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16',
        'openocdscript': 'scripts/openocd/daplink_samd51.cfg',
    },

    'SAME51': {
        'flash_size': 507904,
        'data_size': 0,
        'offset': '0x4000',
        'build_mcu': 'cortex-m4',
        'f_cpu': '120000000L',
        'extra_flags': '-D__SAMD51__ -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16',
        'openocdscript': 'scripts/openocd/daplink_samd51.cfg',
    },
}


def build_header(mcu, name, vendor, product, vid, pid_list):
    prettyname = f"{vendor} {product} ({mcu})"
    print()
    print("# -----------------------------------")
    print(f"# {prettyname}")
    print("# -----------------------------------")
    print(f"{name}.name={prettyname}")
    print()

    print("# VID/PID for Bootloader, Arduino & CircuitPython")
    for i in range(len(pid_list)):
        print(f"{name}.vid.{i}={vid}")
        print(f"{name}.pid.{i}={pid_list[i]}")
    print()


def build_upload(mcu, name, extra_flags):
    print("# Upload")    
    print(f"{name}.upload.tool=bossac18")
    print(f"{name}.upload.protocol=sam-ba")
    
    if ('SAMD51P20A' in extra_flags) or ('SAMD51J20A' in extra_flags):
        flash_size = 1032192
    else:
        flash_size = mcu_dict[mcu]['flash_size']
    print(f"{name}.upload.maximum_size={flash_size}")
    #print(f"{name}.upload.maximum_data_size={mcu_dict[mcu]['data_size']}")
    
    print(f"{name}.upload.offset={mcu_dict[mcu]['offset']}")
    print(f"{name}.upload.use_1200bps_touch=true")
    print(f"{name}.upload.wait_for_upload_port=true")
    print(f"{name}.upload.native_usb=true")
    print()


def build_build(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader):
    mcu_properties = mcu_dict[mcu]

    print("# Build")
    print(f"{name}.build.mcu={mcu_properties['build_mcu']}")
    print(f"{name}.build.f_cpu={mcu_properties['f_cpu']}")
    print(f'{name}.build.usb_product="{product}"')
    print(f'{name}.build.usb_manufacturer="{vendor}"')
    print(f"{name}.build.board={boarddefine}")
    print(f"{name}.build.core=arduino")

    # Due to fastLed issue https://github.com/FastLED/FastLED/issues/1363
    # although there is a simple fix already https://github.com/FastLED/FastLED/pull/1424
    # fastLED is not well maintained, and we need to skip ARDUINO_SAMD_ZERO for affected boards
    # in the long run we should move all of our libraries away from ARDUINO_SAMD_ZERO
    if variant in [ 'gemma_m0', 'trinket_m0', 'qtpy_m0', 'itsybitsy_m0' ]:
        print(f"{name}.build.extra_flags={extra_flags} -DARM_MATH_CM0PLUS {{build.usb_flags}}")
    else:
        print(f"{name}.build.extra_flags={extra_flags} {mcu_properties['extra_flags']} {{build.usb_flags}}")

    print(f"{name}.build.ldscript=linker_scripts/gcc/flash_with_bootloader.ld")
    print(f"{name}.build.openocdscript={mcu_properties['openocdscript']}")
    print(f"{name}.build.variant={variant}")
    print(f"{name}.build.variant_system_lib=")
    print(f"{name}.build.vid={vid}")
    print(f"{name}.build.pid={pid_list[0]}")
    print(f"{name}.bootloader.tool=openocd")
    print(f"{name}.bootloader.file={bootloader}")
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print(f'{name}.compiler.arm.cmsis.ldflags="-L{{runtime.tools.CMSIS-5.4.0.path}}/CMSIS/Lib/GCC/" "-L{{build.variant.path}}" -larm_cortexM4lf_math -mfloat-abi=hard -mfpu=fpv4-sp-d16')
    print()
    

def build_menu(mcu, name):
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print("# Menu: Cache")
        print(f"{name}.menu.cache.on=Enabled")
        print(f"{name}.menu.cache.on.build.cache_flags=-DENABLE_CACHE")
        print(f"{name}.menu.cache.off=Disabled")
        print(f"{name}.menu.cache.off.build.cache_flags=")
        print()

        print("# Menu: Speed")
        print(f"{name}.menu.speed.120=120 MHz (standard)")
        print(f"{name}.menu.speed.120.build.f_cpu=120000000L")
        print(f"{name}.menu.speed.150=150 MHz (overclock)")
        print(f"{name}.menu.speed.150.build.f_cpu=150000000L")
        print(f"{name}.menu.speed.180=180 MHz (overclock)")
        print(f"{name}.menu.speed.180.build.f_cpu=180000000L")
        print(f"{name}.menu.speed.200=200 MHz (overclock)")
        print(f"{name}.menu.speed.200.build.f_cpu=200000000L")
        print()

    print("# Menu: Optimization")
    print(f"{name}.menu.opt.small=Small (-Os) (standard)")
    print(f"{name}.menu.opt.small.build.flags.optimize=-Os")
    print(f"{name}.menu.opt.fast=Fast (-O2)")
    print(f"{name}.menu.opt.fast.build.flags.optimize=-O2")
    print(f"{name}.menu.opt.faster=Faster (-O3)")
    print(f"{name}.menu.opt.faster.build.flags.optimize=-O3")
    print(f"{name}.menu.opt.fastest=Fastest (-Ofast)")
    print(f"{name}.menu.opt.fastest.build.flags.optimize=-Ofast")
    print(f"{name}.menu.opt.dragons=Here be dragons (-Ofast -funroll-loops)")
    print(f"{name}.menu.opt.dragons.build.flags.optimize=-Ofast -funroll-loops")
    print()
    
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print("# Menu: QSPI Speed")
        print(f"{name}.menu.maxqspi.50=50 MHz (standard)")
        print(f"{name}.menu.maxqspi.50.build.flags.maxqspi=-DVARIANT_QSPI_BAUD_DEFAULT=50000000")
        print(f"{name}.menu.maxqspi.fcpu=CPU Speed / 2")
        print(f"{name}.menu.maxqspi.fcpu.build.flags.maxqspi=-DVARIANT_QSPI_BAUD_DEFAULT=({{build.f_cpu}})")
        print()

    print("# Menu: USB Stack")
    print(f"{name}.menu.usbstack.arduino=Arduino")
    print(f"{name}.menu.usbstack.tinyusb=TinyUSB")
    print(f"{name}.menu.usbstack.tinyusb.build.flags.usbstack=-DUSE_TINYUSB")
    print()

    print("# Menu: Debug")
    print(f"{name}.menu.debug.off=Off")
    print(f"{name}.menu.debug.on=On")
    print(f"{name}.menu.debug.on.build.flags.debug=-g")
    print()

    # comment out for now since debugger selection does not work, debug does not pickup the right openocd script
    # print("# Menu: Debugger")
    # script_mcu = 'samd21' if mcu == 'SAMD21' else 'samd51'
    # print(f"{name}.menu.debugger.daplink=CMSIS-DAP (DAPLink)")
    # print(f"{name}.menu.debugger.daplink.build.openocdscript=scripts/openocd/daplink_{script_mcu}.cfg")
    # print(f"{name}.menu.debugger.jlink=J-Link")
    # print(f"{name}.menu.debugger.jlink.build.openocdscript=scripts/openocd/jlink_{script_mcu}.cfg")


def build_global_menu():
    print("menu.cache=Cache")
    print("menu.speed=CPU Speed")
    print("menu.opt=Optimize")
    print("menu.maxqspi=Max QSPI")    
    print("menu.usbstack=USB Stack")
    print("menu.debug=Debug")
    #print("menu.debugger=Debugger")


def make_board(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader):
    build_header(mcu, name, vendor, product, vid, pid_list)
    build_upload(mcu, name, extra_flags)
    build_build(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader)    
    build_menu(mcu, name)


# ------------------------------
# main
# ------------------------------

build_global_menu()

d51_board_list = [

    ["adafruit_feather_m4", "feather_m4", "Adafruit", "Feather M4 Express",
     "0x239A", ["0x8022", "0x0022", "0x8026"],
     "FEATHER_M4", "-D__SAMD51J19A__ -DADAFRUIT_FEATHER_M4_EXPRESS",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["hello_robot_hello_pimu", "hello_pimu", "Hello-Robot", "Hello_Pimu",
     "0x239A", ["0x8022", "0x0022", "0x8026"],
     "HELLO_PIMU", "-D__SAMD51J19A__ -DHELLOROBOT_HELLO_PIMU",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],
]

for b in d51_board_list:
    make_board("SAMD51", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8])
