### Install - packages
apt-get install build-essential libnewlib-dev gcc-riscv64-unknown-elf libusb-1.0-0-dev libudev-dev gdb-multiarch

### Install - Visual Studio Code 
https://code.visualstudio.com/docs/setup/linux

### Install - Platform IO 
https://platformio.org/install/ide?install=vscode

### Install - CH32V-Platform
https://github.com/Community-PIO-CH32V/ch32-pio-projects?tab=readme-ov-file#installing-the-ch32v-platform

To build firmware.bin and firmware.elf select 
> PlatformIO > PROJECT TASKS > Default > Advanced > Verbose build

Usage
===================================

I2C address - 0x33

Registers
-----------------------------------
0x00 - API version (major)
0x01 - API version (minor)
0x02 - Pin select
0x03 - GPIO config
       0x00 - output open drain
       0x01 - output push pull
       0x80 - input float
       0x81 - input pull down
       0x82 - input pull up
0x04 - Data
0x05 - Pins as bits MSB 
0x06 - Pins as bits LSB 
0x07 - BATSENS in percent 
0x08 - BATSENS in mV MSB 
0x09 - BATSENS in mV LSB 

All pins are initally configured as inputs pull down.
NOTE: Valid pins are from 3 to 14. 
NOTE: Virtual pin 15 is PWRSENS.
NOTE: VBAT range 3.2V - 4.2V => BATSENS range 0.89V - 1.14V

To change configuration:
------------------------
1. Write to register 0x02 (Pin select) pin you want to configure
2. Write to register 0x03 (GPIO config) desired config value

To get pin state:
-----------------
1. Write to register 0x02 (Pin select) pin you want to check
2. Read register 0x04 (Data)
If the pin is configured as input (default) readed value is current pin state
If the pin is configured as output the value is last set state

To change pin state:
--------------------
1. Write to register 0x02 (Pin select) pin you want to change
2. Write to register 0x04 (Data) desired new state
If the pin is configured as input (default) nothing will be done
If the pin is configured as output the pin state will be set

Alternative method:
--------------------
You can use registers 0x05 and 0x06 to get/set pin state at once
Register 0x05 contains MSB - pins 15 - 8
Register 0x06 contains LSB - pins  7 - 0

NOTE: Valid pins are from 3 to 14.
NOTE: Virtual pin 15 is PWRSENS.
