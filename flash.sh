#!/bin/bash
cargo build --release &&
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/blinky blinky.bin &&
#dfu-util -D blinky.bin -s 0x08000000 --alt 0
stm32flash /dev/ttyUSB0 -b 115200 -w blinky.bin -S 0x8000000:`stat -c '%s' blinky.bin`
