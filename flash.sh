#!/bin/bash
cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/blinky blinky.bin
dfu-util -D blinky.bin -s 0x08000000 --alt 0
