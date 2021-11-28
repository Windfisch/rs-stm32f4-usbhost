# Rust STM32F4 / Synopsis usbhost

I am building a crate to support the USB host feature of the Synopsis-OTG peripherals
that can be found in STM32F4, GD32VF103 and lots of other microcontrollers.

[A usb-device implementation](https://crates.io/crates/synopsys-usb-otg) already exists,
but only allows to use the chip as a usb device.

This may or may not support [usb-host](https://crates.io/crates/usb-host) some day,
or I may send patches for usb-host if required. We will see.

## Current status

I have a working proof-of-concept that can receive USB-MIDI on a WeAct STM32F411 "black
pill" board.

Try building it in release mode and then flash the binary onto your black pill board. Then
plug in a Novation Launchpad X or any other MIDI device into a correctly-wired USB port.

**Note:** While you *could* use the board's USB-C port for this, the port does not provide
power to the downstream device. You either can bridge the small diode near the USB port,
use a USB MIDI device which is self-powered (and not bus-powered), or solder your own USB-A
socket to the appropriate pins.

I am developing this on a STM32F411 and currently I do not care
about any other chip. These OTG peripherals come with different feature sets, so this
may or may not cause issues. Patches and discussion are welcome.

What works:

  - Initializing the peripheral.
  - Detecting connect and disconnect events.
  - Device enumeration consisting of:
    - Setting the device address to a hard-coded value.
    - Retrieving descriptors.
    - Setting the desired configuration.
  - Doing actual communication using IN and OUT endpoints.
  - Receiving USB-MIDI messages as a demonstration.

What needs to be done yet:

  - Good API.
  - Integration with the usb-host crate.


