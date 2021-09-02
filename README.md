# Rust STM32F4 / Synopsis usbhost

I am building a crate to support the USB host feature of the Synopsis-OTG peripherals
that can be found in STM32F4, GD32VF103 and lots of other microcontrollers.

[A usb-device implementation](https://crates.io/crates/synopsys-usb-otg) already exists,
but only allows to use the chip as a usb device.

This may or may not support [usb-host](https://crates.io/crates/usb-host) some day,
or I may send patches for usb-host if required. We will see.

## Current status

Not at all usable yet. I am developing this on a STM32F411 and currently I do not care
about any other chip. These OTG peripherals come with different feature sets, so this
may or may not cause issues. Patches are currently not, but will be welcome.

What works:

  - Initializing the peripheral.
  - Detecting connect and disconnect events.
  - Setting the device address to a hard-coded value.

What does not yet work:

  - Retrieving descriptors.
  - Doing actual communication.
  - Usable API.
  - Integration with the usb-host crate


