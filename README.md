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

## "Screenshots"

There are no screenshots on an embedded device without a screen, duh! But let me copy-paste
the debug output that comes in via USART1 (PA9/10) at 115200 baud:

```
hello world!
setting PPWR and waiting for PCDET
got PCDET!
resetting the port and waiting for PENCHNG
enumerated speed is 1
gintsts = 05000021
hprt = 0002140d
hprt = 0002140d
hprt 0002140d01b2 00080040
#436, penchng, port enabled is true
0 / 0 (1023)
18 / 18 (0)
Descriptor: [12, 01, 10, 02, 00, 00, 00, 40, 35, 12, 12, 01, 00, 02, 01, 02, 03, 01]; max packet size on ep0 is 64
9 / 9 (0)
64 / 126 (1)
126 / 126 (0)
Config descriptor: [09, 02, 7E, 00, 03, 01, 04, 80, FA, 09, 04, 00, 00, 00, 01, 01, 00, 05, 09, 24, 01, 00, 01, 09, 00, 01, 01, 09, 04, 01, 00, 02, 01, 03, 00, 06, 07, 24, 01, 00, 01, 43, 00, 06, 24, 02, 01, 01, 07, 09, 24, 03, 01, 02, 01, 01, 01, 08, 06, 24, 02, 01, 03, 09, 09, 24, 03, 01, 04, 01, 03, 01, 0A, 09, 05, 01, 02, 40, 00, 01, 00, 00, 06, 25, 01, 02, 01, 03, 09, 05, 81, 02, 40, 00, 01, 00, 00, 06, 25, 01, 02, 02, 04, 09, 04, 02, 00, 02, 08, 06, 50, 0B, 07, 05, 82, 02, 40, 00, 00, 07, 05, 02, 02, 40, 00, 00]
0 / 0 (1023)

======== Configuration #1 with 3 interfaces ========


==== Interface #0 ====
CS_INTERFACE

==== Interface #1 ====
CS_INTERFACE
midi in jack #1 (1)
midi out jack #2 (1)
midi in jack #3 (1)
midi out jack #4 (1)
endpoint #01
  endpoint has 2 embedded midi jacks
  0 -> jack 1
  1 -> jack 3
endpoint #81
  endpoint has 2 embedded midi jacks
  0 -> jack 2
  1 -> jack 4

==== Interface #2 ====
endpoint #82
endpoint #02
4 / 64 (0)
received: [19, 90, 3C, 20]
4 / 64 (0)
received: [1A, A0, 3C, 5C]
4 / 64 (0)
received: [1A, A0, 3C, 72]
4 / 64 (0)
received: [1A, A0, 3C, 78]
4 / 64 (0)
received: [1A, A0, 3C, 7C]
4 / 64 (0)
received: [1A, A0, 3C, 7F]
4 / 64 (0)
received: [1A, A0, 3C, 7E]
4 / 64 (0)
received: [1A, A0, 3C, 67]
4 / 64 (0)
received: [1A, A0, 3C, 0E]
8 / 64 (0)
received: [1A, A0, 3C, 00, 19, 90, 3C, 00]
```
