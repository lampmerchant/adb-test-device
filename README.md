# ADB Test Device

A test device for the Apple Desktop Bus.  It behaves as two more-or-less conventional ADB devices under control of another system via UART.

## HOWTO

### Requirements

* PIC
   * Microchip MPASM (bundled with MPLAB)
      * Note that you **must** use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM
   * A PIC12F1840 microcontroller
   * A device to program the PIC with
* Python
   * Python 3.x
   * PySerial

### Steps

* Build the code using Microchip MPASM and download the result into a PIC12F1840.
* Connect the Tx and Rx lines to a UART on a PC.
* Run the Python code.

### Examples

#### Basic

This does very little; DemoDevice is a class made as a very basic demonstration of how to write an AdbDevice subclass.

```
$ python3 -i testdevice.py
>>> import serial
>>> serial_obj = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=1, rtscts=True)  # RTS/CTS is important!
>>> atd = AdbTestDevice(serial_obj, DemoDevice(), DummyDevice())
>>> atd.start()
```

#### Using TelePort A300 Modem Class

This creates an emulated TelePort A300 modem that simply echoes back whatever data is sent to it.

```
$ python3 -i a300.py
>>> import serial
>>> serial_obj = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=1, rtscts=True)  # RTS/CTS is important!
>>> a300 = GlobalVillageA300()
>>> atd = AdbTestDevice(serial_obj, a300, DummyDevice())
>>> atd.start()
```

## Serial Protocol

Uses 8-N-1 at 115200 baud.  Data in both directions is sent in packets with single-byte headers in this format:

| Bits | Description                               |
| ---- | ----------------------------------------- |
| 7    | Device Number (0 or 1)                    |
| 6:4  | Packet Type (see below)                   |
| 3:0  | Packet length (not including header byte) |

### Test Device to UART Host

#### 0x0: Listen 0

This packet represents a Listen 0 command from the Mac, the packet contains the command's payload.

#### 0x1: Listen 1

This packet represents a Listen 1 command from the Mac, the packet contains the command's payload.

#### 0x2: Listen 2

This packet represents a Listen 2 command from the Mac, the packet contains the command's payload.

#### 0x3: Handler Change

This packet is sent when the Mac changes the handler ID of the device to one of the allowable handler IDs; its length is always 1.  The contents are the new handler ID (which is already reflected in its register 3).

#### 0x7: Reset

This packet is sent when the Mac sends a reset condition or a SendReset command on the bus.  Its length is always 0.

### UART Host to Test Device

#### 0x0: Talk 0

This packet should be 2-8 bytes in length and represent a string to be given as an answer to a Talk 0 command from the Mac.  It is given as an answer to Talk 0 once and then discarded.  If the Mac is polling another device and service requests have not been disabled for the device, the device will make a service request (SRQ).

#### 0x1: Talk 1

This packet should be either 0 or 2-8 bytes in length and represent a string to be given as an answer to a Talk 1 command from the Mac.  It is given as an answer to Talk 1 commands until replaced by another packet of this type.

#### 0x2: Talk 2

This packet should be either 0 or 2-8 bytes in length and represent a string to be given as an answer to a Talk 2 command from the Mac.  It is given as an answer to Talk 2 commands until replaced by another packet of this type.

#### 0x3: Register 3/Configure

This packet should be either 0 or 2-15 bytes in length.

If this packet is 0 bytes in length, it sets the "exceptional event" flag in register 3 to 0, where it remains until it is reset by a read of register 3 by the Mac.

If this packet is 2 or more bytes in length, the bytes are interpreted as follows: The first byte sets the default address (and also the current address) for the device.  Each byte that follows the first is an allowable handler ID for the device, with the last being set as the default handler ID (and also the current handler ID) for the device.  There can be up to 14 allowable handler IDs.
