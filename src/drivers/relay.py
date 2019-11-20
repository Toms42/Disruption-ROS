import serial
from serial import SerialException
from sys import version_info

PY2 = version_info[0] == 2  # Running Python 2.x?


class Relay:
    def __init__(self, ttyStr):
        try:
            self.usb = serial.Serial(ttyStr)
            self.usb.baudrate = 9600
        except SerialException as se:
            raise SerialException("Failed to setup relay: {}".format(se.message))
        self.channels = [1, 2]

    def setRelay(self, channel, value):
        if channel not in self.channels:
            raise ValueError("Invalid relay channel! Acceptable channels are {}".format(self.channels))

        hdr = 0xA0
        ch = channel & 0xf
        value = 0x1 if value else 0x0
        chk = hdr + ch + value

        cmd = bytearray([hdr, ch, value, chk])
        cmd

        try:
            self.usb.write(cmd)
        except SerialException:
            try:
                self.usb = serial.Serial(ttyStr)
                self.usb.baudrate = 9600
            except SerialException as se:
                raise SerialException("Failed to setup relay: {}".format(se.message))
