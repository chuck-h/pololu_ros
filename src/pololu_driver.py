#!/usr/bin/python
from __future__ import division
import serial

"""Pololu driver module for motor controllers using pyserial

This module handles the lower level logic of writing to a pololu motor
controller using python.
"""



BAUD_SYNC = chr(0x80)  # Required to sync the baud rate for older devices
PROTOCOL = chr(0xAA)  # First part of command to write using Pololu Protocol
FORWARD = chr(0x05)  # Drive motor forward command
BACKWARD = chr(0x06)  # Drive motor reverse command
START = chr(0x03)  # Exit Safe-Start mode so the motor can run
STOP = chr(0x60)   # Stops the motor and enters Safe-Start mode

class Single():
    """Represents a single Pololu Simple Motor Controller

    Example:
       controller = Single(0, port="/dev/ttyUSB0")
         dev_num is the Device number of Pololu board
    """

    def __init__(self, dev_num, port):
        """Set motor id and open serial connection"""
        if dev_num < 0 or dev_num > 127:
            raise Exception("Invalid motor id, must set to id of motor (0-127) for daisy chaining")

        self.dev_num = chr(dev_num)  # set device number to use in other commands

        # if serial connection has not been made yet
        self.ser = serial.Serial(port)
        self.ser.write(BAUD_SYNC)  # sync old devices by writing 0x80
        self._exit_safe_start()  # make it so pololu reacts to commands

    def __del__(self):
        self._stop_motor()  # safely stop current motor
        if self.ser is not None and Daisy.ser.isOpen():
                Single.ser.close()
                print("Serial connection closed")

    def _send_command(self, command, databyte3, databyte4):
        """Sends a two-byte command using the Pololu protocol."""
        cmd = PROTOCOL + self.dev_num + command + chr(databyte3) + chr(databyte4)
        self.ser.write(cmd)

    def _send_command_single(self, command):
        """Sends a one-byte command using the Pololu protocol."""
        cmd = PROTOCOL + self.dev_num + command
        self.ser.write(cmd)

    def _exit_safe_start(self):
        """Exit safe start so you can freely send commands to Pololu.
        This must be run before run other commands."""
        self._send_command_single(START)

    def _stop_motor(self):
        """Immediately stops the motor and enters safe start mode"""
        self._send_command_single(STOP)

    # USER METHODS

    def forward(self, speed):
        """Drive motor forward at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds

        self._exit_safe_start()
        # This is how the documentation recommends doing it.
        # The 1st byte will be from 0 to 31
        # The 2nd byte will be from 0 to 100
        self._send_command(FORWARD, speed % 32, speed // 32)  # low bytes, high bytes

    def backward(self, speed):
        """Drive motor backward (reverse) at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()
        self._send_command(BACKWARD, speed % 32, speed // 32)  # low bytes, high bytes

    def drive(self, speed):
        """Drive motor in direction based on speed (-3200, 3200)"""
        if speed < 0:
            self.backward(-speed)
        else:
            self.forward(speed)

    def stop(self):
        """Stop the motor"""
        self._stop_motor()

class Daisy(object):
    """Represents a single Pololu Simple Motor Controller in a daisy chain

    This class offers an interface to daisy chain several Pololu Simple
    Motor Controllers. The daisy chained modules all share the same
    serial connection.  To target specific devices, every command
    is sent with the device number of the device.  You must configure the
    devices to have different numbers. This must be done using the Simple
    Motor Controller setup softare (https://www.pololu.com/docs/0J44/3).

    Initialize every separate board you want to talk to with its
    set device number.

    Example:
        Here is an example of usage. The first Daisy object sets the port and
        all the of the rest use the same port.
        ::
        motor0 = Daisy(0, port="/dev/ttyUSB0") # only first one has to be set
        motor1 = Daisy(1) # all following will use the first port
        motor2 = Daisy(2)

        motor1.forward(1600) # drive forward the motor with device number 1

        # All motors are using port "/dev/ttyUSB0" to send commands
        # Each device is addressed by their device number
"""

    Attributes:
        dev_num (chr): Device number of Pololu board to be commanded
    """

    count = 0  # static variable to keep track of number of Daisys open
    ser = None  # initialize static variable ser to None

    def __init__(self, dev_num, port="/dev/ttyUSB0"):
        """Set motor id and open serial connection if not already open"""
        if dev_num < 0 or dev_num > 127:
            raise Exception("Invalid motor id, must set to id of motor (0-127) for daisy chaining")

        Daisy.count += 1  # increment count of controllers
        self.dev_num = chr(dev_num)  # set device number to use in other commands

        # if serial connection has not been made yet
        if Daisy.ser is None or not Daisy.ser.isOpen():
            Daisy.ser = serial.Serial(port)
            Daisy.ser.write(BAUD_SYNC)  # sync old devices by writing 0x80
        self._exit_safe_start()  # make it so pololu reacts to commands

    def __del__(self):
        """Decrement count, stop motor, and close port if it's the last connection"""
        Daisy.count -= 1  # decrement count of controllers
        self._stop_motor()  # safely stop current motor
        # if this is the last controller open
        if Daisy.count <= 0:
            if Daisy.ser is not None and Daisy.ser.isOpen():
                Daisy.ser.close()
                print("Serial connection closed")

    def _send_command(self, command, databyte3, databyte4):
        """Sends a two-byte command using the Pololu protocol."""
        cmd = PROTOCOL + self.dev_num + command + chr(databyte3) + chr(databyte4)
        Daisy.ser.write(cmd)

    def _send_command_single(self, command):
        """Sends a one-byte command using the Pololu protocol."""
        cmd = PROTOCOL + self.dev_num + command
        Daisy.ser.write(cmd)

    def _exit_safe_start(self):
        """Exit safe start so you can freely send commands to Pololu.
        This must be run before run other commands."""
        self._send_command_single(START)

    def _stop_motor(self):
        """Immediately stops the motor and enters safe start mode"""
        self._send_command_single(STOP)

    # USER METHODS

    def forward(self, speed):
        """Drive motor forward at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds

        self._exit_safe_start()
        # This is how the documentation recommends doing it.
        # The 1st byte will be from 0 to 31
        # The 2nd byte will be from 0 to 100
        self._send_command(FORWARD, speed % 32, speed // 32)  # low bytes, high bytes

    def backward(self, speed):
        """Drive motor backward (reverse) at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()
        self._send_command(BACKWARD, speed % 32, speed // 32)  # low bytes, high bytes

    def drive(self, speed):
        """Drive motor in direction based on speed (-3200, 3200)"""
        if speed < 0:
            self.backward(-speed)
        else:
            self.forward(speed)

    def stop(self):
        """Stop the motor"""
        self._stop_motor()


