#/usr/bin/python
import serial

__maintainer__ = 'Matt Wilson'
__email__ = 'mattwilsonmbw@gmail.com'

PROTOCOL = chr(0xAA) # First part of command to write using Pololu Protocol
FORWARD = chr(0x05) # Drive motor forward command
START = chr(0x03) # Exit Safe-Start mode so the motor can run 
STOP = chr(0x60)  # Stops the motor and enters Safe-Start mode  


class Daisy:
    """Represents a single Pololu Simple Motor Controller in daisy chain
    
    This class offers an interface to daisy chain several Pololu Simple
    Motor Controllers. The daisy chained modules all share the same
    serial connection.  To target specific devices, every command 
    is sent with the device number of the device.


    Initialize every separate board you want to talk to with its 
    device number.

    Example:
	Here is an example of usage
        ::
            pololu = Daisy(0) # initialize a Pololu with device number 0
    
    Todo:
        * For module TODOs
        * You have to also use ``sphinx.ext.todo`` extension
    
    .. _Github Repo
       http://github.com/matwilso
    
    """
    
    count = 0 # static variable to keep track of number of Daisys open
    ser = None # initialize static variable ser to None  
    
    def __init__(self, dev_num, port="ttyACM0"):
	"""Set motor id and open serial connection if not already open"""
	if (dev_num is not int):
	    raise Exception("Invalid motor id, must set to id of motor for daisy chaining") 

	Daisy.count += 1
	self.dev_num = chr(dev_num)
	if (Daisy.ser is None or not Pololu.ser.isOpen()):
	    Daisy.ser = serial.Serial(port)

    def __del__(self):
	"""Decrement count and close port if it's the last connection"""
	Daisy.count -= 1 # decrement running total of Pololu objects
	if (Daisy.count <= 0):
	    if (Daisy.ser is not None and Daisy.ser.isOpen()):
		Daisy.ser.close() 

   def _send_command(command, databyte3, databyte4):
        """Sends a two-byte command using the Pololu protocol."""
	cmd = PROTOCOL + self.dev_num + command + chr(databyte3) + chr(databyte4)
	Daisy.ser.write(cmd)
    
    def _send_command_single(command):
        """Sends a one-byte command using the Pololu protocol."""
	cmd = PROTOCOL + self.dev_num + command
	Daisy.ser.write(cmd)

    def _exit_safe_start():
	self._send_command_single(START) 
    def _stop_motor():
	self._send_command_single(STOP)
    
    """ USER METHODS """
        # TODO


