###########################################################
#
# ptcmd.py
#
#   Pan-tilt controller serial communication interface.
#   This module contains a class that encapsulate the
#   serial command interface to the embedded pat-tilt
#   system controller.
#   The module defines __enter__ and __exit__ functions
#   so that it can be used with a 'with' statement.
#
#   CLI reference:
#       home                       - Home position
#       get pos                    - Print motor positions
#       get a2d <ch>               - Read A-to-D channel
#       get microsteps             - Get micro-steps setting
#       move <motor> <rate> <step> - Move relative steps
#       sync <p-rate> <p-step> 
#            <t-rate> <t-step>     - Synchronized motor start
#       pan  <rate> <step>         - Pan system
#       tilt <rate> <step>         - Tilt system
#       wait all|pan|tilt          - Wait for motor to stop
#       stop                       - Stop all motors
#       version                    - Show version
#
#       ** 'rate' parameter is signed: negative CCW, positive CW
#
#   April 18, 2020
#
###########################################################

import serial
import time

class PTCMD(object):
    """Implements communication and command protocol to the pan-tilt motor controller."""


    def __init__(self, ser_port = '/dev/ttyS5', baud = 19200):
        """Initialize class variables."""

        # Serial port definitions
        self.__ser = None
        self.SER_PORT = ser_port
        self.BAUD_RATE = baud
        self.STOP_BIT = 1
        self.PARITY_BITS = 'N'
        self.DATA_BITS = 8
        self.TIME_OUT = 0.1

        # Motors
        self.PAN = 0
        self.TILT = 1
        self.BOTH = 2

        # Directions
        self.STOP = 0
        self.CW = 1
        self.CCW = -1

        # Limits

        # Open serial communication
        self.__ser = serial.Serial(self.SER_PORT, baudrate=self.BAUD_RATE, timeout=self.TIME_OUT)
        self.__ser.reset_input_buffer()
        self.__ser.reset_output_buffer()


    def __dir__(self):
        return ['__init__', '__enter__', '__exit__', '__send_cmd', '__get_response', 'close', 'raw_command', 'home', 'get_position', 'get_analog_sensor', 'get_microsteps', 'move', 'sync', 'pan', 'tilt', 'wait', 'stop', 'version']


    def __enter__(self):
        return self


    def __exit__(self, exception_type, exception_value, traceback):
        """Close buffers and serial communication."""

        self.close()


    def __send_cmd(self, cmd_string):
        """Send a command string to the controller."""

        #print('__send_cmd()', cmd_string)

        cmd_string = cmd_string.strip('\n\r')
        self.__ser.write(f'{cmd_string}\r'.encode('utf-8'))
        self.__ser.flush()


    def __get_response(self, timeout=0.1):
        """
        Wait to receive a response from the controller and return the response as a list.
        Return a tuple with True for good response, or False for time-out.
        """

        # TODO Add exception handling for failed reads.
        #      raise SerialException('read failed: {}'.format(e))
        #         serial.serialutil.SerialException: read failed: device reports readiness to read but returned no data (device disconnected or multiple access on port?)

        self.__ser.timeout = timeout

        # Less than three bytes in the input buffer
        # means that a response has not been received yet.
        # TODO Consider a timeout test to escape the loop
        while self.__ser.in_waiting < 3:
            pass

        response = self.__ser.readlines()

        self.__ser.timeout = self.TIME_OUT

        #print('__get_response()', response)

        return True, response[1:]


    def close(self):
        """Close buffers and serial communication."""

        self.__ser.close()


    def raw_command(self, cmd_string, timeout=0.1):
        """
        Send command strings as-is to controller.
        Return a tuple indicating status of True if ok or False if time-out or syntax error in command,
        and a list containing the command output.
        """

        self.__send_cmd(cmd_string)

        status_ok, response = self.__get_response(timeout)

        #print('raw_command()', status_ok, response)

        if status_ok:
            response_length = len(response)
            prompt = response_length - 1
            if response[-1:] == [b'>']:
                parameters = []
                for i in range(response_length - 1):
                    parameters.append(response[i].decode('utf-8').strip('\n\r'))
                return True, parameters
            else:
                return False, ['bad command or syntax']
        else:
            return False, ['comm error']


    def home(self):
        """
        Home the pan-tilt system.
        Return True if operation was successful or False if operation timed out.
        """

        status, _ = self.raw_command('home')
        return status


    def get_position(self):
        """
        Returns a tuple of motor pan and tilt positions in step count relative to home.
        Positive count is CW position, negative count in CCW position.
        """
        _, position = self.raw_command('get pos')
        return (int(position[0].split(',')[1]), int(position[1].split(',')[1]))


    def get_analog_sensor(self, channel = 0):
        """
        Return A-to-D conversion reading from specified channel.
        """

        _, conversion = self.raw_command(f'get a2d {channel}')
        return conversion[0]


    def get_microsteps(self):
        """
        Return micro-step setting of A4988 motor driver.
        """

        _, microsteps = self.raw_command('get microsteps')
        return microsteps[0]


    def move(self, motor, rate, steps):
        """
        Move a motor at a step rate for optional step count.
        Step count can be '-1' for unlimited stepping up to system limits.
        """

        status, _ = self.raw_command(f'move {motor} {rate} {steps}')
        return status


    def sync(self, pan_rate, pan_steps, tilt_rate, tilt_steps):
        """
        Start both pan and tilt motors at the same time with a step rate for optional step count.
        Use this instead of issuing two 'move' commands to reduce delay between motors starts
        Step count can be '-1' for unlimited stepping up to system limits.
        """

        status, _ = self.raw_command(f'sync {pan_rate} {pan_steps} {tilt_rate} {tilt_steps}')
        return status


    def pan(self, rate, steps):
        """
        Pan system.
        Step count can be '-1' for unlimited stepping up to system limits.
        """

        status, _ = self.raw_command(f'pan {rate} {steps}')
        return status


    def tilt(self, rate, steps):
        """
        Tilt system.
        Step count can be '-1' for unlimited stepping up to system limits.
        """

        status, _ = self.raw_command(f'tilt {rate} {steps}')
        return status


    def wait(self, motor='all'):
        """
        Wait for motors to stop before function returns.
        Motor specification can be 'all' (default), 'pan', or 'tilt'
        """

        status, _ = self.raw_command(f'wait {motor}')
        return status


    def stop(self):
        """
        Stop all motors.
        """

        status, _ = self.raw_command('stop')
        return status


    def version(self):
        """Return controller version string."""

        _, version = self.raw_command('version')
        return version[0]


