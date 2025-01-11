"""
A library to control hardware (Arduino, ELMO, thermometers ets)

BIONAUT LABS. RAMAT GAN. SHUSTOV. 2023
"""

from abc import abstractmethod
# PySerial
import serial
# Test module Serial
#import serial_mock as serial
import array
import time
import random
from math import cos, sin, pi
import scipy
import numpy as np
from queue import PriorityQueue
from collections import deque
import math


class CoilsControllerInterface:
    """
    Interface of the coils controller
    """

    @abstractmethod
    def new_coil_state(self, coil_num: int, state: float) -> str:
        """
        Changes the state of the coil. Does not change the state of other coils
        :param coil_num: number of the coil to change
        :param state: new state of the coil
        :return: result of the operation as string
        """
        raise NotImplementedError

    @abstractmethod
    def new_coil_state_single(self, coil_num: int, state: float) -> str:
        """
        Changes the state of the coil. All other coils are turned off
        :param coil_num: number of the coil to change
        :param state: new state of the coil
        :return:result of the operation as string
        """
        raise NotImplementedError

    @abstractmethod
    def new_coil_state_all(self, coils_values_list: list[float]) -> str:
        """
        Changes the state of all coils together
        :param coils_values_list: list of new values of the coils
        :return:result of the operation as string
        """
        raise NotImplementedError

    @abstractmethod
    def all_off(self) -> str:
        """
        Turns off all coils
        :return:result of the operation as string
        """
        raise NotImplementedError

    @abstractmethod
    def close_connection(self):
        """
        Close the connection with the device
        :return:
        """


class VoltmeterInterface:
    """
    Interface of the voltmeter
    """

    @abstractmethod
    def measure_voltage_single(self, channel_num: int):
        """
        Measure voltage of the single channel
        :param channel_num: number of the channel to measure voltage
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def measure_voltage_all(self):
        """
        Measure all the voltage channels
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def close_connection(self):
        """
        Close connection to the device
        :return:
        """


class LightControllerInterface:
    """
    Interface of the light controller
    """

    @abstractmethod
    def new_light_state(self, light_num: int, state: float):
        """
        change the state of the light at one channel
        :param light_num: number of the light to change the state
        :param state: new state of the light
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def new_light_state_single(self, light_num: int, state: float):
        """
        Changes the state of the light at one channel, other light are turned off
        :param light_num: number of the light to change the state
        :param state: new state of the light
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def light_off(self):
        """
        all lights are turned off
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def close_connection(self):
        """
        Close connection to the device
        :return:
        """


class ThermometerInterface:
    """
    Interface of the thermometer
    """

    @abstractmethod
    def measure_temperature_single(self, sensor_num: int):
        """
        Measure temperature of one sensor
        :param sensor_num: number of the sensor to measure temperature
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def measure_temperature_all(self):
        """
        Measure temperature of all sensors
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def close_connection(self):
        """
        Close connection to the device
        :return:
        """


class MagnetometerInterface:
    """
    Interface of magnetometer
    """

    @abstractmethod
    def measure_m_field_single(self, **kwargs):
        """
        Measure magnetic field of one sensor
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def measure_m_field_all(self):
        """
        Measure magnetic field of all sensors
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def close_connection(self):
        """
        Close connection to the device
        :return:
        """


class DeviceInterface:
    """
    Interface for a hardware device
    """

    device_reference = None
    device_connected = False

    @abstractmethod
    def set_connection(self):
        """
        connects to the device
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def close_connection(self):
        """
        disconnects from the device
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def send_request(self, request):
        """
        sends a request to the device
        :param request: request
        :return:
        """
        raise NotImplementedError

    @abstractmethod
    def get_response(self, **kwargs):
        """
        read response from the device
        :return: response from the device
        """
        raise NotImplementedError


class ArduinoMegaBoard(DeviceInterface):
    """
    Arduino board class, used to communicate with ArduinoMega.
    See API of ArduinoMega from Bionaut IL
    """
    def __init__(self, port: str, baudrate=19200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.device_reference = self.set_connection()
        print(f'ArduinoMegaBoard initiated at port {port}')

    def set_connection(self):
        device_reference = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)
        self.device_connected = True
        return device_reference

    def send_request(self, request: list[int]) -> bool:
        """
        Send request to the Arduino
        :param request: list of the specific request, for example [231,3,0], according
        to the Arduino API
        :return: boolean indicator OK/NOT OK
        """
        ar = array.array('B', request).tobytes()
        try:
            self.device_reference.reset_output_buffer()
            self.device_reference.write(ar)
            return True
        except serial.SerialException:
            print(f'Error sending request to the board at port {self.port}')
            return False

    def get_response(self) -> str:
        """
        read response from the Arduino port
        :return: response as a string
        """
        res = self.device_reference.readline()
        return res.decode()

    def test_led(self):
        self.send_request([253, 13, 150])

    def close_connection(self):
        """
        Stop the serial connection
        :return:
        """
        if self.device_connected:
            self.device_reference.close()
            self.device_connected = False


class MagnetometerI2CArduino(MagnetometerInterface):
    """
    Class to use a single magnetometer connected to Arduino with I2C
    """

    def __init__(self, arduino_board: DeviceInterface):

        self.arduino_board = arduino_board

    def measure_m_field_single(self, **kwargs):
        """
        Measure magnetic field of one sensor
        :return:
        """
        request = [220, 0, 0]
        self.arduino_board.send_request(request)
        res = self.arduino_board.get_response()
        res_list = res.split()
        return res_list

    def measure_m_field_all(self):
        """
        Measure magnetic field of all sensors
        Not implemented yet
        :return:
        """
        res = self.measure_m_field_single()
        return res

    def close_connection(self):
        """
        Close connection to the device
        :return:
        """
        self.arduino_board.close_connection()


class CoilBTS7960ArduinoController(CoilsControllerInterface):
    """
    Coils controller based on 4 BTS7960 boards connected to Arduino board
    Coils are numbered starting from 1
    """
    def __init__(self,
                 arduino_board: DeviceInterface,
                 ch_plus: tuple[int] = (2, 5, 7, 9),
                 ch_minus: tuple[int] = (3, 6, 8, 10),
                 scale_255: bool = False):
        """
        :param arduino_board: Arduino object
        :param ch_plus: pins of Arduino which send PWM to form positive current on coils [1,2,3,4]
        :param ch_minus: pins of Arduino which send PWM to form negative current on coils [1,2,3,4]
        :param scale_255: if True the PWM is scaled 0..255, otherwise the PWM is scaled 0..1
        """
        self.arduino_board = arduino_board
        self.ch_plus = list()
        self.ch_minus = list()
        self.ch_plus.append(0)
        self.ch_minus.append(0)

        if len(ch_plus) != 4 or len(ch_minus) != 4:
            raise ValueError("Number of Arduino channels must be 4")

        for i in range(4):
            self.ch_plus.append(ch_plus[i])
            self.ch_minus.append(ch_minus[i])

        if scale_255:
            self.PWM_scaler = 1
        else:
            self.PWM_scaler = 255

    def __new_request(self, coil_num: int = 0, state: float = 0, operation_code: int = 0) -> str:
        """
        Send a new request to the Arduino
        :param self:
        :param coil_num: number of the coil
        :param state: new coil's state
        :param operation_code: code of the Arduino operation according to the
        Bionaut IL Arduino API
        :return:
        """
        state = int(state * self.PWM_scaler)
        if state > 255:
            state = 255
        elif state < -255:
            state = -255

        if state >= 0:
            if operation_code !=250 and operation_code != 251:
                request = [operation_code, self.ch_minus[coil_num], 0]
                self.arduino_board.send_request(request)
            request = [operation_code, self.ch_plus[coil_num], state]
            self.arduino_board.send_request(request)
        if state < 0:
            if operation_code != 250 and operation_code != 251:
                request = [operation_code, self.ch_plus[coil_num], 0]
                self.arduino_board.send_request(request)
            request = [operation_code, self.ch_minus[coil_num], -state]
            self.arduino_board.send_request(request)

        return f'State PWM of the coil#{coil_num} is set to {state}'

    def new_coil_state(self, coil_num: int = 0, state: float = 0) -> str:
        """
        New state of the coil. Other coils states are not changed
        :param coil_num: number of the coil 1..4
        :param state: new coil's state 0..255 or 1..0 (if scale_255=False)
        :return: result of the operation
        """

        operation_code = 253
        res = self.__new_request(coil_num=coil_num, state=state,
                                 operation_code=operation_code)
        res = 'New coil state:' + res

        return res

    def new_coil_state_single(self, coil_num: int = 0, state: float = 0) -> str:
        """
        Set single coil state. All other coils are turned off
         :param coil_num: number of the coil 1..4
        :param state: new coil's state 0..255 or 1..0 (if scale_255=False)
        :return: result of the operation
        """

        operation_code = 251
        res = self.__new_request(coil_num=coil_num, state=state,
                                 operation_code=operation_code)
        res = 'New coil state:' + res

        return res

    def new_coil_state_all(self, coils_values_list: list[float]):
        """
        Set state for all 4 coils
        :param coils_values_list: list of new values for coils 1..4
        :return: result of the operation
        """
        if len(coils_values_list) != 4:
            return 'Wrong format of input list'

        for i in range(4):
            coil_num = i+1
            state = coils_values_list[i]
            self.new_coil_state(coil_num=coil_num, state=state)

        return 'New coils states were set'


    def all_off(self) -> str:
        """
        Turn all coils off
        :return:
        """
        operation_code = 250
        self.__new_request(coil_num=1, state=0, operation_code=operation_code)
        res = 'All off'

        return res

    def close_connection(self):
        self.arduino_board.close_connection()


class CoilPololuArduinoController(CoilsControllerInterface):
    """
    Coils controller based on 8 Pololu boards connected to ArduinoMega board (All coils are controlled
    with 1 Arduino board)
    Coils are numbered starting from 1
    """
    def __init__(self,
                 arduino_board: DeviceInterface,
                 pwm_pins: tuple[int] = (2, 3, 5, 6, 7, 8, 9, 10),
                 direction_pins: tuple[int] = (22, 24, 26, 28, 30, 32, 34, 36),
                 scale_255: bool = False):
        """
        :param arduino_board: Arduino object
        :param ch_plus: pins of Arduino which send PWM signal to the driver (to control amplitude)
        :param ch_minus: pins of Arduino which send direction signal to the driver (high/low signal)
        :param scale_255: if True the PWM is scaled 0..255, otherwise the PWM is scaled 0..1
        """
        self.arduino_board = arduino_board
        self.pwm_pins = list()
        self.direction_pins = list()
        self.pwm_pins.append(0)
        self.direction_pins.append(0)

        if len(pwm_pins) != 8 or len(direction_pins) != 8:
            raise ValueError("Number of Arduino channels must be 8/8")

        for i in range(8):
            self.pwm_pins.append(pwm_pins[i])
            self.direction_pins.append(direction_pins[i])

        if scale_255:
            self.PWM_scaler = 1
        else:
            self.PWM_scaler = 255

    def __new_request(self, coil_num: int = 0, state: float = 0, operation_code: int = 0) -> str:
        """
        Send a new request to the Arduino
        :param self:
        :param coil_num: number of the coil
        :param state: new coil's state
        :param operation_code: code of the Arduino operation according to the
        Bionaut IL Arduino API
        :return:
        """
        state = int(state * self.PWM_scaler)
        if state > 255:
            state = 255
        elif state < -255:
            state = -255

        pwm_pin = self.pwm_pins[coil_num]
        direction_pin = self.direction_pins[coil_num]
        direction = 0 if state <= 0 else 1

        request = [operation_code, pwm_pin, direction_pin, abs(state), direction]
        self.arduino_board.send_request(request)

        return f'State PWM of the coil#{coil_num} is set to {state}. Bytes sent {request}'

    def new_coil_state(self, coil_num: int = 0, state: float = 0) -> str:
        """
        New state of the coil. Other coils states are not changed
        :param coil_num: number of the coil 1..8
        :param state: new coil's state 0..255 or 1..0 (if scale_255=False)
        :return: result of the operation
        """

        operation_code = 223
        res = self.__new_request(coil_num=coil_num, state=state,
                                 operation_code=operation_code)
        res = 'New coil state:' + res

        return res

    def new_coil_state_single(self, coil_num: int = 0, state: float = 0) -> str:
        """
        Set single coil state. All other coils are turned off
         :param coil_num: number of the coil 1..8
        :param state: new coil's state 0..255 or 1..0 (if scale_255=False)
        :return: result of the operation
        """

        operation_code = 221
        res = self.__new_request(coil_num=coil_num, state=state,
                                 operation_code=operation_code)
        res = 'New coil state:' + res

        return res

    def new_coil_state_all(self, coils_values_list: list[float]):
        """
        Set state for all 8 coils
        :param coils_values_list: list of new values for coils 1..8
        :return: result of the operation
        """
        if len(coils_values_list) != 8:
            return 'Wrong format of input list'

        for i in range(8):
            coil_num = i+1
            state = coils_values_list[i]
            self.new_coil_state(coil_num=coil_num, state=state)

        return 'New coils states were set'


    def all_off(self) -> str:
        """
        Turn all coils off
        :return:
        """
        operation_code = 220
        self.__new_request(coil_num=1, state=0, operation_code=operation_code)
        res = 'All off'

        return res

    def close_connection(self):
        self.arduino_board.close_connection()


class ArduinoMegaVoltmeter(VoltmeterInterface):
    """
    Implements voltmeter based on Arduino board
    """
    def __init__(self, arduino_board: DeviceInterface):
        self.arduino_board = arduino_board

    def __new_request(self, command: list) -> str:
        """
        Prepare the Serial port and send a command to measure
        :param command: list (command) to Arduino
        :return: 
        """
        self.arduino_board.device_reference.reset_output_buffer()
        self.arduino_board.device_reference.reset_input_buffer()
        self.arduino_board.send_request(command)
        # in some cases the Serial needs a delay between write and read
        time.sleep(1)
        return 'ok'

    def measure_voltage_single(self, channel_num: int) -> int:
        """
        Measure voltage on a single analog input. Result is returned in units of Arduino counts
        :param channel_num: number of the channel to measure
        :return: measured value (counts)
        """
        request = [234, channel_num, 0]
        self.__new_request(request)
        # read first line from Serial to check if it is voltage
        #res = self.arduino_board.device_reference.readline()
        res = self.arduino_board.get_response()
        # if the data is voltage from a single channel - read value
        r = 9999
        print(f'res {res}')
        if res == 'VoltageSingle\r\n':
            res = self.arduino_board.get_response()
            try:
                r = int(res)
            except ValueError:
                print('Arduino send wrong data - not a number')
        return r

    def measure_voltage_all(self) -> list[int]:
        """
        Measure all the voltage channels of ArduinoMega. Result is returned in units of Arduino counts
        :return: list of measured values (counts)
        """
        request = [235, 0, 0]
        self.__new_request(request)
        # read first line from Serial to check if it is voltage
        res = self.arduino_board.get_response()
        # if the data is voltage from all channels - read other 16 lines
        res_list = []
        r = 9999
        if res == 'VoltageAll\r\n':
            for i in range(16):
                res = self.arduino_board.get_response()
                try:
                    r = int(res)
                except ValueError:
                    print('Arduino send wrong data - not a number')
                res_list.append(r)
        return res_list

    def measure_voltage_4(self, channels_list: list) -> list[int]:
        """
        Measure voltage from 4 channels. Result is returned in units of Arduino counts
        :param: channels_list - list of analog pins to measure voltage
        :return: list of measured values (counts)
        """
        request = [233, channels_list[0], channels_list[1], channels_list[2], channels_list[3]]
        self.__new_request(request)
        # read first line from Serial to check if it is voltage
        res = self.arduino_board.get_response()
        # if the data is voltage from 4 channels - read values
        res_list = []
        r = 9999
        if res == 'Voltage\r\n':
            for i in range(4):
                res = self.arduino_board.get_response()
                try:
                    r = int(res)
                except ValueError:
                    print('Arduino send wrong data - not a number')
                res_list.append(r)
        return res_list

    def close_connection(self):
        """
        close device connection
        :return:
        """
        self.arduino_board.close_connection()


class LightBTS7960ArduinoController(LightControllerInterface):
    """
    Implements LightControllerInterface based on Arduino Mega
    """

    def __init__(self, arduino_board: DeviceInterface, light_pins: tuple, scale_255: bool = False):
        """
        Initialize the object
        :param arduino_board: Arduino board to control the light connected to BTS7960 board
        :param light_pins: PWM pins of Arduino, which control the light for example (44,45,46)
        """
        self.arduino_board = arduino_board
        self.light_pins = light_pins
        self.pwm_scaler = 1 if scale_255 else 255

    def __new_request(self, light_num: int = 0, state: float = 0, operation_code: int = 0) -> str:
        """
        Send a new request to the Arduino
        :param light_num: number of the light to change the state, indexing from 1
        :param state: new light state
        :param operation_code: code according to Bionaut Arduino API
        :return:
        """
        pwm_state = state * self.pwm_scaler
        pwm_state = pwm_state if pwm_state <= 255 else 255
        pwm_state = pwm_state if pwm_state >= 0 else 0
        request = [operation_code, self.light_pins[light_num - 1], pwm_state]
        self.arduino_board.send_request(request)
        return 'ok'

    def new_light_state(self, light_num: int, state: float):
        """
        change the state of the light at one channel
        :param light_num: number of the light to change the state, indexing from 1
        :param state: new state of the light
        :return:
        """
        if (light_num > len(self.light_pins)) or (light_num < 0):
            print('Wrong light number')
        else:
            self.light_off()
            self.__new_request(light_num=light_num, state=state, operation_code=231)
        return f'Light {light_num} state changed to {state}'

    def new_light_state_single(self, light_num: int, state: float):
        """
        Changes the state of the light at one channel, other light are turned off
        :param light_num: number of the light to change the state, indexing from 1
        :param state: new state of the light
        :return:
        """
        self.new_light_state(light_num=light_num, state=state)

    def light_off(self):
        """
        all lights are turned off
        :return:
        """
        request = [230, 0, 0]
        self.arduino_board.send_request(request)

    def close_connection(self):
        """

        :return:
        """
        self.arduino_board.close_connection()


class BDCoilsControllerBTS7960(CoilsControllerInterface):
    """
    Coils controller for BD system based on BTS7960 and 2 Arduino boards
    """
    def __init__(self,coils_1_4_controller: CoilsControllerInterface,
                        coils_5_8_controller: CoilsControllerInterface):
        """

        :param coils_1_4_controller: Arduino A BTS7960 coils controller (coils 1..4)
        :param coils_5_8_controller: Arduino B BTS7960 coils controller (coils 5..8)
        """

        self.coils_1_4_controller = coils_1_4_controller
        self.coils_5_8_controller = coils_5_8_controller


    def new_coil_state(self, coil_num, state=0):
        """
        Turns on or off the coil.Index starts from 1
        :param coil_num:
        :param state:
        :return:
        """
        if coil_num < 1 or coil_num > 8:
            print('Wrong coil number, must be in range 1..8')
            return 'Wrong coil number, must be in range 1..8'
        if coil_num < 5:
            self.coils_1_4_controller.new_coil_state(coil_num=coil_num, state=state)
        else:
            self.coils_5_8_controller.new_coil_state(coil_num=coil_num - 4, state=state)
        return f'Coil {coil_num} set to {state}'

    def new_coil_state_single(self, coil_num, state=0):
        """
        Turns on or off the single coil, when all other coils are turned off.Index starts from 1
        :param coil_num:
        :param state:
        :return:
        """
        if coil_num < 1 or coil_num > 8:
            print('Wrong coil number, must be in range 1..8')
            return 'Wrong coil number, must be in range 1..8'
        if coil_num < 5:
            self.coils_1_4_controller.new_coil_state_single(coil_num=coil_num, state=state)
        else:
            self.coils_5_8_controller.new_coil_state_single(coil_num=coil_num - 4, state=state)
        return f'Coil {coil_num} set to {state}'

    def all_off(self):
        """
        All coils off
        :return:
        """
        self.coils_1_4_controller.all_off()
        self.coils_5_8_controller.all_off()
        return 'All coils off'

    def new_coil_state_all(self, coils_values_list):
        """
        Set values of all coils at once
        :param coils_value_list: list of coils state
        :return:
        """
        if len(coils_values_list) > 8 or len(coils_values_list) == 0:
            print('Wrong coils_value_list, must be a list of len = 8')
            return ''
        values_a = coils_values_list[0:4]
        values_b = coils_values_list[4:8]
        self.coils_1_4_controller.new_coil_state_all(values_a)
        self.coils_5_8_controller.new_coil_state_all(values_b)
        return f'New coils values {coils_values_list}'

    def close_connection(self):
        """
        Close serial connections of Arduino boards
        :return:
        """
        self.coils_1_4_controller.close_connection()
        self.coils_5_8_controller.close_connection()
        return 'Controller closed'



class ALISystem:
    """
    Class for pulling motion in CSF (ALI bot)
    """
    def __init__(self, controller: CoilsControllerInterface):
        """
        :param controller: coils controller (CoilsControllerInterface)
        """
        self.controller = controller


    def new_coil_state(self, coil_num, state=0):
        """
        Turns on or off the coil.Index starts from 1
        :param coil_num:
        :param state:
        :return:
        """
        if coil_num < 1 or coil_num > 8:
            print('Wrong coil number, must be in range 1..8')
            return ''
        else:
            self.controller.new_coil_state(coil_num=coil_num,state=state)


    def new_coil_state_single(self, coil_num, state=0):
        """
        Turns on or off the single coil, when all other coils are turned off.Index starts from 1
        :param coil_num:
        :param state:
        :return:
        """
        if coil_num < 1 or coil_num > 8:
            print('Wrong coil number, must be in range 1..8')
            return ''
        else:
            self.controller.new_coil_state_single(coil_num=coil_num, state=state)


    def all_off(self):
        """
        All coils off
        :return:
        """
        self.controller.all_off()


    def new_coil_state_all(self, coils_values_list):
        """
        Set values of all coils at once
        :param coils_value_list: list of coils state
        :return:
        """
        if len(coils_values_list) > 8 or len(coils_values_list) == 0:
            print('Wrong coils_value_list, must be a list of len = 8')
            return ''
        else:
            self.controller.new_coil_state_all(coils_values_list=coils_values_list)


    def close_connection(self):
        """
        Close connections to controller board/boards
        :return:
        """
        self.controller.close_connection()


class ALISystemStepMotion:
    """
    Class for pulling motion with a discreet (step) change of direction
    """

    def __init__(self, controller: CoilsControllerInterface):
        """
        :param controller: coils controller (CoilsControllerInterface)
        """
        self.controller = controller
        self.decompose_coils_list, self.projections_list = [], []


    def decompose_direction(self, vector, coils, n):
        """ decompose vector (force) into directions of n coils
        vector - initial vector for decomposition
        coils - list of coils vectors
        n - number of decomposition directions
        """
        angles_queue = PriorityQueue()
        for i in range(8):
            angle = LinAlg.vec_angle_deg(vector, coils[i])
            angles_queue.put((180 - angle, i))

        decompose_coils_list = []
        projections_list = []
        for i in range(n):
            angle, coil_num = angles_queue.get()
            decompose_coils_list.append(coil_num)
            angle_rad = np.deg2rad(180 - angle)
            projection_length = np.linalg.norm(vector * np.cos(angle_rad))
            projections_list.append(projection_length)

        return decompose_coils_list, projections_list

    def update_direction(self, force_vector, coils, force_decompose_components_number):
        self.decompose_coils_list, self.projections_list = self.decompose_direction(force_vector,
                                                                        coils,
                                                                        force_decompose_components_number)
        self.current_coil_index = 0


    def new_coil_state(self, coil_num, state=0):
        """
        Turns on or off the coil.Index starts from 1
        :param coil_num:
        :param state:
        :return:
        """
        if coil_num < 1 or coil_num > 8:
            print('Wrong coil number, must be in range 1..8')
            return ''
        else:
            self.controller.new_coil_state(coil_num=coil_num,state=state)


    def new_coil_state_single(self, coil_num, state=0):
        """
        Turns on or off the single coil, when all other coils are turned off.Index starts from 1
        :param coil_num:
        :param state:
        :return:
        """
        if coil_num < 1 or coil_num > 8:
            print('Wrong coil number, must be in range 1..8')
            return ''
        else:
            self.controller.new_coil_state_single(coil_num=coil_num, state=state)


    def all_off(self):
        """
        All coils off
        :return:
        """
        self.controller.all_off()


    def close_connection(self):
        """
        Close connections to controller board/boards
        :return:
        """
        self.controller.close_connection()


class PELESystem:
    """
    Class for the screw motion (PELE bot)
    """

    def __init__(self, controller: CoilsControllerInterface):
        """

        """
        self.controller = controller
        self.new_move_dir_vec(new_dir_vec=[0, 1, 0])
        self.current_solver = GeomCurrentDist8EM()
        self.move_dir_vec = np.array([0, 0, 1])
        self.field_vec = LinAlg.vec_to_1(LinAlg.perp2vec(self.move_dir_vec))


    def new_move_dir_vec(self, new_dir_vec):
        """ Set new movement direction vector (axis of the bot movement). Used to initialize the directionvector
        :param new_dir_vec:
        :return:
        """
        self.move_dir_vec = np.array(new_dir_vec)
        self.field_vec = LinAlg.vec_to_1(LinAlg.perp2vec(self.move_dir_vec))


    def update_dir_vec(self, new_dir_vec):
        """ Update direction vector and field vector. Used to turn the bot
        :return:
        """
        new_vec_a = np.array(new_dir_vec)
        angle_rad = LinAlg.vec_angle_rad(self.move_dir_vec, new_vec_a)
        rot_axis = np.cross(self.move_dir_vec, new_vec_a)
        self.field_vec = LinAlg.vec_to_1(LinAlg.rotate_vec(self.field_vec, rot_axis, angle_rad))
        self.move_dir_vec = new_vec_a


    def rotate_field_vec(self, step_rad):
        """
        Rotate the field vector on a step_rad. used to make screw motion
        :param step_rad:
        :return:
        """
        self.field_vec = LinAlg.vec_to_1(LinAlg.rotate_vec(self.field_vec, self.move_dir_vec, step_rad))


    def new_coil_state_all(self, pre_normal_force=False, max_current=1, show_lambda_values=False):
        """
        Send new state to magnets. Convert the field direction vector to currents distribution (solve)
        and send values to arduino boards
        :param pre_normal_force: boolean flag, normalize the force vector before calculation of currents
        :param max_current: maximal current in coils. 0-255 or 0-1 (depends on scale_255)
        :return:
        """

        lambda_sol = self.current_solver.solve_system(self.field_vec, pre_normal_force=pre_normal_force)
        lambda_sol=np.array(lambda_sol) * max_current
        currents_list = [0,0,0,0,0,0,0,0]
        currents_list[0] = lambda_sol[0]
        currents_list[5] = -currents_list[0]
        currents_list[4] = lambda_sol[1]
        currents_list[1] = -currents_list[4]
        currents_list[2] = lambda_sol[3]
        currents_list[7] = -currents_list[2]
        currents_list[6] = lambda_sol[2]
        currents_list[3] = -currents_list[6]
        self.controller.new_coil_state_all(coils_values_list=currents_list)

        if show_lambda_values:
            print(f'Lambda solution {lambda_sol}, \n Currents list {currents_list}')


    def all_off(self):
        """
        Turn magnets off
        :return:
        """
        self.controller.all_off()


    def close_connection(self):
        """
        Close serial connection
        :return:
        """
        self.controller.close_connection()


class RotationSystem:
    """
    Class for the rotation of the bot on spot
    """

    def __init__(self, controller: CoilsControllerInterface):
        """
        """
        self.controller = controller
        self.current_solver = GeomCurrentDist8EM()
        self.dir_vec = np.array([0, 0, 1])


    def update_dir_vec(self, new_dir_vec):
        """ Update direction vector. Used to turn the bot
        :return:
        """
        self.dir_vec = LinAlg.vec_to_1(np.array(new_dir_vec))


    def new_coil_state_all(self, pre_normal_force=True, max_current=1, show_lambda_values=False):
        """
        Send new state to magnets. Convert the field direction vector to currents distribution (solve)
        and send values to arduino boards
        :param pre_normal_force: boolean flag, normalize the force vector before calculation of currents
        :param max_current: maximal current in coils. 0-255 or 0-1 (depends on scale_255)
        :return:
        """

        lambda_sol = self.current_solver.solve_system(self.dir_vec, pre_normal_force=pre_normal_force)
        lambda_sol = np.array(lambda_sol) * max_current
        currents_list = [0, 0, 0, 0, 0, 0, 0, 0]
        currents_list[0] = lambda_sol[0]
        currents_list[5] = -currents_list[0]
        currents_list[4] = lambda_sol[1]
        currents_list[1] = -currents_list[4]
        currents_list[2] = lambda_sol[3]
        currents_list[7] = -currents_list[2]
        currents_list[6] = lambda_sol[2]
        currents_list[3] = -currents_list[6]
        self.controller.new_coil_state_all(coils_values_list=currents_list)

        if show_lambda_values:
            print(f'Lambda solution {lambda_sol}, \n Currents list {currents_list}')


    def all_off(self):
        """
        Turn magnets off
        :return:
        """
        self.controller.all_off()


    def close_connection(self):
        """
        Close serial connection
        :return:
        """
        self.controller.close_connection()



class GeomCurrentDist8EM:
    """
    Calculator for coils current distribution - simple geometrical approach
    """

    def __init__(self):
        self.Fx = 0
        self.Fy = 0
        self.Fz = 0

    def fun(self, variables):
        """function to create system of equations"""

        Fl1, Fl2, Fl3, Fl4 = variables
        c = cos(pi / 4)
        eqn_1 = Fl1 * c - Fl2 * c - self.Fx
        eqn_2 = Fl1 * c + Fl2 * c - 0.5 * self.Fy
        eqn_3 = Fl3 * c - Fl4 * c - self.Fz
        eqn_4 = Fl3 * c + Fl4 * c - 0.5 * self.Fy

        return [eqn_1, eqn_2, eqn_3, eqn_4]


    def solve_system(self, vec_3d, pre_normal_force=False):
        """
        Find current distribution in 8Em for the simplified geometrical approach.
        Input is the force vector [Fx,Fy,Fz] in 8EM associated coordinate system.
        Output is returned in Lambda-coordinates - coordinate system, associated with magnetic
        azes of coils [Fl1, Fl2, Fl3, Fl4]. Coordinates of Fl axes in X-Y-Z basis are
        Fl1 = [1,1,0], Fl2 = [-1,1,0], Fl3 = [0,1,1], Fl4 = [0,1,-1]

        :param vec_3d: Desired force vector in 8EM
        :param pre_normal_force: boolean flag to normalize the force vector before calculation
        :return: 1-normalized list [Fl1, Fl2, Fl3, Fl4]
        """

        vec_3d = np.array(vec_3d)
        vec_len = np.linalg.norm(vec_3d)

        if pre_normal_force and vec_len != 0:
            vec_3d_norm = vec_3d / vec_len
        else:
            vec_3d_norm = vec_3d
        c = cos(pi / 4)
        self.Fx = vec_3d_norm[0]
        self.Fy = vec_3d_norm[1]
        self.Fz = vec_3d_norm[2]
        # initial approximation
        r1 = random.uniform(-10, 10)
        r2 = random.uniform(-10, 10)
        r3 = random.uniform(-10, 10)
        r4 = random.uniform(-10, 10)
        # solver
        Fl1, Fl2, Fl3, Fl4 = scipy.optimize.anderson(self.fun, (r1, r2, r3, r4))
        res = (Fl1, Fl2, Fl3, Fl4)
        abs_res = [abs(Fl1), abs(Fl2), abs(Fl3), abs(Fl4)]
        if max(abs_res) > 1:
            res = [number / max(abs_res) for number in res]

        return res


class LinAlg:
    """
    Class with some aux functions to work with vectors
    """

    @classmethod
    def vec_to_1(cls, v):
        """ Normalizes vector to length=1"""
        v1 = np.array(v)
        vres = [-1, -1, -1]
        if np.linalg.norm(v1) > 0:
            vres = v1 / np.linalg.norm(v1)
        else:
            if len(v) == 2:
                vres = np.array([0, 0])
            if len(v) == 3:
                vres = np.array([0, 0, 0])

        return vres


    @classmethod
    def vec_angle_rad(cls, v1, v2):
        """ Returns angle between v1 and v2 (radians)"""

        v11 = cls.vec_to_1(v1)
        v22 = cls.vec_to_1(v2)

        c = np.dot(v11, v22) / (np.linalg.norm(v11) * np.linalg.norm(v22))

        if -1 <= c <= 1:
            angle = math.acos(c)
        else:
            angle = 0
        return angle


    @classmethod
    def vec_angle_deg(cls, v1, v2):
        """Returns angle between v1 and v2 in degrees"""

        return math.degrees(cls.vec_angle_rad(v1, v2))


    @classmethod
    def rotate_vec(cls, vec, axis, angle):
        """ Rotates vector vec around axis on angle (radian)"""

        x, y, z = axis
        teta = angle

        M_1 = [cos(teta) + (1 - cos(teta)) * (x ** 2),
               (1 - cos(teta)) * x * y - (sin(teta)) * z,
               (1 - cos(teta)) * x * z + (sin(teta)) * y]
        M_2 = [(1 - cos(teta)) * y * x + (sin(teta)) * z,
               cos(teta) + (1 - cos(teta)) * y * y,
               (1 - cos(teta)) * y * z - (sin(teta)) * x]
        M_3 = [(1 - cos(teta)) * z * x - (sin(teta)) * y,
               (1 - cos(teta)) * z * y + (sin(teta)) * x,
               cos(teta) + (1 - cos(teta)) * z * z]

        M = np.array([M_1, M_2, M_3])

        return M.dot(vec)


    @classmethod
    def perp2vec(cls, vec):
        """ Returns random perpendicular for the vector """
        vec_1 = np.array(vec)
        vec_2 = vec_1.copy()
        if vec_2[0] != -1:
            vec_2[0] = vec_2[0] + 1
        elif vec_2[1] != -1:
            vec_2[1] = vec_2[1] + 1
        elif vec_2[2] != -1:
            vec_2[2] = vec_2[2] + 1
        else:
            vec_2[0] = vec_2[0] + 1.5
        perp_vec = np.cross(vec_1, vec_2)

        return perp_vec



if __name__ == '__main__':
    # How to use this module:
    # For example we have a system, where coils are controlled with BTS7960 drivers
    # connected to Arduino-A and Arduino-B, temperature is measured with thermistors with
    # same Arduino boards and light is controlled with a single BTS7960 driver from Arduino-A.

    # Create Arduino device objects for communication with phisical devices:
    arduino_a = ArduinoMegaBoard(port='COM1')
    arduino_b = ArduinoMegaBoard(port='COM2')

    # Create coils contorollers for coils 1..4 and 5..8
    controller_1_4 = CoilBTS7960ArduinoController(arduino_board=arduino_a)
    controller_5_8 = CoilBTS7960ArduinoController(arduino_board=arduino_b)
    # and make one low-level controller for the whole system
    all_controller = BDCoilsControllerBTS7960(controller_1_4=controller_1_4, controller_5_8=controller_5_8)
    # create high-level controller for the ALI bot operation
    ali_system = ALISystem(controller=all_controller)
    # Now we can control coils using ali_system object

    # We need to create also light controller and temperature controller
    light_controller = LightBTS7960ArduinoController(arduino_board=arduino_a)
    thermistor_voltmeter_1_4 = ArduinoMegaVoltmeter(arduino_board=arduino_a)
    thermistor_voltmeter_5_8 = ArduinoMegaVoltmeter(arduino_board=arduino_a)

