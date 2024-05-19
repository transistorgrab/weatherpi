# this script takes data from several sensors and transmits them to opensensmap.org
# Author: transistorgrab
# Date: 2024-05-12

# tested with Python 3.11

# requires: RPi.GPIO, gpiozero, DHT11, requests

from gpiozero import Button ## used to read the input from the anemometer (wind information)
import DHT11    ## reads DHT11 temperature and humidity sensor
import serial   ## required for reading the air quality sensor
import requests ## used to connect to opensensmap
import collections ## used to implement a fifo data buffer
import time
import threading ## used to run periodic tasks like fetching serial data
import json

debug = True

# parameters 
anemometer_radius_cm   = 9
anemometer_corr_factor = 1.18

class Ser:
    ''' holds the serial interface parameters, provides data from the serial interface'''
    def __init__(self):
        self.port   = "/dev/serial0"
        self.bauds  = 9600
        self.size   = serial.EIGHTBITS
        self.parity = serial.PARYTY_NONE
        self.stop   = serial.STOPBITS_ONE
        self.data   = collections.deque()   ## this is a fifo that holds the data read from serial
        self.period_min = self.period_min_seconds() ## minimum period on seconds how often serial data can be read
        self.period = 3                     ## period in seconds how often to read serial data
    def period_min_seconds(self):
        return 3 ## implementing a static value here, serial cannot be read more often than that 
    def post (data):
        self.data.appendleft(self,data)
    def get (self):
        return self.data.pop
    def get_connection(self):
        return serial.Serial(port = self.port, baudrate = self.bauds,
                               parity = self.parity, stopbits = self.stop,
                               bytesize = self.size, timeout = 1)
 
    def start_cyclic_read(self):
        connection = self.get_connection()
        bytes_received = connection.inWaiting()
        self.post(connection.read(bytes_received))
        wait_period = self.period if self.period > self.period_min_seconds() else self.period_min_seconds()
        self.cyclic_read = threading.Timer(wait_period, lambda: self.start_cyclic_read) ## lambda prevents spawning multiple threads
    def stop_cyclic_read(self):
        self.cyclic_read.cancel()
    def send_data(self,data):
        connection = self.get_connection()
        connection.write(data)

class Air:
    ''' holds data and provides methods to communicate to the air sensor'''
    def __init__(self):
        self.air_request_inverval_s = 3 ## not less than 3 seconds
        self.commands = { ## per default commands address all devices (0xff in the address bytes)
            "get_work_mode"       : [0xaa,0xb4,0x02,0x00,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_mode_active"     : [0xaa,0xb4,0x02,0x01,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_mode_query"      : [0xaa,0xb4,0x02,0x01,0x01,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "get_particle_data"   : [0xaa,0xb4,0x04,0x00,0x01,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_device_id"       : [0xaa,0xb4,0x05,0x01,0x01,0,0,0,0,0,0,0,0,0x01,0x01,0xff,0xff,0x00,0xab],
            "get_sleep_mode"      : [0xaa,0xb4,0x06,0x00,0x00,0,0,0,0,0,0,0,0,0x01,0x01,0xff,0xff,0x00,0xab],
            "set_sleep_mode_on"   : [0xaa,0xb4,0x06,0x01,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_sleep_mode_off"  : [0xaa,0xb4,0x06,0x01,0x01,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "get_firmware_version": [0xaa,0xb4,0x07,0x00,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "get_working_period"  : [0xaa,0xb4,0x08,0x00,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_working_period"  : [0xaa,0xb4,0x08,0x01,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
                        }
        self.air_reply_type_id   = {0xC5:"report mode",
                                    0xC0:"sensor data"}
        self.sleep_mode_response = {[0x00,0x00]:"sleeping",
                                    [0x00,0x01]:"working",}
         
        ''' 
            10 data bytes from sensor
            air sensor data format example: "aa c0 8c 00 94 00 bc f9 d5 ab"
                                aa: head,
                                c0: reply type ID,
                                8c 00: PM2.5,
                                94 00: PM10,
                                00 bc: device ID,
                                d5: checksum (sum of "8c" to "f9", lowest 8 bits),
                                ab: tail
            19 data bytes to sensor
            command to sensor example: "aa b4 04 00 00 00 00 00 00 00 00 00 00 00 00 ff ff 00 ab"
                                aa: head,
                                b4: command ID (query sensor data),
                                04: query data command,
                                00...00 : reserved bytes,
                                ff ff: device ID: FF -> all devices,
                                00: checksum byte,
                                ab: tail
        '''
    
    def get_checksum (self,data):
        ''' returns the checksum for a given data list, either 19 byte to sensor or 10 bytes from sensor
            checksum is low 8 bits of sum of data bytes '''
        if (not len(data) == 19) & (not len(data) == 10):
            raise ValueError(f"data must have 19 or 10 bytes, given data has {len(data)} bytes.")
            return False
        checksum = sum(data[2:-2]) & 0xff
        return checksum
    
    def get_command_data(self,command:str,dev_id:int=0xFFFF,work_period:int=0):
        ''' returns a list of bytes for the given command and device ID of the air sensor '''
        if command in self.commands:
            command_data = self.commands[command]
            if work_period:
                work_data = 30 if work_period > 30 else work_period
                command_data[4] = work_data
            command_data[-4] = dev_id >> 8
            command_data[-3] = dev_id & 0xFF
            command_data[-2] = get_checksum(command_data)
            return command_data
        else: 
            return f"unknown command '{command}', available commands are {self.commands.keys()}" 





