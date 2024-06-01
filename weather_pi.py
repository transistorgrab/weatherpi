# this script takes data from several sensors and transmits them to opensensmap.org
# Author: transistorgrab
# Date: 2024-05-12

# tested with Python 3.11

# requires: gpiozero, DHT11, requests

from gpiozero import Button ## used to read the input from the anemometer (wind information),
                            #  automatically sets pull-up active
import RPi.GPIO as GPIO ## required for the DHT11 sensor, since gpiozero is not useable for this application
import dht11    ## reads DHT11 temperature and humidity sensor
import serial   ## required for reading the air quality sensor
import requests ## used to connect to opensensmap
import collections ## used to implement a fifo data buffer
import time
import math
import threading ## used to run periodic tasks like fetching serial data
import json

debug = True

class Wind:
    def __init__(self):
        self.radius_mm   = 90
        self.pulses_per_revolution = 2.0
        self.corr_factor = 1.18
        self.port        = "GPIO18"
        self.impulses    = 0
        self.started     = time.time()
        print (f"wind sensor initiated") if debug else ()
    def init_sensor(self):
        self.sensor      = Button(self.port)
        self.sensor.when_pressed = self.count_impulse
        self.started     = time.time()
        self.impulses    = 0
        self.circumfence = ((2 * math.pi) * self.radius_mm) / 1000.0
    def count_impulse(self):
        print(f"pulses: {self.impulses}") if debug else ()
        self.impulses += 1
    def get_wind_speed(self):
        ''' first reads the collected pulses and time period,
            and starts the new count cycle 
            by setting a new "started" time and resetting the counter
            then calculates the wind speed in m/s'''
        revolutions   = self.impulses / self.pulses_per_revolution
        read_time     = time.time()
        started       = self.started
        self.started  = read_time
        self.impulses = 0 ## reset impulses for next read cycle
        ## now we calculate the result: way traveled divided by time elapsed
        speed = (self.circumfence * revolutions) / (read_time - started) * self.corr_factor
        return (speed)

class Ser:
    ''' holds the serial interface parameters, provides data from the serial interface'''
    def __init__(self):
        self.port   = "/dev/serial0"
        self.bauds  = 9600
        self.size   = serial.EIGHTBITS
        self.parity = serial.PARITY_NONE
        self.stop   = serial.STOPBITS_ONE
        self.data   = collections.deque()   ## this is a fifo that holds the data read from serial
        self.period_min = self.period_min_seconds() ## minimum period on seconds how often serial data can be read
        self.period = 3                     ## period in seconds how often to read serial data
        self.connection = serial.Serial(port = self.port, baudrate = self.bauds,
                               parity = self.parity, stopbits = self.stop,
                               bytesize = self.size, timeout = 1)
        print (f"serial port {self.port} initiated") if debug else ()
    def period_min_seconds(self):
        return 3 ## implementing a static value here, serial cannot be read more often than that 
    def post (self,data):
        self.data.appendleft(self,data)
    def get (self):
        if self.data:
            return self.data.pop()
        else:
            return None
#    def get_connection(self):
#        print (f"setting up serial port {self.port}") if debug else ()
#        return serial.Serial(port = self.port, baudrate = self.bauds,
#                               parity = self.parity, stopbits = self.stop,
#                               bytesize = self.size, timeout = 1)
 
    def start_cyclic_read(self):
        bytes_received = self.connection.inWaiting()
        self.post(connection.read(bytes_received))
        wait_period = self.period if self.period > self.period_min_seconds() else self.period_min_seconds()
        self.cyclic_read = threading.Timer(wait_period, lambda: self.start_cyclic_read) ## lambda prevents spawning multiple threads
    def stop_cyclic_read(self):
        self.cyclic_read.cancel()
    def send_data(self,data):
        self.connection.write(data)

class Air:
    ''' holds data and provides methods to communicate to the air sensor'''
    def __init__(self):
        self.air_request_inverval_s = 3 ## not less than 3 seconds
        self.old_data     = list() ## holds data that does not fit a whole data frame (tail missing)
        self.command_len  = 19 ## length of command packets
        self.response_len = 10 ## length of response packets
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
            "get_sleep_mode"      : [0xaa,0xb4,0x06,0x00,0x00,0,0,0,0,0,0,0,0,0x01,0x01,0xff,0xff,0x00,0xab],
            "set_sleep_mode_on"   : [0xaa,0xb4,0x06,0x01,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_sleep_mode_off"  : [0xaa,0xb4,0x06,0x01,0x01,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "get_firmware_version": [0xaa,0xb4,0x07,0x00,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "get_working_period"  : [0xaa,0xb4,0x08,0x00,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
            "set_working_period"  : [0xaa,0xb4,0x08,0x01,0x00,0,0,0,0,0,0,0,0,0x00,0x00,0xff,0xff,0x00,0xab],
                        }
        self.reply_type_id   = {0xC5:"report mode",
                                    0xC0:"sensor data"}
        self.sleep_mode_response = {0x00:"sleeping",
                                    0x01:"working",}
         
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
        if (not len(data) == self.command_len) & (not len(data) == self.response_len):
            raise ValueError(f"data must have {self.command_len} or {self.response_len} bytes,"
                            +f" given data has {len(data)} bytes.")
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
            command_data[-2] = self.get_checksum(command_data)
            return command_data
        else: 
            return f"unknown command '{command}', available commands are {self.commands.keys()}" 

    def decode (self,data,head = 0xaa, tail = 0xab):
        ''' takes a bytes object of data, decodes it and returns a dict of values '''
        if self.old_data: ## if we have old data from an earlier run
            print (f"found stored data from earlier: {self.old_data}") if debug else ()
            byte_list = self.old_data.extend(list(data))
        else:
            byte_list = list(data)
        print (byte_list) if debug else ()
        packet_list = list()
        data_packet = list()
        reply = dict()
        if byte_list:
            for byte in byte_list:
                data_packet.append(byte)
                if byte == tail: ## we found the tail byte
                    packet_list.append(data_packet) ## appends the packet to the list
                    print (f"packet: {data_packet}") if debug else ()
                    data_packet = [] ## clears the packet storage
            if data_packet: ## we have some bytes left, last byte was not == tail
                print (f"storing data for later: {data_packet}") if debug else ()
                self.old_data = data_packet
            print (f"Packet list: {len(packet_list)} packet(s)") if debug else ()
    
            for packet in packet_list: ## we now inspect the collected packets
                if (not packet[0] == head): ## we have a defective packet, skip this
                    continue
                if (not len(packet) == self.response_len): ## packet is too short/long
                    print (f"Packet length mismatch: {len(packet)}, {packet}") if debug else ()
                    continue
                if (not packet[-2] == self.get_checksum(packet)): ## checking if checksum matches data
                    print (f"Packet checksum mismatch: {packet[-2]} vs {self.get_checksum(packet)}") if debug else ()
                    continue
                if (self.reply_type_id[packet[1]] == "report mode"):
                    print (f"found report: {packet}") if debug else ()
                    pass
                if (self.reply_type_id[packet[1]] == "sensor data"):
                    print (f"found sensor data: {packet}") if debug else ()
                    reply["PM2.5"] = int(packet[3]<<8) + int(packet[2])
                    print (f"PM2.5: {reply['PM2.5']}") if debug else ()
                    reply["PM10"]  = int(packet[5]<<8) + int(packet[4])
                    print (f"PM10: {reply['PM10']}") if debug else ()
                    reply["ID"]    = int(packet[7]<<8) + int(packet[6])
                    print (f"ID: {reply['ID']}") if debug else ()
            return reply


class TH:
    ''' provides data and methods to get temperature and humidity information '''
    def __init__(self):
        self.port = 4  ## pin 7 == "GPIO4"
        self.error = {1:"missing data",2:"CRC faulty"}
    def init_sensor(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # GPIO.cleanup()
        self.sensor = dht11.DHT11(pin = self.port)
    def get(self):
        return self.sensor.read()

if __name__ == "__main__":
    temp_hum = TH()
    temp_hum.init_sensor()
    wind = Wind()
    wind.init_sensor()
    air = Air()
    ser = Ser()
    ser.send_data(air.get_command_data("set_mode_query"))
    
    while (1):
        print(f"Wind reading: {wind.get_wind_speed()}")
        th_read = temp_hum.get()
        if th_read.is_valid():
            print(f"Temperature reading: {th_read.temperature}, Humidity: {th_read.humidity}")
        else:
            print(f"There was an error: {temp_hum.error[th_read.error_code]}")
        print(f"work mode originally: {air.get_command_data('get_work_mode')}")
        ser.send_data(air.get_command_data("get_work_mode"))
        ser.send_data(air.get_command_data("get_particle_data"))
        if ser.connection.in_waiting:
            air_response=ser.connection.read(ser.connection.in_waiting)
            print(air_response)
            decoded = air.decode(air_response)
            print (f"decoded:\n{decoded}")
        else: 
            print(f"no air data response")
    
        time.sleep(5)
    
