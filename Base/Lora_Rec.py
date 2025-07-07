import threading
import time
import argparse
import serial
import re
import sys
import os
import base64
from queue import Queue
sys.path.append(os.path.dirname(os.path.abspath("weather_data.py")))
from weather_data import WeatherData

# wQueue = Queue()
# imgQueue = Queue()

class LoRaReceiver:

    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        self.ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)
        
        self.received_packets = {}
        self.expected_total = None
        self.img_name = None
        self.wQueue = Queue()
        self.imgQueue = Queue()
        self.objQueue = Queue()

    def receiver(self):

        while True:
            data_read = self.ser.readline()
            if not data_read:
                continue
            try:
                data = data_read.decode("utf-8").strip()
                # if the data starts with W:, call weather receive function
                if data.startswith("W:"):
                    self.weather_receive(data)
                    continue
                # if the data starts with a fraction followed by a colon, call image receive function
                if re.match(r"^\d+/\d+:", data):
                    self.image_receive(data)
                    continue
                if data.startswith("ID:"):
                    self.ID_receive(data)
                    continue
                if data.startswith("OBJ:"):
                    self.OBJ_rec(data)
            except Exception as e:
                print(f"Error: {e}")
                continue

    def weather_receive(self, data):
        wData = data[2:]
        # parse data into weather struct
        try:
            weatherCounter, time, temp, humidity, pressure, altitude = wData.split(',')
            weather_data = WeatherData(time, float(temp), float(humidity), float(pressure), float(altitude))
            self.wQueue.put(weather_data)
            ack_msg = f"ACK {weatherCounter}\n"
            self.ser.write(ack_msg.encode('utf-8'))
            print(f"Weather Data Received: {weather_data.__dict__}")
        except ValueError as e:
            print(f"Error parsing weather data: {e}")

    def image_receive(self, data):

        header, packet = data.split(':', 1)
        idx, total = header.split('/')
        idx = int(idx)
        total = int(total)

        if self.expected_total != total:
            self.expected_total = total
            self.received_packets.clear()
        self.received_packets[idx] = packet
        ack_msg = f"ACK {idx}\n"
        self.ser.write(ack_msg.encode('utf-8'))
        print(f"Received packet {idx}/{total}")
        if len(self.received_packets) == self.expected_total:
            print("All packets received. Reconstructing image...")
            b64_data = ''.join([self.received_packets[i] for i in range(1, self.expected_total + 1)])
            img_data = base64.b64decode(b64_data)
            self.expected_total = None
            self.received_packets.clear()
            # save image as img name
            if self.img_name:
                with open(self.img_name, 'wb') as img_file:
                    img_file.write(img_data)
                print(f"Image saved as {self.img_name}")
            else:
                with open('unknown_image.jpg', 'wb') as img_file:
                    img_file.write(img_data)
                    self.img_name = 'unknown_image.jpg'
                print("Image name not set. saved as unknown image.jpg")
            self.imgQueue.put(self.img_name)
            self.img_name = None  # Reset image name for next transmission
        
    def ID_receive(self, data):
        self.img_name = data[3:].strip()
        print(f"Image name set to: {self.img_name}")
        self.ser.write(f"ACK {data}\n".encode('utf-8'))
    
    def OBJ_rec(self, data):
        obj_data = data[4:].strip()
        print(f"Object Data Received: {obj_data}")
        self.objQueue.put(obj_data)
        self.ser.write(f"ACK {data}\n".encode('utf-8'))