import threading
import time
import argparse
import serial
import re

class LoRaReceiver:

    def __init__(self, port="/dev/ttyS0", baudrate=9600):
        self.ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)
        
    def receiver(self):
        received_packets = {}
        expected_total = None

        while running:
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
                else:
                    self.ID_receive(data)
            except Exception as e:
                print(f"Error: {e}")
                continue
