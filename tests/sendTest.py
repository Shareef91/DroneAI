import argparse
import sys
import time
import board
from functools import lru_cache
from datetime import datetime
import os
import cv2
import numpy as np
import serial
import base64
import Adafruit_DHT
import threading
from queue import Queue
from adafruit_bme280 import basic as adafruit_bme280


last_detections = []
last_sent_time = 0
COOLDOWN_SEC = 20
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
CHUNK_SIZE = 200
MAX_RETRIES = 3

running = True

wQueue = Queue()
imgQueue = Queue()
objQueue = Queue()

class WeatherData:
    def __init__(self, time="0", temp=0, humidity=0, pressure=0, altitude=0):
        self.time = time
        self.temp = temp
        self.humidity = humidity
        self.pressure = pressure
        self.altitude = altitude

WeatherStruct = WeatherData()

def read_weather():
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)
    bme280.sea_level_pressure = 1018.5

    while running:
        temperature = bme280.temperature
        humidity = bme280.relative_humidity
        pressure = bme280.pressure
        altitude = bme280.altitude
        timestamp = datetime.now()

        # add to struct
        WeatherStruct.time = timestamp
        WeatherStruct.temp = temperature
        WeatherStruct.humidity = humidity
        WeatherStruct.pressure = pressure
        WeatherStruct.altitude = altitude

        # print(f"Temperature: {temperature:.1f} C")
        # print(f"Humidity: {humidity:.1f} %")
        # print(f"Pressure: {pressure:.1f} hPa")
        # print(f"Altitude: {altitude:.2f} meters")

        # add data to a csv file with a column for each data point as well as time.
        with open("weather_data.csv", "a") as f:
                f.write(f"{timestamp},{float(temperature):.1f},{float(humidity):.1f},{float(pressure):.1f},{float(altitude):.2f}\n")
        # push the data to a queue for transmission (first in first out)
        wQueue.put(WeatherStruct)

        time.sleep(5)



class LoRaTransmitter:
    def __init__(self, port="/dev/ttyS0", baudrate=9600):
        self.ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)
        self.weatherCounter = 0
        self.detected_objects = set()
        self.detected_recently = {}

    def send(self, message):
        retry_count = 0
        while retry_count < MAX_RETRIES:
            self.ser.write((message + "\n").encode('utf-8'))
            start_time = time.time()
            ack_received = False
            while time.time() - start_time < 2:
                if self.ser.in_waiting:
                    ack = self.ser.readline().decode('utf-8').strip()
                    if ack == f"ACK {message}":
                        ack_received = True
                        break
            if ack_received:
                print(f"acknowledged.")
                break
            else:
                retry_count += 1
                print(f"not acknowledged. Retry {retry_count}/{MAX_RETRIES}...")
                time.sleep(0.2)


    def send_weather(self, weatherToSend):
        message = f"W:{self.weatherCounter},{weatherToSend.time},{weatherToSend.temp:.2f},{weatherToSend.humidity:.2f},{weatherToSend.pressure:.2f},{weatherToSend.altitude:.2f}"
        retry_count = 0
        while retry_count <= MAX_RETRIES:
            self.ser.write((message + "\n").encode('utf-8'))
            start_time = time.time()
            ack_received = False
            while time.time() - start_time < 2:
                if self.ser.in_waiting:
                    ack = self.ser.readline().decode('utf-8').strip()
                    if ack == f"ACK {self.weatherCounter}":
                        ack_received = True
                        break
            if ack_received:
                print(f"Weather Packet {self.weatherCounter} acknowledged.")
                break
            else:
                retry_count += 1
                print(f"Weather Packet {self.weatherCounter} not acknowledged. Retry {retry_count}/{MAX_RETRIES}...")
                time.sleep(0.2)
        self.weatherCounter += 1

    def loop(self):
        while running:
            if not wQueue.empty():
                self.send_weather(wQueue.get())
            if not objQueue.empty():
                objID = objQueue.get()
                now = time.time()

                expired = [k for k, v in self.detected_recently.items() if now - v > COOLDOWN_SEC]
                for k in expired:
                    del self.detected_recently[k]

                if objID not in self.detected_recently:
                    self.send("OBJ:" + objID)  # Send object ID
                    self.detected_recently[objID] = now
            if not imgQueue.empty():
                imgPath = imgQueue.get()
                if imgPath in self.detected_objects:
                    continue
                self.send_image(imgPath)
                #add to detected objects
                self.detected_objects.add(imgPath)

    
# --- Camera Detection Class ---
class CameraDetector:
    def __init__(self):
        self.camera = Picamera2()
        self.camera.start()
        self.image_counter = 0

    def capture_and_queue(self):
        frame = self.camera.capture_array()
        _, buffer = cv2.imencode('.jpg', frame)
        jpg_bytes = buffer.tobytes()
        b64 = base64.b64encode(jpg_bytes).decode('utf-8')
        image_id = f"img_{self.image_counter}.jpg"
        imgQueue.put((b64, image_id))
        objQueue.put(image_id)
        # Optional: Save locally
        cv2.imwrite(f"detections/{image_id}", frame)
        print(f"Frame captured and queued: {image_id}")
        self.image_counter += 1

    def send_image(self, image_path):
        with open(image_path, "rb") as img_file:
            b64_data = base64.b64encode(img_file.read()).decode("utf-8")

        total = (len(b64_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        print(f"Sending image in {total} packets...")
        print("Image transmission ready. Sending object ID")
        self.send("ID:" + image_path.split('/')[-1])  # Send the image filename as object ID

        for i in range(total):
            while not wQueue.empty():
                self.send_weather(wQueue.get())
            while not objQueue.empty():
                self.send("OBJ:" + objQueue.get())
            part = b64_data[i * CHUNK_SIZE:(i + 1) * CHUNK_SIZE]
            packet = f"{i+1}/{total}:{part}\n"
            retry_count = 0
            while retry_count <= MAX_RETRIES:
                self.ser.write(packet.encode('utf-8'))
                start_time = time.time()
                ack_received = False
                while time.time() - start_time < 2:
                    if self.ser.in_waiting:
                        ack = self.ser.readline().decode('utf-8').strip()
                        if ack == f"ACK {i+1}":
                            ack_received = True
                            break
                if ack_received:
                    print(f"Packet {i+1}/{total} acknowledged.")
                    break
                else:
                    retry_count += 1
                    print(f"Packet {i+1}/{total} not acknowledged. Retry {retry_count}/{MAX_RETRIES}...")
                    time.sleep(0.2)
            time.sleep(0.1)

    def close(self):
        self.ser.close()


if __name__ == "__main__":
    LoRaSend = LoRaTransmitter()
    threading.Thread(target=read_weather, daemon=True).start()
    threading.Thread(target=LoRaSend.loop, daemon=True).start()
    objQueue.put("I SEE YOU")  # Example object ID to send
    objQueue.put("I SEE YOU AGAIN")  # Example object ID to send
    objQueue.put("I SEE YOU A THIRD TIME")  # Example object ID to send
    objQueue.put("I SEE YOU A THIRD TIME")
    objQueue.put("TESTYPOO")
    #imgQueue.put("img_14.png")  # Example image to send
    #wait 25 sec
    time.sleep(25)
    objQueue.put("I SEE YOU A THIRD TIME")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
        running = False
        LoRaSend.close()
        sys.exit(0)
