# imports (unchanged)
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
from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
from picamera2.devices.imx500 import (
    NetworkIntrinsics,
    postprocess_nanodet_detection
)

# Constants
last_detections = []
last_sent_time = 0
DET_WAIT_SEC = 20
COOLDOWN_SEC = 3
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
CHUNK_SIZE = 200
MAX_RETRIES = 3
running = True

wQueue = Queue()
imgQueue = Queue()
objQueue = Queue()

class Detection:
    def __init__(self, coords, category, conf, metadata):
        self.category = category
        self.conf = conf
        self.box = intrinsics.convert_inference_coords(coords, metadata, picam2)

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
    bme280.sea_level_pressure = 1015

    while running:
        try:
            temperature = bme280.temperature
            humidity = bme280.relative_humidity
            pressure = bme280.pressure
            altitude = bme280.altitude
            timestamp = datetime.now()

            WeatherStruct.time = timestamp
            WeatherStruct.temp = temperature
            WeatherStruct.humidity = humidity
            WeatherStruct.pressure = pressure
            WeatherStruct.altitude = altitude

            with open("weather_data.csv", "a") as f:
                f.write(f"{timestamp},{temperature:.1f},{humidity:.1f},{pressure:.1f},{altitude:.2f}\n")

            wQueue.put(WeatherStruct)
        except Exception as e:
            print(f"Weather read error: {e}")
        time.sleep(10)

def save_cropped_detections(picam2, detections, folder="detections"):
    frame = picam2.capture_array()
    os.makedirs(folder, exist_ok=True)
    paths = []
    for det in detections:
        x1, y1, x2, y2 = map(int, det.box)
        cropped = frame[y1:y2, x1:x2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(folder, f"{det.category}_{timestamp}.jpg")
        cv2.imwrite(filename, cropped, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        paths.append((filename, det.category))
    return paths

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
        ack_tag = f"ACK {self.weatherCounter}"

        while retry_count <= MAX_RETRIES:
            self.ser.write((message + "\n").encode('utf-8'))
            start_time = time.time()
            ack_received = False
            while time.time() - start_time < 2:
                if self.ser.in_waiting:
                    ack = self.ser.readline().decode('utf-8').strip()
                    print(f"Received: {ack}")
                    if ack == ack_tag:
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
                self.obj_Check(objID)
            if not imgQueue.empty():
                imgPath = imgQueue.get()
                if imgPath in self.detected_objects:
                    continue
                self.send_image(imgPath)
                self.detected_objects.add(imgPath)

    def obj_Check(self, objID):
        now = time.time()
        expired = [k for k, v in self.detected_recently.items() if now - v > COOLDOWN_SEC]
        for k in expired:
            del self.detected_recently[k]
        if objID not in self.detected_recently:
            self.send("OBJ:" + objID)
            self.detected_recently[objID] = now

    def send_image(self, image_path):
        with open(image_path, "rb") as img_file:
            b64_data = base64.b64encode(img_file.read()).decode("utf-8")

        total = (len(b64_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        image_name = os.path.basename(image_path)
        object_id = image_name.split('_')[0]
        print(f"Sending image in {total} packets...")
        print("Sending object ID:", object_id)
        self.send(f"ID:{object_id}")

        for i in range(total):
            while not wQueue.empty():
                self.send_weather(wQueue.get())
            while not objQueue.empty():
                self.obj_Check(objQueue.get())
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

def parse_detections(metadata: dict):
    global last_detections, last_sent_time
    bbox_normalization = intrinsics.bbox_normalization
    bbox_order = intrinsics.bbox_order
    threshold = args.threshold
    iou = args.iou
    max_detections = args.max_detections

    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    input_w, input_h = imx500.get_input_size()
    if np_outputs is None:
        return last_detections
    if intrinsics.postprocess == "nanodet":
        boxes, scores, classes = postprocess_nanodet_detection(
            outputs=np_outputs[0], conf=threshold, iou_thres=iou, max_out_dets=max_detections
        )[0]
        from picamera2.devices.imx500.postprocess import scale_boxes
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
    else:
        boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
        if bbox_normalization:
            boxes = boxes / input_h
        if bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]
        boxes = np.array_split(boxes, 4, axis=1)
        boxes = zip(*boxes)

    last_detections = [
        Detection(box, category, score, metadata)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold
    ]

    if last_detections and time.time() - last_sent_time > COOLDOWN_SEC:
        categories = list(set([det.category for det in last_detections]))
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Detected: {', '.join(categories)}")
        objQueue.put(",".join(categories))
        for path, category in save_cropped_detections(picam2, last_detections):
            imgQueue.put(path)
            objQueue.put(category)
        last_sent_time = time.time()

def draw_detections(request):
    with MappedArray(request, "main") as m:
        for detection in last_detections:
            x1, y1, x2, y2 = detection.box
            cv2.rectangle(m.array, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0, 0), 2)
            label = f"{detection.category}: {detection.conf:.2f}"
            cv2.putText(m.array, label, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI Detection with LoRa transmission")
    parser.add_argument("--model", type=str, default="/home/team3box/Documents/DroneAI/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")
    parser.add_argument("--threshold", type=float, default=0.55, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="IoU threshold")
    parser.add_argument("--max-detections", type=int, default=10)
    parser.add_argument("--lora-port", type=str, default="/dev/ttyS0")
    parser.add_argument("--lora-baudrate", type=int, default=9600)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()

    try:
        print("Initializing camera...")
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480)}, controls={"FrameRate": 30})
        config['ai'] = {"model": args.model}
        picam2.configure(config)

        intrinsics = NetworkIntrinsics()
        imx500 = picam2

        lora = LoRaTransmitter(port=args.lora_port, baudrate=args.lora_baudrate)

        if args.preview:
            picam2.post_callback = draw_detections

        picam2.start(show_preview=args.preview)

        while not picam2.capture_metadata():
            time.sleep(0.1)

        print("Camera and AI initialized.")

        weather_thread = threading.Thread(target=read_weather, daemon=True)
        lora_thread = threading.Thread(target=lora.loop, daemon=True)
        weather_thread.start()
        lora_thread.start()

        while True:
            metadata = picam2.capture_metadata()
            if metadata and "ai.outputs" in metadata:
                outputs = imx500.get_outputs(metadata)
                if outputs is not None:
                    parse_detections(metadata)
                else:
                    print("No AI outputs found.")
            else:
                print("No metadata or ai.outputs")

    except KeyboardInterrupt:
        print("Stopping...")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        running = False
        if 'lora' in locals():
            lora.close()
        if 'picam2' in locals():
            picam2.stop()
        print("Shutdown complete.")


