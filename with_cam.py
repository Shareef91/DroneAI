import argparse
import sys
import time
import os
import cv2
import numpy as np
import serial
import base64
import threading
from functools import lru_cache
from datetime import datetime
from queue import Queue

import board
import Adafruit_DHT
from adafruit_bme280 import basic as adafruit_bme280

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics, postprocess_nanodet_detection)

# === GLOBALS ===
last_detections = []
last_sent_time = 0
COOLDOWN_SEC = 5
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
CHUNK_SIZE = 200
MAX_RETRIES = 3
running = True

wQueue = Queue()
imgQueue = Queue()

class WeatherData:
    def __init__(self, time="0", temp=0, humidity=0, pressure=0, altitude=0):
        self.time = time
        self.temp = temp
        self.humidity = humidity
        self.pressure = pressure
        self.altitude = altitude

WeatherStruct = WeatherData()

class Detection:
    def __init__(self, coords, category, conf, metadata, imx500, picam2):
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)

# Should be run as a separate thread
def read_weather():
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)
    bme280.sea_level_pressure = 1018.5

    while running:
        temperature = bme280.temperature
        humidity = bme280.relative_humidity
        pressure = bme280.pressure
        altitude = bme280.altitude
        timestamp = datetime.now().strftime("%H:%M:%S")

        WeatherStruct.time = timestamp
        WeatherStruct.temp = temperature
        WeatherStruct.humidity = humidity
        WeatherStruct.pressure = pressure
        WeatherStruct.altitude = altitude

        # Save to CSV
        with open("weather_data.csv", "a") as f:
            f.write(f"{timestamp},{temperature:.1f},{humidity:.1f},{pressure:.1f},{altitude:.2f}\n")

        wQueue.put(WeatherStruct)
        time.sleep(5)

def save_detection_image(picam2, folder):
    frame = picam2.capture_array()
    os.makedirs(folder, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(folder, f"detection_{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    return filename

class LoRaTransmitter:
    def __init__(self, port="/dev/ttyS0", baudrate=9600):
        self.ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)
        self.weatherCounter = 0

    def send(self, message):
        retry_count = 0
        while retry_count <= MAX_RETRIES:
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
            if not imgQueue.empty():
                self.send_image(imgQueue.get())

    def send_image(self, image_path):
        with open(image_path, "rb") as img_file:
            b64_data = base64.b64encode(img_file.read()).decode("utf-8")

        total = (len(b64_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        print(f"Sending image in {total} packets...")

        for i in range(total):
            if not wQueue.empty():
                self.send_weather(wQueue.get())
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
        print("Image transmission completed. Sending object ID")
        self.send("ID:" + image_path.split('/')[-1])

    def close(self):
        self.ser.close()

def parse_detections(metadata, imx500, picam2, intrinsics, args, lora):
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
        boxes, scores, classes = \
            postprocess_nanodet_detection(outputs=np_outputs[0], conf=threshold, iou_thres=iou,
                                          max_out_dets=max_detections)[0]
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
        Detection(box, category, score, metadata, imx500, picam2)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold
    ]

    if len(last_detections) > 0 and time.time() - last_sent_time > COOLDOWN_SEC:
        filename = save_detection_image(picam2, "detections")
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        weather_info = f" Temp: {WeatherStruct.temp}C Hum: {WeatherStruct.humidity}%"
        lora.send(f"{timestamp}: Detected object.{weather_info}")
        lora.send_image(filename)
        last_sent_time = time.time()

    return last_detections

def draw_detections(request):
    """Draw detection boxes on the camera preview"""
    with MappedArray(request, "main") as m:
        for detection in last_detections:
            x1, y1, x2, y2 = detection.box
            cv2.rectangle(m.array, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0, 0), 2)
            label = f"{detection.category}: {detection.conf:.2f}"
            cv2.putText(m.array, label, (int(x1), int(y1) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI Detection with LoRa transmission")
    parser.add_argument("--model", type=str, help="Path to the AI model",
                       default="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpn_uint8.rpk")
    parser.add_argument("--threshold", type=float, default=0.55, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="IoU threshold for NMS")
    parser.add_argument("--max-detections", type=int, default=10, help="Maximum number of detections")
    parser.add_argument("--lora-port", type=str, default="/dev/ttyS0", help="LoRa serial port")
    parser.add_argument("--lora-baudrate", type=int, default=9600, help="LoRa baudrate")
    parser.add_argument("--preview", action="store_true", help="Show camera preview with detections")

    args = parser.parse_args()

    try:
        print("Initializing camera and AI model...")
        if not IMX500.is_available():
            print("ERROR: IMX500 not available. Make sure you're running on a compatible device.")
            sys.exit(1)

        # === Correct IMX500 and Picamera2 Initialization ===
        imx500 = IMX500(args.model)
        intrinsics = imx500.network_intrinsics
        if not intrinsics:
            intrinsics = NetworkIntrinsics()
            intrinsics.task = "object detection"
        elif intrinsics.task != "object detection":
            print("Network is not an object detection task", file=sys.stderr)
            sys.exit(1)

        picam2 = Picamera2(imx500.camera_num)
        config = picam2.create_preview_configuration(controls={"FrameRate": 30})
        picam2.configure(config)
        imx500.show_network_fw_progress_bar()
        picam2.start(config, show_preview=args.preview)

        print(f"Initializing LoRa transmitter on {args.lora_port}...")
        lora = LoRaTransmitter(port=args.lora_port, baudrate=args.lora_baudrate)

        if args.preview:
            picam2.pre_callback = draw_detections

        print("AI Detection system started. Press Ctrl+C to stop.")
        print(f"Detection threshold: {args.threshold}")
        print(f"Cooldown period: {COOLDOWN_SEC} seconds")

        weather_thread = threading.Thread(target=read_weather, daemon=True)
        lora_thread = threading.Thread(target=lora.loop, daemon=True)
        weather_thread.start()
        lora_thread.start()

        while True:
            metadata = picam2.capture_metadata()
            if metadata:
                detections = parse_detections(metadata, imx500, picam2, intrinsics, args, lora)
                if detections:
                    print(f"Detected {len(detections)} objects")
                    for i, det in enumerate(detections):
                        print(f"  {i+1}: {det.category} (confidence: {det.conf:.2f})")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down...")

    except serial.SerialException as e:
        print(f"LoRa communication error: {e}")
        print("Check LoRa device connection and port settings.")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            running = False
            if 'lora' in locals():
                try:
                    lora.close()
                except:
                    pass
            if 'picam2' in locals():
                try:
                    picam2.stop()
                except:
                    pass
            print("Cleanup completed.")
        except Exception as cleanup_error:
            print(f"Error during cleanup: {cleanup_error}")

