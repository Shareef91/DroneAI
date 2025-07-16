import argparse
import sys
import time
import board
from datetime import datetime
import os
import cv2
import numpy as np
import serial
import base64
import threading
from queue import Queue
from adafruit_bme280 import basic as adafruit_bme280
from picamera2 import Picamera2
from picamera2.devices import IMX500

# --- Configs ---
COOLDOWN_SEC = 5
CHUNK_SIZE = 200
MAX_RETRIES = 3
DETECTION_SAVE_DIR = "detections"
os.makedirs(DETECTION_SAVE_DIR, exist_ok=True)

running = True
wQueue = Queue()
imgQueue = Queue()
objQueue = Queue()
last_sent_time = 0

# --- Weather Sensor Data ---
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
        WeatherStruct.time = datetime.now()
        WeatherStruct.temp = bme280.temperature
        WeatherStruct.humidity = bme280.relative_humidity
        WeatherStruct.pressure = bme280.pressure
        WeatherStruct.altitude = bme280.altitude
        with open("weather_data.csv", "a") as f:
            f.write(f"{WeatherStruct.time},{WeatherStruct.temp:.1f},{WeatherStruct.humidity:.1f},{WeatherStruct.pressure:.1f},{WeatherStruct.altitude:.2f}\n")
        wQueue.put(WeatherStruct)
        time.sleep(5)

# --- LoRa Class ---
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
            if not objQueue.empty():
                self.send("OBJ:" + objQueue.get())
            if not imgQueue.empty():
                self.send_image(imgQueue.get())

    def send_image(self, image_path):
        with open(image_path, "rb") as img_file:
            b64_data = base64.b64encode(img_file.read()).decode("utf-8")
        total = (len(b64_data) + CHUNK_SIZE - 1) // CHUNK_SIZE
        print(f"Sending image in {total} packets...")
        print("Image transmission ready. Sending object ID")
        self.send("ID:" + image_path.split('/')[-1])
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

# --- Camera/Detection Logic ---
def run_ai_camera(model_path, detection_threshold=0.5):
    picam2 = Picamera2(IMX500.load_network(model_path))
    config = picam2.create_preview_configuration(controls={"FrameRate": 30})
    picam2.configure(config)
    picam2.start(show_preview=False)

    intrinsics = picam2.camera_manager.active_cameras[0].camera.controls.active_camera.model.net.intrinsics
    imx500 = picam2.camera_manager.active_cameras[0].camera.controls.active_camera.model

    global last_sent_time

    while running:
        metadata = picam2.capture_metadata()
        np_outputs = imx500.get_outputs(metadata, add_batch=True)
        if np_outputs is None:
            continue
        boxes, scores, classes = None, None, None
        if hasattr(intrinsics, "postprocess") and intrinsics.postprocess == "nanodet":
            from picamera2.devices.imx500.postprocess import scale_boxes, postprocess_nanodet_detection
            input_w, input_h = imx500.get_input_size()
            boxes, scores, classes = postprocess_nanodet_detection(
                outputs=np_outputs[0],
                conf=detection_threshold,
                iou_thres=0.6,
                max_out_dets=10
            )[0]
            boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
        else:
            boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
        # Loop over detections
        detected = False
        if boxes is not None and scores is not None and classes is not None:
            for box, score, cls in zip(boxes, scores, classes):
                if score > detection_threshold:
                    detected = True
                    print(f"[DETECTION] class={cls}, score={score:.2f}, box={box}")
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    image_path = os.path.join(DETECTION_SAVE_DIR, f"detection_{timestamp}.jpg")
                    frame = picam2.capture_array()
                    cv2.imwrite(image_path, frame)
                    imgQueue.put(image_path)
                    objQueue.put(str(cls))
                    break
        if detected:
            last_sent_time = time.time()
        time.sleep(0.2)

# --- Main Entrypoint ---
def main():
    parser = argparse.ArgumentParser(description="AI Camera LoRa Weather System")
    parser.add_argument("--model", type=str, default="/usr/share/imx500-models/imx500_network_yolov11n_pp.rpk", help="Path to IMX500 .rpk model")
    parser.add_argument("--threshold", type=float, default=0.5, help="Detection confidence threshold")
    args = parser.parse_args()
    # Start threads for weather and LoRa
    lora_send = LoRaTransmitter()
    threading.Thread(target=read_weather, daemon=True).start()
    threading.Thread(target=lora_send.loop, daemon=True).start()
    try:
        run_ai_camera(args.model, detection_threshold=args.threshold)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        global running
        running = False
        lora_send.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
