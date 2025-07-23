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
from PIL import Image

from adafruit_bme280 import basic as adafruit_bme280
from picamera2 import MappedArray, Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput


from picamera2.devices import IMX500
from picamera2.devices.imx500 import (
    NetworkIntrinsics,
    postprocess_nanodet_detection
)

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
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)


# should be run as a seperate thread
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


def save_detection_image(picam2, label, detection=None, folder="detections"):
    frame = picam2.capture_array()
    os.makedirs(folder, exist_ok=True)

    # If a detection is provided, crop the image to the bounding box
    if detection is not None and hasattr(detection, "box"):
        x, y, w, h = map(int, detection.box)
        # Ensure bounding box is within image bounds
        x = max(0, x)
        y = max(0, y)
        w = max(1, w)
        h = max(1, h)
        x2 = min(frame.shape[1], x + w)
        y2 = min(frame.shape[0], y + h)
        cropped = frame[y:y2, x:x2]
    else:
        cropped = frame

    # Check if a .jpg file with this label already exists in the folder
    existing_files = [f for f in os.getcwd() if f.startswith(label) and f.endswith(".jpg")]
    if len(existing_files) < 5:
        # If exists, append timestamp to filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = str(f"{label}_{timestamp}.jpg")
    elif existing_files is None:
        # If not, just use label.jpg
        filename = str(f"{label}.jpg")
        # Push to imgQueue only for the first detection
        imgQueue.put(filename)

    cv2.imwrite(filename, cropped)
    
    img = Image.open(filename)
    img = img.resize((img.width // 4, img.height // 4))
    # 112 quality for farther objects or resize it less
    img.save(filename, format='JPEG', quality=72) 
    print(f"[INFO] Saved detection image: {filename}")
    return filename


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
                self.obj_Check(objID)
            if not imgQueue.empty():
                imgPath = imgQueue.get()
                if imgPath in self.detected_objects:
                    continue
                self.send_image(imgPath)
                #add to detected objects
                self.detected_objects.add(imgPath)
    
    def obj_Check(self, objID):
        now = time.time()

        expired = [k for k, v in self.detected_recently.items() if now - v > COOLDOWN_SEC]
        for k in expired:
            del self.detected_recently[k]
        if objID not in self.detected_recently:
            self.send("OBJ:" + objID)  # Send object ID
            self.detected_recently[objID] = now

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
    global last_detections
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
        Detection(box, category, score, metadata)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold
    ]
    return last_detections
    

def draw_detections(request, stream="main"):
    global last_results
    detections = last_results
    if detections is None:
        return
    labels = get_labels()
    with MappedArray(request, stream) as m:
        for detection in detections:
            x, y, w, h = detection.box
            label = f"{labels[int(detection.category)]} ({detection.conf:.2f})"
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x = x + 5
            text_y = y + 15
            overlay = m.array.copy()
            cv2.rectangle(overlay, (text_x, text_y - text_height), (text_x + text_width, text_y + baseline), (255, 255, 255), cv2.FILLED)
            alpha = 0.30
            cv2.addWeighted(overlay, alpha, m.array, 1 - alpha, 0, m.array)
            cv2.putText(m.array, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0, 0), thickness=4)
        if intrinsics.preserve_aspect_ratio:
            b_x, b_y, b_w, b_h = imx500.get_roi_scaled(request)
            color = (255, 0, 0)
            cv2.putText(m.array, "ROI", (b_x + 5, b_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.rectangle(m.array, (b_x, b_y), (b_x + b_w, b_y + b_h), (255, 0, 0, 0))

def get_outputs(metadata, add_batch=True):
    return metadata.get("ai.outputs", None)

def get_input_size():
    return 320, 320  # Modify based on your actual model resolution

# The rest of your code remains the same up until camera initialization

def get_args():
    parser = argparse.ArgumentParser()
    model_path = "/home/team3box/Documents/DroneAI/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
    parser.add_argument("--model", type=str, help="Path of the model", default=model_path)
    parser.add_argument("--fps", type=int, help="Frames per second")
    parser.add_argument("--bbox-normalization", action=argparse.BooleanOptionalAction, help="Normalize bbox")
    parser.add_argument("--bbox-order", choices=["yx", "xy"], default="yx", help="Set bbox order yx -> (y0, x0, y1, x1) xy -> (x0, y0, x1, y1)")
    parser.add_argument("--threshold", type=float, default=0.55, help="Detection threshold")
    parser.add_argument("--iou", type=float, default=0.65, help="Set iou threshold")
    parser.add_argument("--max-detections", type=int, default=10, help="Set max detections")
    parser.add_argument("--ignore-dash-labels", action=argparse.BooleanOptionalAction, help="Remove '-' labels ")
    parser.add_argument("--postprocess", choices=["", "nanodet"], default=None, help="Run post process of type")
    parser.add_argument("-r", "--preserve-aspect-ratio", action=argparse.BooleanOptionalAction, help="preserve the pixel aspect ratio of the input tensor")
    parser.add_argument("--labels", type=str, help="Path to the labels file")
    parser.add_argument("--print-intrinsics", action="store_true", help="Print JSON network_intrinsics then exit")
    return parser.parse_args()

@lru_cache
def get_labels():
    labels = intrinsics.labels
    if intrinsics.ignore_dash_labels:
        labels = [label for label in labels if label and label != "-"]
    return labels

if __name__ == "__main__":
    args = get_args()

    try:

        lora = LoRaTransmitter()
        weather_thread = threading.Thread(target=read_weather, daemon=True)
        lora_thread = threading.Thread(target=lora.loop, daemon=True)
        weather_thread.start()
        lora_thread.start()
        imx500 = IMX500(args.model)
        intrinsics = imx500.network_intrinsics
        if not intrinsics:
            intrinsics = NetworkIntrinsics()
            intrinsics.task = "object detection"
        elif intrinsics.task != "object detection":
            print("Network is not an object detection task", file=sys.stderr)
            exit()


        # Override intrinsics from args
        for key, value in vars(args).items():
            if key == 'labels' and value is not None:
                with open(value, 'r') as f:
                    intrinsics.labels = f.read().splitlines()
            elif hasattr(intrinsics, key) and value is not None:
                setattr(intrinsics, key, value)

        # Defaults
        if intrinsics.labels is None:
            with open("assets/coco_labels.txt", "r") as f:
                intrinsics.labels = f.read().splitlines()
        intrinsics.update_with_defaults()

        if args.print_intrinsics:
            print(intrinsics)
            exit()

        picam2 = Picamera2(imx500.camera_num)
        config = picam2.create_preview_configuration(controls={"FrameRate": intrinsics.inference_rate}, buffer_count=12)
        imx500.show_network_fw_progress_bar()
        picam2.start(config, show_preview=False)

        if intrinsics.preserve_aspect_ratio:
            imx500.set_auto_aspect_ratio()


        last_results = None
        picam2.pre_callback = draw_detections

    
        while True:
            last_results = parse_detections(picam2.capture_metadata())
            # Save a frame every time detections are present
            if last_results:
                for detection in last_results:
                    label = get_labels()[int(detection.category)]
                    if detection.conf > args.threshold:
                        print(f"Detected object: {label} (confidence: {detection.conf:.2f})")
                        objQueue.put(label)
                        save_detection_image(picam2, label, detection)

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
            if 'weather_thread' in locals() and weather_thread.is_alive():
                weather_thread.join(timeout=1)
            if 'lora_thread' in locals() and lora_thread.is_alive():
                lora_thread.join(timeout=1)
            print("Cleanup completed.")
        except Exception as cleanup_error:
            print(f"Error during cleanup: {cleanup_error}")

