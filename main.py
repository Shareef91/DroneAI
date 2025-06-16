import time
from sensor_module import get_sensor_data
from camera_object_detection import detect_objects
from lora_transmitter import send_data
import csv
#hi
def log_data(data):
    with open('log.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=data.keys())
        writer.writerow(data)

def main():
    while True:
        sensor_data = get_sensor_data()
        objects = detect_objects()
        payload = {
            **sensor_data,
            "detected_objects": objects
        }
        send_data(payload)
        log_data(payload)
        print("Sent:", payload)
        time.sleep(10)

if __name__ == "__main__":
    main()
