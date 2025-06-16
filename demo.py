# demo.py
# Test all modules individually

from sensor_module import get_sensor_data
from camera_object_detection import detect_objects
from lora_transmitter import send_data

print("=== SENSOR TEST ===")
sensor_data = get_sensor_data()
print(sensor_data)

print("\n=== CAMERA TEST ===")
detected_objects = detect_objects()
print("Detected Objects:", detected_objects)

print("\n=== LORA TEST ===")
test_payload = {
    "sensor_id": "Test001",
    "temperature_C": 25.0,
    "humidity_percent": 50.0,
    "pressure_hPa": 1000.0,
    "detected_objects": detected_objects
}
send_data(test_payload)