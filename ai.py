from picamera2 import Picamera2
import time
import cv2

model_path = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpn_uint8.rpk"
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (1280, 720)},
    controls={"FrameRate": 15},
    ai_model=model_path
)
picam2.configure(config)
picam2.start()
time.sleep(1)
frame = picam2.capture_array()
cv2.imwrite("ai_test.jpg", frame)
metadata = picam2.capture_metadata()
print("Full metadata with AI model:")
print(metadata)
picam2.stop()
