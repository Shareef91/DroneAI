from picamera2 import Picamera2
import time
import cv2

picam2 = Picamera2()
picam2.start()
time.sleep(1)
frame = picam2.capture_array()
cv2.imwrite("no_ai_test.jpg", frame)
print("Saved no_ai_test.jpg")
picam2.stop()
