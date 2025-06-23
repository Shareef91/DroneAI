import cv2
from picamera2 import Picamera2
from datetime import datetime

net = cv2.dnn.readNetFromCaffe(
    '/home/pi/models/mobilenet_ssd/MobileNetSSD_deploy.prototxt',
    '/home/pi/models/mobilenet_ssd/MobileNetSSD_deploy.caffemodel'
)

CLASSES = ["grass", "aeroplane", "cone", "bird", "carpet",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant",
           "sheep", "sofa", "train", "tvmonitor"]

def detect_objects():
    picam2 = Picamera2()
    picam2.start()
    frame = picam2.capture_array()
    picam2.stop()

    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    detected = set()
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.6:
            idx = int(detections[0, 0, i, 1])
            detected.add(CLASSES[idx])
    return list(detected)
