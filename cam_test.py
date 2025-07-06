"""
cam_test_yolov8n_save.py
------------------------
YOLOv8n camera test for Raspberry Pi 4.
- Saves a detection frame in 'detections/' every time at least one object is detected.
- Also saves the last frame on quit.
- Shows live window with boxes/labels.
"""

from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import time
import os
from datetime import datetime

def main():
    # Detect camera modules
    cameras = Picamera2.global_camera_info()
    print("Detected camera modules:", cameras)
    if not cameras:
        print("ERROR: No camera modules detected! Check your camera hardware.")
        return

    print("Loading YOLOv8n model (auto-download if missing)...")
    model = YOLO("yolov8n.pt")

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)}, controls={"FrameRate": 15})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    print("Camera started. Press 'q' in the window to exit.")

    last_frame = None
    os.makedirs("detections", exist_ok=True)
    last_detection_time = 0
    cooldown_sec = 2  # avoid spamming for every frame (adjust if you want)

    try:
        while True:
            frame = picam2.capture_array()
            last_frame = frame.copy()
            results = model(frame)

            detected = False
            for r in results:
                for box in r.boxes:
                    detected = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = model.names[cls]
                    print(f"Detected: {label} | Confidence: {conf:.2f} | Box: ({x1},{y1})-({x2},{y2})")
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Save frame when there's a detection (with cooldown)
            now = time.time()
            if detected and (now - last_detection_time > cooldown_sec):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = f"detections/detection_{timestamp}.jpg"
                cv2.imwrite(save_path, frame)
                print(f"Saved detection frame as '{save_path}'")
                last_detection_time = now

            cv2.imshow("YOLOv8n Camera Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nQuitting live preview.")
                break

    except KeyboardInterrupt:
        print("\nScript interrupted by user. Exiting...")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        if last_frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = f"detections/yolo_cam_test_output_{timestamp}.jpg"
            cv2.imwrite(save_path, last_frame)
            print(f"Last frame saved as '{save_path}'.")
        print("Camera released and all windows closed.")

if __name__ == "__main__":
    main()

