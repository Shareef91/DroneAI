import cv2
import os
from datetime import datetime
from ultralytics import YOLO

# ---- CONFIG ----
MODEL_PATH = "yolov8n.pt"    # Put your YOLOv8n weights in same folder or give full path
SAVE_DIR = "detections"      # Folder where images will be saved
CAM_INDEX = 0                # 0 = Pi cam, USB cam; try 1 or 2 if you have multiple
# -----------------

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    print(f"[INFO] Loading YOLOv8n model from: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("[INFO] Starting camera stream. Press 'q' to quit, 's' to save manually.")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("[ERROR] Camera not found. Try changing CAM_INDEX.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to capture frame from camera.")
            break

        # Run YOLOv8n detection
        results = model(frame)[0]
        annotated = results.plot()

        # If objects detected, auto-save
        if len(results.boxes) > 0:
            now = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            img_path = os.path.join(SAVE_DIR, f"auto_{now}.jpg")
            cv2.imwrite(img_path, annotated)
            print(f"[AUTO-SAVE] Detected objects! Image saved: {img_path}")

        # Show window (needs VNC or monitor attached)
        cv2.imshow("YOLOv8n Live (Pi)", annotated)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("[INFO] Quitting live stream.")
            break
        elif key == ord('s'):
            now = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            img_path = os.path.join(SAVE_DIR, f"manual_{now}.jpg")
            cv2.imwrite(img_path, annotated)
            print(f"[MANUAL SAVE] Image saved: {img_path}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
