"""
imx500_ai_camera_test.py
------------------------
Test script for Sony IMX500 AI Camera on Raspberry Pi 4 with virtualenv.
- Loads default MobileNet SSD AI model
- Shows live pop-up window with detection boxes and labels
- Prints detected objects/labels/confidences to terminal

USAGE:
    python imx500_ai_camera_test.py
"""

from picamera2 import Picamera2
import time
import cv2

# MobileNet SSD model path (provided by apt package imx500-models)
model_path = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpn_uint8.rpk"
output_image = "ai_test_output.jpg"

def main():
    # List available cameras
    cameras = Picamera2.global_camera_info()
    print("Detected camera modules:", cameras)
    if not cameras:
        print("ERROR: No camera modules detected! Check cable, hardware, or drivers.")
        return

    print(f"Loading MobileNet SSD model: {model_path}")
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (1280, 720)},
            controls={"FrameRate": 15},
            ai_model=model_path
        )
        picam2.configure(config)
        picam2.start()
    except Exception as e:
        print("ERROR: Failed to initialize camera or load AI model.")
        print(e)
        return

    print("Camera started. Press 'q' in the window to exit.")
    time.sleep(1)

    last_frame = None

    try:
        while True:
            frame = picam2.capture_array()
            last_frame = frame.copy()
            metadata = picam2.capture_metadata()

            det_key = None
            if metadata:
                if "AI" in metadata:
                    det_key = "AI"
                elif "ai" in metadata:
                    det_key = "ai"

            if det_key and "detections" in metadata[det_key]:
                detections = metadata[det_key]["detections"]
                print(f"\nDetected {len(detections)} object(s):")
                for det in detections:
                    x1, y1, x2, y2 = det["bbox"]
                    label = det.get("label", "object")
                    conf = det.get("confidence", 0)
                    print(f"  - {label} | Confidence: {conf:.2f} | Box: ({x1},{y1})-({x2},{y2})")
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            else:
                print(".", end="", flush=True)

            cv2.imshow('IMX500 AI Camera Live', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nQuitting live preview.")
                break

    except KeyboardInterrupt:
        print("\nScript interrupted by user. Exiting...")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        if last_frame is not None:
            cv2.imwrite(output_image, last_frame)
            print(f"Last frame saved as '{output_image}'.")
        print("Camera released and all windows closed.")

if __name__ == "__main__":
    main()
