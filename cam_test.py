"""
imx500_ai_camera_test.py
------------------------
Test script for Sony IMX500 AI Camera on Raspberry Pi 4.
- Loads default object detection model
- Shows live pop-up window with detection boxes and labels (OpenCV)
- Prints all detected objects/labels/confidences to terminal
- Saves last frame as 'ai_test_output.jpg' when you quit

USAGE:
    python3 imx500_ai_camera_test.py

Press 'q' in the pop-up window to exit.
"""

from picamera2 import Picamera2
from picamera2.devices import IMX500
import time
import cv2

# === Configuration ===

# Path to the default AI detection model (change if your .rpk is elsewhere)
model_path = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpn_uint8.rpk"

# Optional: Change this for a different save filename
output_image = "ai_test_output.jpg"

# === Main Code ===

def main():
    # Check for IMX500 camera hardware
    if not IMX500.is_available():
        print("ERROR: IMX500 camera NOT available! Check cable, hardware, or drivers.")
        return
    print("IMX500 camera detected!")

    # Initialize camera with AI model
    print(f"Loading AI model: {model_path}")
    picam2 = Picamera2(IMX500.load_network(model_path))
    picam2.start()
    print("Camera started. Pop-up will appear. Press 'q' in the window to exit.")
    time.sleep(1)  # Allow camera to warm up

    last_frame = None  # For saving when quitting

    try:
        while True:
            # Capture a video frame
            frame = picam2.capture_array()
            last_frame = frame.copy()

            # Capture latest AI inference metadata
            metadata = picam2.capture_metadata()

            # Draw detection results (if any)
            if metadata and "ai" in metadata and "detections" in metadata["ai"]:
                detections = metadata["ai"]["detections"]
                print(f"\nDetected {len(detections)} object(s):")
                for det in detections:
                    x1, y1, x2, y2 = det["bbox"]
                    label = det.get("label", "object")
                    conf = det.get("confidence", 0)
                    print(f"  - {label} | Confidence: {conf:.2f} | Box: ({x1},{y1})-({x2},{y2})")
                    # Draw box and label on frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            else:
                print(".", end="", flush=True)  # Print dot if no detections (quiet idle)

            # Show the frame in a pop-up window
            cv2.imshow('IMX500 AI Camera Live', frame)

            # Quit if 'q' is pressed in the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nQuitting live preview.")
                break

    except KeyboardInterrupt:
        print("\nScript interrupted by user. Exiting...")

    finally:
        # Cleanup
        picam2.stop()
        cv2.destroyAllWindows()
        # Save the last frame shown
        if last_frame is not None:
            cv2.imwrite(output_image, last_frame)
            print(f"Last frame saved as '{output_image}'.")
        print("Camera released and all windows closed.")

if __name__ == "__main__":
    main()
