import cv2
import csv
import os
from datetime import datetime

from utils import nothing
from detection import detect_objects

def main():

    # --- Setup camera ---
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # --- UI trackbars ---
    cv2.namedWindow("Parameters")
    cv2.resizeWindow("Parameters", 400, 200)

    cv2.createTrackbar("Canny Min", "Parameters", 50, 255, nothing)
    cv2.createTrackbar("Canny Max", "Parameters", 150, 255, nothing)
    cv2.createTrackbar("Min Area", "Parameters", 1000, 20000, nothing)

    # --- CSV Setup ---
    os.makedirs("Detected_Objects", exist_ok=True)
    csv_file = "Detected_Objects/labels.csv"

    if not os.path.isfile(csv_file):
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Label"])

    last_detection = None  # Stores last detected object for SPACEBAR saving

    while True:

        ret, frame = cap.read()
        if not ret:
            break

        frame_resized = cv2.resize(frame, (640, 480))
        blur = cv2.GaussianBlur(frame_resized, (7, 7), 1)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        canny_min = cv2.getTrackbarPos("Canny Min", "Parameters")
        canny_max = cv2.getTrackbarPos("Canny Max", "Parameters")
        min_area = cv2.getTrackbarPos("Min Area", "Parameters")

        edges = cv2.Canny(gray, canny_min, canny_max)

        # Detect objects
        detections = detect_objects(frame_resized, hsv, edges, min_area)

        # Draw detections
        display = frame_resized.copy()
        for (x, y, w, h, label) in detections:
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(display, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            last_detection = label  # Save latest detection

        # --- Show windows ---
        cv2.imshow("Camera", frame_resized)
        cv2.imshow("Edges", edges)
        cv2.imshow("Detected Shapes", display)

        key = cv2.waitKey(1) & 0xFF

        # --- Save on SPACEBAR ---
        if key == ord(' '):
            if last_detection:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                with open(csv_file, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, last_detection])
                print(f"Saved: {last_detection}")
            else:
                print("No object detected to save.")

        # Quit
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
