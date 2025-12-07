import cv2
from ultralytics import YOLO
import csv
import os
from datetime import datetime
from collections import defaultdict


def draw_confidence_bar(frame, confidence, x=10, y=60, width=200, height=20):
    conf = int(confidence * 100)
    bar_fill = int(width * confidence)

    cv2.rectangle(frame, (x, y), (x + width, y + height), (255, 255, 255), 2)
    cv2.rectangle(frame, (x, y), (x + bar_fill, y + height), (0, 255, 0), -1)
    cv2.putText(frame, f"{conf}%", (x + width + 10, y + height),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    # Charge YOLO model
    model = YOLO("yolov8n.pt")  # Change to yolov8s.pt for more accuracy

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Camera could not be opened.")
        return

    os.makedirs("Detected_Objects", exist_ok=True)
    csv_file = "Detected_Objects/labels.csv"

    # Create CSV if doesnt exists
    if not os.path.isfile(csv_file):
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Label"])

    last_label = None
    object_counter = defaultdict(int)

    stable_label = None
    stable_frames = 0
    REQUIRED_STABLE_FRAMES = 5

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # --- YOLO Detection ---
        results = model(frame, verbose=False)

        best_conf = 0
        best_label = None

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                if conf > 0.5 and conf > best_conf:
                    best_conf = conf
                    best_label = label

        # --- Stability ---
        if best_label == stable_label:
            stable_frames += 1
        else:
            stable_label = best_label
            stable_frames = 1

        if stable_frames >= REQUIRED_STABLE_FRAMES:
            last_label = stable_label

        # --- Draw Bar ---
        if last_label:
            cv2.putText(frame, f"{last_label}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            draw_confidence_bar(frame, best_conf)

        # --- Object counter ---
        y_offset = 100
        cv2.putText(frame, "Detected:", (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        for obj, count in object_counter.items():
            y_offset += 30
            cv2.putText(frame, f"{obj}: {count}", (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("YOLO Object Detection", frame)

        key = cv2.waitKey(1) & 0xFF

        # --- Save labels ---
        if key == ord(" "):
            if last_label:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                object_counter[last_label] += 1

                with open(csv_file, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, last_label])

                print(f"Saved: {last_label}")
            else:
                print("No object detected to save.")

        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()