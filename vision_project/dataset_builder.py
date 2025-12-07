import cv2
import os
import csv
from datetime import datetime


def ensure_folder(path):
    if not os.path.exists(path):
        os.makedirs(path)


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    ensure_folder("Dataset/images")

    annotations_file = "Dataset/annotations.csv"
    if not os.path.isfile(annotations_file):
        with open(annotations_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Filename", "Label"])

    print("=== Dataset Builder Mode ===")
    print("Press SPACE to capture and label an image.")
    print("Press q to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Frame capture failed, stopping.")
            break

        cv2.imshow("Webcam (Dataset Builder)", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            label = input("Enter label for captured image: ").strip()
            if label == "":
                print("Empty label — skipping.")
                continue

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_filename = f"{timestamp}.jpg"
            img_path = os.path.join("Dataset/images", img_filename)

            cv2.imwrite(img_path, frame)

            with open(annotations_file, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, img_filename, label])

            print(f"Saved: {img_path} | Label: {label}")

        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()