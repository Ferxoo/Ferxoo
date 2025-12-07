# 📘 YOLO Object Detection & Dataset Builder – README📝 
## Project Overview
This project implements a real-time object detection system using Ultralytics YOLO (YOLOv8) via a webcam.The system detects objects, displays the most stable label, and allows saving detections to a CSV file simply by pressing the spacebar.Additionally, the project includes a Dataset Builder that allows capturing images from the camera and labeling them manually to build custom datasets.
## 🔧 Features
### ✅ YOLO Object Detection Mode
- Real-time detector based on YOLO (Ultralytics)
- Automatic selection of the object with highest confidence
- Stable detection system (waits several frames before confirming)
- Graphical bar showing the detection confidenceCounter for each saved object
- Labels saved only when SPACE is pressed
- Results stored in Detected_Objects/labels.csv
### ✅ Dataset Builder Mode
- Captures images from the webcam
- Allows entering labels manually
- Saves images and annotations (CSV)
- Ideal for creating your own dataset for YOLO training

## 📂 Project Structure
```bash
vision_project/
│
├── main.py                 # YOLO detection system
├── dataset_builder.py      # Manual dataset capture tool
│
├── Detected_Objects/
│     └── labels.csv        # Detection log (auto-generated)
│
└── Dataset/
      ├── images/           # Captured images (builder mode)
      └── annotations.csv   # Labels for your dataset
```
## ⚙️ Dependencies
- Python 3.8+ recommended
- Install necessary dependencies:
```bash
pip install ultralytics opencv-python
```
## ▶️ How to Run
| Mode            | Archive            | Command to Execute |
|:---------------:|:------------------:|:---------------------:|
| YOLO Detection  | main.py            | python main.py |
| Dataset Builder | dataset_builder.py | python dataset_builder.py |

## 🎮 Key Controls
| Key            | Action            |
|:---------------:|:------------------:|
| SPACE  | Saves the detection            |
| q | Closes the program |
## 🧠 Model Used
By default, the following is used:\
**yolov8n.pt**\
You can replace it with other models:
- yolov8s.pt → More precision
- yolov8m.pt → Better performance
- yolov8n.pt → Faster

Simply modify in main.py:
```bash
model = YOLO("yolov8n.pt")
```
## 🖼 Demonstration Screenshot
!(assets/foto1.png)

## 📄 License
This project is under the MIT License, which allows you to modify, distribute, and use the code freely for personal or academic purposes.