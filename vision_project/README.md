📘 Object Shape & Color Detection – README
📝 Project Overview

This project implements real-time object detection using a webcam, identifying shape and color of objects in the frame.
The system uses Canny edge detection, contour analysis, and HSV color classification to detect objects and only saves the result when the SPACEBAR is pressed.

This project does not store images, only metadata (timestamp + detected label) inside labels.csv.

The code is fully implemented by me, except for the following external libraries:

OpenCV (cv2)

NumPy

Python Standard Library (os, csv, datetime)

All portions of the code written for this project are my own original work.
No external code has been copied without proper reference.

🔧 Features

Live webcam detection

Shape recognition:

Triangle

Square

Rectangle

Circle

Color recognition (based on HSV):

Red, Orange, Yellow, Green, Blue, Purple

White / Black

Press SPACE to save the detection

Results stored as CSV (no images saved)

Adjustable parameters via UI trackbars

Modular structure (main.py, detection.py, utils.py)

📂 Project Structure
project/
│
├── main.py           # Webcam loop, UI, CSV saving
├── detection.py      # Shape + color detection logic
├── utils.py          # Helper functions (HSV color naming, etc.)
└── Detected_Objects/
      └── labels.csv  # Stored detections

▶️ How to Run

1. Install dependencies
pip install opencv-python numpy

2. Run the program
python main.py

3. Key controls
Key Action
SPACE Save the most recent detection to CSV
q Quit the program
🖼 Demonstration Screenshot

(Replace the placeholder with a real screenshot before submission!)

📊 Example CSV Output
Timestamp,Label
20250112_153210,Blue Circle
20250112_153242,Red Rectangle

⚙️ Dependencies

Python 3.8+

OpenCV

NumPy

📄 License

This project is released under the MIT License.
You may freely modify, distribute, and use the code for academic and personal purposes.
