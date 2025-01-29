# License Plate Detection Pipeline

## Overview
This application is designed for real-time license plate detection and recognition using ROS2, YOLOv8, and EasyOCR. The pipeline involves:

1. **License Plate Detection**: Using YOLOv8 to identify and extract the license plate region.
2. **Tilt Correction**: Correcting any detected tilt in the extracted plate.
3. **Character Segmentation**: Extracting individual characters from the plate using OpenCV.
4. **OCR Processing**: Recognizing the characters with EasyOCR.
5. **Data Publishing**: Publishing detected license plate information, along with GPS coordinates, to a ROS2 topic.

## Features
- Real-time processing with ROS2
- YOLOv8-based license plate detection
- Tilt correction for better character recognition
- Character segmentation and recognition using EasyOCR
- Integration with GPS data
- ROS2-based messaging for detected plates

## Installation
### Prerequisites
Ensure you have the following dependencies installed:
- Python 3.8+
- ROS2 (Foxy, Humble, or compatible distribution)
- OpenCV
- NumPy
- EasyOCR
- ultralytics (for YOLOv8)
- cv_bridge (for ROS2 image conversion)

### Installation Steps
```sh
pip install opencv-python numpy easyocr ultralytics cv_bridge rclpy sensor_msgs std_msgs
```

## Usage
### Running the Node
To launch the License Plate Detection Pipeline, run:
```sh
ros2 run usb_cam LPD_Pipeline.py

```
To launch the application (caméra+LPD_pipeline):
```sh
ros2 launch geicar_start_jetson geicar.jetson.launch.py

```

### Topics
#### Subscribed Topics
- `/image_raw` (sensor_msgs/Image): Receives input images from the camera.
- `/gps/fix_adjusted` (sensor_msgs/NavSatFix): Receives GPS data for location tagging.

#### Published Topics
- `/verified_text` (std_msgs/String): Publishes recognized license plate text along with GPS coordinates.
- `/plate_detection/compressed` (sensor_msgs/CompressedImage): Sends the detected plate images to the web server.

## Modules
### 1. `LPD_Pipeline.py`
This is the main ROS2 node for the pipeline, handling:
- Image buffering
- License plate detection using YOLOv8
- Integration with tilt correction and character segmentation
- OCR processing and text verification
- Publishing results to ROS2 topics

### 2. `Segmentor.py`
Handles character segmentation:
- Finds contours of individual characters
- Filters and extracts the characters from the detected license plate

### 3. `Tilt_Corrector.py`
Responsible for correcting tilt in the detected license plate image:
- Identifies bottom-most points of the plate
- Computes the rotation angle
- Applies an affine transformation to correct orientation

## Configuration
- Update the YOLOv8 model path in `LPD_Pipeline.py`:
  ```python
  self.model = YOLO("/path/to/your/model/best.pt")
  ```
- Adjust frame skipping for optimized processing:
  ```python
  self.frame_skip = 3
  ```

