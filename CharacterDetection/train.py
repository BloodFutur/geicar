from ultralytics import YOLO
import os

# load YOLOv8 model 
model = YOLO("yolov8n.yaml")  # Initialization without weights

project_root = os.path.dirname(os.path.abspath(__file__))  # Current directory of the script
data_path = os.path.join(project_root, "data.yaml")
# Train the model
model.train(data=data_path, epochs=25, batch=16, imgsz=640)
