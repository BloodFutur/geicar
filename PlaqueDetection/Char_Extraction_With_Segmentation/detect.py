import cv2
import os
import numpy as np
from ultralytics import YOLO


def load_model():
    #Loads the YOLOv8 model.
    model = YOLO(r'C:\Users\33750\Documents\Projet_5SIEC\Char_extract\best.pt')
    return model


def detect_license_plate(image_path, output_folder="cropped_outputs"):
    """
    Detects and crops the license plate from the input image using our costum Yolov8n trained model.

    Args:
        image_path (str): Path to the input image.
        output_folder (str): Path to save the cropped license plate.

    Returns:
        tuple: (output_image_path (str), NumberPlateCnt (list of points))
    """
    # Load the YOLO model
    model = load_model()

    # Load image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found at {image_path}")

    # Perform inference
    results = model(image_path)

    # Ensure output folder exists
    os.makedirs(output_folder, exist_ok=True)

    # Process results
    if len(results) == 0 or len(results[0].boxes.xyxy) == 0:
        raise ValueError("No license plate detected!")

    # Assume the first detection is the license plate
    box = results[0].boxes.xyxy[0]  # [x_min, y_min, x_max, y_max]
    x_min, y_min, x_max, y_max = map(int, box[:4])

    # Crop license plate
    cropped_plate = image[y_min:y_max, x_min:x_max]

    # Save the cropped image
    output_path = os.path.join(output_folder, os.path.basename(image_path))
    cv2.imwrite(output_path, cropped_plate)

    # License Plate Bounding Box format: list of 4 points [top-left, top-right, bottom-right, bottom-left]
    LP_BB = [
        [x_min, y_min],
        [x_max, y_min],
        [x_max, y_max],
        [x_min, y_max]
    ]

    return output_path, LP_BB



#if __name__ == "__main__":
    #image_path = ""
    #output_path, bounding_box = detect_license_plate(image_path)
    #print(f"Cropped image saved at: {output_path}")
    #print(f"Bounding box coordinates: {bounding_box}")
