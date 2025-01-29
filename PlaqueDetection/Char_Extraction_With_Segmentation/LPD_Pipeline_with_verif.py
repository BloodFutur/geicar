#!/usr/bin/env python3

import os
import cv2
import re
import numpy as np
from collections import defaultdict
from ultralytics import YOLO  # YOLOv8 for detection
from Tilt_Corrector import Rotator  # Import Tilt Correction
from Segmentor import CharacterSegmentation  # Import Character Segmentation
import easyocr  # For OCR
import torch


class PlateDetection:
    def __init__(self, model_path, image_folder):
        # Initialize YOLOv8 model
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = YOLO(model_path)
        self.model.to(device)
        print("YOLOv8 model loaded successfully!")

        # Initialize EasyOCR
        self.reader = easyocr.Reader(['en'], gpu=True)
        print("EasyOCR loaded and initialized successfully!")

        # Image folder
        self.image_folder = image_folder

    def validate_plate(self, plate):
        """Validate the French license plate format (AA123AA)."""
        plate_format = re.compile(r"^[A-Z]{2}\d{3}[A-Z]{2}$")
        return bool(plate_format.match(plate))

    def verify_text(self, detected_texts, confidences):
        """Verify and refine the detected license plate texts."""
        preprocessed_texts = [
            re.sub(r"[^A-Z0-9]", "", text.upper()) for text in detected_texts
        ]

        # Extract valid substrings
        valid_substrings = []
        for text in preprocessed_texts:
            matches = re.findall(r"[A-Z]{2}\d{3}[A-Z]{2}", text)
            valid_substrings.extend(matches)

        # Initialize weighted scores
        plate_length = 7
        weighted_scores = [defaultdict(float) for _ in range(plate_length)]

        # Apply weighted voting
        for text, conf in zip(valid_substrings, confidences):
            for i, char in enumerate(text):
                if i in [0, 1, 5, 6]:  # Letter positions
                    if char.isalpha():
                        weighted_scores[i][char] += conf[i]
                elif i in [2, 3, 4]:  # Number positions
                    if char.isdigit():
                        weighted_scores[i][char] += conf[i]

        # Construct the verified plate
        verified_text = ""
        for char_scores in weighted_scores:
            if char_scores:
                verified_text += max(char_scores, key=char_scores.get)
            else:
                verified_text += "_"

        # Validate the final text
        return verified_text if self.validate_plate(verified_text) else "INVALID"

    def process_image(self, image_path):
        """Process a single image and extract the license plate."""
        image = cv2.imread(image_path)
        if image is None:
            print(f"Failed to load image: {image_path}")
            return "INVALID", []

        results = self.model(image)

        if len(results) > 0 and len(results[0].boxes) > 0:
            box = results[0].boxes[0]  # Use the first detected bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates

            # Crop license plate from the image
            cropped_plate = image[y1:y2, x1:x2]
            print("License plate detected and cropped.")

            # Step 2: Tilt correction
            LP_BB = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
            rotator = Rotator(LP_BB, cropped_plate)
            rotated_plate = rotator.rotate_image()
            print("Tilt corrected.")

            # Step 3: Character segmentation
            segmentor = CharacterSegmentation()
            char_list = segmentor.segment_characters(rotated_plate)
            print("Segmentation done.")

            # Step 4: Text recognition
            extracted_text = ""
            confidences = []
            for char_img in char_list:
                ocr_result = self.reader.readtext(
                    char_img, detail=1, allowlist="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
                )
                if ocr_result:
                    extracted_text += ocr_result[0][1]  # Extracted text
                    confidences.append(ocr_result[0][2])  # Confidence score

            print(f"Extracted Text: {extracted_text}")
            return extracted_text, confidences
        else:
            print("No license plate detected.")
            return "INVALID", []

    def process_folder(self):
        """Process all images in the folder."""
        detected_texts = []
        confidences = []

        for file_name in os.listdir(self.image_folder):
            image_path = os.path.join(self.image_folder, file_name)
            print(f"Processing image: {image_path}")

            detected_text, confidence = self.process_image(image_path)
            if detected_text != "INVALID":
                detected_texts.append(detected_text)
                confidences.append(confidence)

        # Verify the final license plate
        verified_text = self.verify_text(detected_texts, confidences)
        print(f"Final Verified Plate: {verified_text}")


if __name__ == "__main__":
    # Path to YOLO model and image folder
    model_path = r"..\PlaqueDetection\runs\detect\train7\weights\best.pt"
    image_folder = ""

    # Create an instance of PlateDetection and process the folder
    detector = PlateDetection(model_path, image_folder)
    detector.process_folder()
