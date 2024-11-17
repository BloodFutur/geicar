import cv2
import os
import time
from datetime import datetime
import threading

# Parameters
interval_seconds = 0.1  # Time interval between each capture in seconds
save_directory = "captured_images"  # Folder where images will be saved

# Create a folder to store images, if none exists
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# Camera initialization (0 for default camera, try 1 if using another webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Unable to access webcam.")
    exit(1)

# Reduce buffer size
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

try:
    print("Press Ctrl+C to stop capturing images.")
    while True:
        # Clearing buffer by reading twice 
        ret, frame = cap.read()
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read a frame.")
            break

        # Generate a unique image name with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        image_path = os.path.join(save_directory, f"image_{timestamp}.jpg")

        # Image saving
        cv2.imwrite(image_path, frame)
        print(f"Captured and saved image : {image_path}")

        # Pause for specified interval
        threading.Event().wait(interval_seconds)
        
except KeyboardInterrupt:
    print("Image capture stopped by user.")

# Releasing the camera and closing the windows
cap.release()
cv2.destroyAllWindows()
