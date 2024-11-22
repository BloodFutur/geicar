from ultralytics import YOLO
import cv2
import os

# Get the absolute directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load the trained YOLO model with a path based on the script's directory
model_path = os.path.join(script_dir, "runs", "detect", "train7", "weights", "best.pt")
model = YOLO(model_path)

# Initialize video capture (1 for USB webcam)
cap = cv2.VideoCapture(0)

# Check if the camera is accessible
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# Loop to capture frames from the camera
while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read a frame.")
        break

    # Perform detection on the captured frame
    results = model.predict(source=frame, save=False, show=False)

    # Display detection results on the image (bounding boxes)
    annotated_frame = results[0].plot()  # Generate an annotated image with detections

    # Display the frame with detections
    cv2.imshow("Real-Time Detection", annotated_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()