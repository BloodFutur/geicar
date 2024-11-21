from ultralytics import YOLO
import cv2
import os

# Get the absolute directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load the trained model with a path based on the script's directory
model_path = os.path.join(script_dir, "runs", "detect", "train7", "weights", "best.pt")
model = YOLO(model_path)

# Load a test image with a path based on the script's directory
image_path = os.path.join(script_dir, "images_test", "plaques.jpg")
image = cv2.imread(image_path)

# Check if the image was loaded successfully
if image is None:
    print(f"Error: Unable to load image at {image_path}")
else:
    # Perform detection
    results = model.predict(source=image_path, save=True)

    # If you want to display the annotated results with OpenCV:
    for result in results:
        annotated_frame = result.plot()  # Generates an image with annotations
        cv2.imshow("Résultats", annotated_frame)
        cv2.waitKey(0)  # Press any key to close the window
        cv2.destroyAllWindows()
        
