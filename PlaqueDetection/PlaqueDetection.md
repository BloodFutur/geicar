# PlaqueDetection

This project is focused on license plate detection using YOLOv8. The goal is to train a deep learning model to detect and recognize vehicle license plates from images. The repository includes Python files for training and testing the model, as well as the necessary configuration files. However, the dataset of images will not be included in this repository for storage reasons.


## Dataset

The dataset for this project, including .jpg images and their corresponding label files in .txt format, is hosted on Roboflow.

To download the dataset:

    Go to the Roboflow project: https://app.roboflow.com/plaquedetection-cj0zn/platedetection-klmg0/overview

    Download the images:
        You can download the dataset as a ZIP file directly from Roboflow

Important Notes:

    The images in the dataset are in .jpg format.

    Each image has a corresponding label file in .txt format, which contains the bounding box information in YOLOv8 format.

    Make sure that when you download the dataset, the images and their label files are placed in the correct folders (images/ and labels/).

## GitHub Repository Contents

The GitHub repository contains the Project PlaqueDetection with the following files:

    - train.py: Python script for training the YOLOv8 model.

    - test.py: Python script for testing the trained model.

    - data.yaml: Configuration file for the dataset and class names.

    Note: The datasets/ folder in this repository will be empty. All the actual images and labels should be downloaded from Roboflow as mentioned above.

# How to Run the Project

    Download the dataset from Roboflow (as explained above).

    Set up the environment:

        - Install the required Python dependencies (ultralytics, matplotlib, pyyaml)

	- Optional : create a virtual environment : python -m venv venv 

	Activate this environment on Windows : .\venv\Scripts\activate
	
    Train the model:

        Run the train.py script to begin training the model on the dataset : python train.py

    Test the model:

	- Put the image you want to test in the repository : /images_test

	- In test.py, line 13, change the name of the image with your new image name (exemple: new_image.jpg) : image_path = os.path.join(script_dir, "images_test", "new_image.jpg")

        - Run the test.py script to evaluate the trained model on test images : python test.py

