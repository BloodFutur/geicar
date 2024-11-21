import cv2
import numpy as np

def resize_image(image, size=(640, 640)):
    try:
        return cv2.resize(image, size)
    except Exception as e:
        print(f"Error: Unable to resize image {e}")
        return image  # Return the same image


def convert_to_gray(image):
    try:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    except Exception as e:
        print(f"Error: Unable to convert your image to grey {e}")
        return image  

def equalize_histogram(image):
    try:
        if len(image.shape) == 2:  
            return cv2.equalizeHist(image)
        else:
            print("Operation ignored: Convert to grey operation failed")
            return image
    except Exception as e:
        print(f"Error: Unable to equalize histogram {e}")
        return image

def reduce_noise(image):
    try:
        return cv2.bilateralFilter(image, d=9, sigmaColor=75, sigmaSpace=75)
    except Exception as e:
        print(f"Error: Noise reduction failed {e}")
        return image

def convert_to_color(image):
    try:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    except Exception as e:
        print(f"Error: Conversion to colors failed {e}")
        return image

def preprocess_image(image):
    try:
        resized = resize_image(image)
        gray = convert_to_gray(resized)
        equalized = equalize_histogram(gray)
        denoised = reduce_noise(equalized)
        processed = convert_to_color(denoised)
        return processed
    except Exception as e:
        return image
    
# This part will be used for integration of the pipeline of licence plate detection. 

#Fonction pour lancer une instance du model
#def detect_objects(image, model, confidence_threshold=, iou_threshold=):

# Fonction pour afficher les résultats
#def draw_detections(image, detections): #TODO

# Fonction pour capture d'images depuis la caméra
#def run_camera_detection(camera_index=0): #TODO



#if __name__ == "__main__":
 #   try:
  #      preprocess_image(image=1)  
   # except Exception as e:
    #    print(f"Critical Error :  Déclancher l'erreur handler{e}")
     #   raise SystemExit