import easyocr
import cv2
import matplotlib.pyplot as plt
import os
from Tilt_Corrector import Rotator
from Segmentor import CharacterSegmentation
from detect import detect_license_plate

# Function to display an image
def display_image(image, title="Input Image"):
    plt.figure(figsize=(10, 10))
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title(title)
    plt.axis("off")
    plt.show()

# Function to write results to a text file
def write_results(results, filename="results.txt"):
    """
    Write the detection results to a text file.
    
    Parameters:
    results: str - Results to be written into the file
    filename: str - Path of the file to write the results to results.txt
    """
    with open(filename, 'a') as file:
        file.write(results + "\n")
    print(f"Results written to {filename}")

# Function to perform OCR on an image
def perform_ocr_on_image(image_path):
    # Load the image
    image = cv2.imread(image_path)

    # Initialize the EasyOCR reader
    reader = easyocr.Reader(['en'])  

    # Perform OCR on the image
    results = reader.readtext(image_path, detail=1, allowlist='-ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789')

    # Prepare result string for saving to file
    result_info = f"OCR Results for Image: {image_path}\n"
    
    # Display detected text and annotate the image
    print("Detected Text:")
    for result in results:
        bbox, text, confidence = result[0], result[1], result[2]  
        result_info += f"Text: {text}, Confidence: {confidence:.2f}\n"
        print(f"Text: {text}, Confidence: {confidence:.2f}")
        # Draw bounding boxes and text on the image
        (top_left, top_right, bottom_right, bottom_left) = bbox
        top_left = tuple(map(int, top_left))
        bottom_right = tuple(map(int, bottom_right))
        cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
        cv2.putText(image, text, (top_left[0], top_left[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    

    # Display the image with annotations
    #display_image(image, title="Detected Text with Bounding Boxes")

    # Write results to the text file
    write_results(result_info)
    breakpoint
    return text

# Main function to call perform_ocr_on_image
def main():
    input_path=""
    display_image(cv2.imread(input_path))
    output_cropped_path,LP_BB=detect_license_plate(input_path)
    rotator = Rotator(LP_BB, output_cropped_path)
    rotated_plate = rotator.rotate_image()
    segmentor = CharacterSegmentation()
    char_list = segmentor.segment_characters(rotated_plate)
    segmented_dir=segmentor.save_segmented_characters(char_list)
    extracted_text=""
    for i in range(1, 8):
        char_path = os.path.join(segmented_dir, f"char_{i}.jpg")
        text=perform_ocr_on_image(char_path)
        extracted_text=extracted_text+text
    print("The License Plate text is:",extracted_text)

# Entry point for the script
if __name__ == "__main__":
    main()