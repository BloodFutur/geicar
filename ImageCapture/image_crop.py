import os
import xml.etree.ElementTree as ET
from PIL import Image

def crop_images_for_detection(image_dir, xml_file, output_dir):
    """
    Crops images based on bounding boxes specified in the XML file.
    The bounding boxes are the result of Yolo inference

    Args:
        image_dir (str): Directory containing the images.
        xml_file (str): Path to the XML file containing bounding boxes.
        output_dir (str): Directory to save the cropped images.

    Returns:
        None
    """
    try:
        # Check if the output directory exists, create one if not
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"Output directory '{output_dir}' created.")

        # Parse the XML file: This file contains image, bouding box. 
        tree = ET.parse(xml_file)
        root = tree.getroot()

        # Iterate over each image element in the XML
        for image_element in root.findall('image'):
            image_file = image_element.get('file')
            image_name = os.path.basename(image_file)

            # Load the image
            image_path = os.path.join(image_dir, image_name)
            if not os.path.exists(image_path):
                print(f"Image file '{image_name}' not found. Skipping.")
                continue

            try:
                with Image.open(image_path) as img:
                    # Iterate over each text element in the image
                    for text_element in image_element.findall('text'):
                        # Get bounding box coordinates
                        bounding_box = text_element.find('bounding_box')
                        x = int(bounding_box.get('x'))
                        y = int(bounding_box.get('y'))
                        w = int(bounding_box.get('w'))
                        h = int(bounding_box.get('h'))

                        # Crop the image
                        cropped_image = img.crop((x, y, x + w, y + h))

                        # Save the cropped image
                        cropped_image_name = f"{os.path.splitext(image_name)[0]}_crop_{x}_{y}.png"
                        cropped_image_path = os.path.join(output_dir, cropped_image_name)
                        cropped_image.save(cropped_image_path)
                        print(f"Cropped image saved as '{cropped_image_path}'.")

            except Exception as e:
                print(f"Error processing image '{image_name}': {e}")

    except Exception as e:
        print(f"An error occurred while processing the XML file: {e}")

def main():
    image_dir = './test_crop'  # Directory containing the original images
    xml_file = './test_crop/test_crop.xml'  # Path to the XML file
    output_dir = './cropped_images'  # Directory to save cropped images

    crop_images_for_detection(image_dir, xml_file, output_dir)

if __name__ == "__main__":
    main()