import os
import xml.etree.ElementTree as ET

def construct_dataset_fromtxt(image_dir, txt_dir, output_xml_file):
    """
    Creates an XML file from a text file containing image names, bounding box coordinates, and text data.

    Args:
        image_dir (str): Directory containing the images.
        txt_dir (str): Directory containing the text files.
        output_xml_file (str): Output XML file path.

    Returns:
        None
    """

    try:
         # Check if an old output XML file already exists and delete it
        if os.path.exists(output_xml_file):
            print("Looking for older datasets")
            os.remove(output_xml_file)
            print(f"Existing dataset XML file '{output_xml_file}' deleted. To be replaced with the new")
        # Root element of the XML file
        root = ET.Element('annotation')

        # Check if the text files directory exists, otherwise raise a failure
        if not os.path.exists(txt_dir):
            raise FileNotFoundError(f"Text directory '{txt_dir}' not found.")

        # Iterate over the text files
        for filename in os.listdir(txt_dir):
            if filename.endswith('.txt'):
                try:
                    with open(os.path.join(txt_dir, filename), 'r') as f:
                        lines = f.readlines()
                except Exception as e:
                    print(f"Error reading text file '{filename}': {e}")
                    continue

                # Iterate over the lines in the text file
                for line in lines:
                    
                    try:
                        values = line.strip().split('\t') # Split the line into components
                        # Text file should have at least 6 columns: image_name, bounding box: x_min, y_min, x_max, y_max and the text label
                        if len(values) < 6: 
                            raise ValueError("Invalid line format.")
                        image_name = values[0]
                        x, y, w, h = map(int, values[1:5])
                        text = ' '.join(values[5:])
                    except Exception as e:
                        print(f"Error parsing line in text file '{filename}': {e}")
                        continue

                    # Check if the image file exists, otherwise raise a failure
                    image_path = os.path.join(image_dir, image_name)
                    if not os.path.exists(image_path):
                        print(f"Image file '{image_name}' not found.")
                        continue

                    # Create an image element
                    image_element = ET.SubElement(root, 'image')
                    image_element.set('file', image_path)

                    # Create a text element
                    text_element = ET.SubElement(image_element, 'text')
                    bounding_box_element = ET.SubElement(text_element, 'bounding_box')
                    bounding_box_element.set('x', str(x))
                    bounding_box_element.set('y', str(y))
                    bounding_box_element.set('w', str(w))
                    bounding_box_element.set('h', str(h))
                    text_data_element = ET.SubElement(text_element, 'text_data')
                    text_data_element.text = text

        # Write the XML file
        try:
            tree = ET.ElementTree(root)
            tree.write(output_xml_file)
            print(f"XML file dataset '{output_xml_file}' created successfully.")
        except Exception as e:
            print(f"Error writing XML file: {e}")

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    image_dir = './eu'
    txt_dir = './eu'
    output_xml_file = 'output.xml'
    print("Creating you dataset please wait")
    construct_dataset_fromtxt(image_dir, txt_dir, output_xml_file)
    print("Dataset created succesfully")

if __name__ == "__main__":
    main()