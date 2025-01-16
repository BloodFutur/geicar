import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

class CharacterSegmentation:
    def __init__(self, segmented_dir='./segmented_chars_result'):
        
        # Directory to save segmented characters
        #Used only for offline will be deleted later.
        self.segmented_dir = segmented_dir
        if not os.path.exists(self.segmented_dir):
            os.makedirs(self.segmented_dir)

    def find_contours(self, Char_Dimensions, img):
        """
        Find contours in the image and extract characters.

        Parameters:
        Char_Dimensions (list): The expected size range for the character contours [lower_width, upper_width, lower_height, upper_height].
        img (numpy array): The binary image containing the license plate.

        Returns:
        segmented_characters (numpy array): Array of extracted character images.
        """
        # Find contours in the image
        cntrs, _ = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area and take the largest 15
        cntrs = sorted(cntrs, key=cv2.contourArea, reverse=True)[:15]

        # Initialize lists for storing results
        x_cntr_list = []
        segmented_characters = []

        # Load a blank image for visualization (optional)
        ii = cv2.imread('contour.jpg')

        for cntr in cntrs:
            # Get bounding box of the contour
            intX, intY, intWidth, intHeight = cv2.boundingRect(cntr)

            # Filter contours by Char_Dimensions
            if Char_Dimensions[0] < intWidth < Char_Dimensions[1] and Char_Dimensions[2] < intHeight < Char_Dimensions[3]:
                x_cntr_list.append(intX)  # Store x-coordinate for sorting
                char_copy = np.zeros((44, 24))

                # Extract character region and resize
                char = img[intY:intY + intHeight, intX:intX + intWidth]
                char = cv2.resize(char, (20, 40))
                cv2.rectangle(ii, (intX, intY), (intX + intWidth, intY + intHeight), (50, 21, 200), 2)

                # Invert colors and add black padding
                char = cv2.subtract(255, char)
                char_copy[2:42, 2:22] = char
                char_copy[0:2, :] = 0
                char_copy[:, 0:2] = 0
                char_copy[42:44, :] = 0
                char_copy[:, 22:24] = 0

                segmented_characters.append(char_copy)

        # Sort characters based on x-coordinate: Char[i] will be the i character of the license plate
        indices = sorted(range(len(x_cntr_list)), key=lambda k: x_cntr_list[k])
        segmented_characters_sorted = [segmented_characters[idx] for idx in indices]

        plt.imshow(ii, cmap='gray')
        plt.title('Predict Segments')
        plt.show()

        return np.array(segmented_characters_sorted)

    def segment_characters(self, image):
        """
        Preprocess the input license plate image and extract characters.

        Parameters:
        image (numpy array): The license plate image.

        Returns:
        char_list (numpy array): Array of segmented characters.
        """
        # Resize and preprocess image
        img_lp = cv2.resize(image, (333, 75)) # Number may be changed later, the resize is done so the license plate character are more clear
        img_gray_lp = cv2.cvtColor(img_lp, cv2.COLOR_BGR2GRAY)
        """
        Binarization of the gray scaled image: Pixels having  a value > Threshold (200) will have the value 1 ( white) and pixels having a value <= Threshold (200) will have the value 0 (black)
        """
        _, img_binary_lp = cv2.threshold(img_gray_lp, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU) # Binary thresholding: The value is 200 may be changed later
        img_binary_lp = cv2.erode(img_binary_lp, (3, 3)) #Eroding to removepixels that should have a value of 0 but are having a value of 1.
        img_binary_lp = cv2.dilate(img_binary_lp, (3, 3)) #Dilation to remove pixels that should have a value of 1 but are having a value of 0.

        # Make borders white
        img_binary_lp[0:3, :] = 255 
        img_binary_lp[:, 0:3] = 255
        img_binary_lp[72:75, :] = 255
        img_binary_lp[:, 330:333] = 255

        # Define character size ranges
        LP_WIDTH = img_binary_lp.shape[0]
        LP_HEIGHT = img_binary_lp.shape[1]
        Char_Dimensions = [LP_WIDTH / 6, LP_WIDTH / 2, LP_HEIGHT / 10, 2 * LP_HEIGHT / 3] # Standarized Characters of license plate dimensions

        #Displaying the preprocessed image and saving it for tests
        plt.imshow(img_binary_lp, cmap='gray')
        plt.title('Contour')
        plt.show()
        cv2.imwrite('contour.jpg', img_binary_lp)

        # Extract contours and characters
        char_list = self.find_contours(Char_Dimensions, img_binary_lp)

        return char_list

    def save_segmented_characters(self, char_list):
        """
        Save segmented character images and display them.

        Parameters:
        char_list (numpy array): Array of segmented character images.
        """
        for i, char in enumerate(char_list):
            if len(char.shape) != 2:
                print("No character segmented.")
                break

            # Save and display each character
            char_path = os.path.join(self.segmented_dir, f"char_{i + 1}.jpg")
            cv2.imwrite(char_path, char)
            plt.figure()
            plt.imshow(char, cmap='gray')
            plt.title(f'Character {i + 1}')
            plt.axis('off')
            plt.show()
        return self.segmented_dir

# Testing
#segmentation = CharacterSegmentation()
#input_path=""
#input_image=cv2.imread(input_path)
#char_list = segmentation.segment_characters(input_image)
#segmentation.save_segmented_characters(char_list)
