import cv2
import numpy as np

class CharacterSegmentation:
    def __init__(self):
        """
        Initialize the CharacterSegmentation class.
        """
        pass

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

        for cntr in cntrs:
            # Get bounding box of the contour
            intX, intY, intWidth, intHeight = cv2.boundingRect(cntr)

            # Filter contours by Char_Dimensions
            if Char_Dimensions[0] < intWidth < Char_Dimensions[1] and Char_Dimensions[2] < intHeight < Char_Dimensions[3]:
                x_cntr_list.append(intX)  # Store x-coordinate for sorting
                char_copy = np.zeros((44, 24),dtype=np.uint8)

                # Extract character region and resize
                char = img[intY:intY + intHeight, intX:intX + intWidth]
                char = cv2.resize(char, (20, 40))

                # Invert colors and add black padding
                char = cv2.subtract(255, char)
                char_copy[2:42, 2:22] = char
                char_copy[0:2, :] = 0
                char_copy[:, 0:2] = 0
                char_copy[42:44, :] = 0
                char_copy[:, 22:24] = 0

                segmented_characters.append(char_copy)

        # Sort characters based on x-coordinate
        indices = sorted(range(len(x_cntr_list)), key=lambda k: x_cntr_list[k])
        segmented_characters_sorted = [segmented_characters[idx] for idx in indices]

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
        img_lp = cv2.resize(image, (333, 75))  # Resizing for clarity
        img_gray_lp = cv2.cvtColor(img_lp, cv2.COLOR_BGR2GRAY)
        _, img_binary_lp = cv2.threshold(img_gray_lp, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        img_binary_lp = cv2.erode(img_binary_lp, (3, 3))  # Erosion
        img_binary_lp = cv2.dilate(img_binary_lp, (3, 3))  # Dilation

        # Make borders white
        img_binary_lp[0:3, :] = 255
        img_binary_lp[:, 0:3] = 255
        img_binary_lp[72:75, :] = 255
        img_binary_lp[:, 330:333] = 255

        # Define character size ranges
        LP_WIDTH = img_binary_lp.shape[0]
        LP_HEIGHT = img_binary_lp.shape[1]
        Char_Dimensions = [LP_WIDTH / 6, LP_WIDTH / 2, LP_HEIGHT / 10, 2 * LP_HEIGHT / 3]

        # Extract contours and characters
        char_list = self.find_contours(Char_Dimensions, img_binary_lp)

        return char_list
