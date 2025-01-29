import numpy as np
import cv2
import math

class Rotator:
    """
    The Rotator is responsible for correcting the tilt of license plate images. 
    It identifies the bottom-most points of the plate's contour, specifically the bottom-left and bottom-right coordinates, 
    by comparing their y-coordinates. 
    The rotator computes the angle of rotation needed to align the plate horizontally using trigonometric functions. 
    Once the angle is determined, OpenCV's affine transformation is used to rotate the image accordingly. 
    After rotation, the image is cropped to remove excess height introduced during the process.
    """

    def __init__(self, License_Plate_Bounding_Box, cropped_output_path):
        """
        Initialize the Rotator class with the contour points of the detected plate and image path.

        Args:
            License_Plate_Bounding_Box (list): List of 4 coordinates representing the corners of the detected plate.
            cropped_output_path (str): Path to the cropped license plate image.
        """
        self.LP_BBox = License_Plate_Bounding_Box  # 4 corner points of the license plate
        self.cropped_output_path = cropped_output_path  # Path to the cropped image

    def dist(self, x1, x2, y1, y2):
        """
        Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).

        Args:
            x1, y1: Coordinates of the first point.
            x2, y2: Coordinates of the second point.

        Returns:
            float: Euclidean distance between the two points.
        """
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def find_bottom_coordinates(self):
        """
        Find the bottom-left and bottom-right coordinates from the contour points.

        Returns:
            tuple: Bottom-left and bottom-right coordinates.
        """
        idx = 0  # Index of the point with the maximum y-coordinate
        m = 0  # Maximum y-coordinate value

        for i in range(4):
            if self.LP_BBox[i][1] > m:
                idx = i
                m = self.LP_BBox[i][1]

        previous_cor = 3 if idx == 0 else idx - 1
        next_cor = 0 if idx == 3 else idx + 1

        p = self.dist(self.LP_BBox[idx][0], self.LP_BBox[previous_cor][0], self.LP_BBox[idx][1], self.LP_BBox[previous_cor][1])
        n = self.dist(self.LP_BBox[idx][0], self.LP_BBox[next_cor][0], self.LP_BBox[idx][1], self.LP_BBox[next_cor][1])

        if p > n:
            if self.LP_BBox[previous_cor][0] < self.LP_BBox[idx][0]:
                left = self.LP_BBox[previous_cor]
                right = self.LP_BBox[idx]
            else:
                left = self.LP_BBox[idx]
                right = self.LP_BBox[previous_cor]
        else:
            if self.LP_BBox[next_cor][0] < self.LP_BBox[idx][0]:
                left = self.LP_BBox[next_cor]
                right = self.LP_BBox[idx]
            else:
                left = self.LP_BBox[idx]
                right = self.LP_BBox[next_cor]

        return left, right

    def calculate_rotation_angle(self, left, right):
        """
        Calculate the angle of rotation based on the bottom-most coordinates.

        Args:
            left (tuple): Bottom-left coordinate.
            right (tuple): Bottom-right coordinate.

        Returns:
            float: Angle of rotation in degrees.
        """
        opp = right[1] - left[1]  # Opposite side length
        hyp = ((left[0] - right[0]) ** 2 + (left[1] - right[1]) ** 2) ** 0.5  # Hypotenuse
        sin_theta = opp / hyp  # sin(theta)
        theta = math.asin(sin_theta) * 57.2958  # Convert radians to degrees
        return theta

    def rotate_image(self):
        """
        Rotate the license plate image based on the calculated angle.

        Returns:
            np.ndarray: Rotated image.
        """
        left, right = self.find_bottom_coordinates()
        theta = self.calculate_rotation_angle(left, right)

        Cropped_LP = self.cropped_output_path
        if Cropped_LP is None:
            raise FileNotFoundError(f"Image not found at path: {self.cropped_output_path}")

        image_center = tuple(np.array(Cropped_LP.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, theta, 1.0)
        result = cv2.warpAffine(Cropped_LP, rot_mat, Cropped_LP.shape[1::-1], flags=cv2.INTER_LINEAR)

        opp = right[1] - left[1]
        h = result.shape[0] - opp // 2 if opp > 0 else result.shape[0] + opp // 2
        result = result[0:h, :]

        return result
