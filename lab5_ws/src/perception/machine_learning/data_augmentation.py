import os
import glob
import cv2
import numpy as np
import random

class DataAugmentation():

    # Input image, output a list of zoomed images
    # TODO: ONE OF THE OPTIONAL AUGMENTATIONS
    def zoom(self, img):
    # Create a list to hold the output
    # Define the zoom percentages
    # For each zoom percentage
        # Get the image shape
        # Compute the new shape based on the zoom
        # Make the image the new shape
        # Pad or crop the image depending on original size
        # Save new image to list
    # Return list

        return None
    # Input image, output a list of translated images
    # TODO: ONE OF THE OPTIONAL AUGMENTATIONS
    def translate_image(self, img):
        # Create a list to hold the output
        # Define the translations
        # For each translation in X
            # For each translation in Y
                # Get the current image shape
                # Compute the new shape of the image
                # Warp the image to the new shape
                # Append black pixels to the edge
                # Save the new image
        # Return the output
        return None

    # Input image, output a list of rotated images
    # TODO: ONE OF THE OPTIONAL AUGMENTATIONS
    def rotate_image(self, img):
        # Create a list to hold the output
        # Define the rotation we want to consider
        # For each rotation
            # Get the image shape
            # Compute the rotation matrix centered around the middle of the image HINT: cv2.getRotationMatrix2D() 
            # Warp the image using the rotation matrix
            # Append the image to the output list
        # Return the output
        return None

    # Input image, output a list of blurred images
    # TODO: REQUIRED
    def blur_image(self, img):
        # Create a list to hold the output
        # Define the blur amounts
        # For each blur amount
            # Blur the image HINT: cv2.blur
            # Append the image to the output list
        # Return the output
        return None

    # Input image, output a list of images with different brightness
    # TODO: REQUIRED
    def change_brightness(self, img):
        # Create a list to hold the output
        # Define the brightness levels
        # For each brightness level
            # Convert the image into HSV
            # Add the brightness level to the V component
            # Revert to BRG format
            # Append the image to the output list
        # Return the output
        return None

    # Input image, output a list of noisy images
    # TODO: REQUIRED
    def add_noise(self, img):
        # Create a list to hold the output
        # Define the mean and sigma values
        # For each mean value
            # For each sigma value
                # Create random noise with the same shape as the image HINT: np.random.normal
                # Add the noise to the image
                # Save the new image to the output list
        # Return the output
        return None

    # Input image, output a list of images with a combination of the above functions
    # TODO: REQUIRED
    def combine_data_augmentation_functions(self, img):
        # Combine the above functions to return different combintations of transformations
        return None
