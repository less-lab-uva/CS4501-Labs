import os
import glob
import cv2
import numpy as np
import random
from tqdm import tqdm
from data_augmentation import DataAugmentation

# Added debug_data
if not os.path.exists('debug_data'):
    os.makedirs('debug_data')

# Get the file names
file_names = glob.glob("./raw_data/*.jpeg")

# Create the data augmentation class
augment = DataAugmentation()

# Load the image
f = file_names[0]
img = cv2.imread(f)
img = cv2.resize(img, (200, 200))

# Run each of the functions
output = {}
output["rotated_image"]     = augment.rotate_image(img)
output["translated_image"]  = augment.translate_image(img)
output["blurred_image"]     = augment.blur_image(img)
output["brightness_image"]  = augment.change_brightness(img)
output["noisy_image"]       = augment.add_noise(img)
output["zoomed_image"]      = augment.zoom(img)
output["combination_image"] = augment.combine_data_augmentation_functions(img)

# Save the data
save_name = "./debug_data/original.jpeg"
cv2.imwrite(save_name, img)
for key in tqdm(output):
    index = 0
    if output[key] is not None:
        for i in output[key]:
            index += 1
            save_name = "./debug_data/" + key + str(index) + ".jpeg"
            cv2.imwrite(save_name, i)


