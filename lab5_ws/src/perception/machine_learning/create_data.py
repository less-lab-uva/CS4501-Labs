import os
import glob
import cv2
import random
import numpy as np

from tqdm import tqdm
from data_augmentation import DataAugmentation

# Create the folders where we will save the data
if not os.path.exists('training_data'):
    os.makedirs('training_data')

if not os.path.exists('testing_data'):
    os.makedirs('testing_data')

# Get the file names
file_names = glob.glob("./raw_data/*.jpeg")

# Create the raw data
raw_data = {}
raw_data["seal"] = []
raw_data["dolphin"] = []
raw_data["penguin"] = []

save_counter = 0

# For each of the files
for f in file_names:

    # Get the label
    label = f[f.rfind("/")+1: f.find("0")]

    # Get the data by class
    raw_data[label].append(f)

    # Create the structure
    if not os.path.exists('training_data/' + label):
        os.makedirs('training_data/' + label)
    if not os.path.exists('testing_data/' + label):
        os.makedirs('testing_data/' + label)

# Create output data
output_data = {}
output_data["seal"] = []
output_data["dolphin"] = []
output_data["penguin"] = []

# Create the data augmentation class
augment = DataAugmentation()

# Loop over each class
for key in tqdm(raw_data, position=0, desc="Animals"):

    # Loop over every file
    for i in tqdm(range(len(raw_data[key])), position=1, desc="Images"):
        
        # Load the data
        img = cv2.imread(raw_data[key][i])
        img = cv2.resize(img, (200, 200))
        output_data[key].append(img)

        # Rotate the image
        rotated_images = augment.rotate_image(img)
        for r in rotated_images:
            output_data[key].append(r)

        # Translate the image
        translated_images = augment.translate_image(img)
        for t in translated_images:
            output_data[key].append(t)

        # Blur the image
        blurred_images = augment.blur_image(img)
        for b in blurred_images:
            output_data[key].append(b)

        # Update the brightness
        brightness_images = augment.change_brightness(img)
        for b in brightness_images:
            output_data[key].append(b)

        # Add noise
        noisy_images = augment.add_noise(img)
        for n in noisy_images:
            output_data[key].append(n)

        # Add zoom
        zoomed_images = augment.zoom(img)
        for z in zoomed_images:
            output_data[key].append(z)

        # Combination of all of them
        combination_images = augment.combine_data_augmentation_functions(img)
        for c in combination_images:
            output_data[key].append(c)

print("Saving data")  
# Save the data
for key in tqdm(output_data, position=0, desc="Animals"):
    
    # Shuffle the data
    shuffled_data = random.sample(output_data[key], len(output_data[key]))

    # Save the first 10% as test and 90% as train
    test_percentage = 0.1
    
    # Used to save final images
    final_image = 0

    # Create a save index
    for index in tqdm(range(len(shuffled_data)), position=1, desc="Images"):

        # Test data
        if index < (len(shuffled_data) * test_percentage):
            save_name = "./testing_data/" + key + "/" + str(index) + ".jpeg"
        else:
            save_name = "./training_data/" + key + "/" + str(index) + ".jpeg"
        
        # Save the data
        img = shuffled_data[index]
        cv2.imwrite(save_name, img)

        # If this is the one of the last 3 images also save it as a single prediction image
        if index >= len(shuffled_data) - 3:
            save_name = "./testing_data/{}_{}.jpeg".format(key, final_image)
            final_image += 1
            cv2.imwrite(save_name, img)