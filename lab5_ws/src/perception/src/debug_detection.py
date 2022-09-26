#!/usr/bin/env python
import cv2
from marine_life_detection import MarineLifeDetection
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np

# This function takes in an image and plots the HSV pixels values for the image
def debug_hsv(image, fig):
  plt.clf()
  img = cv2.resize(image, (50, 50))
  img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  pixel_colors = img.reshape((np.shape(img)[0]*np.shape(img)[1], 3))
  norm = colors.Normalize(vmin=-1.,vmax=1.)
  norm.autoscale(pixel_colors)
  pixel_colors = norm(pixel_colors).tolist()
  hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
  h, s, v = cv2.split(hsv_img)
  axis = fig.add_subplot(1, 1, 1, projection="3d")
  axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
  axis.set_xlabel("Hue")
  axis.set_ylabel("Saturation")
  axis.set_zlabel("Value")

  # Value vs Saturation
  # axis.view_init(0, 0)
  # Value vs Hue
  # axis.view_init(0, 90)
  # Saturation vs Hue
  # axis.view_init(90, 0)

  return h,s,v,pixel_colors

# Create the marine life detection class
d = MarineLifeDetection()

# For the 8 test images
for i in np.arange(1, 9):

  # Load the test image
  image = cv2.imread('../test_data/test' + str(i) + '.jpeg')

  # Convert to smaller size for easier viewing
  img = cv2.resize(image, (200, 200))

  # Create a figure showing the HSV values
  fig = plt.figure(1)
  h,s,v,pixel_colors = debug_hsv(img, fig)
  plt.pause(0.05)

  # Use the marine life detection class functions
  # NOTE: These are the functions you will need to implement
  mask = d.create_marine_life_mask(img)
  seg_img = d.segment_image(img, mask)
  detected = d.marine_life_detected(mask)
  print("Animal Detected: " + str(detected))

  # Display returned images
  if img is not None:
    cv2.imshow('Raw Image',img)
  if seg_img is not None:
    cv2.imshow('Marine Life', seg_img)

  # Wait 5 seconds before continuing the loop
  key = cv2.waitKey(10000)
  if key == 27:
      cv2.destroyAllWindows()
      break

cv2.destroyAllWindows()