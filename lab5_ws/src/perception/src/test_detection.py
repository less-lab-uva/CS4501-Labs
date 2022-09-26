#!/usr/bin/env python
import cv2
import unittest
import numpy as np
from marine_life_detection import MarineLifeDetection

class TestDetection(unittest.TestCase):

  def setUp(self):
    self.d = MarineLifeDetection()

  def test_image1(self):
    image = cv2.imread('../test_data/test1.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(detected)

  def test_image2(self):
    image = cv2.imread('../test_data/test2.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(not detected)

  def test_image3(self):
    image = cv2.imread('../test_data/test3.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(detected)

  def test_image4(self):
    image = cv2.imread('../test_data/test4.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(detected)

  def test_image5(self):
    image = cv2.imread('../test_data/test5.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(detected)

  def test_image6(self):
    image = cv2.imread('../test_data/test6.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(detected)

  def test_image7(self):
    image = cv2.imread('../test_data/test7.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(not detected)

  def test_image8(self):
    image = cv2.imread('../test_data/test8.jpeg')
    mask = self.d.create_marine_life_mask(image)
    detected = self.d.marine_life_detected(mask)
    self.assertTrue(detected)