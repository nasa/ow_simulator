#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import roslib
import unittest
import sys
import os
import numpy as np
from ow_dynamic_terrain.msg import modify_terrain_patch
sys.path.append(os.path.abspath('../scripts'))
from modify_terrain_patch_pub import scale_image_intensities

PKG = 'ow_dynamic_terrain'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.


# ModifyTerrainPatch
class ModifyTerrainPatch(unittest.TestCase):

  def check_image_equal_to_value(self, image, value):
    h = image.shape[0]
    w = image.shape[1]
    for y in range(0, h):
      for x in range(0, w):
        self.assertAlmostEqual(image[y, x], value,
            msg="pixel(x = %d, y = %d) = %d != %d" % (x, y, image[y, x], value))

  def test_scale_image_intensities(self):
    image = np.ones((3, 3, 1), np.float32)
    self.check_image_equal_to_value(image, 1.0)
    scale_image_intensities(image, 2.0)
    self.check_image_equal_to_value(image, 2.0)


if __name__ == '__main__':
  import rosunit
  rosunit.unitrun(PKG, 'test_modify_terrain_patch', ModifyTerrainPatch)
