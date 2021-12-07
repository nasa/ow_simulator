#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from ow_dynamic_terrain.msg import modify_terrain_patch

cv_bridge = CvBridge()

def convert_tiff_to_grayscale(tiff_img):
    min_intensity, max_intensity, _, _ = cv2.minMaxLoc(tiff_img)
    img_scaled = (tiff_img - min_intensity) * (255.0 / (max_intensity - min_intensity))
    return img_scaled.astype(np.uint8)

def callback(msg):
    rospy.loginfo("modify_terrain_patch message received")
    rospy.loginfo("Position: (%d, %d, %d)" % (msg.position.x, msg.position.y, msg.position.z))
    rospy.loginfo("Orientation: %d" % msg.orientation)

    image = cv_bridge.imgmsg_to_cv2(msg.patch)
    if msg.patch.encoding == "64FC1" or msg.patch.encoding == "32FC1":
        rospy.loginfo("Converting to a displayable format ...")
        image = convert_tiff_to_grayscale(image)
    cv2.imshow("view", image)
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("modify_terrain_patch_sub", anonymous=True)
    sub = rospy.Subscriber('/ow_dynamic_terrain/modify_terrain_patch', modify_terrain_patch, callback)
    rospy.spin()
