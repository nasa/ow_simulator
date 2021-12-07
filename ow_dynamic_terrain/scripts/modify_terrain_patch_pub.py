#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import os
import argparse
import time
import rospy
import rospkg

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from ow_dynamic_terrain.msg import modify_terrain_patch

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "image", help="path of the image file to send", type=str)
    parser.add_argument(
        "--position_x", help="x value of center of the image", type=float, default=0)
    parser.add_argument(
        "--position_y", help="y value of center of the image", type=float, default=0)
    parser.add_argument(
        "--position_z", help="z value of center of the image", type=float, default=0)
    parser.add_argument(
        "--orientation", help="rotates the image around its center (degrees)", type=float, default=0)
    parser.add_argument("--z_scale", help="a scale factor that will be applied to image intensities", type=float,
                        default=0)
    parser.add_argument("--merge_method", help=("decides how to merge the height values of supplied image with the"
                                                "current height values of the terrain"), type=str, default="add")
    return parser.parse_args()

def scale_image_intensities(image, scale):
    h = image.shape[0]
    w = image.shape[1]
    for y in range(0, h):
        for x in range(0, w):
            image[y, x] *= scale

def compose_modify_terrain_patch_message(image_path, position_x, position_y, position_z, orientation, z_scale,
                                         merge_method):
    msg = modify_terrain_patch()
    msg.position = Point(position_x, position_y, position_z)
    msg.orientation = orientation
    msg.merge_method = merge_method
    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if z_scale == 0:    # compute the automatic scale factor
        min_intensity, max_intensity, _, _ = cv2.minMaxLoc(image)
        z_scale = 1.0 / (max_intensity - min_intensity)
    scale_image_intensities(image, z_scale)
    cv_bridge = CvBridge()
    msg.patch = cv_bridge.cv2_to_imgmsg(image)
    return msg

def publish_image(args):
    rospy.init_node("modify_terrain_patch_pub", anonymous=True)
    pub = rospy.Publisher(
        '/ow_dynamic_terrain/modify_terrain_patch', modify_terrain_patch, queue_size=1)

    try:
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            if connections > 0:
                msg = compose_modify_terrain_patch_message(args.image, args.position_x, args.position_y,
                                                           args.position_z, args.orientation, args.z_scale,
                                                           args.merge_method)
                pub.publish(msg)
                rospy.loginfo("modify_terrain_patch message sent")
                break
            time.sleep(0.1)
    except rospy.ROSInterruptException as e:
        raise e

if __name__ == '__main__':
    args = parse_args()
    if not os.path.exists(args.image):
        rospy.logerror("image file not found")
        quit()
    publish_image(args)
