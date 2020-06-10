#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import os, argparse, time
import rospy, rospkg
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from ow_dynamic_terrain.msg import modify_terrain_patch

def get_image_path():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="path of the image file to send", type=str)
    args = parser.parse_args()
    return args.image

def scale_image_intensities(image, scale):
    h = image.shape[0]
    w = image.shape[1]
    for y in range(0, h):
        for x in range(0, w):
            image[y, x] *= scale

def compose_modify_terrain_patch_message(image_path):
    msg = modify_terrain_patch()
    msg.position = Point(0, 0, 0)
    msg.orientation = 45.0
    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    min_intensity, max_intensity, _, _ = cv2.minMaxLoc(image)
    z_scale = 1.0 / (max_intensity - min_intensity)
    scale_image_intensities(image, z_scale)
    cv_bridge = CvBridge()
    msg.patch = cv_bridge.cv2_to_imgmsg(image)
    return msg

def publish_image(image_path):
    rospy.init_node("modify_terrain_patch_pub", anonymous=True)
    pub = rospy.Publisher('/ow_dynamic_terrain/modify_terrain_patch', modify_terrain_patch, queue_size=1)

    try:
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            if connections > 0:
                msg = compose_modify_terrain_patch_message(image_path)
                pub.publish(msg)
                rospy.loginfo("modify_terrain_patch message sent")
                break
            time.sleep(0.1)
    except rospy.ROSInterruptException, e:
        raise e

if __name__ == '__main__':
    image_path = get_image_path()
    if not os.path.exists(image_path):
        print("file not found")
        quit()
    publish_image(image_path)
    
