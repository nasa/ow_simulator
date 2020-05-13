#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import os
import rospy, rospkg
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from ow_dynamic_terrain.msg import modify_terrain_patch

def trench_image_path():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("ow_dynamic_terrain")
    return os.path.join(pkg_path, "misc/trench.png")


if __name__ == '__main__':
    rospy.init_node("modify_terrain_patch_pub", anonymous=True)
    pub = rospy.Publisher('/ow_dynamic_terrain/modify_terrain_patch', modify_terrain_patch, queue_size=1)
    loop_rate = rospy.Rate(0.1)
    msg = modify_terrain_patch()
    msg.position = Point(0, 0, 0)
    msg.z_scale = 1.0
    br = CvBridge()
    
    image = cv2.imread(trench_image_path())
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    msg.patch = br.cv2_to_imgmsg(image)
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("message sent")
        loop_rate.sleep()
