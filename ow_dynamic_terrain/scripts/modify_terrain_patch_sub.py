#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
from ow_dynamic_terrain.msg import modify_terrain_patch
from cv_bridge import CvBridge
import cv2

def callback(msg):
    rospy.loginfo("message received")
    rospy.loginfo("Position (%d, %d, %d)" % (msg.position.x, msg.position.y, msg.position.z))
    rospy.loginfo("Z-Scale (%d)" % msg.z_scale)
    
    br = CvBridge()
    image = br.imgmsg_to_cv2(msg.patch)
    cv2.imshow("view", image)
    cv2.waitKey(5)

if __name__ == '__main__':
    rospy.init_node("modify_terrain_patch_sub", anonymous=True)
    pub = rospy.Subscriber('/ow_dynamic_terrain/modify_terrain_patch', modify_terrain_patch, callback)
    rospy.spin()
