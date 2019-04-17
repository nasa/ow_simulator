#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import rospy
from std_msgs.msg import Float64
from ow_lander.srv import *
import csv
import time

def talker(req):
    pubs = []
    nb_links = 6
    rows = [] 
    if req.use_latest :
        #os.sys(blablablablablablablablab)
    else :
        filename = req.trajectory_filename

    print "Start publishing trajectory with filename = %s"%(filename)

    # reading csv file 
    with open(filename, 'r') as csvfile: 
        # creating a csv reader object 
        csvreader = csv.reader(csvfile) 
  
        # extracting each data row one by one 
        for row in csvreader: 
            rows.append(row) 

    pubs.append(rospy.Publisher('/shou_yaw_position_controller/command', Float64, queue_size=40))
    pubs.append(rospy.Publisher('/shou_pitch_position_controller/command', Float64, queue_size=40))
    pubs.append(rospy.Publisher('/prox_pitch_position_controller/command', Float64, queue_size=40))
    pubs.append(rospy.Publisher('/dist_pitch_position_controller/command', Float64, queue_size=40))
    pubs.append(rospy.Publisher('/hand_yaw_position_controller/command', Float64, queue_size=40))
    pubs.append(rospy.Publisher('/scoop_yaw_position_controller/command', Float64, queue_size=40))
    rate = rospy.Rate(2) # Hz

    for row in rows[1:]: 
        for x in range(nb_links):
            pubs[x].publish(float("%14s\n"%row[12+x]))
        rate.sleep()

    #close(filename)
    return True, "Done publishing trajectory"


if __name__ == '__main__':
    rospy.init_node('trajectory_publisher', anonymous=True)
    start_srv = rospy.Service('publish_trajectory', PublishTrajectory, talker)
    rospy.spin()

# # Get all the yaml files from .ros
# var=$(ls -r ~/.ros/*.yaml)
