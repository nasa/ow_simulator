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
import glob
import os
import constants

def check_for_contact():
  return False 


def talker(req):
  pubs = []
  nb_links = 6
  rows = [] 
  pub_rate = 10 # Hz
  if req.use_latest :
    files = glob.glob('*.csv')
    latest = max(files, key=os.path.getctime)
    filename = latest
  else :
    filename = req.trajectory_filename

  #TODO: Add file check on trajectory

  print "Start publishing trajectory with filename = %s"%(filename)

  # reading csv file 
  with open(filename, 'r') as csvfile: 
    # creating a csv reader object 
    csvreader = csv.reader(csvfile) 

    # extracting each data row one by one 
    for row in csvreader: 
      rows.append(row) 

  # Extract guard_start_time
  guard_start_time = rows[-1][1]

  pubs.append(rospy.Publisher('/shou_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/shou_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/prox_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/dist_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/hand_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/scoop_yaw_position_controller/command', Float64, queue_size=40))
  rate = rospy.Rate(pub_rate) # Hz

  for row in rows[1:-2]: # Cycles on all the rows except header and guard
    if row[0][0] == '1' : # If the row is a command
      
      if guard_start_time > 0 : # If the activity is a move_guarded
        # If guard move sequence has started
        if int(row[0]) > int(guard_start_time) + constants.GUARD_INIT_INTERVAL : 
          if check_for_contact() == True :
            print "Found ground, stopped motion..."
            return True, "Done publishing move_guarded"


      for x in range(nb_links):
        pubs[x].publish(float("%14s\n"%row[12+x]))
      print "Sent %s on joint[0] publisher"%(float("%14s\n"%row[12+0]))
      rate.sleep()

  #close(filename)
  return True, "Done publishing trajectory"


if __name__ == '__main__':
  rospy.init_node('trajectory_publisher', anonymous=True)
  start_srv = rospy.Service('publish_trajectory', PublishTrajectory, talker)
  rospy.spin()