#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from ow_lander.srv import *
import csv
import time
import glob
import os
import constants
from numpy import mean 

effort_arr = [constants.GUARD_MAX_SLOPE_BEFORE_CONTACT] * constants.GUARD_FILTER_AV_WIDTH # Used to store efforts for filtering
previous_effort = 0
array_index = 0
slope = 0

def joint_states_cb(data):
  global effort_arr
  global array_index
  global previous_effort
  global slope  
  effort_arr[array_index] = data.effort[6]
  array_index = (array_index+1)%constants.GUARD_FILTER_AV_WIDTH

  new_effort = mean(effort_arr)
  slope = abs(new_effort - previous_effort)
  previous_effort = new_effort

# Returns True if a torque spike is detected in j_shou_pitch
def check_for_contact():
  global slope
  if slope > constants.GUARD_MAX_SLOPE_BEFORE_CONTACT:
    return True
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
  guard_start_time = int(rows[-1][1])

  pubs.append(rospy.Publisher('/shou_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/shou_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/prox_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/dist_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/hand_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/scoop_yaw_position_controller/command', Float64, queue_size=40))
  rate = rospy.Rate(pub_rate) # Hz

  if int(guard_start_time) > 0 : # If the activity is a move_guarded
    # Start subscriber for the joint_states
    rospy.Subscriber("/joint_states", JointState, joint_states_cb)

  start_guard_delay_acc = 0 
  for row in rows[1:-2]: # Cycles on all the rows except header and guard
    if row[0][0] == '1' : # If the row is a command
      
      if int(guard_start_time) > 0 : # If the activity is a move_guarded
        # If guard move sequence has started
        if int(row[0]) > int(guard_start_time) + constants.GUARD_INIT_INTERVAL : 
          print int(row[0])
          print "and"
          print int(guard_start_time) + constants.GUARD_INIT_INTERVAL

          start_guard_delay_acc += 1
          rate = rospy.Rate(int(pub_rate/2)) # Hz
          if start_guard_delay_acc > constants.GUARD_FILTER_AV_WIDTH:
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