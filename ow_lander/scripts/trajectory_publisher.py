#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

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

effort_arr = [0] * constants.GUARD_FILTER_AV_WIDTH # Used to store efforts for filtering
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
def check_for_contact(max_slope):
  global slope
  print slope
  print max_slope
  if slope > max_slope * constants.GUARD_MAX_SLOPE_BEFORE_CONTACT_COEFF:
    return True
  return False 

# The talker runs once the publish service is called. It starts a publisher 
# per joint controller, then reads teh trajectory csvs. 
# If the traj csv is a guarded move, it reads both parts.
# When publishing the safe (second) part of a guarded_move, it
# monitors the torque on /shou_pitch, and cuts off the publishing
# if constants.GUARD_MAX_SLOPE_BEFORE_CONTACT is reached. The 
# slope after averaging the last 10 values of /shou_pitch effort
def talker(req):
  pubs = []
  pubs.append(rospy.Publisher('/shou_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/shou_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/prox_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/dist_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/hand_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/scoop_yaw_position_controller/command', Float64, queue_size=40))
  pub_rate = constants.TRAJ_PUB_RATE # Hz
  rate = rospy.Rate(pub_rate) # Hz
  nb_links = constants.NB_ARM_LINKS
  rows = [] 
  guard_rows = []
  guarded_move_bool = False
  max_slope = 0

  # === READ TRAJ FILE(S) ===============================
  if req.use_latest :
    files = glob.glob('*.csv')
    filename = max(files, key=os.path.getctime)
  else :
    filename = req.trajectory_filename

  #TODO: Add file check on trajectory
  print "Start publishing trajectory with filename = %s"%(filename)

  # Use two files instead of one if this is a guarded move
  if filename.startswith('guarded_move_traj_'):
    # reading csv file 
    guard_filename = filename
    prefix = "pre_"
    filename = prefix + filename
    guarded_move_bool = True

  # Reading csv file 
  with open(filename, 'r') as csvfile: 
    # creating a csv reader object 
    csvreader = csv.reader(csvfile) 

    # extracting each data row one by one 
    for row in csvreader: 
      rows.append(row) 

  # If guarded_move, read the guarded motion traj csv
  if guarded_move_bool :
    with open(guard_filename, 'r') as csvfile: 
      # creating a csv reader object 
      csvreader = csv.reader(csvfile) 

      # extracting each data row one by one 
      for row in csvreader: 
        guard_rows.append(row) 

  # === PUBLISH TRAJ FILE(S) ===============================
  for row in rows[1:]: # Cycles on all the rows except header
    if row[0][0] == '1' : # If the row is a command
      for x in range(nb_links):
        pubs[x].publish(float("%14s\n"%row[12+x]))
      print "Sent %s on joint[0] publisher"%(float("%14s\n"%row[12+0]))
      rate.sleep()

  if guarded_move_bool : # If the activity is a guarded_move
    # Start subscriber for the joint_states
    rate = rospy.Rate(int(pub_rate/2)) # Hz
    rospy.Subscriber("/joint_states", JointState, joint_states_cb)
    time.sleep(2)
    start_guard_delay_acc = 0 
    for row in guard_rows[1:]: # Cycles on all the rows except header
      if row[0][0] == '1' : # If the row is a command
        if start_guard_delay_acc < constants.GUARD_FILTER_AV_WIDTH:
          start_guard_delay_acc += 1
          max_slope = slope
        else:
          if check_for_contact(max_slope) == True :
            print "Found ground, stopped motion..."
            return True, "Done publishing guarded_move"

        for x in range(nb_links):
          pubs[x].publish(float("%14s\n"%row[12+x]))
        print "Guarded. Sent %s on joint[0] publisher"%(float("%14s\n"%row[12+0]))
        rate.sleep()

  return True, "Done publishing trajectory"


if __name__ == '__main__':
  rospy.init_node('trajectory_publisher', anonymous=True)
  start_srv = rospy.Service('arm/publish_trajectory', PublishTrajectory, talker)
  rospy.spin()
