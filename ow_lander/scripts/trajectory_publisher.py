#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from ow_lander.srv import *
import csv
import time
import glob
import os
import constants
from ow_lander.msg import GuardedMoveResult
from ground_detection import GroundDetector
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
import time 
import numpy as np
import pylab
import yaml

ground_detector = None

velocity_array = np.array([0.0] *1)
current_position=  np.array([0.0] * 6)


def joint_states_cb(data):

  global velocity_array
  global current_position
  velocity_array  = np.append(velocity_array , data.velocity[6])
  j_dist_pitch    = data.position[2]
  j_hand_yaw      = data.position[3]
  j_prox_pitch    = data.position[4]
  j_scoop_yaw     = data.position[5]
  j_shou_pitch    = data.position[6]
  j_shou_yaw      = data.position[7]
  
  current_position[0] = j_shou_yaw 
  current_position[1] = j_shou_pitch 
  current_position[2] = j_prox_pitch
  current_position[3] = j_dist_pitch 
  current_position[4] = j_hand_yaw 
  current_position[5] = j_scoop_yaw 
# The talker runs once the publish service is called. It starts a publisher
# per joint controller, then reads the trajectory csvs.
# If the traj csv is a guarded move, it reads both parts.
# When publishing the safe (second) part of a guarded_move, it
# monitors the velocity, and cuts off the publishing after is ground is detected.
#The ground is detected after the peak_detection algorithm detects a spike in the velocity data


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
  
  #rospy.Subscriber("/joint_states", JointState, joint_states_cb)
  
  file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan.yaml')
  
  #with open(file_path, 'w') as file_save:
    #yaml.dump(plan, file_save, default_flow_style=True)

  with open(file_path, 'r') as file_open:
    loaded_plan = yaml.load(file_open)
  print loaded_plan  
  
  for index in range(len(loaded_plan.joint_trajectory.points)):
    for ind in range (len(loaded_plan.joint_trajectory.joint_names)):
      #print loaded_plan.joint_trajectory.joint_names[ind] , loaded_plan.joint_trajectory.points[index].positions[ind]
      #for x in range(nb_links):
      #pubs[x].publish(float("%14s\n"%row[12+x]))
      
      pubs[ind].publish(float("%14s\n"%loaded_plan.joint_trajectory.points[index].positions[ind]))     # this works
      
      #desired_pos = loaded_plan.joint_trajectory.points[index].positions[ind]
      #print ind
      #print index
      ###current_pos = 
      #error = desired_pos - current_position[ind]

  '''
  # === READ TRAJ FILE(S) ===============================
  if req.use_latest:
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
      rate.sleep()


  if guarded_move_bool : # If the activity is a guarded_move

    # Start publisher for guarded_move result message
    guarded_move_pub = rospy.Publisher('/guarded_move_result', GuardedMoveResult, queue_size=10)

    # Start subscriber for the joint_states
    time.sleep(2) # wait for 2 seconds till the arm settles
    # begin tracking velocity values as soon as the scoop moves downward 
    global ground_detector
    ground_detector.reset()
    rate = rospy.Rate(int(pub_rate/2)) # Hz

    for row in guard_rows[1:]: # Cycles on all the rows except header
      if row[0][0] == '1' : # If the row is a command
        if ground_detector.detect():
          print "Ground found! Motion stopped!"
          guarded_move_pub.publish(True, ground_detector.ground_position, 'base_link')
          return True, "Done publishing guarded_move"

        for x in range(nb_links):
          pubs[x].publish(float("%14s\n"%row[12+x]))
        rate.sleep()

    guarded_move_pub.publish(False, Point(0.0, 0.0, 0.0), 'base_link')
  '''
  return True, "Done publishing trajectory"


if __name__ == '__main__':
  global ground_detector
  rospy.init_node('trajectory_publisher', anonymous=True)
  ground_detector = GroundDetector()
  start_srv = rospy.Service('arm/publish_trajectory', PublishTrajectory, talker)
  rospy.spin()
