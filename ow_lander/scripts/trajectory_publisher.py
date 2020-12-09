#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from ow_lander.srv import *
import csv
# import time
import glob
import os
import constants
from ow_lander.msg import GuardedMoveResult
from ground_detection import GroundDetector

ground_detector = None
# The talker runs once the publish service is called. It starts a publisher
# per joint controller, then reads the trajectory csvs.
# If the traj csv is a guarded move, it reads both parts.
# When publishing the safe (second) part of a guarded_move, it
# monitors the velocity, and cuts off the publishing after is ground is detected.
#The ground is detected after the peak_detection algorithm detects a spike in the velocity data


def talker(req):     
  """
  :type req: class 'ow_lander.srv._PublishTrajectory.PublishTrajectoryRequest'
  """
  pubs = []
  pubs.append(rospy.Publisher('/shou_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/shou_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/prox_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/dist_pitch_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/hand_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/scoop_yaw_position_controller/command', Float64, queue_size=40))
  pubs.append(rospy.Publisher('/grinder_yaw_position_controller/command', Float64, queue_size=40))
  pub_rate = constants.TRAJ_PUB_RATE # Hz
  rate = rospy.Rate(pub_rate) # Hz
  nb_links = constants.NB_ARM_LINKS
  rows = []
  guard_rows = []
  guarded_move_bool = False


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
        pubs[x].publish(float("%14s\n"%row[13+x]))
      rate.sleep()


  if guarded_move_bool : # If the activity is a guarded_move

    # Start publisher for guarded_move result message
    rospy.set_param('use_sim_time', True)
    guarded_move_pub = rospy.Publisher('/guarded_move_result', GuardedMoveResult, queue_size=10)

    # Start subscriber for the joint_states
    rospy.sleep(2) # wait for 2 seconds till the arm settles
    # begin tracking velocity values as soon as the scoop moves downward 
    global ground_detector
    ground_detector.reset()
    rate = rospy.Rate(int(pub_rate/2)) # Hz

    for row in guard_rows[1:]: # Cycles on all the rows except header
      current_time = rospy.get_time()
      # print (current_time)
      if row[0][0] == '1' : # If the row is a command
        if ground_detector.detect():
          print "Ground found! Motion stopped!"
          guarded_move_pub.publish(True, ground_detector.ground_position, 'base_link')
          return True, "Done publishing guarded_move"

        for x in range(nb_links):
          pubs[x].publish(float("%14s\n"%row[13+x]))
        rate.sleep()

    guarded_move_pub.publish(False, Point(0.0, 0.0, 0.0), 'base_link')

  return True, "Done publishing trajectory"


if __name__ == '__main__':
  global ground_detector
  rospy.set_param('gazebo/use_sim_time', True)
  rospy.init_node('trajectory_publisher', anonymous=True)
  ground_detector = GroundDetector()
  start_srv = rospy.Service('arm/publish_trajectory', PublishTrajectory, talker)
  rospy.spin()
