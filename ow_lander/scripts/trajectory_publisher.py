#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf2_ros
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from ow_lander.srv import *
import csv
import time
import glob
import os
import constants
import numpy as np
import pylab
from ow_lander.msg import GuardedMoveResult


velocity_array = np.array([0.0] *1)


def thresholding_algo(y, lag, threshold, influence):
  ground_found = 0
  if (len(y) < lag):
    return
  filteredY = np.array(y)
  avgFilter = [0]*len(y)
  stdFilter = [0]*len(y)
  avgFilter[lag - 1] = np.mean(y[0:lag])
  stdFilter[lag - 1] = np.std(y[0:lag])
  for i in range(lag, len(y)):
    if abs(y[i] - avgFilter[i-1]) > threshold * stdFilter [i-1]:
      if y[i] > avgFilter[i-1]:
        ground_found = 1
      filteredY[i] = influence * y[i] + (1 - influence) * filteredY[i-1]
      avgFilter[i] = np.mean(filteredY[(i-lag+1):i+1])
      stdFilter[i] = np.std(filteredY[(i-lag+1):i+1])
    else:
      filteredY[i] = y[i]
      avgFilter[i] = np.mean(filteredY[(i-lag+1):i+1])
      stdFilter[i] = np.std(filteredY[(i-lag+1):i+1])

  return ground_found

def check_for_contact(y):
  # lag, threshold and innfluence can be tuned to detect contact with the ground. These numbers were tested with
  #different planning algorithms to check for ground. If these number doesnot work for your particular configuration,
  #you can re-tune the numbers. Save velocity array (uncomment line 60 np.savetxt ....), and run   peak_detect.py. see
  #peak_detect.py for more information.
  lag = 100
  threshold = 20
  influence = 1.0
  # Run algo with settings from above
  result = thresholding_algo(y, lag=lag, threshold=threshold, influence=influence)
  return result

def joint_states_cb(data):

  global velocity_array
  velocity_array  = np.append(velocity_array , data.velocity[6])
  #c = np.savetxt('velocity_array.txt', velocity_array)  # save the velocity_array to tune the ground detection


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

    # Start publisher for guarded_move result message
    guarded_move_pub = rospy.Publisher('/guarded_move_result', GuardedMoveResult, queue_size=10)

    # Start subscriber for the joint_states
    rate = rospy.Rate(int(pub_rate/2)) # Hz
    rospy.Subscriber("/joint_states", JointState, joint_states_cb)
    time.sleep(2)
    for row in guard_rows[1:]: # Cycles on all the rows except header
      if row[0][0] == '1' : # If the row is a command
        ground = check_for_contact(velocity_array)
        if (ground == 1):
          print "Found ground, stopped motion..."

          tfBuffer = tf2_ros.Buffer()
          listener = tf2_ros.TransformListener(tfBuffer)
          trans = tfBuffer.lookup_transform("base_link", "l_scoop_tip", rospy.Time(0), rospy.Duration(10.0))
          Point_X = trans.transform.translation.x
          Point_Y = trans.transform.translation.y
          Point_Z = trans.transform.translation.z
          guarded_move_pub.publish(ground, Point(Point_X, Point_Y, Point_Z), 'base_link')

          return True, "Done publishing guarded_move"

        for x in range(nb_links):
          pubs[x].publish(float("%14s\n"%row[12+x]))
        print "Guarded. Sent %s on joint[0] publisher"%(float("%14s\n"%row[12+0]))
        rate.sleep()

    guarded_move_pub.publish(ground, Point(0.0, 0.0, 0.0), 'base_link')

  return True, "Done publishing trajectory"


if __name__ == '__main__':
  rospy.init_node('trajectory_publisher', anonymous=True)
  start_srv = rospy.Service('arm/publish_trajectory', PublishTrajectory, talker)
  rospy.spin()
