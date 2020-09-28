#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import constants
import math
import time
import subprocess, os, signal
from std_msgs.msg import String
import rospy
import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF

# Basic check if input trench arguments are numbers
def check_arguments(tx,       # type: float
                    ty,       # type: float
                    td):      # type: float

  try:
    float(tx)
    float(ty)
    float(td)
    return True

  except ValueError:
    return False

def start_traj_recording(delete_prev_traj,      # type: bool
                         bagname):              # type: str

  # If argument is true, delete all traj files in /.ros, to prevent sending wrong traj
  if delete_prev_traj == True:
    os.system("rm ~/.ros/*.csv")

  # Start rosbag recording
  command = "rosbag record -O " + bagname + " /planning/joint_states"
  p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd='.')

def stop_traj_recording(result,       # type: bool
                        bagname):     # type: str

  time.sleep(1)
  # Stop rosbag recording (TODO: clean process)
  os.system("killall -s SIGINT record")

  if result == False :
    time.sleep(1)
    print "[ERROR] No plan found. Exiting path_planning_commander..."
    command = "rm " + bagname + ".bag"
    os.system(command)
    return

  time.sleep(1)

  # rosbag to csv
  trajname = bagname + ".csv"
  command = "rostopic echo -p -b " + bagname + ".bag /planning/joint_states > " + trajname
  os.system(command)
  time.sleep(1)

  # Cleanup bag and csv
  command = "rm " + bagname + ".bag"
  os.system(command)

def is_shou_yaw_goal_in_range(joint_goal):      # type: List[float, float, float, float, float, float]
  
  print "utils>is_shou_yaw_goal_in_range"
  print type(joint_goal)
  for i in joint_goal:
    print type(i)
  
  # If shoulder yaw goal angle is out of joint range, abort
  upper = URDF.from_parameter_server().joint_map["j_shou_yaw"].limit.upper
  lower = URDF.from_parameter_server().joint_map["j_shou_yaw"].limit.lower

  if (joint_goal[constants.J_SHOU_YAW]<lower) or (joint_goal[constants.J_SHOU_YAW]>upper):
    return False
    
  else:
    return True
