#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import constants
import math
import subprocess, os, signal
import datetime
import time
from std_msgs.msg import String

# Basic check if input trench arguments are numbers
def check_arguments(tx, ty, td):
  try:
    float(tx)
    float(ty)
    float(td)
    return True
  except ValueError:
    return False

def start_traj_recording(delete_prev_traj,prefix): 	
 # If argument is true, delete all traj files in /.ros, to prevent sending wrong traj
	if delete_prev_traj == True :
	  os.system("rm ~/.ros/" + prefix + "*")

	# Start rosbag recording
	currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
	location = prefix
	bagname = location + currentDT
	command = "rosbag record -O " + bagname + " /ns_planning/joint_states"    
	p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd='.')
	return bagname

def stop_traj_recording(result, bagname):
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
	command = "rostopic echo -p -b " + bagname + ".bag /ns_planning/joint_states > " + trajname
	os.system(command)
	time.sleep(1)

	# Cleanup bag and csv
	command = "rm " + bagname + ".bag"
	os.system(command)
