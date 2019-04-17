#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import sys
import rosbag
import subprocess, os, signal
import copy
import rospy
import time
import csv
import datetime
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from ow_lander.srv import *

## GLOBAL VARS ##
J_SCOOP_YAW = 5
J_HAND_YAW = 4
J_DIST_PITCH = 3 
J_PROX_PITCH = 2
J_SHOU_PITCH = 1
J_SHOU_YAW = 0

X_SHOU = 0.79
Y_SHOU = 0.175
HAND_Y_OFFSET = 0.0249979319838 
GROUND_POSITION = -0.175
SCOOP_OFFSET = 0.215

X_DELIV = 0.2
Y_DELIV = 0.2
Z_DELIV = 1.2
SHOU_YAW_DELIV = 0.4439

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_arm = moveit_commander.MoveGroupCommander("arm")
    move_limbs = moveit_commander.MoveGroupCommander("limbs")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    self.move_arm = move_arm
    self.move_limbs = move_limbs

  def go_home(self):
    # Move to home position
    move_arm = self.move_arm
    joint_goal = move_arm.get_current_joint_values()
    joint_goal[J_DIST_PITCH] = 3.1416
    joint_goal[J_HAND_YAW] = 0
    joint_goal[J_PROX_PITCH] = -2.75
    joint_goal[J_SHOU_PITCH] = 1.5708
    joint_goal[J_SHOU_YAW] = -1.5
    joint_goal[J_SCOOP_YAW] = 0
    move_arm.go(joint_goal, wait=True)
    move_arm.stop()

  def dig_trench(self, x_tr, y_tr, depth):
    move_arm = self.move_arm
    move_limbs = self.move_limbs

    # Compute shoulder yaw angle to trench
    alpha = math.atan2(y_tr-Y_SHOU, x_tr-X_SHOU)
    h = math.sqrt( pow(y_tr-Y_SHOU,2) + pow(x_tr-X_SHOU,2) )
    l = Y_SHOU - HAND_Y_OFFSET
    beta = math.asin (l/h)

    # Move to pre trench position, align shoulder yaw
    joint_goal = move_arm.get_current_joint_values()
    joint_goal[J_DIST_PITCH] = 0
    joint_goal[J_HAND_YAW] = math.pi/2.2
    joint_goal[J_PROX_PITCH] = -math.pi/2
    joint_goal[J_SHOU_PITCH] = math.pi/2
    joint_goal[J_SHOU_YAW] = alpha + beta
    
    # If out of joint range, abort (TODO: parse limit from urdf)
    if (joint_goal[J_SHOU_YAW]<-1.8) or (joint_goal[J_SHOU_YAW]>1.8): 
      return False
    
    joint_goal[J_SCOOP_YAW] = 0
    move_arm.go(joint_goal, wait=True)
    move_arm.stop()

    # Once aligned to trench goal, place hand above trench middle point
    goal_pose = move_limbs.get_current_pose().pose
    goal_pose.position.x = x_tr
    goal_pose.position.y = y_tr
    goal_pose.position.z = GROUND_POSITION + SCOOP_OFFSET - depth
    move_limbs.set_pose_target(goal_pose)
    plan = move_limbs.plan()

    if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
      return False

    plan = move_limbs.go(wait=True)
    move_limbs.stop()
    move_limbs.clear_pose_targets()

    # Rotate hand yaw to dig in
    joint_goal = move_arm.get_current_joint_values()
    joint_goal[J_HAND_YAW] = 0
    move_arm.go(joint_goal, wait=True)
    move_arm.stop()

    # Insert here for linear trenching

    # Rotate hand yaw to dig out
    joint_goal = move_arm.get_current_joint_values()
    joint_goal[J_HAND_YAW] = -math.pi/2.2
    move_arm.go(joint_goal, wait=True)
    move_arm.stop()

    # # Go back to safe position and align yaw to deliver
    # joint_goal = move_arm.get_current_joint_values()
    # joint_goal[J_DIST_PITCH] = 0
    # joint_goal[J_HAND_YAW] = -math.pi/2
    # joint_goal[J_PROX_PITCH] = -math.pi/2
    # joint_goal[J_SHOU_PITCH] = math.pi/2
    # joint_goal[J_SHOU_YAW] = SHOU_YAW_DELIV
    # joint_goal[J_SCOOP_YAW]= 0
    # move_arm.go(joint_goal, wait=True)
    # move_arm.stop()

    # # Go to deliver position
    # joint_goal = move_arm.get_current_joint_values()
    # joint_goal[J_PROX_PITCH]= math.pi/2 - 0.1
    # joint_goal[J_SCOOP_YAW]= math.pi - 0.05
    # move_arm.go(joint_goal, wait=True)
    # move_arm.stop()

    # # Deliver (high amplitude)
    # joint_goal = move_arm.get_current_joint_values()
    # joint_goal[J_HAND_YAW] = -math.pi
    # move_arm.go(joint_goal, wait=True)
    # move_arm.stop()
    # joint_goal[J_HAND_YAW] = math.pi/2
    # move_arm.go(joint_goal, wait=True)
    # move_arm.stop()

    return True

# Converts csv trajectory file to 6 yaml joint trajectories
def csv_to_yamls(filename): 
  rows = [] 
  out = []
  nb_links = 6

  for x in range(nb_links):
    out.append(open(filename[:-4] + "_j" + str(x+1) + ".yaml","w"))

  # reading csv file 
  with open(filename, 'r') as csvfile: 
    # creating a csv reader object 
    csvreader = csv.reader(csvfile) 
  
    # extracting each data row one by one 
    for row in csvreader: 
      rows.append(row) 
  
  for row in rows[1:]: 
    for x in range(nb_links):
      out[x].write("---\n")
      out[x].write("%14s\n"%row[12+x])

  for x in range(nb_links):
    out[x].write("...")
    out[x].close()

# Basic check if input trench arguments are numbers
def check_arguments(tx, ty, td):
  try:
    float(tx)
    float(ty)
    float(td)
    return True
  except ValueError:
    return False

def handle_start_planning(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting planning session with trench_x=%s, trench_y=%s, trench_d=%s and delete_prev_traj=%s"%(req.trench_x, req.trench_y, req.trench_d, req.delete_prev_traj)
    
    if req.use_defaults :
      # Default trenching values
      trench_x=1.5
      trench_y=0
      trench_d=0.02
      delete_prev_traj=True
    else :
      trench_x=req.trench_x
      trench_y=req.trench_y
      trench_d=req.trench_d
      delete_prev_traj=req.delete_prev_traj

    

    #trench_x = rospy.get_param('/path_planning_commander/trench_x')
    #trench_y = rospy.get_param('/path_planning_commander/trench_y')
    #trench_d = rospy.get_param('/path_planning_commander/trench_d')

    # If argument is true, delet all traj files in /.ros, to prevent sending wrong traj
    #if rospy.get_param('/path_planning_commander/delete_prev_traj') == True :
    if delete_prev_traj == True :
      os.system("rm ~/.ros/traj*")


    if check_arguments(trench_x, trench_y, trench_d) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      #os.system("ps -ef | grep rosmaster | grep -v grep | awk '{print $2}' | xargs kill")
      return

    # Home robot
    interface.go_home()
    
    # Start rosbag recording
    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    location = "trajectory_"
    bagname = location + currentDT
    command = "rosbag record -O " + bagname + " /joint_states"    
    p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd='.')
    
    if (interface.dig_trench(trench_x,trench_y,trench_d) == True) :
      time.sleep(1)
      # Stop rosbag recording (TODO: clean process)
      os.system("killall -s SIGINT record") 
    
    else: # Clean empty bag and exit
      time.sleep(1)
      # Stop rosbag recording (TODO: clean process)
      os.system("killall -s SIGINT record") 
      time.sleep(1)
      print "[ERROR] No plan found. Exiting path_planning_commander..."
      command = "rm " + bagname + ".bag"
      os.system(command)
      #os.system("ps -ef | grep rosmaster | grep -v grep | awk '{print $2}' | xargs kill")
      return
    
    time.sleep(1)

    # rosbag to csv
    trajname = bagname + ".csv" 
    command = "rostopic echo -p -b " + bagname + ".bag /joint_states > " + trajname
    print command
    os.system(command)
    time.sleep(1)

    # csv to yaml
    # csv_to_yamls(trajname) 

    # Cleanup bag and csv
    command = "rm " + bagname + ".bag"
    os.system(command)
    #command = "rm " + trajname
    #os.system(command)

    # Kill rosmaster and exit
    # time.sleep(1)
    # os.system("ps -ef | grep rosmaster | grep -v grep | awk '{print $2}' | xargs kill")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session succesfully..."
  return True, "Hi"

def main():
  rospy.init_node('path_planning_commander', anonymous=True)

  # Setup planner triggering service
  #start_srv = rospy.Service('start_plannning_session', StartPlanning, handle_start_planning)
  start_srv = rospy.Service('start_plannning_session', StartPlanning, handle_start_planning)

  rospy.spin()

if __name__ == '__main__':
  main()



