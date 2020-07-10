#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rosbag
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from ow_lander.srv import *
import datetime
import time

import constants
import utils
import activity_full_digging_traj
import activity_guarded_move
import activity_deliver_sample
import activity_grind

# === MAIN COMMANDER CLASS =============================
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

# === SERVICE ACTIVITIES - guarded move =============================
def handle_guarded_move(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting guarded move planning session"
    args = activity_guarded_move.arg_parsing(req)

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "pre_guarded_move_traj_"
    bagname = location + currentDT

    # Approach
    utils.start_traj_recording(args[1], bagname)
    result = activity_guarded_move.pre_guarded_move(interface.move_arm,interface.move_limbs,args)
    utils.stop_traj_recording(result, bagname)

    # Safe move, monitoring torques
    location = "guarded_move_traj_"
    bagname = location + currentDT
    utils.start_traj_recording(False, bagname)
    result = activity_guarded_move.guarded_move(interface.move_arm,interface.move_limbs,args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished guarded move planning session succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - Dig circular trench =============================
def handle_dig_circular(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    args = activity_full_digging_traj.arg_parsing_circ(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[5], bagname)
    result = activity_full_digging_traj.dig_circular(interface.move_arm,interface.move_limbs,args[1],args[2],args[3],args[4])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - Dig Linear Trench =============================
def handle_dig_linear(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    args = activity_full_digging_traj.arg_parsing_lin(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[5], bagname)
    result = activity_full_digging_traj.dig_linear(interface.move_arm,interface.move_limbs,args[1],args[2],args[3],args[4])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session for linear trenching succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - deliver sample =============================
def handle_deliver_sample(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting sample delivery session"
    args = activity_deliver_sample.arg_parsing(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid sample delivery input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[4], bagname)
    result = activity_deliver_sample.deliver_sample(interface.move_arm,interface.move_limbs,args[1],args[2],args[3])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session for linear trenching succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - Stow =============================
def handle_stow(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    args = activity_full_digging_traj.arg_parsing_stow(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[4], bagname)
    result = activity_full_digging_traj.go_home(interface.move_arm)#,interface.move_limbs,args[1],args[2],args[3])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning stow succesfully..."
  return True, "Done"


# === SERVICE ACTIVITIES - Unstow =============================
def handle_unstow(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Moving to unstowed configuration..."

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(False, bagname)
    result = activity_full_digging_traj.unstow(interface.move_arm)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Moved to unstowed configuration succesfully..."
  return True, "Done"






  # === SERVICE ACTIVITIES - Grind =============================
def handle_grind(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting grinder planning session"
    args = activity_full_digging_traj.arg_parsing_lin(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid grinder trajectory input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[5], bagname)
    result = activity_grind.grind(interface.move_arm,interface.move_limbs,args[1],args[2],args[3],args[4])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Grinder planning succesfully finished..."
  return True, "Done"



# === MAIN ================================================
def main():
  rospy.init_node('path_planning_commander', anonymous=True)

  # Setup planner triggering service
  dig_circular_srv = rospy.Service('arm/dig_circular', DigCircular, handle_dig_circular)
  dig_linear_srv = rospy.Service('arm/dig_linear', DigLinear, handle_dig_linear)
  guarded_move_srv = rospy.Service('arm/guarded_move', GuardedMove, handle_guarded_move)
  stow_srv = rospy.Service('arm/stow', Stow, handle_stow)
  unstow_srv = rospy.Service('arm/unstow', Unstow, handle_unstow)
  deliver_sample_srv = rospy.Service('arm/deliver_sample', DeliverSample, handle_deliver_sample)
  grind_srv = rospy.Service('arm/grind', Grind, handle_grind)

  rospy.spin()

if __name__ == '__main__':
  main()
