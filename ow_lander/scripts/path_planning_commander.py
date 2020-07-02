#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

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
#import activity_dig_trench
import activity_move_guarded
import activity_sample_delivery
import activity_saw_motion_planning
#import activity_reset

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

# === SERVICE ACTIVITIES - MOVE GUARDED =============================
def handle_move_guarded(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting move guarded planning session"
    args = activity_move_guarded.arg_parsing(req)

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "pre_move_guarded_traj_"
    bagname = location + currentDT

    # Approach
    utils.start_traj_recording(args[1], bagname)
    result = activity_move_guarded.pre_move_guarded(interface.move_arm,interface.move_limbs,args)
    utils.stop_traj_recording(result, bagname)

    # Safe move, monitoring torques
    location = "move_guarded_traj_"
    bagname = location + currentDT
    utils.start_traj_recording(False, bagname)
    result = activity_move_guarded.move_guarded(interface.move_arm,interface.move_limbs,args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished move guarded planning session succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - FULL circular TRAJ =============================
def handle_start_planning(req):
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
    result = activity_full_digging_traj.dig_trench(interface.move_arm,interface.move_limbs,args[1],args[2],args[3],args[4])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - Dig Linear Trench =============================
def handle_dig_linear_trench(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    args = activity_full_digging_traj.arg_parsing_lin(req)
    #args = activity_dig_trench.arg_parsing(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[5], bagname)
    #result = activity_full_digging_traj.dig_trench(interface.move_arm,interface.move_limbs,args[1],args[2],args[3])
    #result = activity_dig_trench.dig_trench(interface.move_arm,interface.move_limbs,args[1],args[2],args[3])
    result = activity_full_digging_traj.dig_linear_trench(interface.move_arm,interface.move_limbs,args[1],args[2],args[3],args[4])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session for linear trenching succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - SampleDelivery =============================
def handle_sample_delivery(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting sample delivery session"
    args = activity_sample_delivery.arg_parsing(req)
    #args = activity_dig_trench.arg_parsing(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid sample delivery input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[4], bagname)
    result = activity_sample_delivery.sample_delivery(interface.move_arm,interface.move_limbs,args[1],args[2],args[3])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session for linear trenching succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - Reset=============================
def handle_reset(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    #args = activity_reset.arg_parsing(req)
    args = activity_full_digging_traj.arg_parsing_reset(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[4], bagname)
    #result = activity_full_digging_traj.dig_trench(interface.move_arm,interface.move_limbs,args[1],args[2],args[3])
    #result = activity_reset.go_home(interface.move_arm)#,interface.move_limbs,args[1],args[2],args[3])
    result = activity_full_digging_traj.go_home(interface.move_arm)#,interface.move_limbs,args[1],args[2],args[3])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning reset succesfully..."
  return True, "Done"


# === SERVICE ACTIVITIES - Unstowed=============================
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






  # === SERVICE ACTIVITIES - Saw motion planning =============================
def handle_saw_motion_planning(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting saw motion planning session"
    args = activity_full_digging_traj.arg_parsing_lin(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid saw trajectory input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(args[5], bagname)
    result = activity_saw_motion_planning.saw_motion_planning(interface.move_arm,interface.move_limbs,args[1],args[2],args[3],args[4])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Saw motion planning succesfully finished..."
  return True, "Done"



# === MAIN ================================================
def main():
  rospy.init_node('path_planning_commander', anonymous=True)

  # Setup planner triggering service
  start_srv = rospy.Service('start_plannning_session', StartPlanning, handle_start_planning)
  start_dig_srv = rospy.Service('start_dig_trench_session', DigTrench, handle_dig_linear_trench)
  move_guarded_srv = rospy.Service('start_move_guarded', MoveGuarded, handle_move_guarded)
  move_reset_srv = rospy.Service('start_reset_session', MoveReset, handle_reset)
  move_reset_srv = rospy.Service('start_sample_delivery', SampleDelivery, handle_sample_delivery)
  saw_motion_plan_srv = rospy.Service('start_saw_motion_planning',SawMotionPlanning, handle_saw_motion_planning)
  unstow_srv = rospy.Service('start_unstow_session', Unstow, handle_unstow)

  rospy.spin()

if __name__ == '__main__':
  main()
