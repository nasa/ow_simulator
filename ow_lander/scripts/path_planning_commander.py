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

import constants
import utils
import activity_full_digging_traj
import activity_move_guarded

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

    bagname = utils.start_traj_recording(args[1], "move_guarded_traj_")
    result, guard_start_time = activity_move_guarded.move_guarded(interface.move_arm,interface.move_limbs,args)
    utils.stop_traj_recording(result, bagname, guard_start_time)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished move guarded planning session succesfully..."
  return True, "Done"

# === SERVICE ACTIVITIES - FULL TRAJ =============================
def handle_start_planning(req):
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    args = activity_full_digging_traj.arg_parsing(req)

    if utils.check_arguments(args[1],args[2],args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    bagname = utils.start_traj_recording(args[4], "full_traj_")
    result = activity_full_digging_traj.dig_trench(interface.move_arm,interface.move_limbs,args[1],args[2],args[3])
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session succesfully..."
  return True, "Done"

# === MAIN ================================================
def main():
  rospy.init_node('path_planning_commander', anonymous=True)

  # Setup planner triggering service
  start_srv = rospy.Service('start_plannning_session', StartPlanning, handle_start_planning)
  move_guarded_srv = rospy.Service('start_move_guarded', MoveGuarded, handle_move_guarded)

  rospy.spin()

if __name__ == '__main__':
  main()

