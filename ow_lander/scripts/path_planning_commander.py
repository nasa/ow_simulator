#!/usr/bin/env python2

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

class PathPlanningCommander(object):
  
  def __init__(self):
    super(PathPlanningCommander, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_arm = moveit_commander.MoveGroupCommander("arm")
    # move_limbs = moveit_commander.MoveGroupCommander("limbs")
    # move_grinder = moveit_commander.MoveGroupCommander("grinder")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.arm_move_group = move_arm
    # self.move_limbs = move_limbs
    # self.move_grinder = move_grinder

  # === SERVICE ACTIVITIES - Stow =============================
  def handle_stow(self, req): 
    """
    :type req: class 'ow_lander.srv._Stow.StowRequest'
    """
    move_group = self.arm_move_group
    stowed_goal = move_group.get_named_target_values("arm_stowed")
    move_group.go(stowed_goal, wait=True)
    move_group.stop()
    return True, "Done"

  # === SERVICE ACTIVITIES - Unstow =============================
  def handle_unstow(self, req):
    """
    :type req: class 'ow_lander.srv._Unstow.UnstowRequest'
    """
    move_group = self.arm_move_group
    stowed_goal = move_group.get_named_target_values("arm_unstowed")
    move_group.go(stowed_goal, wait=True)
    move_group.stop()
    return True, "Done"

  def run(self):
    rospy.init_node('path_planning_commander', anonymous=True)
    self.stow_srv = rospy.Service('arm/stow', Stow, self.handle_stow)
    self.unstow_srv = rospy.Service('arm/unstow', Unstow, self.handle_unstow)
    rospy.spin()


# === SERVICE ACTIVITIES - guarded move =============================
def handle_guarded_move(req): 
  """
  :type req: class 'ow_lander.srv._GuardedMove.GuardedMoveRequest'
  """
  try:
    interface = MoveGroupPythonInteface()
    print "Starting guarded move planning session"
    guarded_move_args = activity_guarded_move.arg_parsing(req)

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "pre_guarded_move_traj_"
    bagname = location + currentDT

    # Approach
    utils.start_traj_recording(guarded_move_args[1], bagname)
    result = activity_guarded_move.pre_guarded_move(interface.move_arm, guarded_move_args)
    utils.stop_traj_recording(result, bagname)

    # Safe move, monitoring torques
    location = "guarded_move_traj_"
    bagname = location + currentDT
    utils.start_traj_recording(False, bagname)
    result = activity_guarded_move.guarded_move(interface.move_arm, guarded_move_args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished guarded move planning session succesfully..."

  return True, "Done"

# === SERVICE ACTIVITIES - Dig circular trench =============================
def handle_dig_circular(req): 
  """
  :type req: class 'ow_lander.srv._DigCircular.DigCircularRequest'
  """
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    dig_circular_args = activity_full_digging_traj.arg_parsing_circ(req)

    if utils.check_arguments(dig_circular_args[1], dig_circular_args[2], dig_circular_args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(dig_circular_args[6], bagname)
    result = activity_full_digging_traj.dig_circular(interface.move_arm, interface.move_limbs, dig_circular_args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session succesfully..."

  return True, "Done"

# === SERVICE ACTIVITIES - Dig Linear Trench =============================
def handle_dig_linear(req):
  """
  :type req: class 'ow_lander.srv._DigLinear.DigLinearRequest'
  """
  try:
    interface = MoveGroupPythonInteface()
    print "Starting full traj planning session"
    dig_linear_args = activity_full_digging_traj.arg_parsing_lin(req)

    if utils.check_arguments(dig_linear_args[1], dig_linear_args[2], dig_linear_args[3]) != True:
      print "[ERROR] Invalid trench input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(dig_linear_args[6], bagname)
    result = activity_full_digging_traj.dig_linear(interface.move_arm, interface.move_limbs, dig_linear_args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Finished planning session for linear trenching succesfully..."

  return True, "Done"

# === SERVICE ACTIVITIES - deliver sample =============================
def handle_deliver_sample(req): 
  """
  :type req: class 'ow_lander.srv._DeliverSample.DeliverSampleRequest'
  """
  try:
    interface = MoveGroupPythonInteface()
    print "Starting sample delivery session"
    deliver_sample_args = activity_deliver_sample.arg_parsing(req)

    if utils.check_arguments(deliver_sample_args[1], deliver_sample_args[2], deliver_sample_args[3]) != True:
      print "[ERROR] Invalid sample delivery input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(deliver_sample_args[4], bagname)
    result = activity_deliver_sample.deliver_sample(interface.move_arm, deliver_sample_args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Sample succesfully delivered..."

  return True, "Done"

  # === SERVICE ACTIVITIES - Grind =============================
def handle_grind(req): 
  """
  :type req: class 'ow_lander.srv._Grind.GrindRequest'
  """
  try:
    interface = MoveGroupPythonInteface()
    print "Starting grinder planning session"
    grind_args = activity_grind.arg_parsing(req)

    if utils.check_arguments(grind_args[1], grind_args[2], grind_args[3]) != True:
      print "[ERROR] Invalid grinder trajectory input arguments. Exiting path_planning_commander..."
      return

    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    location = "full_traj_"
    bagname = location + currentDT

    utils.start_traj_recording(grind_args[6], bagname)
    result = activity_grind.grind(interface.move_arm, interface.move_limbs, interface.move_grinder, grind_args)
    utils.stop_traj_recording(result, bagname)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  print "Grinder planning succesfully finished..."

  return True, "Done"

# === MAIN ================================================
def main():

  # Setup planner triggering service
  # dig_circular_srv = rospy.Service('arm/dig_circular', DigCircular, handle_dig_circular)
  # dig_linear_srv = rospy.Service('arm/dig_linear', DigLinear, handle_dig_linear)
  # guarded_move_srv = rospy.Service('arm/guarded_move', GuardedMove, handle_guarded_move)
  # deliver_sample_srv = rospy.Service('arm/deliver_sample', DeliverSample, handle_deliver_sample)
  # grind_srv = rospy.Service('arm/grind', Grind, handle_grind)

if __name__ == '__main__':
  ppc = PathPlanningCommander()
  ppc.run()
