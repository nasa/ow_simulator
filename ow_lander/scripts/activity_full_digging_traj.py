#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import constants
import math
import copy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from utils import is_shou_yaw_goal_in_range


def go_to_Z_coordinate(move_group,x_tr,y_tr,z_tr):
  goal_pose = move_group.get_current_pose().pose
  goal_pose.position.x = x_tr
  goal_pose.position.y = y_tr
  goal_pose.position.z = z_tr
  move_group.set_pose_target(goal_pose)
  plan = move_group.plan()
  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False
  plan = move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

def change_joint_value(move_group,joint_name,target_value):
  joint_goal = move_group.get_current_joint_values()
  joint_goal[joint_name] = target_value
  move_group.go(joint_goal, wait=True)
  move_group.stop()

def arg_parsing_lin(req):
  if req.use_defaults :
    # Default trenching values
    x_start=1.5
    y_start=0
    depth=0.01
    length=0.3
    delete_prev_traj=False

  else :
    x_start=req.x
    y_start=req.y
    depth=req.depth
    length=req.length
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,x_start,y_start,depth,length,delete_prev_traj]

def arg_parsing_circ(req):
  if req.use_defaults :
    # Default trenching values
    x_start=1.5
    y_start=0
    depth=0.02
    radial=False
    delete_prev_traj=False

  else :
    x_start=req.x
    y_start=req.y
    depth=req.depth
    radial=req.radial
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,x_start,y_start,depth,radial,delete_prev_traj]

def move_to_pre_trench_configuration(move_arm, x_tr, y_tr):
  # Compute shoulder yaw angle to trench
  alpha = math.atan2(y_tr-constants.Y_SHOU, x_tr-constants.X_SHOU)
  h = math.sqrt( pow(y_tr-constants.Y_SHOU,2) + pow(x_tr-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)
    # Move to pre trench position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = math.pi/2.2
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
    return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  return True


def plan_cartesian_path_lin(move_arm, length, alpha):

  waypoints = []
  wpose = move_arm.get_current_pose().pose

  wpose.position.x += length*math.cos(alpha)
  wpose.position.y += length*math.sin(alpha)

  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # end effector follow step (meters)
                               0.0)         # jump threshold
  return plan, fraction

def dig_linear(move_arm,move_limbs,x_tr, y_tr, depth, length):

  pre_move_complete = move_to_pre_trench_configuration(move_arm, x_tr, y_tr)
  if pre_move_complete == False:
    return False

  ## Rotate hand yaw to dig in
  change_joint_value(move_arm,constants.J_HAND_YAW,0.0)

  #rotate scoop
  change_joint_value(move_arm,constants.J_SCOOP_YAW,math.pi/2)

  #rotate dist pith to pre-trenching position.
  change_joint_value(move_arm,constants.J_DIST_PITCH,-math.pi/2)

  ## Once aligned to trench goal, place hand above trench middle point
  z_tr = constants.GROUND_POSITION + constants.SCOOP_OFFSET - depth
  go_to_Z_coordinate(move_limbs,x_tr,y_tr,z_tr)

  #  rotate to dig in the ground
  change_joint_value(move_arm,constants.J_DIST_PITCH,55.0/180.0*math.pi)

  # determine linear trenching direction (alpha)
  current_pose = move_arm.get_current_pose().pose
  quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
  current_euler = euler_from_quaternion(quaternion)
  alpha = current_euler[2]

  # linear trenching
  cartesian_plan, fraction = plan_cartesian_path_lin(move_arm, length, alpha)
  move_arm.execute(cartesian_plan, wait=True)
  move_arm.stop()

  #  rotate to dig out
  change_joint_value(move_arm,constants.J_DIST_PITCH,math.pi/2)

  return True


def dig_circular(move_arm,move_limbs,x_tr,y_tr,depth,radial):

  pre_move_complete = move_to_pre_trench_configuration(move_arm, x_tr, y_tr)
  if pre_move_complete == False:
    return False

  if radial==False:

    # Once aligned to trench goal, place hand above trench middle point
    z_tr = constants.GROUND_POSITION + 3*constants.SCOOP_HEIGHT - depth
    go_to_Z_coordinate(move_limbs,x_tr,y_tr,z_tr)

    # Rotate hand perpendicular to radial direction
    change_joint_value(move_arm,constants.J_HAND_YAW,-math.pi/2.2)

  else:
    # Rotate hand so scoop is in middle point
    change_joint_value(move_arm,constants.J_HAND_YAW,0.0)

    # Rotate scoop
    change_joint_value(move_arm,constants.J_SCOOP_YAW, math.pi/2)

    # Rotate dist so scoop is back
    change_joint_value(move_arm,constants.J_DIST_PITCH,-19.0/54.0*math.pi)

    # Once aligned to trench goal, place hand above trench middle point
    z_tr = constants.GROUND_POSITION + 3*constants.SCOOP_HEIGHT - depth
    go_to_Z_coordinate(move_limbs,x_tr,y_tr,z_tr)

    # Rotate dist to dig
    joint_goal = move_arm.get_current_joint_values()
    dist_now = joint_goal[3]
    change_joint_value(move_arm,constants.J_DIST_PITCH,dist_now + 2*math.pi/3)

  return True


def go_home(move_arm):
  # Move to home position
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_SHOU_YAW] = rospy.get_param('/stowed_shou_yaw', default=0)
  joint_goal[constants.J_SHOU_PITCH] = rospy.get_param('/stowed_shou_pitch', default=0)
  joint_goal[constants.J_PROX_PITCH] = rospy.get_param('/stowed_prox_pitch', default=0)
  joint_goal[constants.J_DIST_PITCH] = rospy.get_param('/stowed_dist_pitch', default=0)
  joint_goal[constants.J_HAND_YAW] = rospy.get_param('/stowed_hand_yaw', default=0)
  joint_goal[constants.J_SCOOP_YAW] = rospy.get_param('/stowed_scoop_yaw', default=0)
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()


def unstow(move_arm):

  change_joint_value(move_arm,constants.J_SHOU_YAW,0.0)
  move_to_pre_trench_configuration(move_arm, 2.0, 0.0)

  return True
