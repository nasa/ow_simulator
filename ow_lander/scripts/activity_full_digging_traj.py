#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import rospy
import constants
import math
import copy
from tf.transformations import quaternion_from_euler
from utils import is_shou_yaw_goal_in_range
import tf


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
    trench_x=1.5
    trench_y=0
    trench_d=0.01
    length=0.3
    delete_prev_traj=False

  else :
    trench_x=req.trench_x
    trench_y=req.trench_y
    trench_d=req.trench_d
    length=req.length
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,trench_x,trench_y,trench_d,length,delete_prev_traj]

def arg_parsing_circ(req):
  if req.use_defaults :
    # Default trenching values
    trench_x=1.5
    trench_y=0
    trench_d=0.02
    radial=False
    delete_prev_traj=False

  else :
    trench_x=req.trench_x
    trench_y=req.trench_y
    trench_d=req.trench_d
    radial=req.radial
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,trench_x,trench_y,trench_d,radial,delete_prev_traj]

def arg_parsing_reset(req):
  if req.use_defaults :
    # Default trenching values
    trench_x=1.5
    trench_y=0
    trench_d=0.02
    delete_prev_traj=False

  else :
    trench_x=req.trench_x
    trench_y=req.trench_y
    trench_d=req.trench_d
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,trench_x,trench_y,trench_d,delete_prev_traj]

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


def plan_cartesian_path_lin(move_arm, length,x_tr,y_tr):

  waypoints = []
  wpose = move_arm.get_current_pose().pose

  x_shou = 10.0 * constants.SCOOP_HEIGHT
  y_shou = 2.8 * constants.SCOOP_HEIGHT
  alpha = math.atan2(y_tr-y_shou,x_tr-x_shou)
  wpose.position.x += x_tr + length*math.cos(alpha) # Second move forward/backwards in (x)
  wpose.position.y += y_tr + length*math.sin(alpha) # Second move forward/backwards in (x)

  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  #ROS_INFO("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  # Note: We are just planning, not asking move_group to actually move the robot yet:
  return plan, fraction

def dig_linear_trench(move_arm,move_limbs,x_tr, y_tr, depth, length):
  # # now = rospy.Time.now()
  # listener = tf.TransformListener()
  # now = rospy.Time(0)
  # listener.waitForTransform("base_link", "j_shou_yaw", now, rospy.Duration(5.0) );
  # (trans,rot) = listener.lookupTransform("base_link", "j_shou_yaw", now)
  # print(trans)

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

  # linear trenching
  cartesian_plan, fraction = plan_cartesian_path_lin(move_arm, length, x_tr, y_tr)
  move_arm.execute(cartesian_plan, wait=True)
  move_arm.stop()

  #  rotate to dig out
  change_joint_value(move_arm,constants.J_DIST_PITCH,math.pi/2)

  return True


def dig_trench(move_arm,move_limbs,x_tr,y_tr,depth,radial):

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


  # # Go back to safe position and align yaw to deliver
  # joint_goal = move_arm.get_current_joint_values()
  # joint_goal[constants.J_DIST_PITCH] = 0
  # joint_goal[constants.J_HAND_YAW] = -math.pi/2
  # joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  # joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  # joint_goal[constants.J_SHOU_YAW] = constants.SHOU_YAW_DELIV
  # joint_goal[constants.J_SCOOP_YAW]= 0
  # move_arm.go(joint_goal, wait=True)
  # move_arm.stop()

  # # Go to deliver position
  # joint_goal = move_arm.get_current_joint_values()
  # joint_goal[constants.J_PROX_PITCH]= math.pi/2 - 0.1
  # joint_goal[constants.J_SCOOP_YAW]= math.pi - 0.05
  # move_arm.go(joint_goal, wait=True)
  # move_arm.stop()

  # # Deliver (high amplitude)
  # joint_goal = move_arm.get_current_joint_values()
  # joint_goal[constants.J_HAND_YAW] = -math.pi
  # move_arm.go(joint_goal, wait=True)
  # move_arm.stop()
  # joint_goal[constants.J_HAND_YAW] = math.pi/2
  # move_arm.go(joint_goal, wait=True)
  # move_arm.stop()
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
