#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import constants
import math
import datetime
import time
import rospy

def arg_parsing(req):
  if req.use_defaults :
    # Default trenching values
    delete_prev_traj=False
    target_x=2
    target_y=0
    target_z=0.02
    surface_normal_x=0
    surface_normal_y=0
    surface_normal_z=1
    offset_distance = 0.2
    overdrive_distance = 0.2
    retract = False
    

  else :
    delete_prev_traj = req.delete_prev_traj
    target_x = req.target_x
    target_y = req.target_y
    target_z = req.target_z
    surface_normal_x = req.surface_normal_x
    surface_normal_y = req.surface_normal_y
    surface_normal_z = req.surface_normal_z
    offset_distance = req.offset_distance
    overdrive_distance = req.overdrive_distance
    retract = req.retract
		
  return [req.use_defaults, delete_prev_traj, target_x, target_y, target_z, surface_normal_x,
          surface_normal_y, surface_normal_z, offset_distance, overdrive_distance, 
          retract]

# Approach
def pre_move_guarded(move_arm,move_limbs,args):
  targ_x = args[2]
  targ_y = args[3]
  targ_z = args[4]
  norm_x = args[5]
  norm_y = args[6]
  norm_z = args[7]
  offset = args[8]
  overdrive = args[9]
  retract = args[10]
  
  # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
  targ_elevation = -0.2
  
  # Compute shoulder yaw angle to target
  alpha = math.atan2( (targ_y+norm_y*offset)-constants.Y_SHOU, (targ_x+norm_x*offset)-constants.X_SHOU)
  h = math.sqrt(pow( (targ_y+norm_y*offset)-constants.Y_SHOU,2) + pow( (targ_x+norm_x*offset)-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)
  
  # Move to pre move position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = 0
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal  [constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort (TODO: parse limit from urdf)
  if (joint_goal[constants.J_SHOU_YAW]<-1.8) or (joint_goal[constants.J_SHOU_YAW]>1.8): 
    return False
  
  joint_goal[constants.J_SCOOP_YAW] = 0
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  
  # Once aligned to move goal and offset, place scoop tip at surface target offset
  goal_pose = move_limbs.get_current_pose().pose
  goal_pose.position.x = targ_x+norm_x*offset
  goal_pose.position.y = targ_y+norm_y*offset
  
  # Change to limbs, not arm, and figure quat out
  # oal_pose.orientation.x = norm_x
  # goal_pose.orientation.y = norm_y
  # goal_pose.orientation.x = norm_z
  # goal_pose.orientation.x = norm_x
  
  # TODO: trigo on scoop offset
  goal_pose.positi  on.z = targ_elevation + constants.SCOOP_OFFSET + norm_z*offset
  move_limbs.set_pose_target(goal_pose)

  plan = move_limbs.plan()
  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_limbs.go(wait=True)
  move_limbs.stop()
  move_limbs.clear_pose_targets()
  print "Done planning approach of move_guarded"
  return True

def move_guarded(move_arm,move_limbs,args):
  targ_x = args[2]
  targ_y = args[3]
  targ_z = args[4]
  norm_x = args[5]
  norm_y = args[6]
  norm_z = args[7]
  offset = args[8]
  overdrive = args[9]
  retract = args[10]
  # Drive scoop tip along norm vector, distance is offset+overdrive
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.position.x -= norm_x*(offset+overdrive)
  goal_pose.position.y -= norm_y*(offset+overdrive)
  goal_pose.position.z -= norm_z*(offset+overdrive)
  move_arm.set_max_velocity_scaling_factor(0.1) # Limit speed for approach
  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()
  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()
  print "Done planning safe part of move_guarded"
  return True

