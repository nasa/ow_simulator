#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import constants
import math
from utils import is_shou_yaw_goal_in_range
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_deliver_sample import cascade_plans
import action_dig_linear
import action_grind
from action_dig_circular import calculate_joint_state_end_pose_from_plan_arm
from std_msgs.msg import Header

pre_guarded_move_traj = RobotTrajectory()
guarded_move_traj = RobotTrajectory()

def guarded_move_plan(move_arm, robot, moveit_fk, args):
    
  robot_state = robot.get_current_state()
  move_arm.set_start_state(robot_state)
    
  ### pre-guarded move starts here ### 
    
  targ_x = args.start.x
  targ_y = args.start.y
  targ_z = args.start.z
  direction_x = args.normal.x
  direction_y = args.normal.y
  direction_z = args.normal.z
  search_distance = args.search_distance
  
  # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
  targ_elevation = -0.2
  if (targ_z+targ_elevation) == 0:
    offset = search_distance
  else:
    offset = (targ_z*search_distance)/(targ_z+targ_elevation)

  # Compute shoulder yaw angle to target
  alpha = math.atan2((targ_y+direction_y*offset)-constants.Y_SHOU,
                     (targ_x+direction_x*offset)-constants.X_SHOU)
  h = math.sqrt(pow((targ_y+direction_y*offset)-constants.Y_SHOU, 2) +
                pow((targ_x+direction_x*offset)-constants.X_SHOU, 2))
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin(l/h)

  # Move to pre move position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = 0
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
    return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  
  plan_a = move_arm.plan(joint_goal)
  if len(plan_a.joint_trajectory.points) == 0:  # If no plan found, abort
    return False
  
  # Once aligned to move goal and offset, place scoop tip at surface target offset
  cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_arm (robot, plan_a, move_arm, moveit_fk)
  move_arm.set_start_state(cs)
  goal_pose.position.x = targ_x
  goal_pose.position.y = targ_y
  goal_pose.position.z = targ_z

  move_arm.set_pose_target(goal_pose)
  plan_b = move_arm.plan()
  if len(plan_b.joint_trajectory.points) == 0:  # If no plan found, abort
    return False
  pre_guarded_move_traj = cascade_plans (plan_a, plan_b)
  
  ### pre-guarded move ends here ###  
  
  # Drive scoop tip along norm vector, distance is search_distance
  
  cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_arm (robot, pre_guarded_move_traj, move_arm, moveit_fk)
  move_arm.set_start_state(cs)
  goal_pose.position.x = targ_x
  goal_pose.position.y = targ_y
  goal_pose.position.z = targ_z
  goal_pose.position.x -= direction_x*search_distance
  goal_pose.position.y -= direction_y*search_distance
  goal_pose.position.z -= direction_z*search_distance

  move_arm.set_pose_target(goal_pose)
  plan_c  = move_arm.plan()
  
  guarded_move_traj = cascade_plans (pre_guarded_move_traj, plan_c)
  
  return guarded_move_traj
