#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import constants
import math
import datetime
import time
import rospy
from utils import is_shou_yaw_goal_in_range
import yaml
import os
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

guarded_move_traj = RobotTrajectory()

 
def cascade_plans (plan1, plan2):
     # Create a new trajectory object
    new_traj = RobotTrajectory()
    # Initialize the new trajectory to be the same as the planned trajectory
    traj_msg = JointTrajectory()
    # Get the number of joints involved
    n_joints1 = len(plan1.joint_trajectory.joint_names)
    n_joints2 = len(plan2.joint_trajectory.joint_names)
    # Get the number of points on the trajectory
    n_points1 = len(plan1.joint_trajectory.points)
    n_points2 = len(plan2.joint_trajectory.points)
    # Store the trajectory points
    points1 = list(plan1.joint_trajectory.points)

    points2 = list(plan2.joint_trajectory.points)
    end_time = plan1.joint_trajectory.points[n_points1-1].time_from_start
    start_time =  plan1.joint_trajectory.points[0].time_from_start
    duration =  end_time - start_time

    for i in range(n_points1):
        point = JointTrajectoryPoint()
        point.time_from_start = plan1.joint_trajectory.points[i].time_from_start
        point.velocities = list(plan1.joint_trajectory.points[i].velocities)
        point.accelerations = list(plan1.joint_trajectory.points[i].accelerations)
        point.positions = plan1.joint_trajectory.points[i].positions
        points1[i] = point
        traj_msg.points.append(point)
        end_time = plan1.joint_trajectory.points[i].time_from_start
        
    for i in range(n_points2):
        point = JointTrajectoryPoint()
        point.time_from_start = plan2.joint_trajectory.points[i].time_from_start + end_time
        point.velocities = list(plan2.joint_trajectory.points[i].velocities)
        point.accelerations = list(plan2.joint_trajectory.points[i].accelerations)
        point.positions = plan2.joint_trajectory.points[i].positions
        points1[i] = point
        traj_msg.points.append(point)
    
    traj_msg.joint_names = plan1.joint_trajectory.joint_names
    traj_msg.header.frame_id = plan1.joint_trajectory.header.frame_id
    new_traj.joint_trajectory = traj_msg
    return new_traj   
    


def arg_parsing(req):
  if req.use_defaults :
    # Default trenching values
    delete_prev_traj=False
    target_x=2.0
    target_y=0.0
    target_z=0.3
    direction_x=0.0
    direction_y=0.0
    direction_z=1.0
    search_distance = 0.5

  else :
    delete_prev_traj = req.delete_prev_traj
    target_x = req.x
    target_y = req.y
    target_z = req.z
    direction_x = req.direction_x
    direction_y = req.direction_y
    direction_z = req.direction_z
    search_distance = req.search_distance

  return [req.use_defaults, delete_prev_traj, target_x, target_y, target_z,
          direction_x, direction_y, direction_z, search_distance]

# Approach
def pre_guarded_move(move_arm, args,robot):
  targ_x = args[2]
  targ_y = args[3]
  targ_z = args[4]
  direction_x = args[5]
  direction_y = args[6]
  direction_z = args[7]
  search_distance = args[8]

  # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
  targ_elevation = -0.2
  if (targ_z+targ_elevation)==0:
    offset = search_distance
  else:
    offset = (targ_z*search_distance)/(targ_z+targ_elevation)

  # Compute shoulder yaw angle to target
  alpha = math.atan2( (targ_y+direction_y*offset)-constants.Y_SHOU, (targ_x+direction_x*offset)-constants.X_SHOU)
  h = math.sqrt(pow( (targ_y+direction_y*offset)-constants.Y_SHOU,2) + pow( (targ_x+direction_x*offset)-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)

  # Move to pre move position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = 0
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal  [constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
     return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  #move_arm.go(joint_goal, wait=True)
  #move_arm.stop()
  
  plan = move_arm.plan(joint_goal)

  start_state = plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions

  cs = robot.get_current_state()
  new_value = (0,0) + start_state # adding antenna state to the robot states.
  cs.joint_state.position = new_value # modify current state of robot to the end state of tje plan


  # Once aligned to move goal and offset, place scoop tip at surface target offset
  #move_arm.set_start_state(start_state)
  #move_arm.set_start_state(robot.get_current_state())
  move_arm.set_start_state(cs)
  goal_pose = move_arm.get_current_pose().pose
  
  #orinetation solution obtained from rviz
  goal_pose.orientation.x = -0.69980734388
  goal_pose.orientation.y = 0.71433163078
  goal_pose.orientation.z = 0.0
  goal_pose.orientation.w = 0.0

  goal_pose.position.x = targ_x
  goal_pose.position.y = targ_y
  goal_pose.position.z = targ_z
  
  move_arm.set_pose_target(goal_pose)
  move_arm.set_max_velocity_scaling_factor(0.5)
  #plan = move_arm.plan()
  plan2 = move_arm.plan(goal_pose)


  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
     return False
  
  guarded_move_traj =  cascade_plans (plan, plan2)
  
  file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan.yaml')
  with open(file_path, 'w') as file_save:
    yaml.dump(guarded_move_traj, file_save, default_flow_style=True)
  
  #move_arm.execute(guarded_move_traj, wait=True)

  
  print "Done planning approach of guarded_move"
  return True

def guarded_move(move_arm, args):

  direction_x = args[5]
  direction_y = args[6]
  direction_z = args[7]
  search_distance = args[8]

  # Drive scoop tip along norm vector, distance is search_distance
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.orientation.x = -0.69980734388
  goal_pose.orientation.y = 0.71433163078
  goal_pose.orientation.z = 0.0
  goal_pose.orientation.w = 0.0
  
  goal_pose.position.x -= direction_x*search_distance
  goal_pose.position.y -= direction_y*search_distance
  goal_pose.position.z -= direction_z*search_distance
  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()
  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan3 = move_arm.go(wait=True)
  
  guarded_move_traj =  cascade_plans (guarded_move_traj, plan3)
  
  file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan.yaml')
  with open(file_path, 'w') as file_save:
    yaml.dump(guarded_move_traj, file_save, default_flow_style=True)
    
    
  move_arm.stop()
  move_arm.clear_pose_targets()
  print "Done planning safe part of guarded_move"
  return True
