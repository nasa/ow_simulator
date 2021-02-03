#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

## this file generates a complete grinding trajactory from the robot's 
## current postion to a specified point on the ground. The arguments can be passed ## on through the action goal 


import constants
import math
import copy
from utils import is_shou_yaw_goal_in_range
from activity_full_digging_traj import go_to_Z_coordinate, change_joint_value
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_deliver_sample import cascade_plans



def plan_cartesian_path(move_group, length, alpha, parallel):   
  """
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
  :type length: float
  :type alpha: float
  :type parallel: bool
  """
  if parallel==False:
    alpha = alpha - math.pi/2

  waypoints = []
  wpose = move_group.get_current_pose().pose
  wpose.position.x += length*math.cos(alpha)
  wpose.position.y += length*math.sin(alpha)
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # end effector follow step (meters)
                               0.0)         # jump threshold

  return plan, fraction

def grind(move_grinder, robot, args):          
  """
  :type move_grinder: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, float, float, float, bool, float, bool]
  """
  x_start = args.x_start
  y_start = args.y_start
  depth = args.depth
  length = args.length
  parallel = args.parallel
  ground_position = args.ground_position

  # Compute shoulder yaw angle to trench
  alpha = math.atan2(y_start-constants.Y_SHOU, x_start-constants.X_SHOU)
  h = math.sqrt( pow(y_start-constants.Y_SHOU,2) + pow(x_start-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)
  alpha = alpha+beta
  
  if parallel:
    R = math.sqrt(x_start*x_start+y_start*y_start)
    # adjust trench to fit scoop circular motion
    dx = 0.04*R*math.sin(alpha) # Center dig_circular in grind trench 
    dy = 0.04*R*math.cos(alpha)
    x_start = 0.9*(x_start + dx) # Move starting point back to avoid scoop-terrain collision
    y_start = 0.9*(y_start - dy)
  else:
    dx = 5*length/8*math.sin(alpha)
    dy = 5*length/8*math.cos(alpha)
    x_start = 0.97*(x_start - dx) # Move starting point back to avoid scoop-terrain collision
    y_start = 0.97*(y_start + dy)   

  # Place the grinder vertical, above the desired starting point, at
  # an altitude of 0.25 meters in the base_link frame. 
  goal_pose = move_grinder.get_current_pose().pose
  goal_pose.position.x = x_start # Position
  goal_pose.position.y = y_start
  goal_pose.position.z = 0.25 
  goal_pose.orientation.x = 0.70616885803 # Orientation
  goal_pose.orientation.y = 0.0303977418722
  goal_pose.orientation.z = -0.706723318474
  goal_pose.orientation.w = 0.0307192507001
  move_grinder.set_pose_target(goal_pose)
  plan = move_grinder.plan()
  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False
  #plan = move_grinder.go(wait=True)
  #move_grinder.stop()
  #move_grinder.clear_pose_targets()

  ## entering terrain
  #z_start = ground_position + constants.GRINDER_OFFSET - depth
  #go_to_Z_coordinate(move_grinder, x_start, y_start, z_start, False)

  ## grinding ice forward
  #cartesian_plan, fraction = plan_cartesian_path(move_grinder, length, alpha, parallel)
  #move_grinder.execute(cartesian_plan, wait=True)
  #move_grinder.stop()

  #joint_goal = move_grinder.get_current_joint_values()
  #if parallel:
    #change_joint_value(move_grinder, constants.J_SHOU_YAW, joint_goal[0]+0.08)
  #else:
    #x_now = move_grinder.get_current_pose().pose.position.x
    #y_now = move_grinder.get_current_pose().pose.position.y
    #z_now = move_grinder.get_current_pose().pose.position.z
    #x_goal = x_now + 0.08*math.cos(alpha)
    #y_goal = y_now + 0.08*math.sin(alpha)
    #go_to_Z_coordinate(move_grinder, x_goal, y_goal, z_now, False)

  ## grinding ice backwards
  #cartesian_plan, fraction = plan_cartesian_path(move_grinder, -length, alpha, parallel)
  #move_grinder.execute(cartesian_plan, wait=True)
  #move_grinder.stop()

  ## exiting terrain
  #x_now = move_grinder.get_current_pose().pose.position.x
  #y_now = move_grinder.get_current_pose().pose.position.y
  #go_to_Z_coordinate(move_grinder, x_start, y_start, 0.22, False)

  return plan
