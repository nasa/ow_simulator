#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import constants
import math
import copy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from utils import is_shou_yaw_goal_in_range
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_deliver_sample import cascade_plans
import action_dig_linear
import action_grind
from action_deliver_sample import cascade_plans

circ_traj = RobotTrajectory()

def calculate_starting_state_arm (plan,robot):
  #joint_names: [j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw, j_grinder]
  #robot full state name: [j_ant_pan, j_ant_tilt, j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw,
  #j_grinder, j_scoop_yaw]

  start_state = plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions 
  cs = robot.get_current_state()
  ## adding antenna state (0, 0) and j_scoop_yaw  to the robot states.
  ## j_scoop_yaw  state obstained from rviz
  new_value =  new_value =  (0,0) + start_state[:5] + (-0.1407739555464298,) + (start_state [5],)
  # modify current state of robot to the end state of the previous plan
  cs.joint_state.position = new_value 
  return cs, start_state

def go_to_Z_coordinate(move_arm, cs, x_start, y_start, z_start, approximate=False):
  """
  :param approximate: use an approximate solution. default True
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
  :type x_start: float
  :type y_start: float
  :type z_start: float
  :type approximate: bool
  """
  
  move_arm.set_start_state(cs)
  
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.position.x = x_start
  goal_pose.position.y = y_start
  goal_pose.position.z = z_start
  
  goal_pose.orientation.x = -0.70685
  goal_pose.orientation.y = 0.011349
  goal_pose.orientation.z = 0.70711
  goal_pose.orientation.w = 0.014891
  # Ask the planner to generate a plan to the approximate joint values generated
  # by kinematics builtin IK solver. For more insight on this issue refer to:
  # https://github.com/nasa/ow_simulator/pull/60
  if approximate:
    move_arm.set_joint_value_target(goal_pose, True)
  else:
    move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()
  if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
    return False
  #plan = move_group.go(wait=True)
  #plan = move_arm.plan(joint_goal)
  return plan

def dig_circular(move_arm, move_limbs, robot, args):
    
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, int, float, float, float]
  """
  x_start = args.x_start
  y_start = args.y_start
  depth = args.depth
  parallel = args.parallel
  ground_position = args.ground_position
  
  plan_a= action_dig_linear.move_to_pre_trench_configuration(
      move_arm, x_start, y_start)
  
  if not parallel:
  
    #start_state = return_start_state(plan_a) 
    cs, start_state = calculate_starting_state_arm (plan_a, robot)
    #if not parallel:
    # Once aligned to trench goal, place hand above trench middle point
    z_start = ground_position + constants.R_PARALLEL_FALSE - depth
    plan_b = go_to_Z_coordinate(move_arm, cs, x_start, y_start, z_start)
    circ_traj = cascade_plans (plan_a, plan_b)

    # Rotate hand perpendicular to arm direction
    cs, start_state = calculate_starting_state_arm (plan_a, robot)
    plan_c = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, -0.29*math.pi)
    circ_traj = cascade_plans (circ_traj, plan_c)
  else:
    # Rotate hand so scoop is in middle point
    cs, start_state = calculate_starting_state_arm (plan_a, robot)
    plan_b = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, 0.0)
    circ_traj = cascade_plans (plan_a, plan_b)
    
    # Rotate scoop
    cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    plan_c = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_SCOOP_YAW, math.pi/2)
    circ_traj = cascade_plans (circ_traj, plan_c)
    
    # Rotate dist so scoop is back
    cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    plan_d = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, -19.0/54.0*math.pi)
    circ_traj = cascade_plans (circ_traj, plan_d)
    
    # Once aligned to trench goal, place hand above trench middle point
    cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    z_start = ground_position + constants.R_PARALLEL_FALSE - depth
    plan_e = go_to_Z_coordinate(move_arm, cs, x_start, y_start, z_start)
    circ_traj = cascade_plans (circ_traj, plan_e)
    
    # Rotate dist to dig
    cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    dist_now = start_state[3]
    plan_f = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, dist_now + 2*math.pi/3)
    circ_traj = cascade_plans (circ_traj, plan_f)

  return circ_traj
