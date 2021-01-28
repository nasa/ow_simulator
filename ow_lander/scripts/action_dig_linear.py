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

dig_linear_traj = RobotTrajectory()

def move_to_pre_trench_configuration(move_arm, x_start, y_start):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type x_start: float
  :type y_start: float
  """
 # Compute shoulder yaw angle to trench
  alpha = math.atan2(y_start-constants.Y_SHOU, x_start-constants.X_SHOU)
  h = math.sqrt(pow(y_start-constants.Y_SHOU, 2) +
                pow(x_start-constants.X_SHOU, 2))
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin(l/h)
  # Move to pre trench position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0.0
  joint_goal[constants.J_HAND_YAW] = math.pi/2.2
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
    return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  plan = move_arm.plan(joint_goal)
  ##move_arm.asyncExecute(plan, wait=True)
  #move_arm.execute(plan, wait=True)
  ##move_arm.go(joint_goal, wait=True)
  #move_arm.stop()

  #return True
  return plan




def plan_cartesian_path_lin(move_arm, length, alpha):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type length: float
  :type alpha: float
  """
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

def change_joint_value(move_arm, cs, start_state, joint_index, target_value):
  """
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander' 
  :type joint_index: int
  :type target_value: float
  """
  move_arm.set_start_state(cs)
  
  joint_goal = move_arm.get_current_joint_values()
  for k in range (0,len(start_state)):
      joint_goal[k] = start_state[k]
      
      
  joint_goal[joint_index] = target_value
  plan = move_arm.plan(joint_goal)
  #move_group.execute(plan, wait=True)
  ##move_group.go(joint_goal, wait=True)
  #move_group.stop()
  return plan

def go_to_Z_coordinate(move_group, x_start, y_start, z_start, approximate=True):
  """
  :param approximate: use an approximate solution. default True
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
  :type x_start: float
  :type y_start: float
  :type z_start: float
  :type approximate: bool
  """
  goal_pose = move_group.get_current_pose().pose
  goal_pose.position.x = x_start
  goal_pose.position.y = y_start
  goal_pose.position.z = z_start
  # Ask the planner to generate a plan to the approximate joint values generated
  # by kinematics builtin IK solver. For more insight on this issue refer to:
  # https://github.com/nasa/ow_simulator/pull/60
  if approximate:
    move_group.set_joint_value_target(goal_pose, True)
  else:
    move_group.set_pose_target(goal_pose)
  plan = move_group.plan()
  if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
    return False
  #plan = move_group.go(wait=True)
  plan = move_group.plan(joint_goal)
  return plan
  #move_group.execute(plan, wait=True)
  #move_group.stop()
  #move_group.clear_pose_targets()


def dig_linear(move_arm, robot, args):
    
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, int, float, float, float]
  """
  x_start = args.x_start
  y_start = args.y_start
  depth = args.depth
  length = args.length
  ground_position = args.ground_position
  
  plan_a= move_to_pre_trench_configuration(
      move_arm, x_start, y_start)
  #if pre_move_complete == False:
    #return False
    
  start_state = plan_a.joint_trajectory.points[len(plan_a.joint_trajectory.points)-1].positions 
  cs = robot.get_current_state()
  # adding antenna state and grinder state  to the robot states.
  # grider state obstained from rviz
  
  new_value =  (0,0) + start_state[:5] + (-0.1407739555464298,) + (start_state [5],)
  
  
  cs.joint_state.position = new_value # modify current state of robot to the end state of the previous plan
  
  #################### Rotate hand yaw to dig in#################################
  plan_b = change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, 0.0) 
  
  
  if len(plan_b.joint_trajectory.points) == 0:  # If no plan found, send the previous plan only
    return plan_a

  dig_linear_traj = cascade_plans (plan_a, plan_b)
  
  ######################### rotate scoop #######################################
  start_state = plan_b.joint_trajectory.points[len(plan_b.joint_trajectory.points)-1].positions 
  cs = robot.get_current_state()
  # adding antenna state and grinder state  to the robot states.
  # grider state obstained from rviz
  
  new_value =  (0,0) + start_state[:5] + (-0.1407739555464298,) + (start_state [5],)
  
  cs.joint_state.position = new_value # modify current state of robot to the end state of the previous plan
  
  plan_c = change_joint_value(move_arm, cs, start_state, constants.J_SCOOP_YAW, math.pi/2)
  
  dig_linear_traj = cascade_plans (dig_linear_traj, plan_c)
  
  ######################### rotate dist pith to pre-trenching position###########
  start_state = plan_c.joint_trajectory.points[len(plan_c.joint_trajectory.points)-1].positions 
  cs = robot.get_current_state()
  # adding antenna state and grinder state  to the robot states.
  # grider state obstained from rviz
  
  new_value =  (0,0) + start_state[:5] + (-0.1407739555464298,) + (start_state [5],)
  
  cs.joint_state.position = new_value # modify current state of robot to the end
  
  plan_d = change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, -math.pi/2)

  dig_linear_traj = cascade_plans (dig_linear_traj, plan_d)
  
  ##########################Once aligned to trench goal, 
  ##########################place hand above the desired start point
  alpha = math.atan2(constants.WRIST_SCOOP_PARAL, constants.WRIST_SCOOP_PERP)
  distance_from_ground = constants.ROT_RADIUS * \
      (math.cos(alpha) - math.sin(alpha))
  z_start = ground_position + constants.SCOOP_HEIGHT - depth + distance_from_ground
  #go_to_Z_coordinate(move_arm, x_start, y_start, z_start)
  
  return dig_linear_traj
  
  
  
