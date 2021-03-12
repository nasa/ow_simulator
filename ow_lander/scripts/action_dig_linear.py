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
from std_msgs.msg import Header
#from action_dig_circular import calculate_starting_state_arm

dig_linear_traj = RobotTrajectory()

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


def calculate_joint_state_end_pose_from_plan_arm (robot, plan, move_arm, moveit_fk):
  ''' 
  calculate the end pose (position and orientation), joint states and robot states
  from the current plan
  inputs:  current plan, robot, arm interface, and moveit forward kinematics object
  outputs: goal_pose, robot state and joint states at end of the plan
  '''  
  #joint_names: [j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw, j_scoop_yaw]
  #robot full state name: [j_ant_pan, j_ant_tilt, j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw,
  #j_grinder, j_scoop_yaw]

  # get joint states from the end of the plan 
  joint_states = plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions 
  # construct robot state at the end of the plan
  robot_state = robot.get_current_state()
  # adding antenna (0,0) and grinder positions (-0.1) which should not change
  new_value =  new_value =  (0,0) + joint_states[:5] + (-0.1,) + (joint_states [5],)
  # modify current state of robot to the end state of the previous plan
  robot_state.joint_state.position = new_value 
  # calculate goal pose at the end of the plan using forward kinematics
  goal_pose = move_arm.get_current_pose().pose
  header = Header(0,rospy.Time.now(),"base_link")
  fkln = ['l_scoop']
  goal_pose_stamped = moveit_fk(header, fkln, robot_state )
  goal_pose = goal_pose_stamped.pose_stamped[0].pose
  
  return robot_state, joint_states, goal_pose 

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
  return plan




def plan_cartesian_path_lin(move_arm, wpose, length, alpha, z_start, cs):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type length: float
  :type alpha: float
  """
  move_arm.set_start_state(cs)
  waypoints = []
  #wpose = move_arm.get_current_pose().pose
  ## these values were obtained from rviz 
  #wpose.position.x = 1.94233
  #wpose.position.y = -0.0167343
  #wpose.position.z = -0.122421 # -0.0456
  #wpose.orientation.x = 0.9988
  #wpose.orientation.y = -0.0215346
  #wpose.orientation.z = -0.043971
  #wpose.orientation.w = -0.00134208
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
  return plan

def go_to_Z_coordinate(move_arm, cs, goal_pose, x_start, y_start, z_start, approximate=True):
  """
  :param approximate: use an approximate solution. default True
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
  :type x_start: float
  :type y_start: float
  :type z_start: float
  :type approximate: bool
  """
  
  move_arm.set_start_state(cs)

  goal_pose.position.x = x_start
  goal_pose.position.y = y_start
  goal_pose.position.z = z_start

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
  return plan



def dig_linear(move_arm, robot, moveit_fk, args):
    
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
    
  
  #cs, start_state = calculate_starting_state_arm (plan_a, robot)
  cs, start_state, current_pose = calculate_joint_state_end_pose_from_plan_arm (robot, plan_a, move_arm, moveit_fk)
  #################### Rotate hand yaw to dig in#################################
  plan_b = change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, 0.0) 
  
  
  if len(plan_b.joint_trajectory.points) == 0:  # If no plan found, send the previous plan only
    return plan_a

  dig_linear_traj = cascade_plans (plan_a, plan_b)
  
  ######################### rotate scoop #######################################
  #cs, start_state = calculate_starting_state_arm (dig_linear_traj, robot)
  cs, start_state, current_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  plan_c = change_joint_value(move_arm, cs, start_state, constants.J_SCOOP_YAW, math.pi/2)
  
  dig_linear_traj = cascade_plans (dig_linear_traj, plan_c)
  
  ######################### rotate dist pith to pre-trenching position###########
  #cs, start_state = calculate_starting_state_arm (dig_linear_traj, robot)
  cs, start_state, current_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  
  plan_d = change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, -math.pi/2)

  dig_linear_traj = cascade_plans (dig_linear_traj, plan_d)
  
  ##########################Once aligned to trench goal, 
  ##########################place hand above the desired start point
  alpha = math.atan2(constants.WRIST_SCOOP_PARAL, constants.WRIST_SCOOP_PERP)
  distance_from_ground = constants.ROT_RADIUS * \
      (math.cos(alpha) - math.sin(alpha))
  z_start = ground_position + constants.SCOOP_HEIGHT - depth + distance_from_ground
  
  #cs, start_state = calculate_starting_state_arm (dig_linear_traj, robot)
  cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  
  plan_e = go_to_Z_coordinate(move_arm, cs, goal_pose, x_start, y_start, z_start)
  
  dig_linear_traj = cascade_plans (dig_linear_traj, plan_e)
  
  ############################  rotate to dig in the ground
  
  #cs, start_state = calculate_starting_state_arm (dig_linear_traj, robot)
  cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  plan_f = change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, 2.0/9.0*math.pi)
  
  dig_linear_traj = cascade_plans (dig_linear_traj, plan_f)
  
  # determine linear trenching direction (alpha) value obtained from rviz
  #current_pose = move_arm.get_current_pose().pose
  #current_pose.orientation.x = 0.9988
  #current_pose.orientation.y = -0.0215346
  #current_pose.orientation.z = -0.043971
  #current_pose.orientation.w = -0.00134208
  
  cs, start_state, current_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  
  quaternion = [current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w]
  current_euler = euler_from_quaternion(quaternion)
  alpha = current_euler[2]
  
  # linear trenching
  #cs, start_state = calculate_starting_state_arm (dig_linear_traj, robot)
  cs, start_state, current_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  cartesian_plan, fraction = plan_cartesian_path_lin(move_arm, current_pose, length, alpha, z_start, cs)
  dig_linear_traj = cascade_plans (dig_linear_traj , cartesian_plan)
  
  
  #  rotate to dig out
  #cs, start_state = calculate_starting_state_arm (dig_linear_traj, robot)
  cs, start_state, current_pose = calculate_joint_state_end_pose_from_plan_arm (robot, dig_linear_traj, move_arm, moveit_fk)
  plan_g = change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, math.pi/2)
  dig_linear_traj = cascade_plans (dig_linear_traj, plan_g)
  
  return dig_linear_traj
  
  
  
