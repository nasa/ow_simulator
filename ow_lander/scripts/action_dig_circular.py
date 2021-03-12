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
from std_msgs.msg import Header

circ_traj = RobotTrajectory()

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

def calculate_starting_state_arm (plan,robot):
  #joint_names: [j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw, j_grinder]
  #robot full state name: [j_ant_pan, j_ant_tilt, j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw,
  #j_grinder, j_scoop_yaw]

  start_state = plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions 
  cs = robot.get_current_state()
  ## adding antenna state (0, 0) and j_scoop_yaw  to the robot states.
  ## j_scoop_yaw  state obstained from rviz
  new_value =  (0,0) + start_state[:5] + (-0.1407739555464298,) + (start_state [5],)
  # modify current state of robot to the end state of the previous plan
  cs.joint_state.position = new_value 
  return cs, start_state


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

def dig_circular(move_arm, move_limbs, robot, moveit_fk, args):
    
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
    #cs, start_state = calculate_starting_state_arm (plan_a, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, plan_a, move_arm, moveit_fk)
    #if not parallel:
    # Once aligned to trench goal, place hand above trench middle point
    z_start = ground_position + constants.R_PARALLEL_FALSE - depth
    #plan_b = go_to_Z_coordinate(move_arm, cs, x_start, y_start, z_start)
    plan_b = go_to_Z_coordinate(move_arm, cs, end_pose, x_start, y_start, z_start)
    
    circ_traj = cascade_plans (plan_a, plan_b)

    # Rotate hand perpendicular to arm direction
    #cs, start_state = calculate_starting_state_arm (plan_a, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    plan_c = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, -0.29*math.pi)
    circ_traj = cascade_plans (circ_traj, plan_c)
    
  else:
    # Rotate hand so scoop is in middle point
    #cs, start_state = calculate_starting_state_arm (plan_a, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, plan_a, move_arm, moveit_fk)
    plan_b = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, 0.0)
    circ_traj = cascade_plans (plan_a, plan_b)
    
    # Rotate scoop
    #cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    plan_c = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_SCOOP_YAW, math.pi/2)
    circ_traj = cascade_plans (circ_traj, plan_c)
    
    # Rotate dist so scoop is back
    #cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    plan_d = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, -19.0/54.0*math.pi)
    circ_traj = cascade_plans (circ_traj, plan_d)
    
    # Once aligned to trench goal, place hand above trench middle point
    #cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    z_start = ground_position + constants.R_PARALLEL_FALSE - depth
    plan_e = go_to_Z_coordinate(move_arm, cs, end_pose, x_start, y_start, z_start)
    circ_traj = cascade_plans (circ_traj, plan_e)
    
    # Rotate dist to dig
    #cs, start_state = calculate_starting_state_arm (circ_traj, robot)
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    dist_now = start_state[3]
    plan_f = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, dist_now + 2*math.pi/3)
    circ_traj = cascade_plans (circ_traj, plan_f)

  return circ_traj