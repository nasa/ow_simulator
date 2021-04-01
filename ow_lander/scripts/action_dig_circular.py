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
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
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

  
def go_to_XYZ_coordinate(move_arm, cs, goal_pose, x_start, y_start, z_start, approximate=True):
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
  
  goal_pose.orientation.x = goal_pose.orientation.x
  goal_pose.orientation.y = goal_pose.orientation.y
  goal_pose.orientation.z = goal_pose.orientation.z
  goal_pose.orientation.w = goal_pose.orientation.w

  # Ask the planner to generate a plan to the approximate joint values generated
  # by kinematics builtin IK solver. For more insight on this issue refer to:
  # https://github.com/nasa/ow_simulator/pull/60
  if approximate:
    move_arm.set_joint_value_target(goal_pose, True)
  else:
    move_arm.set_pose_target(goal_pose)

  plan = move_arm.plan()
  print (plan)
  
  if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
    return False

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
  #goal_pose.position.x = x_start
  #goal_pose.position.y = y_start
  goal_pose.position.z = z_start
  
  goal_pose.orientation.x = goal_pose.orientation.x
  goal_pose.orientation.y = goal_pose.orientation.y
  goal_pose.orientation.z = goal_pose.orientation.z
  goal_pose.orientation.w = goal_pose.orientation.w

  # Ask the planner to generate a plan to the approximate joint values generated
  # by kinematics builtin IK solver. For more insight on this issue refer to:
  # https://github.com/nasa/ow_simulator/pull/60
  if approximate:
    move_arm.set_joint_value_target(goal_pose, True)
  else:
    move_arm.set_pose_target(goal_pose)
    
  #move_arm.set_path_constraints(upright_constraints(move_arm))  
  upright_constraints = Constraints()
  joint_constraint = JointConstraint()
  upright_constraints.name = "upright"
  joint_constraint.joint_name = move_arm.get_joints()[constants.J_DIST_PITCH]
  joint_constraint.position = 0.25*math.pi
  joint_constraint.tolerance_above = .1
  joint_constraint.tolerance_below = .1
  joint_constraint.weight = 1
  #joint_constraint.joint_name = "j_dist_pitch"
  upright_constraints.joint_constraints.append(joint_constraint)
  #move_arm.set_path_constraints(upright_constraints)
  plan = move_arm.plan()
  print (plan)
  
  if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
    return False

  return plan

def move_to_pre_trench_configuration_dig_circ(move_arm, x_start, y_start):
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
  joint_goal[constants.J_HAND_YAW] = 0.0
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
    return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  plan = move_arm.plan(joint_goal)
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
  
  #plan_a= action_dig_linear.move_to_pre_trench_configuration(
      #move_arm, x_start, y_start)
  
  if not parallel:
     
    plan_a= move_to_pre_trench_configuration_dig_circ(
      move_arm, x_start, y_start) 
    
    # Once aligned to move goal and offset, place scoop tip at surface target offset
    
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, plan_a, move_arm, moveit_fk)
    z_start = ground_position + constants.R_PARALLEL_FALSE_A #- depth
    end_pose.position.x = x_start
    end_pose.position.y = y_start
    end_pose.position.z = z_start
    
    move_arm.set_start_state(cs)
    move_arm.set_pose_target(end_pose)
    plan_b = move_arm.plan()
    if len(plan_b.joint_trajectory.points) == 0:  # If no plan found, abort
      return False
    circ_traj = cascade_plans (plan_a, plan_b)
    
    #### Rotate J_HAND_YAW to correct postion 
   
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    
    plan_c = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW,  math.pi/2.2)
    
    circ_traj = cascade_plans (circ_traj, plan_c)
    
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    ##if not parallel:
    ## Once aligned to trench goal, place hand above trench middle point
    z_start = ground_position + constants.R_PARALLEL_FALSE_A #- depth
    
    plan_d = go_to_Z_coordinate(move_arm, cs, end_pose, x_start, y_start, z_start)
    circ_traj = cascade_plans (circ_traj, plan_d)
    
    ## Rotate hand perpendicular to arm direction
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    plan_e = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, -0.29*math.pi)
    circ_traj = cascade_plans (circ_traj, plan_e)
    
  else:
      
    plan_a= action_dig_linear.move_to_pre_trench_configuration(
      move_arm, x_start, y_start)  
    # Rotate hand so scoop is in middle point
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, plan_a, move_arm, moveit_fk)
    plan_b = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_HAND_YAW, 0.0)
    circ_traj = cascade_plans (plan_a, plan_b)
    
    # Rotate scoop
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    plan_c = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_SCOOP_YAW, math.pi/2)
    circ_traj = cascade_plans (circ_traj, plan_c)
    
    # Rotate dist so scoop is back
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    plan_d = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, -19.0/54.0*math.pi)
    circ_traj = cascade_plans (circ_traj, plan_d)
    
    # Once aligned to trench goal, place hand above trench middle point
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    z_start = ground_position + constants.R_PARALLEL_FALSE_A - depth
    plan_e = go_to_XYZ_coordinate(move_arm, cs, end_pose, x_start, y_start, z_start)
    circ_traj = cascade_plans (circ_traj, plan_e)
    
    # Rotate dist to dig
    cs, start_state, end_pose = calculate_joint_state_end_pose_from_plan_arm (robot, circ_traj, move_arm, moveit_fk)
    dist_now = start_state[3]
    plan_f = action_dig_linear.change_joint_value(move_arm, cs, start_state, constants.J_DIST_PITCH, dist_now + 2*math.pi/3)
    circ_traj = cascade_plans (circ_traj, plan_f)

  return circ_traj
