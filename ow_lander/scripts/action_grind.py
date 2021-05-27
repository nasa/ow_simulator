#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

## this file generates a complete grinding trajactory from the robot's 
## current postion to a specified point on the ground. The arguments can be passed ## on through the action goal 


import constants
import math
import copy
import rospy
from utils import is_shou_yaw_goal_in_range
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_deliver_sample import cascade_plans
from action_dig_linear import go_to_Z_coordinate, change_joint_value
from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from std_msgs.msg import Header
from urdf_parser_py.urdf import URDF
import kdl_parser_py.urdf


grind_traj = RobotTrajectory()

def calculate_starting_state_grinder (plan,robot):
  #joint_names: [j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw, j_grinder]
  #robot full state name: [j_ant_pan, j_ant_tilt, j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw,
  #j_grinder, j_scoop_yaw]

  start_state = plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions 
  cs = robot.get_current_state()
  ## adding antenna state (0, 0) and j_scoop_yaw  to the robot states.
  ## j_scoop_yaw  state obstained from rviz
  new_value =  (0,0) + start_state[:6] + (0.17403329917811217,) 
  # modify current state of robot to the end state of the previous plan
  cs.joint_state.position = new_value 
  return cs, start_state

def calculate_joint_state_end_pose_from_plan_grinder (robot, plan, move_arm, moveit_fk):
  ''' 
  calculate the end pose (position and orientation), joint states and robot states
  from the current plan
  inputs:  current plan, robot, grinder interface, and moveit forward kinematics object
  outputs: goal_pose, robot state and joint states at end of the plan
  '''  
  #joint_names: [j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw, j_scoop_yaw]
  #robot full state name: [j_ant_pan, j_ant_tilt, j_shou_yaw, j_shou_pitch, j_prox_pitch, j_dist_pitch, j_hand_yaw,
  #j_grinder, j_scoop_yaw]

  # get joint states from the end of the plan 
  joint_states = plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions 
  # construct robot state at the end of the plan
  robot_state = robot.get_current_state()
  # adding antenna (0,0) and j_scoop_yaw (0.1) which should not change
  new_value =  (0,0) + joint_states[:6] + (0.1740,) 
  # modify current state of robot to the end state of the previous plan
  robot_state.joint_state.position = new_value 
  # calculate goal pose at the end of the plan using forward kinematics
  goal_pose = move_arm.get_current_pose().pose
  header = Header(0,rospy.Time.now(),"base_link")
  fkln = ['l_grinder']
  goal_pose_stamped = moveit_fk(header, fkln, robot_state )
  goal_pose = goal_pose_stamped.pose_stamped[0].pose
  
  return robot_state, joint_states, goal_pose 

    

def plan_cartesian_path(move_group, wpose, length, alpha, parallel, z_start, cs):   
  """
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
  :type length: float
  :type alpha: float
  :type parallel: bool
  """
  if parallel==False:
    alpha = alpha - math.pi/2
  move_group.set_start_state(cs)
  waypoints = []
  wpose.position.z = z_start
  wpose.position.x += length*math.cos(alpha)
  wpose.position.y += length*math.sin(alpha)
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # end effector follow step (meters)
                               0.0)         # jump threshold

  return plan, fraction



def grind(move_grinder, robot, moveit_fk, args):          
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
  robot_state = robot.get_current_state()
  move_grinder.set_start_state(robot_state)
  goal_pose = move_grinder.get_current_pose().pose
  goal_pose.position.x = x_start # Position
  goal_pose.position.y = y_start
  goal_pose.position.z = 0.25 
  goal_pose.orientation.x = 0.70616885803 # Orientation
  goal_pose.orientation.y = 0.0303977418722
  goal_pose.orientation.z = -0.706723318474
  goal_pose.orientation.w = 0.0307192507001
  move_grinder.set_pose_target(goal_pose)
  plan_a = move_grinder.plan()
  if len(plan_a.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  ## entering terrain
  z_start = ground_position + constants.GRINDER_OFFSET - depth
  cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_grinder (robot, plan_a, move_grinder, moveit_fk)
  plan_b = go_to_Z_coordinate(move_grinder, cs, goal_pose, x_start, y_start, z_start, False)
  
  grind_traj = cascade_plans (plan_a, plan_b)
  
  ## grinding ice forward
  cs, start_state, goal_pose = calculate_joint_state_end_pose_from_plan_grinder (robot, grind_traj, move_grinder, moveit_fk)
  cartesian_plan, fraction = plan_cartesian_path(move_grinder, goal_pose, length, alpha, parallel, z_start, cs)
  
  grind_traj = cascade_plans (grind_traj , cartesian_plan)

  ## grinding sideways
  cs, start_state, joint_goal = calculate_joint_state_end_pose_from_plan_grinder (robot, grind_traj, move_grinder, moveit_fk)
  if parallel:
    plan_c = change_joint_value(move_grinder, cs, start_state, constants.J_SHOU_YAW, start_state[0]+0.08)
  else:
    x_now = joint_goal.position.x
    y_now = joint_goal.position.y
    z_now = joint_goal.position.z
    x_goal = x_now + 0.08*math.cos(alpha)
    y_goal = y_now + 0.08*math.sin(alpha)
    plan_c = go_to_Z_coordinate(move_grinder, cs, joint_goal, x_goal, y_goal, z_now, False)
    

  grind_traj = cascade_plans (grind_traj , plan_c)
  ## grinding ice backwards
  cs, start_state, joint_goal = calculate_joint_state_end_pose_from_plan_grinder (robot, grind_traj, move_grinder, moveit_fk)
  cartesian_plan2, fraction2 = plan_cartesian_path(move_grinder, joint_goal, -length, alpha, parallel, z_start, cs)
  grind_traj = cascade_plans (grind_traj , cartesian_plan2)


  ## exiting terrain
  cs, start_state, joint_goal = calculate_joint_state_end_pose_from_plan_grinder (robot, grind_traj, move_grinder, moveit_fk)
  plan_d = go_to_Z_coordinate(move_grinder, cs, joint_goal, x_start, y_start, 0.22, False)
  grind_traj = cascade_plans (grind_traj , plan_d)

  return grind_traj
