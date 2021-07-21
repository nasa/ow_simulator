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
import numpy as np
from scipy import interpolate

pre_guarded_move_traj = RobotTrajectory()
guarded_move_traj = RobotTrajectory()

def cascade_plans_smooth (plan1, plan2):
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
  # add a time toleracne between  successive plans
  time_tolerance = rospy.Duration.from_sec(0.1)
    

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
    point.time_from_start = plan2.joint_trajectory.points[i].time_from_start + end_time +time_tolerance
    point.velocities = list(plan2.joint_trajectory.points[i].velocities)
    point.accelerations = list(plan2.joint_trajectory.points[i].accelerations)
    point.positions = plan2.joint_trajectory.points[i].positions
    traj_msg.points.append(point)
    
  traj_msg.joint_names = plan1.joint_trajectory.joint_names
  traj_msg.header.frame_id = plan1.joint_trajectory.header.frame_id
  new_traj.joint_trajectory = traj_msg
  
  
  new_traj =  smooth_plans  (new_traj, n_points1)
  return new_traj  


def interp_cubic(p0, p1, t_abs):
    T = (p1.time_from_start - p0.time_from_start).to_sec()
    t = t_abs - p0.time_from_start.to_sec()
    q = [0] * 6
    qdot = [0] * 6
    qddot = [0] * 6
    for i in range(len(p0.positions)):
        a = p0.positions[i]
        b = p0.velocities[i]
        c = (-3*p0.positions[i] + 3*p1.positions[i] - 2*T*p0.velocities[i] - T*p1.velocities[i]) / T**2
        d = (2*p0.positions[i] - 2*p1.positions[i] + T*p0.velocities[i] + T*p1.velocities[i]) / T**3

        q[i] = a + b*t + c*t**2 + d*t**3
        qdot[i] = b + 2*c*t + 3*d*t**2
        qddot[i] = 2*c + 6*d*t
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))

def interpolate_traj(vel):
  y = np.array(vel)
  x = np.linspace (0, y.size,num = y.size, endpoint=True )
  tck = interpolate.splrep(x, y, s=0.1)
  ynew = interpolate.splev(x, tck, der=0)
  ynew_der = interpolate.splev(x, tck, der=1) # Added to compute first derivative

  return ynew,ynew_der

def smooth_plans (plan1,n_points):
  # Create a new trajectory object
  new_traj = RobotTrajectory()
  # Initialize the new trajectory to be the same as the planned trajectory
  traj_msg = JointTrajectory()
  # Get the number of joints involved
  n_joints1 = len(plan1.joint_trajectory.joint_names)
  # Get the number of points on the trajectory
  n_points1 = len(plan1.joint_trajectory.points)
  # Store the trajectory points
  points1 = list(plan1.joint_trajectory.points)
  vel = []
  j_shou_yaw_vel = []
  j_shou_pitch_vel =[] 
  j_prox_pitch_vel = [] 
  j_dist_pitch_vel = [] 
  j_hand_yaw_vel = []
  j_scoop_yaw_vel = []
  for i in range(n_points1):
    j_shou_yaw_vel.append ( plan1.joint_trajectory.points[i].velocities[0])
    j_shou_pitch_vel.append ( plan1.joint_trajectory.points[i].velocities[1])
    j_prox_pitch_vel.append ( plan1.joint_trajectory.points[i].velocities[2])
    j_dist_pitch_vel.append ( plan1.joint_trajectory.points[i].velocities[3])
    j_hand_yaw_vel.append ( plan1.joint_trajectory.points[i].velocities[4])
    j_scoop_yaw_vel.append ( plan1.joint_trajectory.points[i].velocities[5])
    
  j_shou_yaw_vel_s, j_shou_yaw_acc_s  = interpolate_traj(j_shou_yaw_vel)
  j_shou_pitch_vel_s, j_shou_pitch_acc_s = interpolate_traj(j_shou_pitch_vel)
  j_prox_pitch_vel_s, j_prox_pitch_acc_s = interpolate_traj(j_prox_pitch_vel)
  j_dist_pitch_vel_s, j_dist_pitch_acc_s = interpolate_traj(j_dist_pitch_vel)
  j_hand_yaw_vel_s , j_hand_yaw_acc_s = interpolate_traj(j_hand_yaw_vel)
  j_scoop_yaw_vel_s, j_scoop_yaw_acc_s = interpolate_traj(j_scoop_yaw_vel)
    
  smooth_radius =  5
  print (n_points)
  for i in range(n_points1):
    point = JointTrajectoryPoint()
    point.time_from_start = plan1.joint_trajectory.points[i].time_from_start
    #velocity_smooth = interpolate_traj(plan1.joint_trajectory.points[i].velocities)
    point.velocities = list(plan1.joint_trajectory.points[i].velocities)
    if (i > n_points-smooth_radius ) and (i < n_points + smooth_radius):
      point.velocities = ( j_shou_yaw_vel_s[i], j_shou_pitch_vel_s[i], j_prox_pitch_vel_s[i], j_dist_pitch_vel_s[i] ,j_hand_yaw_vel_s[i], j_scoop_yaw_vel_s[i])
      point.accelerations = ( j_shou_yaw_acc_s[i], j_shou_pitch_acc_s[i], j_prox_pitch_acc_s[i], j_dist_pitch_acc_s[i] ,j_hand_yaw_acc_s[i], j_scoop_yaw_acc_s[i])
    else:
      point.velocities = ( j_shou_yaw_vel[i], j_shou_pitch_vel[i], j_prox_pitch_vel[i], j_dist_pitch_vel[i] ,j_hand_yaw_vel[i], j_scoop_yaw_vel[i])
      point.accelerations = list(plan1.joint_trajectory.points[i].accelerations)
    #point.velocities = ( j_shou_yaw_vel_s[i], j_shou_pitch_vel_s[i], j_prox_pitch_vel_s[i], j_dist_pitch_vel_s[i] ,j_hand_yaw_vel_s[i], j_scoop_yaw_vel_s[i])

    #point.accelerations = list(plan1.joint_trajectory.points[i].accelerations)
    point.positions = plan1.joint_trajectory.points[i].positions
    points1[i] = point
    traj_msg.points.append(point)

    
  traj_msg.joint_names = plan1.joint_trajectory.joint_names
  traj_msg.header.frame_id = plan1.joint_trajectory.header.frame_id
  new_traj.joint_trajectory = traj_msg
  return new_traj   

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
  #pre_guarded_move_traj = cascade_plans_smooth (plan_a, plan_b)
  
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
  
  #guarded_move_traj = cascade_plans (pre_guarded_move_traj, plan_c)
  guarded_move_traj = cascade_plans_smooth (pre_guarded_move_traj, plan_c)
  
  #guarded_move_traj =  smooth_plans (guarded_move_traj)
  return guarded_move_traj
