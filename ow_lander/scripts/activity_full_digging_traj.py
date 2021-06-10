#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import constants
import math
import copy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from utils import is_shou_yaw_goal_in_range


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
  
  _, plan, _, _ = move_group.plan()
  if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
    return False
  plan = move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()


def change_joint_value(move_group, joint_index, target_value):
  """
  :type move_group: class 'moveit_commander.move_group.MoveGroupCommander' 
  :type joint_index: int
  :type target_value: float
  """
  joint_goal = move_group.get_current_joint_values()
  joint_goal[joint_index] = target_value
  move_group.set_joint_value_target(joint_goal)
  _, plan, _, _ = move_group.plan()
  move_group.execute(plan, wait=True)
  #move_group.go(joint_goal, wait=True)
  move_group.stop()


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
  move_arm.set_joint_value_target(joint_goal)
  _, plan, _, _ = move_arm.plan()
  #move_arm.asyncExecute(plan, wait=True)
  move_arm.execute(plan, wait=True)
  #move_arm.go(joint_goal, wait=True)
  move_arm.stop()

  return True


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


def arg_parsing_lin(req):
  """
  :type req: class 'ow_lander.srv._DigLinear.DigLinearRequest'
  """
  if req.use_defaults:
    # Default trenching values
    x_start = 1.46
    y_start = 0
    depth = 0.01
    length = 0.1
    ground_position = constants.DEFAULT_GROUND_HEIGHT
  else:
    x_start = req.x
    y_start = req.y
    depth = req.depth
    length = req.length
    ground_position = req.ground_position

  return [req.use_defaults, x_start, y_start, depth, length, ground_position]


def dig_linear(move_arm, args):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, int, float, float, float]
  """
  x_start = args[1]
  y_start = args[2]
  depth = args[3]
  length = args[4]
  ground_position = args[5]

  pre_move_complete = move_to_pre_trench_configuration(
      move_arm, x_start, y_start)
  if pre_move_complete == False:
    return False

  # Rotate hand yaw to dig in
  change_joint_value(move_arm, constants.J_HAND_YAW, 0.0)

  # rotate scoop
  change_joint_value(move_arm, constants.J_SCOOP_YAW, math.pi/2)

  # rotate dist pith to pre-trenching position.
  change_joint_value(move_arm, constants.J_DIST_PITCH, -math.pi/2)

  # Once aligned to trench goal, place hand above the desired start point
  alpha = math.atan2(constants.WRIST_SCOOP_PARAL, constants.WRIST_SCOOP_PERP)
  distance_from_ground = constants.ROT_RADIUS * \
      (math.cos(alpha) - math.sin(alpha))
  z_start = ground_position + constants.SCOOP_HEIGHT - depth + distance_from_ground
  go_to_Z_coordinate(move_arm, x_start, y_start, z_start)

  #  rotate to dig in the ground
  change_joint_value(move_arm, constants.J_DIST_PITCH, 2.0/9.0*math.pi)

  # determine linear trenching direction (alpha)
  current_pose = move_arm.get_current_pose().pose
  quaternion = [current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w]
  current_euler = euler_from_quaternion(quaternion)
  alpha = current_euler[2]

  # linear trenching
  cartesian_plan, fraction = plan_cartesian_path_lin(move_arm, length, alpha)
  move_arm.execute(cartesian_plan, wait=True)
  move_arm.stop()

  #  rotate to dig out
  change_joint_value(move_arm, constants.J_DIST_PITCH, math.pi/2)

  return True


def arg_parsing_circ(req):
  """
  :type req: class 'ow_lander.srv._DigCircular.DigCircularRequest' 
  """
  if req.use_defaults:
    # Default trenching values
    x_start = 1.65
    y_start = 0
    depth = 0.01
    parallel = True
    ground_position = constants.DEFAULT_GROUND_HEIGHT
  else:
    x_start = req.x
    y_start = req.y
    depth = req.depth
    parallel = req.parallel
    ground_position = req.ground_position

  return [req.use_defaults, x_start, y_start, depth, parallel, ground_position]


def dig_circular(move_arm, move_limbs, args, controller_switcher):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type move_limbs: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, int, float, bool, float]
  """
  x_start = args[1]
  y_start = args[2]
  depth = args[3]
  parallel = args[4]
  ground_position = args[5]

  pre_move_complete = move_to_pre_trench_configuration(
      move_arm, x_start, y_start)
  if pre_move_complete == False:
    return False

  if not parallel:

    # Once aligned to trench goal, place hand above trench middle point
    z_start = ground_position + constants.R_PARALLEL_FALSE - depth
    controller_switcher('limbs_controller', 'arm_controller')
    go_to_Z_coordinate(move_limbs, x_start, y_start, z_start)
    controller_switcher('arm_controller', 'limbs_controller')

    # Rotate hand perpendicular to arm direction
    change_joint_value(move_arm, constants.J_HAND_YAW, -0.29*math.pi)

  else:
    # Rotate hand so scoop is in middle point
    change_joint_value(move_arm, constants.J_HAND_YAW, 0.0)

    # Rotate scoop
    change_joint_value(move_arm, constants.J_SCOOP_YAW, math.pi/2)

    # Rotate dist so scoop is back
    change_joint_value(move_arm, constants.J_DIST_PITCH, -19.0/54.0*math.pi)

    # Once aligned to trench goal, place hand above trench middle point
    z_start = ground_position + constants.R_PARALLEL_FALSE - depth
    controller_switcher('limbs_controller', 'arm_controller')
    go_to_Z_coordinate(move_limbs, x_start, y_start, z_start)
    controller_switcher('arm_controller', 'limbs_controller')

    # Rotate dist to dig
    joint_goal = move_arm.get_current_joint_values()
    dist_now = joint_goal[3]
    change_joint_value(move_arm, constants.J_DIST_PITCH,
                       dist_now + 2*math.pi/3)

  return True
