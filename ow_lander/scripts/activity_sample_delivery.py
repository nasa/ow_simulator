#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import constants
import math
import copy
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from shape_msgs.msg import SolidPrimitive

def init_upright_path_constraints(self,pose):

    self.upright_constraints = Constraints()
    self.upright_constraints.name = "upright"
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    #orientation_constraint.link_name = self.arm.get_end_effector_link()
    orientation_constraint.link_name = self.get_end_effector_link()
    print (orientation_constraint.link_name )
    print("Hello World") 
    #orientation_constraint.link_name = lander::l_scoop
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.4
    orientation_constraint.absolute_y_axis_tolerance = 0.4
    orientation_constraint.absolute_z_axis_tolerance = 0.4
    #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
    orientation_constraint.weight = 1

    self.upright_constraints.orientation_constraints.append(orientation_constraint)


def enable_upright_path_constraints(self):
    #self.arm.set_path_constraints(self.upright_constraints)
    self.set_path_constraints(self.upright_constraints)
def disable_upright_path_constraints(self):
    self.arm.set_path_constraints(None)

def arg_parsing(req):
  if req.use_defaults :
    # Default trenching values
    trench_x=1.5
    trench_y=0
    trench_d=0.02
    delete_prev_traj=False

  else :
    trench_x=req.trench_x
    trench_y=req.trench_y
    trench_d=req.trench_d
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,trench_x,trench_y,trench_d,delete_prev_traj]

def plan_cartesian_path(move_arm, move_limbs, scale):

    waypoints = []
    wpose = move_limbs.get_current_pose().pose
    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    #ROS_INFO("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
# Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def sample_delivery(move_arm,move_limbs,x_tr, y_tr, depth):
  # Compute shoulder yaw angle to trench
  alpha = math.atan2(y_tr-constants.Y_SHOU, x_tr-constants.X_SHOU)
  h = math.sqrt( pow(y_tr-constants.Y_SHOU,2) + pow(x_tr-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)
    # Move to pre trench position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = math.pi/2.2
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta
  
  # If out of joint range, abort (TODO: parse limit from urdf)
  if (joint_goal[constants.J_SHOU_YAW]<-1.8) or (joint_goal[constants.J_SHOU_YAW]>1.8): 
    return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()


  ## Rotate hand yaw to dig in
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_HAND_YAW] = 0
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  
  
  
    ###rotate scoop 
  #joint_goal = move_arm.get_current_joint_values()
  #joint_goal[constants.J_SCOOP_YAW] = math.pi/2
  #move_arm.go(joint_goal, wait=True)
  #move_arm.stop()
    ### trying delivery with pose goal
    
      # RPY to convert: 90deg, 0, -90deg
  #q = quaternion_from_euler(1.5707, 0, -1.5707)
  mypi = 3.14159
  r = -180
  p = 90  # 45 worked get
  y = -90
  d2r = mypi/180
  r2d = 180/mypi
  q = quaternion_from_euler(r*d2r, p*d2r, y*d2r)
  #print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
  
  
  goal_pose = move_arm.get_current_pose().pose
  qg= goal_pose.orientation
  #rotation = (qg.x, qg.y, qg.z, qg.w)
  rotation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)
  euler_angle = euler_from_quaternion(rotation)
  #print "Current quaternion is %s." % (qg)
  print "Current euler is ******************************************** %s %s %s." % (euler_angle[0]*r2d, euler_angle[1]*r2d, euler_angle[2]*r2d)
  #print "goal quaternion is %s %s %s %s." % (q[0], q[1], q[2], q[3])
  print "goal euler is ----------------------------------------------  %s %s %s." % (r ,p, y)
  #goal_pose.position.x = x_tr
  #goal_pose.position.y = y_tr
  #goal_pose.position.z = constants.GROUND_POSITION + constants.SCOOP_OFFSET - depth
  #goal_pose.orientation.w = 1;
  goal_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
  
  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()
  '''
  
  # plan with constraint 
  #name: ['irg_sun::ellipsoid', 'irg_jupiter::ellipsoid', 'ow_light_probe::sphere', 'heightmap::link',
  #'gui_camera_sim::link', 'lander::base_link', 'lander::l_ant_foot', 'lander::l_ant_panel',
  #'lander::lander_lights_link', 'lander::l_shou', 'lander::l_prox', 'lander::l_dist',
  #'lander::l_wrist', 'lander::l_hand', 'lander::l_scoop', 'lander::l_scoop_tip']

  
  #moveit_msgs::OrientationConstraint ocm;
  #ocm.link_name = "lander::l_scoop";
  #ocm.header.frame_id = "lander::base_link";
  #ocm.orientation.w = 1.0;
  #ocm.absolute_x_axis_tolerance = 0.1;
  #ocm.absolute_y_axis_tolerance = 0.1;
  #ocm.absolute_z_axis_tolerance = 0.1;
  #ocm.weight = 1.0;
  
  #moveit_msgs::Constraints test_constraints;
  #test_constraints.orientation_constraints.push_back(ocm);
  #move_group.setPathConstraints(test_constraints);
  
 # enable_upright_path_constraints(move_arm)
  

  
  
  #rotate dist pith to pre-trenching position. 
  
  #enable_upright_path_constraints(move_arm)
  
  #upright_constraints = Constraints()
  #upright_constraints.name = "upright"
  #orientation_constraint = OrientationConstraint()
  #orientation_constraint.header = pose.header
  #orientation_constraint.link_name = self.arm.get_end_effector_link()
  #orientation_constraint.orientation = pose.pose.orientation
  #orientation_constraint.absolute_x_axis_tolerance = 0.5
  #orientation_constraint.absolute_y_axis_tolerance = 0.5
  #orientation_constraint.absolute_z_axis_tolerance = 3.14 # allow free rotation around this axis
  #orientation_constraint.weight = 1
  
      #constraint = Constraints()
    #constraint.name = "tilt constraint"
    #tilt_constraint = OrientationConstraint()
    ## 'base_link' is equal to the world link
    #tilt_constraint.header.frame_id = "base_link"
    ## The link that must be oriented upwards
    #tilt_constraint.link_name = "box_gripper_link"
    #tilt_constraint.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
    ## Allow rotation of 45 degrees around the x and y axis
    #tilt_constraint.absolute_x_axis_tolerance = 0.45 #Allow max rotation of 45 degrees
    #tilt_constraint.absolute_y_axis_tolerance = 3.6 #Allow max rotation of 360 degrees
    #tilt_constraint.absolute_z_axis_tolerance = 0.45 #Allow max rotation of 45 degrees
    ## The tilt constraint is the only constraint
    #tilt_constraint.weight = 1
    #constraint.orientation_constraints = [tilt_constraint]
    #self.robot_controller.move_group.set_path_constraints(constraint)
  
  upright_constraints = Constraints()
  upright_constraints.name = "upright"
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = "base_link"
  orientation_constraint.link_name = "l_scoop"
  #orientation_constraint.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
  #orientation_constraint.orientation = Quaternion(0.999424, -0.017672, -0.028959, -0.000525)
  orientation_constraint.orientation.w = 1.0
  orientation_constraint.absolute_x_axis_tolerance = 0.5
  orientation_constraint.absolute_y_axis_tolerance = 0.5
  orientation_constraint.absolute_z_axis_tolerance = 3.14 # allow free rotation around this axis
  orientation_constraint.weight = 1
  upright_constraints.orientation_constraints = [orientation_constraint]
  #move_arm.set_path_constraints(upright_constraints) #this resulted in no paths 
  #upright_constraints.orientation_constraints.append(orientation_constraint)
  
  delivery_constraints = Constraints()
  delivery_constraints.name = "delivery"
  pos_constraint = PositionConstraint()
  pos_constraint.header.frame_id = "base_link"
  pos_constraint.link_name = "l_scoop_tip"
  #orientation_constraint.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
  #orientation_constraint.orientation = Quaternion(0.999424, -0.017672, -0.028959, -0.000525)
  #orientation_constraint.orientation.w = 1.0
     #position_c.header = goal_to_append.request.goal_constraints[0].header
    #position_c.link_name = goal_to_append.request.goal_constraints[0].link_name
    #position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
    #position_c.constraint_region.primitive_poses.append(goal_pose)
  
  pos_constraint.target_point_offset.x = 0.1
  pos_constraint.target_point_offset.y = 0.1
  pos_constraint.target_point_offset.z = 0.1
  #pos_constraint.constraint_region = 
  pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1.0]))
  pos_constraint.weight = 1
  delivery_constraints.position_constraints = [pos_constraint]
  move_arm.set_path_constraints(delivery_constraints)
  
  #joint_goal = move_arm.get_current_joint_values()
  #joint_goal[constants.J_DIST_PITCH] = -math.pi/4
  #move_arm.go(joint_goal, wait=True)
  #move_arm.stop()
  
  #move_arm.clear_path_constraints()
  
  
  ### Once aligned to trench goal, place hand above trench middle point
  #goal_pose = move_limbs.get_current_pose().pose
  #goal_pose.position.x = x_tr
  #goal_pose.position.y = y_tr
  #goal_pose.position.z = constants.GROUND_POSITION + constants.SCOOP_OFFSET - depth
  #move_limbs.set_pose_target(goal_pose)
  #plan = move_limbs.plan()

  #if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    #return False

  #plan = move_limbs.go(wait=True)
  #move_limbs.stop()
  #move_limbs.clear_pose_targets()
  
  ##  rotate to dig in the ground
  #joint_goal = move_arm.get_current_joint_values()
  #joint_goal[constants.J_DIST_PITCH] = math.pi/10 # we want zero so a number very close to zero
  #move_arm.go(joint_goal, wait=True)
  #move_arm.stop()
  
  ## linear trenching 
  
  #cartesian_plan, fraction = plan_cartesian_path(move_arm,move_limbs, scale=100)
  #move_limbs.execute(cartesian_plan, wait=True)
  #move_limbs.stop()
  #move_arm.execute(cartesian_plan, wait=True)
  #move_arm.stop()
  
  ##  rotate to dig out 
  #joint_goal = move_arm.get_current_joint_values()
  #joint_goal[constants.J_DIST_PITCH] = math.pi/4
  #move_arm.go(joint_goal, wait=True)
  #move_arm.stop()
  '''
  
  return True



