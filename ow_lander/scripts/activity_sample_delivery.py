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

# the upright constraint keeps the scoop uright during the motion. It was not needed for this test. 

def init_upright_path_constraints(self,pose): 

    self.upright_constraints = Constraints()
    self.upright_constraints.name = "upright"
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    #orientation_constraint.link_name = self.arm.get_end_effector_link()
    orientation_constraint.link_name = self.get_end_effector_link()
    #orientation_constraint.link_name = l_scoop
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



def plan_cartesian_path(move_arm, move_limbs, scale):

    waypoints = []
    wpose = move_limbs.get_current_pose().pose
    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
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
  
  
  #rotate shoulder joint to dig radially outwards

  #joint_goal = move_arm.get_current_joint_values()
  #joint_goal[constants.J_SHOU_YAW] = math.pi/4
  ## If out of joint range, abort (TODO: parse limit from urdf)
  #if (joint_goal[constants.J_SHOU_YAW]<-1.8) or (joint_goal[constants.J_SHOU_YAW]>1.8): 
    #return False

  #joint_goal[constants.J_SCOOP_YAW] = 0
  #move_arm.go(joint_goal, wait=True)
  #move_arm.stop()
  
      # find return sampe point begin
  goal_pose = move_arm.get_current_pose().pose
  return_pt = (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)
  return_o = goal_pose.orientation
    # find return sampe point end
  
  

    
  #rotate scoop outwards to dig radially outwards
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_SCOOP_YAW] = math.pi/2
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  

  
    #rotate dist pith to pre-trenching position. 
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = -math.pi/2
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  


  
  ## Once aligned to trench goal, place hand above trench middle point
  goal_pose = move_limbs.get_current_pose().pose
  goal_pose.position.x = x_tr
  goal_pose.position.y = y_tr
  goal_pose.position.z = constants.GROUND_POSITION + constants.SCOOP_OFFSET - depth
  move_limbs.set_pose_target(goal_pose)
  plan = move_limbs.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_limbs.go(wait=True)
  move_limbs.stop()
  move_limbs.clear_pose_targets()
  
  #  rotate to dig in the ground  ###rotate scoop to deliver sample at current location begin
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 55.0/180.0*math.pi # we want zero so a number very close to zero
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  
  # linear trenching 
  
  cartesian_plan, fraction = plan_cartesian_path(move_arm,move_limbs, scale=100)
  #move_limbs.execute(cartesian_plan, wait=True)
  #move_limbs.stop()
  move_arm.execute(cartesian_plan, wait=True)
  move_arm.stop()
  
  #  rotate to dig out of ground
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = math.pi/3
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()
  
  # after sample collect  
  
  goal_pose = move_arm.get_current_pose().pose
  #goal_pose.position.x = return_pt[0] # original return point .. can be used later for dumping samples
  #goal_pose.position.y = return_pt[1]
  #goal_pose.position.z = return_pt[2]
  #goal_pose.position.x = 0.52 #top of lander pos 
  #goal_pose.position.y = -0.22
  #goal_pose.position.z = 0.82
  # position of sample ingestion
  goal_pose.position.x = 0.55 #top of lander pos was 0.53
  goal_pose.position.y = -0.36
  goal_pose.position.z = 0.82 # was .78
  goal_pose.orientation = return_o 

  
  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  
  
  ###rotate scoop to deliver sample at current location... 
   
  # adding position constraint on the solution so that the tip doesnot diverge to get to the solution.  
  pos_constraint = PositionConstraint()
  pos_constraint.header.frame_id = "base_link"
  #pos_constraint.link_name = "l_scoop_tip"
  pos_constraint.link_name = "l_scoop"
  pos_constraint.target_point_offset.x = 0.1
  pos_constraint.target_point_offset.y = 0.1
  pos_constraint.target_point_offset.z = 0.1  ###rotate scoop to deliver sample at current location begin
  pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
  pos_constraint.weight = 1
  
  #using euler angles for own verification..
  mypi = 3.14159
  r = +180
  p = 90  # 45 worked get
  y = -90
  d2r = mypi/180
  r2d = 180/mypi
  q = quaternion_from_euler(r*d2r, p*d2r, y*d2r)
  goal_pose = move_arm.get_current_pose().pose
  rotation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)
  euler_angle = euler_from_quaternion(rotation)
  # debug angles 
  #print "Current quaternion is %s." % (qg)
  #print "Current euler is ******************************************** %s %s %s." % (euler_angle[0]*r2d, euler_angle[1]*r2d, euler_angle[2]*r2d)
  #print "goal quaternion is %s %s %s %s." % (q[0], q[1], q[2], q[3])
  #print "target goal euler is ----------------------------------------------  %s %s %s." % (target_euler_angle[0]*r2d, target_euler_angle[1]*r2d, target_euler_angle[2]*r2d)
  #print "current position is ++++++++++++++++++++++++++++++++++++++++ %s %s %s." % (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)
  return_pt = (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)

  goal_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
  #goal_pose.orientation = Quaternion(-0.48, 0.49, 0.5, 0.52) # angles from rviz 
  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()
  #rotate scoop to deliver sample at current location end
  
  return True



