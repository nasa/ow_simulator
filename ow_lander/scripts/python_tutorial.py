import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math
import constants

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

target_x=2.0
target_y=0.0
target_z=0.3
direction_x=0.0
direction_y=0.0
direction_z=1.0
search_distance = 0.5

targ_x = target_x
targ_y = target_y
targ_z = target_z


# STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
targ_elevation = -0.2
if (targ_z+targ_elevation)==0:
  offset = search_distance
else:
  offset = (targ_z*search_distance)/(targ_z+targ_elevation)

# Compute shoulder yaw angle to target
alpha = math.atan2( (targ_y+direction_y*offset)-constants.Y_SHOU, (targ_x+direction_x*offset)-constants.X_SHOU)
h = math.sqrt(pow( (targ_y+direction_y*offset)-constants.Y_SHOU,2) + pow( (targ_x+direction_x*offset)-constants.X_SHOU,2) )
l = constants.Y_SHOU - constants.HAND_Y_OFFSET
beta = math.asin (l/h)

# Move to pre move position, align shoulder yaw
joint_goal = group.get_current_joint_values()
joint_goal[constants.J_DIST_PITCH] = 0
joint_goal[constants.J_HAND_YAW] = 0
joint_goal[constants.J_PROX_PITCH] = -math.pi/2
joint_goal  [constants.J_SHOU_PITCH] = math.pi/2
joint_goal[constants.J_SHOU_YAW] = alpha + beta


# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()



# Once aligned to move goal and offset, place scoop tip at surface target offset
goal_pose = group.get_current_pose().pose
goal_pose.position.x = targ_x
goal_pose.position.y = targ_y
goal_pose.position.z = targ_z
group.set_pose_target(goal_pose)
group.set_max_velocity_scaling_factor(0.5)
plan = group.plan()
#if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
  #return False

plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()

print "Done planning approach of guarded_move"
