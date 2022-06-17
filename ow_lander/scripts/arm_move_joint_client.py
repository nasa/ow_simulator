#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import ow_lander.msg
import argparse
from guarded_move_action_client import print_arguments
from urdf_parser_py.urdf import URDF
from distutils.util import strtobool


def ArmMoveJoint_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('relative', type=strtobool,
                        help='Move joint relative to the current position', nargs='?', default='False', const=0)
    parser.add_argument('joint', type=int,
                        help='Joint index to be moved 0:j_shou_yaw, 1:j_shou_pitch, 2:j_prox_pitch, 3:j_dist_pitch, 4:j_hand_yaw, 5:j_scoop_yaw', nargs='?', 
                        default=0, const=0, choices=range(6))

    parser.add_argument('angle', type=float,
                        help='goal angle of the joint in radians', nargs='?', default=-0.5, const=0)
    args = parser.parse_args()
    print_arguments(args)

    joints_set = {0: 'j_shou_yaw', 1: 'j_shou_pitch', 2: 'j_prox_pitch',
                  3: 'j_dist_pitch', 4: 'j_hand_yaw', 5: 'j_scoop_yaw'}
    robot = URDF.from_parameter_server()
    print('Lower and upper limit of the joint is',
          robot.joint_map[joints_set[args.joint]].limit.lower, robot.joint_map[joints_set[args.joint]].limit.upper)

    client = actionlib.SimpleActionClient(
        'ArmMoveJoint', ow_lander.msg.ArmMoveJointAction)

    client.wait_for_server()

    goal = ow_lander.msg.ArmMoveJointGoal()

    goal.relative = bool(args.relative)
    goal.joint = args.joint
    goal.angle = args.angle
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    if client.get_state() == GoalStatus.ABORTED:
        return ('aborted')
    else:
        return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the discard client can
        # publish and subscribe over ROS.
        rospy.init_node('discard_client_py')
        result = ArmMoveJoint_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
