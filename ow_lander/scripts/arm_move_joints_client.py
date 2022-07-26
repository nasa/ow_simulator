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


def ArmMoveJoints_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('relative', type=strtobool,
                        help='Move joint relative to the current position', nargs='?', default='False', const=0)
    parser.add_argument('j_shou_yaw', type=float,
        help='Shoulder yaw joint angle', nargs='?', default=0.1, const=0)
    parser.add_argument('j_shou_pitch', type=float,
        help='Shoulder pitch joint angle', nargs='?', default=0.1, const=0)
    parser.add_argument('j_prox_pitch', type=float,
        help='Proximal pitch joint angle', nargs='?', default=0.1, const=0)
    parser.add_argument('j_dist_pitch', type=float,
        help='Distal pitch joint angle', nargs='?', default=0.1, const=0)
    parser.add_argument('j_hand_yaw', type=float,
        help='Hand yaw joint angle', nargs='?', default=0.1, const=0)
    parser.add_argument('j_scoop_yaw', type=float,
        help='Scoop yaw joint angle', nargs='?', default=0.1, const=0)
    args = parser.parse_args()
    print_arguments(args)
    client = actionlib.SimpleActionClient(
        'ArmMoveJoints', ow_lander.msg.ArmMoveJointsAction)
    client.wait_for_server()
    goal = ow_lander.msg.ArmMoveJointsGoal()
    goal.relative = bool(args.relative)
    goal.angles = [
        args.j_shou_yaw,
        args.j_shou_pitch,
        args.j_prox_pitch,
        args.j_dist_pitch,
        args.j_hand_yaw,
        args.j_scoop_yaw
    ]
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
        rospy.init_node('arm_move_joints_client_py')
        result = ArmMoveJoints_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
