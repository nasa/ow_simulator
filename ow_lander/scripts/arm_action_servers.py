#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

from ow_lander.actions import arm

rospy.init_node('arm_action_servers')
server_stop = arm.StopServer()
server_unstow = arm.UnstowServer()
server_stow = arm.StowServer()
server_grind = arm.GrindServer()

# TODO: other actions go here

rospy.spin()
