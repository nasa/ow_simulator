#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

from ow_lander import actions

rospy.init_node('arm_action_servers')
server_stop = actions.StopServer()
server_unstow = actions.UnstowServer()
server_stow = actions.StowServer()
server_grind = actions.GrindServer()

# TODO: other actions go here

rospy.spin()
