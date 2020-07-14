#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
The modify_terrain_scoop_pub module updates the terrain in line with the scoop
movement. It accomplishes this by subscribing to link poses of the scoop and
issuing a corresponding modify_terrain_* message to update the terrain.
"""

import time
from math import degrees
import rospy
import numpy as np
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from ow_dynamic_terrain.msg import modify_terrain_ellipse


class ModifyTerrainScoop:

  def __init__(self, *args):
    rospy.init_node("modify_terrain_scoop_pub", anonymous=True)
    self.last_translation = np.zeros(3)
    self.pub = rospy.Publisher(
        'ow_dynamic_terrain/modify_terrain_ellipse', modify_terrain_ellipse, queue_size=1)
    self.states_sub = rospy.Subscriber(
        "/gazebo/link_states", LinkStates, self.handle_link_states)

  def compose_modify_terrain_ellipse_message(self, position, orientation):
    return modify_terrain_ellipse(position=Point(position.x, position.y, position.z),
                                  orientation=orientation,
                                  outer_radius_a=0.002, outer_radius_b=0.005,
                                  inner_radius_a=0.001, inner_radius_b=0.001,
                                  weight=-0.025, merge_method="min")

  def check_and_submit(self, new_position, new_rotation):
    """ Checks if position has changed significantly before submmitting a message """

    current_translation = np.array([new_position.x, new_position.y, new_position.z])
    if all(np.isclose(current_translation, self.last_translation, atol=1.e-4)):
      return  # no significant change, abort

    self.last_translation = current_translation

    _, _, yaw = euler_from_quaternion(
        quaternion=(new_rotation.x, new_rotation.y, new_rotation.z, new_rotation.w))

    msg = self.compose_modify_terrain_ellipse_message(new_position, degrees(yaw))
    self.pub.publish(msg)
    
    rospy.logdebug_throttle(1, "modify_terrain_scoop message:\n" + str(msg))

  def handle_link_states(self, data):
    try:
      idx = data.name.index("lander::l_scoop_tip")
    except ValueError:
      rospy.logerr_throttle(1, "lander::l_scoop_tip not found in link_states")
      return

    position = data.pose[idx].position
    orientation = data.pose[idx].orientation
    self.check_and_submit(position, orientation)


if __name__ == '__main__':
  mtg = ModifyTerrainScoop()
  rospy.spin()
