#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
"""
The modify_terrain_grinder_pub module updates the terrain in line with the  
grinder movement. It accomplishes this by subscribing to link poses of the grinder
and issuing a corresponding modify_terrain_* message to update the terrain.
"""

import rospy
import time
from math import degrees
import numpy as np
from geometry_msgs.msg import Point
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from ow_dynamic_terrain.msg import modify_terrain_circle


class ModifyTerrainGrinder:

  def __init__(self, *args):
    rospy.init_node("modify_terrain_grinder_pub", anonymous=True)
    self.last_translation = np.zeros(3)
    self.pub_visual = rospy.Publisher(
        'ow_dynamic_terrain/modify_terrain_circle/visual', modify_terrain_circle, queue_size=1)
    self.pub_collision = rospy.Publisher(
        'ow_dynamic_terrain/modify_terrain_circle/collision', modify_terrain_circle, queue_size=1)
    self.states_sub = rospy.Subscriber(
        "/gazebo/link_states", LinkStates, self.handle_link_states)

  def compose_modify_terrain_circle_message(self, position, depth, scale=1.0):
    """ Composes a modify_terrain_circle that matches the grinder end effector
    it is positioned downwards ready for digging. When the grinder is position
    downward for digging the shape it projects on the terrain as a circle.

    Parameters:
      position: Corresponds to the center of the modify_terrain_circle operation
      scale: A value that uniformaly scales the generated modify_terrain_circle
        message
    """
    return modify_terrain_circle(position=Point(position.x, position.y, position.z),
                                 outer_radius=0.08*scale, inner_radius=0.04*scale,
                                 weight=depth*scale, merge_method="min")

  def check_and_submit(self, new_position, new_rotation):
    """ Checks if position has changed significantly before submmitting a message """

    _, roll, _ = euler_from_quaternion(
        quaternion=(new_rotation.x, new_rotation.y, new_rotation.z, new_rotation.w))

    current_orientation = np.array([degrees(roll)])
    effective_orientation = np.array([90])
    if not all(np.isclose(current_orientation, effective_orientation, atol=10)):
      return # grinder not aligned for digging, abort

    current_translation = np.array([new_position.x, new_position.y, new_position.z])
    if all(np.isclose(current_translation, self.last_translation, atol=1.e-4)):
      return  # no significant change, abort

    self.last_translation = current_translation

    msg = self.compose_modify_terrain_circle_message(new_position, depth=-0.16, scale=2.0)
    self.pub_collision.publish(msg)
    msg = self.compose_modify_terrain_circle_message(new_position, depth=-0.06, scale=1.0)
    self.pub_visual.publish(msg)
    
    rospy.logdebug_throttle(1, "modify_terrain_grinder message:\n" + str(msg))

  def handle_link_states(self, data):
    try:
      idx = data.name.index("lander::l_grinder")
    except ValueError:
      rospy.logerr_throttle(1, "lander::l_grinder not found in link_states")
      return

    position = data.pose[idx].position
    orientation = data.pose[idx].orientation
    self.check_and_submit(position, orientation)


if __name__ == '__main__':
  mtg = ModifyTerrainGrinder()
  rospy.spin()
