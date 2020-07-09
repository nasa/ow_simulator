#!/usr/bin/env python
# -*- encoding: utf-8 -*-
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
    self.pub = rospy.Publisher(
        'ow_dynamic_terrain/modify_terrain_circle', modify_terrain_circle, queue_size=1)
    self.states_sub = rospy.Subscriber(
        "/gazebo/link_states", LinkStates, self.emit_circle_message_callback)

  def compose_modify_terrain_circle_message(self, position):
    return modify_terrain_circle(position=Point(position.x, position.y, position.z),
                                 outer_radius=0.008, inner_radius=0.001,
                                 weight=-0.2, merge_method="min")

  def check_and_submit(self, new_position, new_rotation):
    """ Checks if position has changed significantly before submmitting a message """

    current_translation = np.array(
        [new_position.x, new_position.y, new_position.z])

    if all(np.isclose(current_translation, self.last_translation, atol=1.e-5)):
      return  # no significant change, abort

    _, roll, _ = euler_from_quaternion(
        quaternion=(new_rotation.x, new_rotation.y, new_rotation.z, new_rotation.w))

    current_orientation = np.array([degrees(roll)])
    effective_orientation = np.array([90])
    if not all(np.isclose(current_orientation, effective_orientation, atol=10)):
      return

    self.last_translation = current_translation

    msg = self.compose_modify_terrain_circle_message(new_position)
    self.pub.publish(msg)
    
    rospy.logdebug_throttle(1, "modify_terrain_scoop message:\n" + str(msg))

  def emit_circle_message_callback(self, data):
    try:
      idx = data.name.index("lander::l_grinder")
    except ValueError:
      return

    position = data.pose[idx].position
    orientation = data.pose[idx].orientation

    self.check_and_submit(position, orientation)


if __name__ == '__main__':
  mtg = ModifyTerrainGrinder()
  rospy.spin()
