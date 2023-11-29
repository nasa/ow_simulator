#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from irg_gazebo_plugins.msg import ShaderParamUpdate


class SunValues:
  def __init__(self):
    # find reference body and sun in parameters file
    self.reference_body = rospy.get_param('~reference_body')
    target_bodies = rospy.get_param('~target_bodies')
    self.sun = ""
    for i in target_bodies:
      if i.lower() == "sun":
        self.sun = i

    self.listener = tf.TransformListener()

    # Must use separate messages or they sometimes interfere with one another
    self.flux_msg = ShaderParamUpdate()
    self.visibility_msg = ShaderParamUpdate()
    self.shader_param_pub = rospy.Publisher("/gazebo/global_shader_param", ShaderParamUpdate, queue_size=1)

    self.lens_flare_color_msg = Vector3()
    self.lens_flare_color_pub = rospy.Publisher("/gazebo/plugins/ow_lens_flare/color", Vector3, queue_size=1)

  # Compute sun intensity at distance to our reference body
  def computeIntensity(self):
    if self.reference_body == "" or self.sun == "":
      return

    # Ignore missing transforms--there will probably be a few missing at startup
    try:
      (trans,rot) = self.listener.lookupTransform(self.reference_body, self.sun, rospy.Time(0))
    except:
      return

    dist = math.sqrt(trans[0] * trans[0] + trans[1] * trans[1] + trans[2] * trans[2])
    # Ratio of 1 AU over distance
    dist_ratio = 149597870700.0 / dist
    # Use the solar constant (in W / m^2) to compute flux at this distance from sun
    flux = 1361.5 * dist_ratio * dist_ratio

    self.flux_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT
    self.flux_msg.paramName = "sunIntensity"
    self.flux_msg.paramValue = str(flux) + ' ' + str(flux) + ' ' + str(flux)
    self.shader_param_pub.publish(self.flux_msg)

    self.lens_flare_color_msg.x = flux
    self.lens_flare_color_msg.y = flux
    self.lens_flare_color_msg.z = flux
    self.lens_flare_color_pub.publish(self.lens_flare_color_msg)

  # Pass sun visibility value from irg_planetary_ephemeris to shaders in our
  # visual simulation. The penumbra of a sun shadow on a moon would be on the
  # order of 100 km wide, so we do not attempt to simulate a shadow gradient--
  # we simply darken the whole scene.
  def __call__(self, imsg):
    self.visibility_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT
    self.visibility_msg.paramName = "sunVisibility"
    self.visibility_msg.paramValue = str(imsg.data)
    self.shader_param_pub.publish(self.visibility_msg)


def main():
  rospy.init_node("sunlight_values")

  sun_values = SunValues()
  rospy.Subscriber("sun_visibility", Float64, sun_values)

  rate = rospy.Rate(1.0)
  while not rospy.is_shutdown():
    try:
      sun_values.computeIntensity()
      rate.sleep()
    except rospy.ROSInterruptException:
      pass


if __name__=='__main__':
  main()

