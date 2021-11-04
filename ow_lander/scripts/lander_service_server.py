#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

from ow_lander.srv import Light
from irg_gazebo_plugins.msg import ShaderParamUpdate


class LanderServiceServer(object):

  def __init__(self):
    super(LanderServiceServer, self).__init__()

  def run(self):
    rospy.init_node('lander_service_server', anonymous=True)
 
    # Create publishers and messages
    self.light_pub = rospy.Publisher("/gazebo/global_shader_param",
                                     ShaderParamUpdate, queue_size=1)
    self.light_msg = ShaderParamUpdate()
    self.light_msg.shaderType = ShaderParamUpdate.SHADER_TYPE_FRAGMENT

    # Advertise services
    self.lander_light_srv = rospy.Service(
        'lander/light', Light, self.handle_light)

    rospy.loginfo("Lander service server has started.")

    rospy.spin()

  def handle_light(self, req):
    """
    :type req: class 'ow_lander.srv.Light.LightRequest'
    """
    if req.name != 'left' and req.name != 'right':
      return False, 'Light intensity setting failed. "{}" is not a light '\
             'identifier.'.format(req.name)

    if req.intensity < 0.0 or req.intensity > 1.0:
      return False, 'Light intensity setting failed. Intensity = {} is out '\
             'of range.'.format(req.intensity)

    self.light_msg.paramName = 'spotlightIntensityScale[0]'
    if req.name == 'right':
        self.light_msg.paramName = 'spotlightIntensityScale[1]'
    self.light_msg.paramValue = str(req.intensity)
    self.light_pub.publish(self.light_msg)
    return True, '{} light intensity setting succeeded.'.format(req.name)


if __name__ == '__main__':
  lss = LanderServiceServer()
  lss.run()
