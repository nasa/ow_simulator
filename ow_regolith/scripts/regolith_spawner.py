#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *

def spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame):
  print("spawn_model called")
  rospy.wait_for_service('/gazebo/spawn_sdf_model')
  try:
    service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    response = service(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
    return response
  except rospy.ServiceException as e:
    print('Service call failed: %s' % e)

if __name__ == "__main__":
  response = spawn_model("regolith", open("/home/starry/oceanwaters/workspace/src/ow_simulator/ow_regolith/models/regolith_sample/model.sdf").read(), "/regolith", Pose(position=Point(0, 0, -0.05), orientation=Quaternion(0, 0, 0, 0)), "lander::l_scoop_tip")
  print("Response = %s" % response)