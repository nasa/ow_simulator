#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Python libs
import sys, time
import numpy as np

VERBOSE=True

class MechanicalPower:
  def __init__(self):
    """Initialize ros publisher, ros subscriber"""
    # topic where we publish
    self.power_pub = rospy.Publisher('/mechanical_power/raw', Float64, queue_size = 10)
    self.power_pub_a = rospy.Publisher('/mechanical_power/average', Float64, queue_size = 10)
        
    self.subscriber = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size = 1)
    self.history = []
    rate = rospy.Rate(100)    
            

  def _moving_average(self, msg):  
    """
    :type msg: float
    """
    #global history
    self.history.append(msg)
    if len(self.history) > 10:
        self.history = self.history[-10:]
        average = sum(self.history) / float(len(self.history))
        #rospy.loginfo('Average of most recent {} samples: {}'.format(len(self.history), average))
    else:
        average = msg

    return average    
        
  def callback(self, ros_data): 
    """
    :type ros_data: class 'sensor_msgs.msg._JointState.JointState'
    """
    self._value = len(ros_data.name)
    num_joints = len(ros_data.name)
    power = 0
    for x in range(0,num_joints):
        power = power + abs(ros_data.velocity[x]*ros_data.effort[x])

        # Publish total power
    power_avg =  self._moving_average(power)
        
    self.power_pub.publish(power)
    self.power_pub_a.publish(power_avg)

def main(args): 
  """
  Initializes and cleanup ros node
  :type args: List[str, str, str]
  """
  rospy.set_param('/use_sim_time', True)
  rospy.init_node('mechanical_power_arm', anonymous=True)
  mp = MechanicalPower()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down mechanical_power module"


if __name__ == '__main__':
  main(sys.argv)
 
