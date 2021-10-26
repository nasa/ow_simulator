#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from math import pi
import ow_lander.msg

def wrap_angle(angle):
    """
    :param angle: (float)
    :return: (float) the angle in [-pi, pi]
    """
    tolerance = 0.01
    while angle > (pi+tolerance):
        angle -= 2 * pi
    while angle < -(pi-tolerance):
        angle += 2 * pi
    return angle 

class antenna_client_multiple_goals():
    
    def __init__(self):
        rospy.init_node('antenna_client_multiple_goal', anonymous = False)
        
        rospy.on_shutdown(self.shutdown)
 
        self.client = actionlib.SimpleActionClient('AntennaPanTiltAction', ow_lander.msg.AntennaPanTiltAction)
        rospy.loginfo("Waiting for AntennaPanTilt action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to AntennaPanTilt action server")
        rospy.loginfo("Starting AntennaPanTilt action")

        goal = ow_lander.msg.AntennaPanTiltGoal()

        goal.pan = 0.5
        goal.tilt = 0.0
        goal.tilt = wrap_angle(goal.tilt)
        goal.pan = wrap_angle(goal.pan)
        
        
        self.move(goal)
        
    def move(self, goal):    

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        success = self.client.wait_for_result()
        
        # If we don't get there in time, abort the goal
        if not success:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.client.get_state()
            result = self.client.get_result()
            if state == 3:
                rospy.loginfo("Goal succeeded!")
                # Prints out the result of executing the action
                rospy.loginfo("Result: %s", result)

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.client.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)    

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the UnstowActionClient can
        # publish and subscribe over ROS.
        #rospy.init_node('antenna_client_py')
        antenna_client_multiple_goals()
        #rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
