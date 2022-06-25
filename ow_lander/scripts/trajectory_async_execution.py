#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from operator import truediv
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from ow_faults_detection.msg import SystemFaults
import dynamic_reconfigure.client

ARM_EXECUTION_ERROR = 4

# continue_in_fault = rospy.get_param('continue_arm_in_fault')
class TrajectoryAsyncExecuter:
    """
    Interfaces with a Joint Trajectory Action Server of a given controller to execute
    a precomputed trajectory.
    """

    def __init__(self):
        self._connected = False
        self._goal_time_tolerance = rospy.Time(0.1)
        self._client = None
        self.arm_fault = False
        # rospy.init_node('trajectoryAsyncExecuter')
        # subscribe to system_fault_status for any arm faults
        rospy.Subscriber("/faults/system_faults_status",
                         SystemFaults, self.faultCheckCallback)
        #self.continue_in_fault = rospy.get_param('faults/continue_arm_in_fault')
        self.continue_in_fault = False
        self.faults_client()

        #topic = rospy.get_param('~topic', 'chatter')
        # Create a subscriber with appropriate topic, custom message and name of callback function.
        #rospy.Subscriber("faults/continue_arm_in_fault", SystemFaults, self.paramCheckCallBack)
        # rospy.spin()

    def stop_arm_if_fault(self, feedback):
        """
        stops arm if arm fault exists during feedback callback
        """
        if self.arm_fault and self.continue_in_fault is False:
        #if self.arm_fault and self.continue_if() is False:    
            #rospy.loginfo(self.continue_if())
            self.stop()

    def fault_client_callback(self, config):
        #rospy.loginfo("Config set to {continue_arm_in_fault}".format(**config))        
        self.continue_in_fault = config['continue_arm_in_fault']


    def faults_client(self):
        #rospy.init_node("dynamic_client_faults")
        client = dynamic_reconfigure.client.Client("faults", timeout=30, config_callback=self.fault_client_callback)
        #r = rospy.Rate(0.5)
        #while not rospy.is_shutdown():
        #    r.sleep()        

    def paramCheckCallBack(self, data):
        #self.continue_in_fault =  data.value        
        self.continue_in_fault =  True

    def faultCheckCallback(self, data):
        """
        If system fault occurs, and it is an arm failure, an arm failure flag is set for the whole class
        """
        self.arm_fault = (data.value & ARM_EXECUTION_ERROR ==
                          ARM_EXECUTION_ERROR)
        #self.continue_in_fault = rospy.get_param('faults/continue_arm_in_fault')        

    def continue_if(self):
        return self.continue_in_fault    

    def success(self):
        return not self.arm_fault

    def connect(self, controller):
        """
        Connects to the action server of a specific controller
        :returns: True if ground connection was established, False otherwise
        :rtype: boolean
        """
        self._controller = controller
        self._client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self._controller),
            FollowJointTrajectoryAction)
        self._connected = self._client.wait_for_server()
        if not self._connected:
            rospy.logerr(
                "Timed out waiting for {} joint trajectory action server".format(
                    self._controller))
        else:
            rospy.loginfo(
                "Successfully connected to {} joint Trajectory action server".format(
                    self._controller))
        return self._connected

    def execute(self, trajectory, done_cb=None, active_cb=None, feedback_cb=None):
        """
        :type trajectory: trajectory_msgs/JointTrajectory
        :type done_cb: function
        :type active_cb: function
        :type feedback_cb: function
        """
        if not self._connected:
            rospy.logwarn_throttle(
                1, "TrajectoryAsyncExecuter: Client failed to connect to the action server .. nothing will be executed!")
            return False
        if self.arm_fault:
            return False
        goal = FollowJointTrajectoryGoal()
        goal.goal_time_tolerance = self._goal_time_tolerance
        goal.trajectory = trajectory
        self._client.send_goal(goal, done_cb, active_cb, feedback_cb)

    def stop(self):
        """
        Stops the execution of the last trajectory submitted for executoin
        """
        action_state = self.get_state()
        if self._connected and action_state == GoalStatus.ACTIVE:
            self._client.cancel_goal()

    def wait(self, timeout=0):
        """
        Blocks until the execution of the current trajectory comes to an end
        :type timeout: int
        """
        if self._connected:
            self._client.wait_for_result(timeout=rospy.Duration(timeout))

        return True

    def result(self):
        """
        Gets the result of the last goal
        """
        if not self._connected:
            return None
        return self._client.get_result()

    def get_state(self):
        """
        returns if the goal is active
        """
        if not self._connected:
            return None
        return self._client.get_state()
