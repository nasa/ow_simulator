# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import dynamic_reconfigure.client
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController
from owl_msgs.msg import SystemFaultsStatus

from ow_lander.common import Singleton

ARM_EXECUTION_ERROR = 4

class ArmTrajectoryExecutor(metaclass=Singleton):
    """Interfaces with the Joint Trajectory Actions of a given controller to
    execute a trajectory provided as a moveit_msgs.msg.RobotTrajectory
    """

    def __init__(self):
        """
        Connects to action server of specified controller and the dynamic
        reconfigure server. If either connection fails false is returned.
        :returns: True if ground connection was established, False otherwise
        :rtype: boolean
        """
        # subscribe to system_fault_status for any arm faults
        SYSTEM_FAULTS_TOPIC = "/system_faults_status"
        self._sub_system_faults = rospy.Subscriber(
            SYSTEM_FAULTS_TOPIC, SystemFaultsStatus, self._system_faults_callback)

        # initialize a dynamic reconfigure client to receive reconfiguration
        DYNRECON_CLIENT_TIMEOUT = 30 # seconds
        DYNRECON_NAME = "faults"
        self._faults_reconfigure_client = dynamic_reconfigure.client.Client(
            DYNRECON_NAME, timeout=DYNRECON_CLIENT_TIMEOUT,
            config_callback=self._dynamic_reconfigure_callback)
        config = self._faults_reconfigure_client.get_configuration(
            DYNRECON_CLIENT_TIMEOUT)
        if not config:
            raise TimeoutError(
                f"Timed out waiting {DYNRECON_CLIENT_TIMEOUT} seconds for a " \
                f"connection with the {DYNRECON_NAME} dynamic reconfigure server."
            )
        rospy.loginfo("Successfully connected to the dynamic reconfigure server.")
        self.arm_motion_continues_in_fault = config['arm_motion_continues_in_fault']

        # initialize action client for all required controllers (arm and grinder)
        ACTION_CLIENT_TIMEOUT = 30 # seconds
        self.active_controller = 'arm_controller'
        self.SUPPORTED_CONTROLLERS = ['arm_controller', 'grinder_controller']
        self._follow_action_clients = dict()
        for controller in self.SUPPORTED_CONTROLLERS:
            self._follow_action_clients[controller] = actionlib.SimpleActionClient(
                f"{controller}/follow_joint_trajectory",
                FollowJointTrajectoryAction
            )
            if not self._follow_action_clients[controller].wait_for_server(
                    rospy.Duration(ACTION_CLIENT_TIMEOUT)):
                raise TimeoutError(
                  f"Timed out waiting {ACTION_CLIENT_TIMEOUT} seconds for connection " \
                  f"the {controller} joint trajectory action server."
                )
            rospy.loginfo(f"Successfully connected to {controller} joint "\
                          f"trajectory action server.")
        # initialize controller switch service proxy to enable grinder trajectories
        SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'
        SERVICE_PROXY_TIMEOUT = 30 # seconds
        rospy.wait_for_service(SWITCH_CONTROLLER_SERVICE, SERVICE_PROXY_TIMEOUT)
        self._switch_controller_srv = rospy.ServiceProxy(
          SWITCH_CONTROLLER_SERVICE, SwitchController)
        self._stopped = False

    def _dynamic_reconfigure_callback(self, config):
        """Update the corresponding flag from dynamic reconfigure server.
        """
        self.arm_motion_continues_in_fault = config['arm_motion_continues_in_fault']
        # TODO: couldn't this callback also assign arm_fault?

    def _system_faults_callback(self, data):
        """If system fault occurs and it is an arm failure, the arm_fault flag
        is set."""
        self.arm_fault = data.value & ARM_EXECUTION_ERROR == ARM_EXECUTION_ERROR

    def switch_controllers(self, switch_to, switch_from):
        if not switch_to in self.SUPPORTED_CONTROLLERS:
            rospy.logerr(f"{switch_to} is not a supported controller")
            return False
        if switch_to == self.active_controller:
            return False
        success = False
        try:
            success = self._switch_controller_srv(
                [switch_to], [switch_from], 2, False, 1.0)
        except rospy.ServiceException as err:
            rospy.loginfo(f"Failed to call switch_controller service: {err}")
        # This sleep is a workaround for "start point deviates from current
        # robot state" error on dig_circular trajectory execution.
        if success:
            self.active_controller = switch_to
            rospy.sleep(0.2)
        return success

    def execute(self, trajectory, goal_time_tolerance=rospy.Time(0.1),
                            done_cb=None, active_cb=None, feedback_cb=None):
        """
        :type trajectory: trajectory_msgs/JointTrajectory
        :type done_cb: function
        :type active_cb: function
        :type feedback_cb: function
        """
        if self.arm_fault:
            return False

        goal = FollowJointTrajectoryGoal(
            trajectory = trajectory,
            goal_time_tolerance = goal_time_tolerance
        )
        self._get_active_follow_client().send_goal(
            goal, done_cb, active_cb, feedback_cb)

        # only return when follow action has been accepted
        rospy.sleep(0.2)

        # FOLLOW_ACTION_ACCEPTED_TIMEOUT = rospy.log
        # while

    def stop_arm_if_fault(self, _feedback):
        """stops arm if arm fault exists during feedback callback"""
        if self.arm_motion_continues_in_fault is False and self.arm_fault:
            self.stop()

    def success(self):
        return not self.arm_fault

    def _get_active_follow_client(self):
        return self._follow_action_clients[self.active_controller]

    def stop(self):
        """Stops the execution of the last trajectory submitted for execution"""
        if self._get_active_follow_client().get_state() == GoalStatus.ACTIVE:
            self._get_active_follow_client().cancel_goal()
            self._stopped = True
        return True
        # return False # nothing was cancelled

    def is_stopped(self):
        return self._stopped

    def reset_stopped_state(self):
        self._stopped = False

    def wait(self, timeout=0):
        """
        Blocks until the execution of the current trajectory comes to an end
        :type timeout: int
        """
        self._get_active_follow_client().wait_for_result(
            timeout=rospy.Duration(timeout))

    def result(self):
        """
        Gets the result of the last goal
        """
        return self._get_active_follow_client().get_result()

    # FIXME: returns 4 (GoalStatus.ABORTED) after the initial movement on complex
    #        trajectories like grind
    def is_active(self):
        return self._get_active_follow_client().get_state() == GoalStatus.ACTIVE

    def is_preempted(self):
        return self._get_active_follow_client().get_state() == GoalStatus.PREEMPTED
