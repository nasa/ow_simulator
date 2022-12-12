# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import dynamic_reconfigure.client
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController
from owl_msgs.msg import SystemFaultsStatus

from ow_lander.common import Singleton

class ArmFaultMonitor(metaclass=Singleton):
    """Checks whether the arm has faulted."""

    CONTINUE_ARM_IN_FAULT_NAME = 'arm_motion_continues_in_fault'

    def __init__(self):
        """Subscribes to system_fault_status for arm fault updates and connects
        to the faults dynamic reconfigure server to retrieve the continue arm in
        fault flag.
        May raise TimeoutError if a server connection fails.
        """
        SYSTEM_FAULTS_TOPIC = "/system_faults_status"
        self._sub_system_faults = rospy.Subscriber(
            SYSTEM_FAULTS_TOPIC,
            SystemFaultsStatus,
            self._system_faults_callback
        )
        self._arm_fault = False

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
                f"connection with the {DYNRECON_NAME} dynamic reconfigure " \
                f"server."
            )
        rospy.loginfo(
            "Successfully connected to the dynamic reconfigure server.")
        self._continue_arm_in_fault = config[
            ArmFaultMonitor.CONTINUE_ARM_IN_FAULT_NAME]

    def _system_faults_callback(self, data):
        """If system fault occurs and it is an arm failure, mark flag true"""
        ARM_FAULT = 4
        self._arm_fault = data.value & ARM_FAULT == ARM_FAULT

    def _dynamic_reconfigure_callback(self, config):
        """Updates the continue arm flag from dynamic reconfigure server."""
        self._continue_arm_in_fault = config[
            ArmFaultMonitor.CONTINUE_ARM_IN_FAULT_NAME]
        # TODO: could this callback also assign arm_fault?

    def is_arm_faulted(self):
        return self._arm_fault

    def should_arm_continue_in_fault(self):
        return self._continue_arm_in_fault

class ArmTrajectoryExecutor(metaclass=Singleton):
    """Invokes the follow_joint_trajectory actions of arm controllers to
    execute a trajectory provided as a moveit_msgs.msg.RobotTrajectory.
    Execution occurs asynchronously and can be ceased with a method call.
    """

    def __init__(self):
        """Connect to action server of specified controller.
        May raise a TimeoutError if a server connection fails.
        """
        # initialize action client for all required controllers (arm and grinder)
        ACTION_CLIENT_TIMEOUT = 30 # seconds
        self.SUPPORTED_CONTROLLERS = ['arm_controller', 'grinder_controller']
        self._active_controller = self.SUPPORTED_CONTROLLERS[0]
        self._follow_action_clients = dict()
        for controller in self.SUPPORTED_CONTROLLERS:
            self._follow_action_clients[controller] = actionlib.SimpleActionClient(
                f"{controller}/follow_joint_trajectory",
                FollowJointTrajectoryAction
            )
            if not self._follow_action_clients[controller].wait_for_server(
                    rospy.Duration(ACTION_CLIENT_TIMEOUT)):
                raise TimeoutError(
                  f"Timed out waiting {ACTION_CLIENT_TIMEOUT} seconds for "\
                  f"connection the {controller} joint trajectory action server."
                )
            rospy.loginfo(f"Successfully connected to {controller} joint "\
                          f"trajectory action server.")
        # initialize controller switch service proxy to enable grinder trajectories
        SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'
        SERVICE_PROXY_TIMEOUT = 30 # seconds
        rospy.wait_for_service(SWITCH_CONTROLLER_SERVICE, SERVICE_PROXY_TIMEOUT)
        self._switch_controller_srv = rospy.ServiceProxy(
          SWITCH_CONTROLLER_SERVICE, SwitchController)

    def _get_active_follow_client(self):
        return self._follow_action_clients[self._active_controller]

    def switch_controllers(self, switch_to, switch_from):
        if not switch_to in self.SUPPORTED_CONTROLLERS:
            rospy.logerr(f"{switch_to} is not a supported controller")
            return False
        if switch_to == self._active_controller:
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
            self._active_controller = switch_to
            rospy.sleep(0.2)
        return success

    def get_active_controller(self):
        return self._active_controller

    def execute(self, trajectory, goal_time_tolerance=rospy.Time(0.1),
                done_cb=None, active_cb=None, feedback_cb=None):
        """Execute the provided trajectory asynchronously.
        trajectory -- An instance of moveit_msgs.msg.RobotTrajectory that
                      describes the desired trajectory.
        """
        goal = FollowJointTrajectoryGoal(
            trajectory = trajectory,
            goal_time_tolerance = goal_time_tolerance
        )
        self._get_active_follow_client().send_goal(
            goal, done_cb, active_cb, feedback_cb)

    # FIXME: May not cancel follow_client if it has failed. See OW-1090
    def cease_execution(self):
        """Stops the execution of the last trajectory submitted for execution"""
        if self._get_active_follow_client().get_state() == GoalStatus.ACTIVE:
            self._get_active_follow_client().cancel_goal()

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

    # FIXME: returns 4 (GoalStatus.ABORTED) after the initial movement on
    #        complex trajectories like grind (see OW-1090 for more details)
    def is_active(self):
        return self._get_active_follow_client().get_state() == GoalStatus.ACTIVE
