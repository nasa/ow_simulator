# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines all OceanWATERS arm trajectories. Computes parameterized trajectories
based on action goal parameters.
"""

import rospy
import math
import copy
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion, Point
from shape_msgs.msg import SolidPrimitive

from ow_lander import constants
from ow_lander.common import Singleton, is_shou_yaw_goal_in_range
from ow_lander.frame_transformer import FrameTransformer
from ow_lander.trajectory_sequence import TrajectorySequence

def _compute_workspace_shoulder_yaw(x, y):
    # Compute shoulder yaw angle to trench
    alpha = math.atan2(y - constants.Y_SHOU, x - constants.X_SHOU)
    h = math.sqrt(pow(y - constants.Y_SHOU, 2) + pow(x - constants.X_SHOU, 2))
    l = constants.Y_SHOU - constants.HAND_Y_OFFSET
    beta = math.asin(l / h)
    return alpha + beta

def _compute_trench_setup_config(x_start, y_start):
    # Compute shoulder yaw angle to trench
    alpha = math.atan2(y_start-constants.Y_SHOU, x_start-constants.X_SHOU)
    h = math.sqrt(pow(y_start-constants.Y_SHOU, 2) +
                    pow(x_start-constants.X_SHOU, 2))
    l = constants.Y_SHOU - constants.HAND_Y_OFFSET
    beta = math.asin(l/h)
    joint_goal = [0.0] * 6
    # Move to pre trench position, align shoulder yaw
    joint_goal[constants.J_DIST_PITCH] = 0.0
    joint_goal[constants.J_HAND_YAW] = 0.0
    joint_goal[constants.J_PROX_PITCH] = -math.pi/2
    joint_goal[constants.J_SHOU_PITCH] = math.pi/2
    joint_goal[constants.J_SHOU_YAW] = alpha + beta
    joint_goal[constants.J_SCOOP_YAW] = 0
    # TODO: generalize this function
    # If out of joint range, abort
    if (is_shou_yaw_goal_in_range(joint_goal) == False):
        return False
    return joint_goal

class ArmTrajectoryPlanner(metaclass = Singleton):
    """Computes trajectories for arm actions and returns the result as a
    moveit_msgs.msg.RobotTrajectory
    """

    def __init__(self):
        # basic moveit interface for arm control
        self._robot = moveit_commander.RobotCommander()
        self._move_arm = moveit_commander.MoveGroupCommander('arm')
        self._move_grinder = moveit_commander.MoveGroupCommander('grinder')

    def get_end_effector_pose(self, end_effector, frame_id,
                              timestamp=rospy.Time(0),
                              timeout=rospy.Duration(0)):
        """Look up the pose of an end-effector
        end_effector -- Name of the end-effector link
        frame_id     -- Frame ID in which to provide the result
        timestamp    -- See method comments in frame_transformer.py for usage
                        default: rospy.Time(0)
        timeout      -- See method comments in frame_tansformer.py for usage
                        default: rospy.Duration(0)
        returns geometry_msgs.PoseStamped
        """
        pose = self._move_arm.get_current_pose(end_effector)
        pose.header.stamp = timestamp
        return FrameTransformer().transform(pose, frame_id, timeout)

    def calculate_joint_state_end_pose_from_plan_arm(self, plan):
        '''
        calculate the end pose (position and orientation), joint states and robot states
        from the current plan
        inputs:  current plan, robot, arm interface, and moveit forward kinematics object
        outputs: goal_pose, robot state and joint states at end of the plan
        '''
        # get joint states from the end of the plan
        joint_states = plan.joint_trajectory.points[-1].positions
        # construct robot state at the end of the plan
        robot_state = self._robot.get_current_state()
        # adding antenna (0,0) and grinder positions (-0.1) which should not change
        new_value = new_value = (
            0, 0) + joint_states[:5] + (-0.1,) + (joint_states[5],)
        # modify current state of robot to the end state of the previous plan
        robot_state.joint_state.position = new_value
        # TODO: may raise ROSSerializationException
        goal_pose = self.compute_forward_kinematics('l_scoop', robot_state)
        return robot_state, joint_states, goal_pose

    def go_to_XYZ_coordinate(self, cs, goal_pose, x_start, y_start, z_start, approximate=True):
        """
        :param approximate: use an approximate solution. default True
        :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
        :type x_start: float
        :type y_start: float
        :type z_start: float
        :type approximate: bool
        """

        self._move_arm.set_start_state(cs)
        goal_pose.position.x = x_start
        goal_pose.position.y = y_start
        goal_pose.position.z = z_start

        goal_pose.orientation.x = goal_pose.orientation.x
        goal_pose.orientation.y = goal_pose.orientation.y
        goal_pose.orientation.z = goal_pose.orientation.z
        goal_pose.orientation.w = goal_pose.orientation.w

        # Ask the planner to generate a plan to the approximate joint values generated
        # by kinematics builtin IK solver. For more insight on this issue refer to:
        # https://github.com/nasa/ow_simulator/pull/60
        if approximate:
            self._move_arm.set_joint_value_target(goal_pose, True)
        else:
            self._move_arm.set_pose_target(goal_pose)

        _, plan, _, _ = self._move_arm.plan()

        if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
            return False

        return plan

    def go_to_Z_coordinate_dig_circular(self, cs, goal_pose, z_start, approximate=True):
        """
        :type cs: class 'moveit_msgs/RobotState'
        :type goal_pose: Pose
        :type z_start: float
        :param approximate: use an approximate solution. default True
        """

        self._move_arm.set_start_state(cs)
        goal_pose.position.z = z_start

        goal_pose.orientation.x = goal_pose.orientation.x
        goal_pose.orientation.y = goal_pose.orientation.y
        goal_pose.orientation.z = goal_pose.orientation.z
        goal_pose.orientation.w = goal_pose.orientation.w

        # Ask the planner to generate a plan to the approximate joint values generated
        # by kinematics builtin IK solver. For more insight on this issue refer to:
        # https://github.com/nasa/ow_simulator/pull/60
        if approximate:
            self._move_arm.set_joint_value_target(goal_pose, True)
        else:
            self._move_arm.set_pose_target(goal_pose)

        _, plan, _, _ = self._move_arm.plan()

        if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
            return False

        return plan

    def dig_circular(self, point, depth, parallel):
        """
        :type self._move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
        :type args: List[bool, float, int, float, float, float]
        """

        # TODO:
        #  1. implement normal parameter
        #  2. implement scoop_angle parameter

        trench_bottom = Point(
            point.x,
            point.y,
            point.z - depth + constants.R_PARALLEL_FALSE_A
        )
        sequence = TrajectorySequence('l_scoop', self._robot, self._move_arm)
        # place end-effector above trench position
        sequence.plan_to_joint_positions(
            j_shou_yaw = _compute_workspace_shoulder_yaw(point.x, point.y),
            j_shou_pitch = math.pi / 2,
            j_prox_pitch = -math.pi / 2,
            j_dist_pitch = 0.0,
            j_hand_yaw = 0.0,
            j_scoop_yaw = 0.0
        )
        if parallel:
            # rotate hand so scoop bottom points down
            sequence.plan_to_joint_positions(j_hand_yaw = 0.0)
            # rotate scoop to face radially out from lander
            sequence.plan_to_joint_positions(j_scoop_yaw = math.pi/2)
            # pitch scoop back with the distal pitch so its blade faces terrain
            sequence.plan_to_joint_positions(j_dist_pitch = -19.0/54.0*math.pi)
            # Once aligned to trench goal, place hand above trench middle point
            sequence.plan_to_position(trench_bottom)
            # perform scoop by rotating distal pitch, and scoop through surface
            sequence.plan_to_joint_translations(j_dist_pitch = 2.0/3.0*math.pi)
        else:
            # lower to trench position, maintaining up-right orientation
            sequence.plan_to_position(trench_bottom)
            # rotate hand yaw so scoop tip points into surface
            sequence.plan_to_joint_positions(j_hand_yaw = math.pi/2.2)
            # lower scoop back to down z-position with new hand yaw position set
            sequence.plan_to_z(trench_bottom.z)
            # perform scoop by rotating hand yaw, and scoop through surface
            sequence.plan_to_joint_positions(j_hand_yaw = -0.29*math.pi)
        return sequence.merge()

    def move_to_pre_trench_configuration(self, x_start, y_start):
        """
        :type x_start: float
        :type y_start: float
        """

        # Initilize to current position
        joint_goal = self._move_arm.get_current_pose().pose
        robot_state = self._robot.get_current_state()
        self._move_arm.set_start_state(robot_state)

        # Compute shoulder yaw angle to trench
        alpha = math.atan2(y_start-constants.Y_SHOU, x_start-constants.X_SHOU)
        h = math.sqrt(pow(y_start-constants.Y_SHOU, 2) +
                        pow(x_start-constants.X_SHOU, 2))
        l = constants.Y_SHOU - constants.HAND_Y_OFFSET
        beta = math.asin(l/h)
        # Move to pre trench position, align shoulder yaw
        joint_goal = self._move_arm.get_current_joint_values()
        joint_goal[constants.J_DIST_PITCH] = 0.0
        joint_goal[constants.J_HAND_YAW] = math.pi/2.2
        joint_goal[constants.J_PROX_PITCH] = -math.pi/2
        joint_goal[constants.J_SHOU_PITCH] = math.pi/2
        joint_goal[constants.J_SHOU_YAW] = alpha + beta

        # If out of joint range, abort
        if (is_shou_yaw_goal_in_range(joint_goal) == False):
            return False

        joint_goal[constants.J_SCOOP_YAW] = 0
        self._move_arm.set_joint_value_target(joint_goal)
        _, plan, _, _ = self._move_arm.plan()
        return plan

    def plan_cartesian_path(self, move_group, wpose, length, alpha, parallel, z_start, cs):
        """
        :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
        :type length: float
        :type alpha: float
        :type parallel: bool
        """
        if parallel == False:
            alpha = alpha - math.pi/2
        move_group.set_start_state(cs)
        waypoints = []
        wpose.position.z = z_start
        wpose.position.x += length*math.cos(alpha)
        wpose.position.y += length*math.sin(alpha)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # end effector follow step (meters)
            0.0)         # jump threshold

        return plan, fraction

    def plan_cartesian_path_lin(self, move_group, wpose, length, alpha, z_start, cs):
        """
        :type length: float
        :type alpha: float
        """
        move_group.set_start_state(cs)
        waypoints = []

        wpose.position.x += length*math.cos(alpha)
        wpose.position.y += length*math.sin(alpha)

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # end effector follow step (meters)
            0.0)         # jump threshold

        return plan, fraction

    def change_joint_value(self, move_group, cs, start_state, joint_index, target_value):
        """
        :type move_group: class 'moveit_commander.move_group.MoveGroupCommander'
        :type joint_index: int
        :type target_value: float
        """
        move_group.set_start_state(cs)

        joint_goal = move_group.get_current_joint_values()
        for k in range(0, len(start_state)):
            joint_goal[k] = start_state[k]

        joint_goal[joint_index] = target_value
        move_group.set_joint_value_target(joint_goal)
        _, plan, _, _ = move_group.plan()
        return plan

    def go_to_Z_coordinate(self, move_group, cs, goal_pose, x_start, y_start, z_start, approximate=True):
        """
        :param approximate: use an approximate solution. default True
        :type x_start: float
        :type y_start: float
        :type z_start: float
        :type approximate: bool
        """

        move_group.set_start_state(cs)

        goal_pose.position.x = x_start
        goal_pose.position.y = y_start
        goal_pose.position.z = z_start

        # Ask the planner to generate a plan to the approximate joint values generated
        # by kinematics builtin IK solver. For more insight on this issue refer to:
        # https://github.com/nasa/ow_simulator/pull/60
        if approximate:
            move_group.set_joint_value_target(goal_pose, True)
        else:
            move_group.set_pose_target(goal_pose)
        _, plan, _, _ = move_group.plan()

        if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
            return False
        return plan

    def dig_linear(self, point, depth, length):
        """
        :type args: List[bool, float, int, float, float, float]
        """

        # NOTE: point must be in world coordinates

        x_start = point.x
        y_start = point.y
        ground_position = point.z

        # TODO:
        #  1. implement normal parameter

        plan_a = self.move_to_pre_trench_configuration(x_start, y_start)
        if not plan_a or len(plan_a.joint_trajectory.points) == 0:  # If no plan found, abort
            return False

        cs, start_state, current_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            plan_a)
        #################### Rotate hand yaw to dig in#################################

        plan_b = self.change_joint_value(
            self._move_arm, cs, start_state, constants.J_HAND_YAW, 0.0)

        # If no plan found, send the previous plan only
        if len(plan_b.joint_trajectory.points) == 0:
            return plan_a

        dig_linear_traj = _cascade_plans(plan_a, plan_b)

        ######################### rotate scoop #######################################

        cs, start_state, current_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)

        plan_c = self.change_joint_value(
            self._move_arm, cs, start_state, constants.J_SCOOP_YAW, math.pi/2)

        dig_linear_traj = _cascade_plans(dig_linear_traj, plan_c)

        ######################### rotate dist pith to pre-trenching position###########

        cs, start_state, current_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)

        plan_d = self.change_joint_value(
            self._move_arm, cs, start_state, constants.J_DIST_PITCH, -math.pi/2)

        dig_linear_traj = _cascade_plans(dig_linear_traj, plan_d)

        # Once aligned to trench goal,
        # place hand above the desired start point
        alpha = math.atan2(constants.WRIST_SCOOP_PARAL,
                             constants.WRIST_SCOOP_PERP)
        distance_from_ground = constants.ROT_RADIUS * \
            (math.cos(alpha) - math.sin(alpha))
        z_start = ground_position + constants.SCOOP_HEIGHT - depth + distance_from_ground

        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)

        plan_e = self.go_to_Z_coordinate(
            self._move_arm, cs, goal_pose, x_start, y_start, z_start)

        dig_linear_traj = _cascade_plans(dig_linear_traj, plan_e)

        # rotate to dig in the ground

        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)

        plan_f = self.change_joint_value(
            self._move_arm, cs, start_state, constants.J_DIST_PITCH, 2.0/9.0*math.pi)

        dig_linear_traj = _cascade_plans(dig_linear_traj, plan_f)

        # determine linear trenching direction (alpha) value obtained from rviz

        cs, start_state, current_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)

        quaternion = [current_pose.orientation.x, current_pose.orientation.y,
                        current_pose.orientation.z, current_pose.orientation.w]
        current_euler = euler_from_quaternion(quaternion)
        alpha = current_euler[2]

        # linear trenching

        cs, start_state, current_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)
        cartesian_plan, fraction = self.plan_cartesian_path_lin(
            self._move_arm, current_pose, length, alpha, z_start, cs)
        dig_linear_traj = _cascade_plans(dig_linear_traj, cartesian_plan)

        #  rotate to dig out
        cs, start_state, current_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            dig_linear_traj)

        plan_g = self.change_joint_value(
            self._move_arm, cs, start_state, constants.J_DIST_PITCH, math.pi/2)
        dig_linear_traj = _cascade_plans(dig_linear_traj, plan_g)

        self._move_arm.clear_pose_targets()

        return dig_linear_traj

    def calculate_joint_state_end_pose_from_plan_grinder(self, plan):
        '''
        calculate the end pose (position and orientation), joint states and robot states
        from the current plan
        inputs:  current plan, robot, grinder interface, and moveit forward kinematics object
        outputs: goal_pose, robot state and joint states at end of the plan

        :type plan: JointTrajectory
        '''
        # get joint states from the end of the plan
        joint_states = plan.joint_trajectory.points[-1].positions
        # construct robot state at the end of the plan
        robot_state = self._robot.get_current_state()
        # adding antenna (0,0) and j_scoop_yaw (0.1) which should not change
        new_value = (0, 0) + joint_states[:6] + (0.1740,)
        # modify current state of robot to the end state of the previous plan
        robot_state.joint_state.position = new_value
        # TODO: may raise ROSSerializationException
        goal_pose = self.compute_forward_kinematics('l_grinder', robot_state)
        return robot_state, joint_states, goal_pose

    def grind(self, args):
        """
        :type args: List[bool, float, float, float, float, bool, float, bool]
        """

        x_start = args.x_start
        y_start = args.y_start
        depth = args.depth
        length = args.length
        parallel = args.parallel
        ground_position = args.ground_position

        # Compute shoulder yaw angle to trench
        alpha = math.atan2(y_start-constants.Y_SHOU, x_start-constants.X_SHOU)
        h = math.sqrt(pow(y_start-constants.Y_SHOU, 2) +
                        pow(x_start-constants.X_SHOU, 2))
        l = constants.Y_SHOU - constants.HAND_Y_OFFSET
        beta = math.asin(l/h)
        alpha = alpha+beta

        if parallel:
            R = math.sqrt(x_start*x_start+y_start*y_start)
            # adjust trench to fit scoop circular motion
            dx = 0.04*R*math.sin(alpha)  # Center dig_circular in grind trench
            dy = 0.04*R*math.cos(alpha)
            # Move starting point back to avoid scoop-terrain collision
            x_start = 0.9*(x_start + dx)
            y_start = 0.9*(y_start - dy)
        else:
            dx = 5*length/8*math.sin(alpha)
            dy = 5*length/8*math.cos(alpha)
            # Move starting point back to avoid scoop-terrain collision
            x_start = 0.97*(x_start - dx)
            y_start = 0.97*(y_start + dy)

        # Place the grinder vertical, above the desired starting point, at
        # an altitude of 0.25 meters in the base_link frame.
        robot_state = self._robot.get_current_state()
        self._move_grinder.set_start_state(robot_state)
        goal_pose = self._move_grinder.get_current_pose().pose
        goal_pose.position.x = x_start  # Position
        goal_pose.position.y = y_start
        goal_pose.position.z = 0.25
        goal_pose.orientation.x = 0.70616885803  # Orientation
        goal_pose.orientation.y = 0.0303977418722
        goal_pose.orientation.z = -0.706723318474
        goal_pose.orientation.w = 0.0307192507001
        self._move_grinder.set_pose_target(goal_pose)

        _, plan_a, _, _ = self._move_grinder.plan()

        if len(plan_a.joint_trajectory.points) == 0:  # If no plan found, abort
            self._move_grinder.clear_pose_targets()
            return False

        # entering terrain
        z_start = ground_position + constants.GRINDER_OFFSET - depth
        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_grinder(
            plan_a)
        plan_b = self.go_to_Z_coordinate(
            self._move_grinder, cs, goal_pose, x_start, y_start, z_start, False)

        grind_traj = _cascade_plans(plan_a, plan_b)

        # grinding ice forward
        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_grinder(
            grind_traj)
        cartesian_plan, fraction = self.plan_cartesian_path(
            self._move_grinder, goal_pose, length, alpha, parallel, z_start, cs)

        grind_traj = _cascade_plans(grind_traj, cartesian_plan)

        # grinding sideways
        cs, start_state, joint_goal = self.calculate_joint_state_end_pose_from_plan_grinder(
            grind_traj)
        if parallel:
            plan_c = self.change_joint_value(
                self._move_grinder, cs, start_state, constants.J_SHOU_YAW, start_state[0]+0.08)
        else:
            x_now = joint_goal.position.x
            y_now = joint_goal.position.y
            z_now = joint_goal.position.z
            x_goal = x_now + 0.08*math.cos(alpha)
            y_goal = y_now + 0.08*math.sin(alpha)
            plan_c = self.go_to_Z_coordinate(
                self._move_grinder, cs, joint_goal, x_goal, y_goal, z_now, False)

        grind_traj = _cascade_plans(grind_traj, plan_c)
        # grinding ice backwards
        cs, start_state, joint_goal = self.calculate_joint_state_end_pose_from_plan_grinder(
            grind_traj)
        cartesian_plan2, fraction2 = self.plan_cartesian_path(
            self._move_grinder, joint_goal, -length, alpha, parallel, z_start, cs)
        grind_traj = _cascade_plans(grind_traj, cartesian_plan2)

        # exiting terrain
        cs, start_state, joint_goal = self.calculate_joint_state_end_pose_from_plan_grinder(
            grind_traj)
        plan_d = self.go_to_Z_coordinate(
            self._move_grinder, cs, joint_goal, x_start, y_start, 0.22, False)
        grind_traj = _cascade_plans(grind_traj, plan_d)

        self._move_grinder.clear_pose_targets()

        return grind_traj

    def guarded_move(self, args):
        """
        :type self._move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
        :type robot: class 'moveit_commander.RobotCommander'
        :type moveit_fk: class moveit_msgs/GetPositionFK
        :type args: List[bool, float, float, float, float, float, float, float]
        """

        robot_state = self._robot.get_current_state()
        self._move_arm.set_start_state(robot_state)
        self._move_arm.set_planner_id("RRTstar")

        ### pre-guarded move starts here ###

        targ_x = args.start.x
        targ_y = args.start.y
        targ_z = args.start.z
        direction_x = args.normal.x
        direction_y = args.normal.y
        direction_z = args.normal.z
        search_distance = args.search_distance

        # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
        targ_elevation = -0.2
        if (targ_z+targ_elevation) == 0:
            offset = search_distance
        else:
            offset = (targ_z*search_distance)/(targ_z+targ_elevation)

        # Compute shoulder yaw angle to target
        alpha = math.atan2((targ_y+direction_y*offset)-constants.Y_SHOU,
                             (targ_x+direction_x*offset)-constants.X_SHOU)
        h = math.sqrt(pow((targ_y+direction_y*offset)-constants.Y_SHOU, 2) +
                        pow((targ_x+direction_x*offset)-constants.X_SHOU, 2))
        l = constants.Y_SHOU - constants.HAND_Y_OFFSET
        beta = math.asin(l/h)

        # Move to pre move position, align shoulder yaw
        joint_goal = self._move_arm.get_current_joint_values()
        joint_goal[constants.J_DIST_PITCH] = 0
        joint_goal[constants.J_HAND_YAW] = 0
        joint_goal[constants.J_PROX_PITCH] = -math.pi/2
        joint_goal[constants.J_SHOU_PITCH] = math.pi/2
        joint_goal[constants.J_SHOU_YAW] = alpha + beta

        # If out of joint range, abort
        if (is_shou_yaw_goal_in_range(joint_goal) == False):
            return False

        joint_goal[constants.J_SCOOP_YAW] = 0

        self._move_arm.set_joint_value_target(joint_goal)

        _, plan_a, _, _ = self._move_arm.plan()
        if len(plan_a.joint_trajectory.points) == 0:  # If no plan found, abort
            return False

        # Once aligned to move goal and offset, place scoop tip at surface target offset
        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            plan_a)
        self._move_arm.set_start_state(cs)
        goal_pose.position.x = targ_x
        goal_pose.position.y = targ_y
        goal_pose.position.z = targ_z

        self._move_arm.set_pose_target(goal_pose)

        _, plan_b, _, _ = self._move_arm.plan()
        if len(plan_b.joint_trajectory.points) == 0:  # If no plan found, abort
            self._move_arm.clear_pose_targets()
            return False
        pre_guarded_move_traj = _cascade_plans(plan_a, plan_b)

        ### pre-guarded move ends here ###

        # Drive scoop tip along norm vector, distance is search_distance

        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            pre_guarded_move_traj)
        self._move_arm.set_start_state(cs)
        goal_pose.position.x = targ_x
        goal_pose.position.y = targ_y
        goal_pose.position.z = targ_z
        goal_pose.position.x -= direction_x*search_distance
        goal_pose.position.y -= direction_y*search_distance
        goal_pose.position.z -= direction_z*search_distance

        self._move_arm.set_pose_target(goal_pose)

        _, plan_c, _, _ = self._move_arm.plan()

        guarded_move_traj = _cascade_plans(pre_guarded_move_traj, plan_c)
        self._move_arm.set_planner_id("RRTConnect")
        '''
        estimated time ratio is the ratio between the time to complete first two parts of the plan
        to the the entire plan. It is used for ground detection only during the last part of the plan.
        It is set at 0.5 after several tests
        '''

        self._move_arm.clear_pose_targets()

        return guarded_move_traj

    def discard_sample(self, point, height):
        """
        :type self._move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
        :type robot: class 'moveit_commander.RobotCommander'
        :type moveit_fk: class moveit_msgs/GetPositionFK
        :type args: List[bool, float, float, float]
        """
        robot_state = self._robot.get_current_state()
        self._move_arm.set_start_state(robot_state)

        # after sample collect
        mypi = 3.14159
        d2r = mypi/180
        r2d = 180/mypi

        goal_pose = self._move_arm.get_current_pose().pose
        # position was found from rviz tool
        goal_pose.position.x = point.x
        goal_pose.position.y = point.y
        goal_pose.position.z = point.z + height

        r = -179
        p = -20
        y = -90

        q = quaternion_from_euler(r*d2r, p*d2r, y*d2r)
        goal_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self._move_arm.set_pose_target(goal_pose)
        self._move_arm.set_planner_id("RRTstar")
        _, plan_a, _, _ = self._move_arm.plan()

        if len(plan_a.joint_trajectory.points) == 0:  # If no plan found, abort
            self._move_arm.clear_pose_targets()
            return False

        # adding position constraint on the solution so that the tip does not
        # diverge to get to the solution.
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "l_scoop"
        pos_constraint.target_point_offset.x = 0.1
        pos_constraint.target_point_offset.y = 0.1
        # rotate scoop to discard sample at current location begin
        pos_constraint.target_point_offset.z = 0.1
        pos_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pos_constraint.weight = 1

        # using euler angles for own verification..

        r = +180
        p = 90
        y = -90
        q = quaternion_from_euler(r*d2r, p*d2r, y*d2r)

        cs, start_state, goal_pose = self.calculate_joint_state_end_pose_from_plan_arm(
            plan_a)

        self._move_arm.set_start_state(cs)

        goal_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self._move_arm.set_pose_target(goal_pose)
        _, plan_b, _, _ = self._move_arm.plan()

        self._move_arm.clear_pose_targets()

        # If no plan found, send the previous plan only
        if len(plan_b.joint_trajectory.points) == 0:
            return plan_a

        discard_sample_traj = _cascade_plans(plan_a, plan_b)
        self._move_arm.set_planner_id("RRTConnect")
        return discard_sample_traj

    def deliver_sample(self):
        """
        :type self._move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
        :type robot: class 'moveit_commander.RobotCommander'
        :type moveit_fk: class moveit_msgs/GetPositionFK
        """
        total_plan = None
        targets = [
            "arm_deliver_staging_1",
            "arm_deliver_staging_2",
            "arm_deliver_final"
        ]
        for t in targets:
            cs = self._robot.get_current_state() if total_plan is None else \
                self.calculate_joint_state_end_pose_from_plan_arm(total_plan)[0]
            self._move_arm.set_start_state(cs)
            self._move_arm.set_planner_id("RRTstar")
            goal = self._move_arm.get_named_target_values(t)
            self._move_arm.set_joint_value_target(goal)

            _, plan, _, _ = self._move_arm.plan()
            if len(plan.joint_trajectory.points) == 0:
                return False
            total_plan = plan if total_plan is None else \
                _cascade_plans(total_plan, plan)
        self._move_arm.set_planner_id("RRTConnect")
        return total_plan
