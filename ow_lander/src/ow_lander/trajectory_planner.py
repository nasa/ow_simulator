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
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
# from shape_msgs.msg import SolidPrimitive

from ow_lander import math3d
from ow_lander import constants
from ow_lander.common import Singleton, is_shou_yaw_goal_in_range
from ow_lander.frame_transformer import FrameTransformer
from ow_lander.trajectory_sequence import TrajectorySequence, PlanningException

def _compute_workspace_shoulder_yaw(x, y):
    # Compute shoulder yaw angle to trench
    alpha = math.atan2(y - constants.Y_SHOU, x - constants.X_SHOU)
    h = math.sqrt(pow(y - constants.Y_SHOU, 2) + pow(x - constants.X_SHOU, 2))
    l = constants.Y_SHOU - constants.HAND_Y_OFFSET
    beta = math.asin(l / h)
    return alpha + beta

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

    def dig_linear(self, point, depth, length):
        """
        :type args: List[bool, float, int, float, float, float]
        """

        # TODO:
        #  1. implement normal parameter

        ## TODO: comment on what these calculations do
        alpha = math.atan2(constants.WRIST_SCOOP_PARAL,
                             constants.WRIST_SCOOP_PERP)
        distance_from_ground = constants.ROT_RADIUS * \
            (math.cos(alpha) - math.sin(alpha))
        # TODO: what point does this reference??
        center = Point(
            point.x,
            point.y,
            point.z + constants.SCOOP_HEIGHT - depth + distance_from_ground
        )
        sequence = TrajectorySequence('l_scoop', self._robot, self._move_arm)
        # place end-effector above trench position
        sequence.plan_to_joint_positions(
            j_shou_yaw = _compute_workspace_shoulder_yaw(point.x, point.y),
            j_shou_pitch = math.pi / 2,
            j_prox_pitch = -math.pi / 2,
            j_dist_pitch = 0.0,
            j_hand_yaw = math.pi/2.2,
            j_scoop_yaw = 0.0
        )
        # rotate hand so scoop bottom points down
        sequence.plan_to_joint_positions(j_hand_yaw = 0.0)
        # rotate scoop to face radially out from lander
        sequence.plan_to_joint_positions(j_scoop_yaw = math.pi / 2)
        # retract scoop back so its blades face terrain
        sequence.plan_to_joint_positions(j_dist_pitch = -math.pi / 2)
        # place scoop at the trench side nearest to the lander
        sequence.plan_to_position(center)
        # TODO: what is this doing?
        sequence.plan_to_joint_positions(j_dist_pitch = 2.0/9.0 * math.pi)
        # compute the far end of the trench
        far_trench_pose = sequence.get_final_pose()
        _, _, yaw = math3d.euler_from_quaternion(far_trench_pose.orientation)
        far_trench_pose.position.x += length * math.cos(yaw)
        far_trench_pose.position.y += length * math.sin(yaw)
        # move the scoop along a linear path to the end of the trench
        sequence.plan_linear_path_to_pose(far_trench_pose)
        # pitch scoop upward to maintain sample
        sequence.plan_to_joint_positions(j_dist_pitch = math.pi / 2)
        return sequence.merge()

    def grind(self, args):
        """
        :type args: List[bool, float, float, float, float, bool, float, bool]
        """
        PREGRIND_HEIGHT = 0.25

        point = Point(args.x_start, args.y_start, args.ground_position)
        depth = args.depth
        length = args.length
        parallel = args.parallel

        yaw = _compute_workspace_shoulder_yaw(point.x, point.y)
        if parallel:
            R = math.sqrt(point.x*point.x+point.y*point.y)
            # adjust trench to fit scoop circular motion
            dx = 0.04*R*math.sin(yaw)  # Center dig_circular in grind trench
            dy = 0.04*R*math.cos(yaw)
            # Move starting point back to avoid scoop-terrain collision
            point.x = 0.9*(point.x + dx)
            point.y = 0.9*(point.y - dy)
        else:
            dx = 5*length/8*math.sin(yaw)
            dy = 5*length/8*math.cos(yaw)
            # Move starting point back to avoid scoop-terrain collision
            point.x = 0.97*(point.x - dx)
            point.y = 0.97*(point.y + dy)

        sequence = TrajectorySequence(
            'l_grinder', self._robot, self._move_grinder)
        # place grinder above the start point
        pregrind_position = math3d.add(point, Vector3(0, 0, PREGRIND_HEIGHT))
        pregrind_pose = Pose(
            position = pregrind_position,
            orientation = Quaternion(
                0.70616885803, 0.0303977418722,
                -0.706723318474, 0.0307192507001
            )
        )
        sequence.plan_to_pose(pregrind_pose)
        # enter terrain
        trench_bottom = math3d.add(
            point, Vector3(0, 0, constants.GRINDER_OFFSET - depth)
        )
        sequence.plan_to_z(trench_bottom.z)
        # grinding ice forward
        yaw_offset = 0 if parallel else -math.pi / 2
        trench_segment = math3d.scalar_multiply(
            length,
            Vector3(math.cos(yaw + yaw_offset), math.sin(yaw + yaw_offset), 0)
        )
        sequence.plan_linear_translation(trench_segment)
        # grind sideways
        if parallel:
            # NOTE: small angle approximation?
            sequence.plan_to_joint_translations(j_shou_yaw = 0.08)
        else:
            sequence.plan_to_translation(math3d.scalar_multiply(0.08,
                Vector3(math.cos(yaw), math.sin(yaw), 0)))
        # grind backwards towards lander
        sequence.plan_linear_translation(
            math3d.scalar_multiply(-1, trench_segment))
        # exit terrain
        sequence.plan_to_z(pregrind_position.z)
        return sequence.merge()

    def guarded_move(self, point, normal, search_distance):
        self._move_arm.set_planner_id("RRTstar")
        try:
            sequence = TrajectorySequence(
                'l_scoop', self._robot, self._move_arm)
            # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
            targ_elevation = -0.2
            if (point.z+targ_elevation) == 0:
                offset = search_distance
            else:
                offset = (point.z*search_distance)/(point.z+targ_elevation)
            # Compute shoulder yaw angle to target
            alpha = math.atan2((point.y+normal.y*offset)-constants.Y_SHOU,
                                 (point.x+normal.x*offset)-constants.X_SHOU)
            h = math.sqrt(pow((point.y+normal.y*offset)-constants.Y_SHOU, 2) +
                            pow((point.x+normal.x*offset)-constants.X_SHOU, 2))
            l = constants.Y_SHOU - constants.HAND_Y_OFFSET
            beta = math.asin(l/h)
            # align scoop above target point
            sequence.plan_to_joint_positions(
                j_shou_yaw = alpha + beta,
                j_shou_pitch = math.pi / 2,
                j_prox_pitch = -math.pi / 2,
                j_dist_pitch = 0.0,
                j_hand_yaw = 0.0,
                j_scoop_yaw = 0.0
            )
            # once aligned to move goal and offset, place scoop tip at surface target offset
            sequence.plan_to_position(point)
            # drive scoop along anti-normal vector by the search distance
            sequence.plan_to_translation(
                math3d.scalar_multiply(-search_distance, normal)
            )
        except PlanningException as err:
            raise err
        finally:
            self._move_arm.set_planner_id("RRTConnect")
        return sequence.merge()

    def discard_sample(self, point, height):
        """
        :type self._move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
        :type robot: class 'moveit_commander.RobotCommander'
        :type moveit_fk: class moveit_msgs/GetPositionFK
        :type args: List[bool, float, float, float]
        """
        # robot_state = self._robot.get_current_state()
        # self._move_arm.set_start_state(robot_state)

        D2R = math.pi / 180

        self._move_arm.set_planner_id("RRTstar")

        try:
            sequence = TrajectorySequence(
                'l_scoop', self._robot, self._move_arm)
            # move scoop to the pose at the discard point that holds the sample
            held_euler = (
                -179 * D2R,
                -20  * D2R,
                -90  * D2R
            )
            held_pose = Pose(
                position = math3d.add(point, Point(0, 0, height)),
                orientation = math3d.quaternion_from_euler(*held_euler)
            )
            sequence.plan_to_pose(held_pose)

            # NOTE: This code does nothing since pos_constraint always remains a
            #       local variable. Saved in case the constraint is useful.
            # # adding position constraint on the solution so that the tip does not
            # # diverge to get to the solution.
            # pos_constraint = PositionConstraint()
            # pos_constraint.header.frame_id = "base_link"
            # pos_constraint.link_name = "l_scoop"
            # pos_constraint.target_point_offset.x = 0.1
            # pos_constraint.target_point_offset.y = 0.1
            # # rotate scoop to discard sample at current location begin
            # pos_constraint.target_point_offset.z = 0.1
            # pos_constraint.constraint_region.primitives.append(
            #     SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
            # pos_constraint.weight = 1

            # point front of scoop downward to discard sample
            dumped_euler = (
                180 * D2R,
                90  * D2R,
                -90 * D2R
            )
            dumped_quat = math3d.quaternion_from_euler(*dumped_euler)
            sequence.plan_to_orientation(dumped_quat)

        except PlanningException as err:
            raise err
        finally:
            # ensure planner type is reset even if an exception occurs
            self._move_arm.set_planner_id("RRTConnect")
        return sequence.merge()

    def deliver_sample(self):
        """
        :type self._move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
        :type robot: class 'moveit_commander.RobotCommander'
        :type moveit_fk: class moveit_msgs/GetPositionFK
        """
        targets = [
            "arm_deliver_staging_1",
            "arm_deliver_staging_2",
            "arm_deliver_final"
        ]
        self._move_arm.set_planner_id("RRTstar")
        try:
            sequence = TrajectorySequence(
                'l_scoop', self._robot, self._move_arm)
            for t in targets:
                sequence.plan_to_target(t)
        except PlanningException as err:
            raise err
        finally:
            self._move_arm.set_planner_id("RRTConnect")
        return sequence.merge()
