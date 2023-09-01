#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import os
import actionlib
import pandas as pd
import tf2_ros
import roslib
import unittest
import owl_msgs.msg
import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose, Quaternion
from action_testing import *
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean

PKG = 'ow_sim_tests'
TEST_NAME = 'arm_consistency'
common_dir = os.path.expanduser('~/Documents')
fig_3d_path = os.path.join(common_dir, '3D_trajectories_RRTConnect_testing.png')
result_path = os.path.join(common_dir, 'test_results_RRTConnect_testing.txt')
data_path = os.path.join(common_dir, 'all_trajectories_RRTConnect_testing.csv.txt')
roslib.load_manifest(PKG)

def _dtw_helper(trajectories, runs):
  baseline_index = 0

  threshold = 10

  while not rospy.is_shutdown():
    baseline_trajectory = trajectories[baseline_index]
    outlier_indices = []
    for i, traj in enumerate(trajectories[1:]):
      distance, _ = fastdtw(baseline_trajectory, traj, dist = euclidean)
      if distance > threshold:
        outlier_indices.append(i+1)
    if len(outlier_indices) >= (runs / 2):
      baseline_index += 1
    else:
      break
      
  return outlier_indices, threshold

def _plot_helper(trajectories):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  
  for i, traj in enumerate(trajectories):
    traj = np.array(traj)
    xs, ys, zs = traj[:, 0], traj[:, 1], traj[:, 2]
    ax.plot(xs, ys, zs)

  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.title('3D Trajectories')
  plt.legend()
  fig.savefig(fig_3d_path)
  
class TestArmConsistency(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    rospy.init_node("arm_unstow_test")

    set_ignore_action_checks(True)
    # proceed with test only when ros clock has been initialized
    
    random.seed(42)
    cls.tfBuffer = tf2_ros.Buffer()
    cls.listener = tf2_ros.TransformListener(cls.tfBuffer)
    cls.client_unstow = actionlib.SimpleActionClient('ArmUnstow', owl_msgs.msg.ArmUnstowAction)
    cls.client_stow = actionlib.SimpleActionClient('ArmStow', owl_msgs.msg.ArmStowAction)
    cls.client_arm_move_cartesian = actionlib.SimpleActionClient('ArmMoveCartesian', 
                                                                 owl_msgs.msg.ArmMoveCartesianAction)
    cls.client_unstow.wait_for_server(timeout = rospy.Duration(50))
    cls.client_stow.wait_for_server(timeout = rospy.Duration(50))
    cls.client_arm_move_cartesian.wait_for_server(timeout=rospy.Duration(50))

    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def test_01_multiple_runs(self):
    num_runs = 10
    trajectories = []
    goal_unstow = owl_msgs.msg.ArmUnstowGoal()
    goal_arm_move = owl_msgs.msg.ArmMoveCartesianGoal(
        frame = 0,
        relative = False,
        pose= Pose(
          position = Point(1.166, 0.356, 0.588),
          orientation = Quaternion(-0.254, 0.727, -0.456, 0.446)
        )
    )
    self.client_unstow.send_goal(goal_unstow)
    self.client_unstow.wait_for_result()
    for i in range(num_runs):
      rospy.sleep(0.1)

      self.client_arm_move_cartesian.send_goal(goal_arm_move)

      trajectory = []
      rate = rospy.Rate(10.0)
      start_time = rospy.Time.now()

      while not self.client_arm_move_cartesian.wait_for_result(rospy.Duration(0.1)):
        try:
          trans = self.tfBuffer.lookup_transform('base_link', 'l_scoop_tip',rospy.Time(0))
          print('x location', trans.transform.translation.x)
          trajectory.append([ trans.transform.translation.x, 
                              trans.transform.translation.y, 
                              trans.transform.translation.z])
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
              print('Could not get transform')
              continue
        rate.sleep()

      rospy.sleep(0.1)
      self.client_unstow.send_goal(goal_unstow)
      self.client_unstow.wait_for_result()
  
      trajectories.append(trajectory)

    # Using Dynamic Time Warping (DTW) to find the outlier trajectories
    outlier_indices, threshold = _dtw_helper(trajectories, num_runs)
    total_outliers = len(outlier_indices)
    try:
      with open(result_path, 'w') as f:
        f.write("Total number of runs: {}\n".format(num_runs))
        f.write("Outliers found at runs: {}\n".format(outlier_indices))
        f.write("Total number of outliers: {}\n".format(total_outliers))
        f.write("Threshold: {}\n".format(threshold))
    except Exception as e:
      rospy.logerr("Failed to save test results: {}".format(e))
    dfs = []
    for i, traj in enumerate(trajectories):
      df = pd.DataFrame(traj, columns=[f'x_run{i}', f'y_run{i}', f'z_run{i}'])
      dfs.append(df)
    
    result = pd.concat(dfs, axis = 1)

    try:
      result.to_csv(data_path)
      print("File saved successfully")
    except Exception as e:
      print("Failed to save file", e)

    _plot_helper(trajectories)

    self.assertTrue(len(trajectories) == num_runs)
if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, TEST_NAME, TestArmConsistency)
