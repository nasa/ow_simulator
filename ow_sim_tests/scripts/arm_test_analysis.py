#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import glob
import os
import re
import subprocess
import time
import pprint
import argparse

def get_ros_log_path():
  ENV_ROS_LOG_DIR = 'ROS_LOG_DIR'
  ENV_ROS_HOME    = 'ROS_HOME'
  ENV_HOME        = 'HOME'
  # logic for finding log directory location derived from http://wiki.ros.org/ROS/EnvironmentVariables
  if ENV_ROS_LOG_DIR in os.environ:
    return os.environ[ENV_ROS_LOG_DIR]
  elif ENV_ROS_HOME in os.environ:
    return os.path.join(os.environ[ENV_ROS_HOME], 'log')
  else:
    return os.path.join(os.environ[ENV_HOME], '.ros', 'log')

def get_latest_test_log(test_name):
  ros_log = get_ros_log_path()
  file_pattern = '%s-*.log' % test_name
  paths = glob.glob(os.path.join(ros_log, 'latest', file_pattern))
  for p in paths:
    if 'stdout' not in p:
      return p
  # log not found
  return ''

def process_latest_log(test_name):
  PATTERN = r"""===(\w+)\ action\ goal\ sent===                                    # start of an action
                (?:.|\n)+?                                                         # fewest possible line skips
                final\ position\ =\ \((-?\d+\.\d*),\ (-?\d+\.\d*),\ (-?\d+\.\d*)\) # final vector
                (?:.|\n)+?                                                         # fewest possible line skips
                ===\1\ action\ completed\ in\ (\d+\.\d*)s===                       # end of the same action
                """
  with open(get_latest_test_log(test_name), 'r') as log:
    final_positions = dict()
    for match in re.finditer(PATTERN, log.read(), flags=re.VERBOSE):
      final_positions[match[1]] = {
        'final_x': float(match[2]),
        'final_y': float(match[3]),
        'final_z': float(match[4]),
        'duration': float(match[5])
      }
  return final_positions

def run_test(test_name, real_time_update_rate):
  test_cmd = ["rostest", "ow_sim_tests", test_name, "gzclient:=false"]
  test_proc = subprocess.Popen(test_cmd)
  # attempt to change physics update rate until it succeeds
  gz_speedup_cmd = ["gz", "physics", "-u", str(real_time_update_rate)]
  while True:
    gz_speedup_proc = subprocess.run(gz_speedup_cmd, capture_output=True)
    if gz_speedup_proc.stderr == b'':
      break
    time.sleep(0.1)
  test_proc.wait()

def average_results(results):
  summations = dict()
  occurrences = dict()
  for test in results:
    for action in test.keys():
      if action not in summations:
        summations[action] = dict()
        occurrences[action] = 1
      else:
        occurrences[action] += 1
      for value in test[action].keys():
        if value not in summations[action]:
          summations[action][value] = test[action][value]
        else:
          summations[action][value] += test[action][value]

  averages = dict()
  for action in summations.keys():
    averages[action] = dict()
    for value in summations[action].keys():
      averages[action][value] = float(summations[action][value]) / occurrences[action]

  return averages

def pretty_dictionary_print(d, indent=0):
  for key, value in d.items():
    print('  ' * indent + str(key))
    if isinstance(value, dict):
      pretty_dictionary_print(value, indent+1)
    else:
      print('  ' * (indent+1) + str(value))

if __name__ == '__main__':

  parser = argparse.ArgumentParser(
    description="""Run a rostest that calls the test_action method from
    common_test_methods.py on arm actions multiple times compute the average
    final position and duration.""",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
  )
  parser.add_argument(
    'test_name', type=str, help="Name of rostest that will be ran"
  )
  parser.add_argument(
    'repeats', type=int, help="Number of times rostest will run"
  )
  parser.add_argument(
    '--real_time_update_rate', '-u', type=float, required=False, default=1200,
    help="Rate each simulation will be ran at."
  )
  args = parser.parse_args()

  results = list()
  for i in range(args.repeats):
    print("Running test %d of %d..." % (i, args.repeats))
    run_test(args.test_name, args.real_time_update_rate)
    results.append(process_latest_log(args.test_name.rsplit('.')[0]))


  # TEST INPUT
  # results = [
  #   {
  #     "dig_linear": {
  #       "final_x"  : 20,
  #       "final_y"  : 0,
  #       "final_z"  : 60,
  #       "duration" : 200,
  #     },
  #     "grind": {
  #       "final_x"  : 2,
  #       "final_y"  : 5,
  #       "final_z"  : 8,
  #       "duration" : 20
  #     }
  #   },
  #   {
  #     "dig_linear": {
  #       "final_x"  : 10,
  #       "final_y"  : 50,
  #       "final_z"  : 20,
  #       "duration" : 100,
  #     }
  #     ,
  #     "grind": {
  #       "final_x"  : 6,
  #       "final_y"  : 10,
  #       "final_z"  : 4,
  #       "duration" : 10
  #     }
  #   }
  # ]
  # expected results
  # dig_linear: 15, 25, 40, 150
  # grind     : 4, 7.5, 6, 15


  print("All %d runs of test %s completed." % (ARG2, ARG1))
  print("Here are the results:")

  pretty_dictionary_print(average_results(results))
