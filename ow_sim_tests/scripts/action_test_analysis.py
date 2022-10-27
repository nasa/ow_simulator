#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import glob
import os
import re
import subprocess
import time
import argparse
import yaml
from statistics import mean, stdev

import rospkg

PKG = 'ow_sim_tests'

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
  return ""

def run_test(test_name, real_time_update_rate, show_gzclient):
  test_cmd = [
    "rostest", "ow_sim_tests", test_name,
    "ignore_action_checks:=true", "gzclient:=%s" % str(show_gzclient)
  ]
  test_proc = subprocess.Popen(test_cmd)
  # attempt to change physics update rate until it succeeds
  gz_speedup_cmd = ["gz", "physics", "-u", str(real_time_update_rate)]
  while True:
    gz_speedup_proc = subprocess.run(gz_speedup_cmd, capture_output=True)
    if gz_speedup_proc.stderr == b'':
      break
    time.sleep(0.1)
  test_proc.wait()

def parse_latest_log(test_name, out_results):
  # NOTE: To see from where this syntax originates inspect print_action_start,
  #       print_action_complete, and print_arm_action_final in
  #       action_testing.py
  PATTERN = r"""===\ (?P<unit>\w+)\ goal\ sent\ ===        # start of an action
                (?:(?:.|\n)+?                              # fewest possible line skips
                ===\ \1\ completed\ with\ arm\ in\ position\ \((?P<final_x>-?\d+\.\d*),\ (?P<final_y>-?\d+\.\d*),\ (?P<final_z>-?\d+\.\d*)\)\ ===)? # final vector
                (?:.|\n)+?                                 # fewest possible line skips
                ===\ \1\ completed\ in\ (?P<duration>\d+\.\d*)s\ ===   # end of the same action
                """
  log_path = get_latest_test_log(test_name)
  if log_path == "":
    print("Log for test %s was not found." % test_name)
    return
  with open(log_path, 'r') as log:
    for match in re.finditer(PATTERN, log.read(), flags=re.VERBOSE):
      groups = match.groupdict()
      unit = groups.pop('unit')
      if not unit in out_results:
        out_results[unit] = dict()
      for param in groups.keys():
        if groups[param] == None:
          continue
        if not param in out_results[unit]:
          out_results[unit][param] = list()
        out_results[unit][param] += [ float(groups[param]) ]

def generate_statistics(results):
  STATS_FORMAT = {
    'mean': mean,
    'std': stdev,
    'samples': len
  }
  stats = dict()
  for unit in results.keys():
    stats[unit] = dict()
    for param in results[unit].keys():
      stats[unit][param] = dict()
      for stat in STATS_FORMAT:
        stats[unit][param][stat] = STATS_FORMAT[stat](results[unit][param])
  return stats

def save_to_yaml(data, output_path):
  # put into correct format for dump method
  with open(output_path, 'w') as output:
    formatted_data = {'test_action_statistics': data}
    yaml.dump(formatted_data, output)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description="""Run a rostest that calls the test_action or test_arm_action
    methods from action_testing.py multiple times to compute statistics on the
    final position and duration of each action called.""",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
  )
  parser.add_argument(
    'test_name', type=str, help="Name of rostest"
  )
  parser.add_argument(
    'repeats', type=int, help="Number of times rostest will be repeated"
  )
  parser.add_argument(
    '--real-time-update-rate', '-u', type=float, required=False, default=1200,
    help="Update rate each test simulation will run at.",
    dest='rt_update_rate'
  )
  parser.add_argument(
    '--output', '-o', type=str, required=False, default=None,
    help="YAML file results will be saved to."
  )
  parser.add_argument(
    '--show-gzclient', action='store_true', dest='show_gzclient',
    help="If true, gzclient will launch and show each test as it happens."
  )
  args = parser.parse_args()

  test_name_noext = os.path.splitext(args.test_name)[0]
  results = dict()
  for i in range(args.repeats):
    print("Running test %d of %d..." % (i, args.repeats))
    run_test(args.test_name, args.rt_update_rate, args.show_gzclient)
    parse_latest_log(test_name_noext, results)

  print("All %d runs of test %s completed." % (args.repeats, args.test_name))

  stats = generate_statistics(results)

  # TODO: Save results to the same file as statistics or make it a script option
  #       to output them to another file

  # SAVED: Helpful for debugging faulty tests in the sequence
  # print("results = ", results)

  output = args.output
  if output == None:
    pkg_path = rospkg.RosPack().get_path(PKG)
    output = os.path.join(pkg_path, 'config', test_name_noext + '.yaml')
  save_to_yaml(stats, output)
  print("Statistical results of test runs saved to %s" % output)
