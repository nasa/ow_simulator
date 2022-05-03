#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

import ow_lander.msg

from ow_regolith.srv import RemoveRegolith
from ow_regolith.msg import Contacts

class DockIngestSampleActionServer(object):

  def __init__(self, name):
    self._action_name = name
    # Action Feedback/Result
    self._fdbk = ow_lander.msg.DockIngestSampleFeedback() # not used
    self._result = ow_lander.msg.DockIngestSampleResult()
    self._contacts_sub = rospy.Subscriber('/ow_regolith/contacts/sample_dock',
      Contacts, self._on_dock_contacts)
    self._collected_sample = list()
    self._server = actionlib.SimpleActionServer(
      self._action_name,
      ow_lander.msg.DockIngestSampleAction,
      execute_cb=self._on_ingest_sample,
      auto_start=False
    )
    self._server.start()

  def _on_dock_contacts(self, msg):
    self._collected_sample = msg.link_names

  def _call_remove_regolith_service(self, link_names):
    REMOVE_REGOLITH_SERVICE = '/ow_regolith/remove_regolith'
    rospy.wait_for_service(REMOVE_REGOLITH_SERVICE)
    try:
      result = None
      service = rospy.ServiceProxy(REMOVE_REGOLITH_SERVICE, RemoveRegolith)
      result = service(link_names)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
    finally:
      return result

  def _on_ingest_sample(self, goal):
    if not self._collected_sample:
      self._result.sample_present = False
      self._server.set_succeeded(self._result)
      return

    srv_result = self._call_remove_regolith_service(self._collected_sample)

    if srv_result.success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._result.sample_present = True
      self._server.set_succeeded(self._result)
    else:
      not_removed_str = not_removed[0]
      for ln in srv_result.not_removed[1:]:
        not_removed_str += ', %s' % ln
      rospy.logwarn(
        '%s: Remove regolith service failed to remove all links in sample dock.'
        'The following links were not removed: %s'
        % (self._action_name, not_removed_str))

if __name__ == '__main__':
  rospy.init_node('DockIngestSampleAction')
  server = DockIngestSampleActionServer(rospy.get_name())
  rospy.spin()
