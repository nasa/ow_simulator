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
    TOPIC_SAMPLE_DOCK_CONTACTS = "/ow_regolith/contacts/sample_dock"

    self.NO_CONTACTS_TIMEOUT = 3.0 # seconds

    self._action_name = name
    # Action Feedback/Result
    self._fdbk = ow_lander.msg.DockIngestSampleFeedback() # not used
    self._result = ow_lander.msg.DockIngestSampleResult()
    self._contacts_sub = rospy.Subscriber(
      TOPIC_SAMPLE_DOCK_CONTACTS,
      Contacts,
      self._on_dock_contacts
    )
    self._dock_contacts = list()
    self._ingesting = False
    self._last_contact_time = 0.0 # seconds
    # Construct action server and start
    self._server = actionlib.SimpleActionServer(
      self._action_name,
      ow_lander.msg.DockIngestSampleAction,
      execute_cb=self._on_ingest_sample,
      auto_start=False
    )
    self._server.start()

  def _on_dock_contacts(self, msg):
    self._dock_contacts = msg.link_names
    if self._ingesting:
      # remove contacts if ingestion is active
      self._remove_dock_contacts()
      self._update_last_contact_time()

  def _on_ingest_sample(self, goal):
    rospy.loginfo("%s: Ingesting sample in dock..." % self._action_name)
    self._ingesting = True
    self._result.sample_ingested = False
    # remove sample already contacting the dock
    self._remove_dock_contacts()
    # action timeout that gets reset everytime a new dock contact is detected
    self._update_last_contact_time()
    while rospy.get_time() - self._last_contact_time < self.NO_CONTACTS_TIMEOUT:
      rospy.sleep(0.2)

    # cease and clean-up ingest action
    self._ingesting = False
    self._server.set_succeeded(self._result)
    rospy.loginfo("%s: Succeeded" % self._action_name)

  def _update_last_contact_time(self):
    self._last_contact_time = rospy.get_time()

  def _remove_dock_contacts(self):
    REMOVE_REGOLITH_SERVICE = '/ow_regolith/remove_regolith'
    # do nothing if there are no contacts to remove
    if not self._dock_contacts:
      return

    # grab this value now in case its update functions gets called
    remove_request_count = len(self._dock_contacts)

    rospy.wait_for_service(REMOVE_REGOLITH_SERVICE)
    result = None
    try:
      service = rospy.ServiceProxy(REMOVE_REGOLITH_SERVICE, RemoveRegolith)
      result = service(self._dock_contacts)
    except rospy.ServiceException as e:
      rospy.logwarn("Service call failed: %s" % e)
      return

    # mark sample as ingested so long as one dock contact was removed
    if result.success or len(result.not_removed) < remove_request_count:
      self._result.sample_ingested = True

if __name__ == '__main__':
  SERVER_NAME = 'DockIngestSampleAction'
  rospy.init_node(SERVER_NAME)
  server = DockIngestSampleActionServer(SERVER_NAME)
  rospy.spin()
