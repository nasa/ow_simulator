// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef PERSISTENT_SERVICE_CLIENT_H
#define PERSISTENT_SERVICE_CLIENT_H

#include <ros/ros.h>

namespace ow_regolith {

template <class T>
class PersistentServiceClient
{
public:
  PersistentServiceClient(ros::NodeHandle* nh, bool persistent = true)
    : m_node_handle(nh), m_persistent(persistent)
  {
    // do nothing
  }

  bool connect(std::string service_path, int timeout = 5.0)
  {
    m_client = m_node_handle->serviceClient<T>(service_path, m_persistent);
    if (!m_client.waitForExistence(ros::Duration(timeout))) {
      ROS_ERROR("Timed out waiting for service %s to advertise",
                service_path.c_str());
      return false;
    }
    return true;
  }

  bool call(T &message)
  {
    if (!m_client.isValid() && !attempt_reconnect()) {
      ROS_ERROR(
        "Connection to service %s has been lost and reconnection failed",
        m_client.getService().c_str()
      );
      return false;
    }

    if (!m_client.call(message)) {
      ROS_ERROR("Failed to call service %s", m_client.getService().c_str());
      return false;
    }
    return true;
  }

private: 
  bool attempt_reconnect()
  {
    constexpr auto RECONNECT_TIMEOUT = 0.5;
    return connect(m_client.getService().c_str(), RECONNECT_TIMEOUT);
  }

  bool m_persistent;

  ros::ServiceClient m_client;

  std::unique_ptr<ros::NodeHandle> m_node_handle;

};

}

#endif //PERSISTENT_SERVICE_CLIENT_H