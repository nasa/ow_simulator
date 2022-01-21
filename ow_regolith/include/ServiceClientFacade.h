// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef SERVICE_CLIENT_FACADE_H
#define SERVICE_CLIENT_FACADE_H

#include <memory>

#include <ros/ros.h>

namespace ow_regolith {

template <class T>
class ServiceClientFacade
{
public:
  ServiceClientFacade() : m_node_handle(nullptr)
  {
    // do nothing
  }

  bool connect(std::shared_ptr<ros::NodeHandle> &nh, std::string service_path,
               ros::Duration timeout, bool persistent)
  {
    m_node_handle = nh;
    m_persistent = persistent;
    m_client = m_node_handle->serviceClient<T>(service_path, m_persistent);
    if (!m_client.waitForExistence(timeout)) {
      ROS_ERROR("Timed out waiting for service %s to advertise",
                service_path.c_str());
      return false;
    }
    return true;
  }

  bool call(T &message)
  {
    if (!m_node_handle) {
      ROS_ERROR(
        "ServiceClientFacade called before being connected."
      );
      return false;
    }

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
    m_client = m_node_handle->serviceClient<T>(m_client.getService().c_str(),
                                               m_persistent);
    return m_client.exists();
  }

  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ros::ServiceClient m_client;

  bool m_persistent;

};

} // namespace ow_regolith

#endif //SERVICE_CLIENT_FACADE_H