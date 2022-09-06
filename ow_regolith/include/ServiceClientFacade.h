// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef SERVICE_CLIENT_FACADE_H
#define SERVICE_CLIENT_FACADE_H

#include <memory>

#include <ros/ros.h>

namespace ow_regolith {

// ServiceClientFacade wraps ros::ServiceClient and implements automatic
// reconnection for when persistent clients drop their connection.
class ServiceClientFacade
{
public:
  ServiceClientFacade() : m_node_handle(nullptr), m_persistent(false) { }
  ~ServiceClientFacade() = default;

  ServiceClientFacade(const ServiceClientFacade&) = delete;
  ServiceClientFacade& operator=(const ServiceClientFacade&) = delete;

  // Connect this instance to a certain ROS service and block for the given
  // timeout until the service is advertised.
  // NOTE: This method's name is a bit of a misgnomer because of the differences
  //       between persistent and non-persistent ros::ServiceClients.
  //       * A ServiceClientFacade "connected" persistently may reconnect when
  //       it is called, but only ever if it's persistent connection has been
  //       dropped.
  //       * A ServiceClientFacade "connected" non-persistently will reconnect
  //       each time it is called. So in the case of a non-persistent
  //       ServiceClientFacade the connect method is really just a wrapper for
  //       ros::NodeHandle::serviceClient followed by a block until the
  //       service has advertised.
  //       For best performance, a ServiceClientFacade should be connected
  //       persistently if it is expected to be called frequently.
  template <typename T>
  bool connect(std::shared_ptr<ros::NodeHandle> nh,
               const std::string &service_path,
               ros::Duration timeout, bool persistent);

  // Call the ROS service this instance is connected to. See the NOTE above the
  // connect method to understand how this method works differently for
  // persistent and non-persistent clients.
  template <typename T>
  bool call(T &message);

  bool isConnected()
  {
    return m_client.exists() && m_client.isValid();
  };

private:

  template <typename T>
  bool attemptReconnect();

  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ros::ServiceClient m_client;

  bool m_persistent;

};

template <typename T>
bool ServiceClientFacade::connect(std::shared_ptr<ros::NodeHandle> nh,
                                  const std::string &service_path,
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
};

template <typename T>
bool ServiceClientFacade::call(T &message)
{
  if (!m_node_handle) {
    ROS_ERROR(
      "ServiceClientFacade called before being connected."
    );
    return false;
  }

  if (!m_client.isValid() && !attemptReconnect<T>()) {
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
};

template <typename T>
bool ServiceClientFacade::attemptReconnect()
{
  m_client = m_node_handle->serviceClient<T>(m_client.getService().c_str(), m_persistent);
  return isConnected();
};


} // namespace ow_regolith

#endif //SERVICE_CLIENT_FACADE_H
