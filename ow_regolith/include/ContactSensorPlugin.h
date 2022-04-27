// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef TERRAIN_CONTACT_PLUGIN_H
#define TERRAIN_CONTACT_PLUGIN_H

#include <string>
#include <set>
#include <memory>
#include <regex>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>

namespace ow_regolith {

// ContactSensorPlugin is a Gazebo sensor plugin that publishes the names of any
// links actively in contact with the collision object the sensor plugin has
// been attached to.
class ContactSensorPlugin : public gazebo::SensorPlugin
{
public:
  ContactSensorPlugin()
    : SensorPlugin(), m_links_in_contact(), m_report_only_set(false) { };
  ~ContactSensorPlugin() = default;

  ContactSensorPlugin(const ContactSensorPlugin&) = delete;
  ContactSensorPlugin& operator=(const ContactSensorPlugin&) = delete;

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  void onUpdate();

  std::set<std::string> m_links_in_contact;

  bool m_report_only_set;
  std::regex m_report_only;

  gazebo::sensors::ContactSensorPtr m_parent_sensor;
  gazebo::event::ConnectionPtr m_update_connection;

  std::unique_ptr<ros::NodeHandle> m_node_handle;
  ros::Publisher m_pub_contacts;
};

} // namespace ow_regolith

#endif // TERRAIN_CONTACT_PLUGIN_H
