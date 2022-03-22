// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef TERRAIN_CONTACT_PLUGIN_H
#define TERRAIN_CONTACT_PLUGIN_H

#include <string>
#include <set>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>

namespace ow_regolith {

  class ContactSensorPlugin : public gazebo::SensorPlugin
  {
  public:
    ContactSensorPlugin() : SensorPlugin(), m_links_in_contact() { };

    void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

  private:
    void onUpdate();

    std::set<std::string> m_links_in_contact;

    gazebo::sensors::ContactSensorPtr m_parent_sensor;
    gazebo::event::ConnectionPtr m_update_connection;

    std::unique_ptr<ros::NodeHandle> m_node_handle;
    ros::Publisher m_pub_contacts;
  };

} // namespace ow_regolith

#endif // TERRAIN_CONTACT_PLUGIN_H
