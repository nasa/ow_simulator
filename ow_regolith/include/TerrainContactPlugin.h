// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef TERRAIN_CONTACT_PLUGIN_H
#define TERRAIN_CONTACT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>

namespace ow_regolith {

  class TerrainContactPlugin : public gazebo::SensorPlugin
  {
  public:
    TerrainContactPlugin() : SensorPlugin() { };

    void Load(gazebo::sensors::SensorPtr _sensor,
              sdf::ElementPtr _sdf) override;

  private:
    void onUpdate();

    gazebo::sensors::ContactSensorPtr m_parent_sensor;
    gazebo::event::ConnectionPtr m_update_connection;

    std::unique_ptr<ros::NodeHandle> m_node_handle;
    ros::Publisher m_pub_terrain_collision;
  };

} // namespace ow_regolith

#endif // TERRAIN_CONTACT_PLUGIN_H
