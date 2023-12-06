// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.
#ifndef BALOVNEV_MODEL_PLUGIN_H
#define BALOVNEV_MODEL_PLUGIN_H 

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ow_dynamic_terrain/modified_terrain_diff.h>
#include <ow_dynamic_terrain/scoop_dig_phase.h>

#include <MovingMaxFilter.h>

namespace gazebo
{
// This plugin is added to a robot description and applies forces calculated 
// from Balovnev force model
class BalovnevModelPlugin : public gazebo::ModelPlugin
{
public:
  BalovnevModelPlugin() = default;
  BalovnevModelPlugin (const BalovnevModelPlugin&) = default;
  BalovnevModelPlugin& operator= (const BalovnevModelPlugin&) = default;
  ~BalovnevModelPlugin() = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  
private:

  void onUpdate();

  double getParameterA(double x, double ifa, double efa) const;

  void computeForces(double vertical_cut_depth);

  void publishForces();

  void resetVerticalForces();
  void resetHorizontalForces();
  void resetForces();

  void resetDepth();

  void onModDiffVisualMsg(
    const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg);

  void onDigPhaseMsg(const ow_dynamic_terrain::scoop_dig_phase::ConstPtr &msg);

  std::unique_ptr<ros::NodeHandle> m_node_handle;

  ros::Subscriber m_sub_mod_diff_visual;
  ros::Subscriber m_sub_dig_phase;

  // true if scoop is performing a dig motion
  bool m_digging;

  ros::Publisher m_pub_horizontal_force;
  ros::Publisher m_pub_vertical_force;
  
  physics::LinkPtr m_link;

  event::ConnectionPtr m_updateConnection;  

  double m_horizontal_force, m_vertical_force;

  std::unique_ptr<MovingMaxFilter> m_moving_max_depth;

};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BalovnevModelPlugin)
}

#endif // BALOVNEV_MODEL_PLUGIN_H
