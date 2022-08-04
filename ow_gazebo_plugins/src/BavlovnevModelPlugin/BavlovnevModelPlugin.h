// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <ros/ros.h>
#include <tf/tf.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ow_dynamic_terrain/modified_terrain_diff.h>


namespace gazebo
{
    // This plugin is added to a robot description and applies forces 
    // calculate from Bavlovnev force model
class BavlovnevModelPlugin : public gazebo::ModelPlugin
{
public:
  
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  
private:

  ros::Subscriber m_sub_mod_diff_visual;

  void OnUpdate();

  double getParameterA(double x, double ifa, double efa);
  
  void getForces(double VERTICAL_CUT_DEPTH);
  
  void onModDiffVisualMsg(const ow_dynamic_terrain::modified_terrain_diff::ConstPtr& msg);
  
  physics::LinkPtr m_link;

  double m_horizontal_force;
  double m_vertical_force;

  std::unique_ptr<ros::NodeHandle> m_node_handle;
  event::ConnectionPtr m_updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BavlovnevModelPlugin)
}