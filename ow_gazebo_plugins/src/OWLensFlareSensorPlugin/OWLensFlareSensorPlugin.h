#ifndef OS_LENS_FLARE_SENSOR_PLUGIN_H
#define OS_LENS_FLARE_SENSOR_PLUGIN_H

#include <gazebo/plugins/LensFlareSensorPlugin.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{

/// \brief Plugin that adds lens flare effect to a camera or multicamera
/// sensor
/// The plugin has the following optional parameter:
/// <scale>       Scale of lens flare. Must be greater than 0
/// <color>       Color of lens flare.
/// The scale and color can be set by ROS topics
class OWLensFlareSensorPlugin : public LensFlareSensorPlugin
{
public:
  /// \brief Constructor.
  OWLensFlareSensorPlugin();

  /// \brief Destructor.
  ~OWLensFlareSensorPlugin();

private:
  void scaleCb(const std_msgs::Float64::ConstPtr& msg);

  void colorCb(const geometry_msgs::Vector3::ConstPtr& msg);

  std::unique_ptr<ros::NodeHandle> m_node_handle;

  /// \brief Subscriber for setting lens flare scale.
  ros::Subscriber m_scale_sub;

  /// \brief Subscriber for setting lens flare color.
  ros::Subscriber m_color_sub;
};

}

#endif
