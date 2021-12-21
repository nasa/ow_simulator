#include "OWLensFlareSensorPlugin.h"


using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(OWLensFlareSensorPlugin)

OWLensFlareSensorPlugin::OWLensFlareSensorPlugin()
{
  m_node_handle.reset(new ros::NodeHandle("ow_lens_flare_sensor_plugin"));

  // Allow lens flare scale to be set in real-time
  std::string scale_topic("/gazebo/plugins/ow_lens_flare/scale");
  m_scale_sub = m_node_handle->subscribe(scale_topic, 1,
    &OWLensFlareSensorPlugin::scaleCb, this);

  // Allow lens flare color to be set in real-time
  std::string color_topic("/gazebo/plugins/ow_lens_flare/color");
  m_color_sub = m_node_handle->subscribe(color_topic, 1,
    &OWLensFlareSensorPlugin::colorCb, this);
}

OWLensFlareSensorPlugin::~OWLensFlareSensorPlugin()
{
}

void OWLensFlareSensorPlugin::scaleCb(const std_msgs::Float64::ConstPtr& msg)
{
  SetScale(msg->data);
}

void OWLensFlareSensorPlugin::colorCb(const geometry_msgs::Vector3::ConstPtr& msg)
{
  SetColor(ignition::math::Vector3d(msg->x, msg->y, msg->z));
}
