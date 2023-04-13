// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OWLightControlPlugin_h
#define OWLightControlPlugin_h

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>


namespace gazebo {
  
class OWLightControlPlugin : public gazebo::VisualPlugin
{
public:
  OWLightControlPlugin() {}
  ~OWLightControlPlugin() {}

  virtual void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf);

protected:
  void onPreRender();

private:
  std::unique_ptr<ros::NodeHandle> m_nodeHandle;

  gazebo::event::ConnectionPtr mPreRenderConnection;

  std::vector<std::string> m_lightName;
  std::vector<std::string> m_paramName;
  std::vector<double> m_lightIntensity;
};

}

#endif // OWLightControlPlugin_h
