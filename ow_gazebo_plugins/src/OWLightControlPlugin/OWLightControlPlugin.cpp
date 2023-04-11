// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "OWLightControlPlugin.h"
#include <gazebo/rendering/rendering.hh>


namespace gazebo
{

GZ_REGISTER_VISUAL_PLUGIN(OWLightControlPlugin)

void OWLightControlPlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  m_nodeHandle = std::make_unique<ros::NodeHandle>("OWLightControlPlugin");

  m_lightName.push_back("lander::lander_lights_link::lander_light_light0");
  m_paramName.push_back("left_light");
  m_lightIntensity.push_back(1.0);
  m_lightName.push_back("lander::lander_lights_link::lander_light_light1");
  m_paramName.push_back("right_light");
  m_lightIntensity.push_back(1.0);

  for (size_t i = 0; i < m_paramName.size(); i++)
  {
    m_nodeHandle->setParam(m_paramName[i], m_lightIntensity[i]);
  }

  mPreRenderConnection = gazebo::event::Events::ConnectPreRender(
    boost::bind(&OWLightControlPlugin::onPreRender, this));
}

void OWLightControlPlugin::onPreRender()
{
  double intensity;
  gazebo::rendering::ScenePtr scene;

  for (size_t i = 0; i < m_paramName.size(); i++) {
    m_nodeHandle->getParamCached(m_paramName[i], intensity);

    // Intensity is a scale factor ranging from off to full power,
    // so we clamp it in a plausible way.
    intensity = std::clamp(intensity, 0.0, 1.0);

    if (m_lightIntensity[i] != intensity)
    {
      m_lightIntensity[i] = intensity;

      if (!scene) {
        scene = gazebo::rendering::get_scene();
        if (!scene || !scene->Initialized()) {
          gzwarn << "Gazebo scene not initialized." << std::endl;
          return;
        }

        // Helper code for outputting light names
        /*uint32_t lightCnt = scene->LightCount();
        for (uint32_t i = 0; i < lightCnt; ++i) {
          gzmsg << scene->LightByIndex(i)->Name() << std::endl;
        }*/
      }

      gazebo::rendering::LightPtr light = scene->LightByName(m_lightName[i]);
      if (!light) {
        gzwarn << "Found no light named " << m_lightName[i] << std::endl;
        continue;
      }

      ignition::math::Color color = ignition::math::Color(intensity, intensity, intensity);
      light->SetDiffuseColor(color);
    }
  }
}

}
