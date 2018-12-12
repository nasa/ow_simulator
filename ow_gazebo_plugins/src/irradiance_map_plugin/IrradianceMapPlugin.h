/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef IRRADIANCE_MAP_PLUGIN_H
#define IRRADIANCE_MAP_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreTexture.h>

namespace gazebo {

class IrradianceMapPlugin : public VisualPlugin
{
public:
  IrradianceMapPlugin();
  ~IrradianceMapPlugin();

  virtual void Load(rendering::VisualPtr visual, sdf::ElementPtr element);

  bool initialize();

  void onUpdate();

private:
  // Connection to the update event
  event::ConnectionPtr mUpdateConnection;

  Ogre::TexturePtr m_texture;

  Ogre::Camera* m_cameras[6];
  Ogre::Viewport* m_viewports[6];
  Ogre::MaterialPtr m_material;
};

}

#endif // IRRADIANCE_MAP_PLUGIN_H

