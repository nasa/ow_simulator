/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "IrradianceMapPlugin.h"

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/common/Image.hh>
#include <OGRE/OgreSceneManager.h>


using namespace std;
using namespace gazebo;
using namespace Ogre;

GZ_REGISTER_VISUAL_PLUGIN(IrradianceMapPlugin)

IrradianceMapPlugin::IrradianceMapPlugin() :
  VisualPlugin()
{
  m_texture.setNull();

  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
  }

  m_timer.Start();
}

IrradianceMapPlugin::~IrradianceMapPlugin()
{
}

void IrradianceMapPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr element)
{
  // Listen to the update event. This event is broadcast every sim iteration.
  this->mUpdateConnection = event::Events::ConnectPreRender(
    boost::bind(&IrradianceMapPlugin::onUpdate, this));

  string name = visual->GetMaterialName();
  try
  {
    m_material = MaterialManager::getSingleton().getByName(name);
  }
  catch(Exception &e)
  {
    gzwarn << "Unable to get Material \"" << name << "\". Object will retain its original cubemap.\n";
    return;
  }
}

bool IrradianceMapPlugin::initialize()
{
  if (!m_texture.isNull())
  {
    // Texture has already been created
    return true;
  }

  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << "IrradianceMapPlugin::initialize: scene pointer is NULL" << endl;
    return false;
  }

  const int size = 1024;
  const int numMipMaps = log2(size);
  m_texture = TextureManager::getSingleton().createManual(
        "SourceEnvironmentCubemap", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        TEX_TYPE_CUBE_MAP, size, size, 0/*numMipMaps*/, PF_FLOAT32_RGB,
        TU_DYNAMIC_WRITE_ONLY | TU_RENDERTARGET | TU_AUTOMIPMAP);

  for (int i = 0; i < 6; i++)
  {
    SceneManager* scene_manager = scene->GetManager();
    m_cameras[i] = scene_manager->createCamera("CameraCubeMap" + StringConverter::toString(i));  // Who cleans up this camera?
    m_cameras[i]->setFOVy(Radian(Math::PI / 2));
    m_cameras[i]->setAspectRatio(1);
    m_cameras[i]->setNearClipDistance(0.1);
    m_cameras[i]->setFarClipDistance(200000);
    m_cameras[i]->setPosition(0, 0, 3);

    // OpenGL cubemaps are arranged using RenderMan's left-handed coordinate
    // system resulting in the entire map being mirrored when rendered looking
    // out from the center like this, so we must somehow mirror the entire
    // cubemap to use it in the scene properly.
    // https://stackoverflow.com/questions/11685608/convention-of-faces-in-opengl-cubemapping
    // http://marcinignac.com/blog/pragmatic-pbr-hdr
    // This problem might be solved in Ogre3D 2.x, but I'm not certain.
    // https://bitbucket.org/sinbad/ogre/pull-requests/650/fixed-cubemap-reflections-so-there-are/activity
    // Since we're using Ogre3D 1.9, we will solve this problem by rendering
    // our cubemaps like below and then negating the y-component of cube lookups
    // in our shaders.
    switch(i)
    {
    case 0: // right
      m_cameras[i]->yaw( Radian( -Math::PI / 2) );
      m_cameras[i]->roll( Radian( -Math::PI / 2) );
      break;
    case 1: // left
      m_cameras[i]->yaw( Radian( Math::PI / 2) );
      m_cameras[i]->roll( Radian( Math::PI / 2) );
      break;
    case 2: // up
      m_cameras[i]->pitch( Radian( Math::PI ) );
      break;
    case 3: // down
      break;
    case 4: // front
      m_cameras[i]->pitch( Radian( Math::PI / 2 ) );
      break;
    case 5: // back
      m_cameras[i]->pitch( Radian( -Math::PI / 2 ) );
      m_cameras[i]->roll( Radian( Math::PI ) );
      break;
    }

    RenderTarget* renderTarget = m_texture->getBuffer(i)->getRenderTarget();
    m_viewports[i] = renderTarget->addViewport(m_cameras[i]);
    m_viewports[i]->setOverlaysEnabled(false);
    m_viewports[i]->setClearEveryFrame(true);
    m_viewports[i]->setBackgroundColour(ColourValue::Black);
  }

  m_cubemap_filter.reset(new CubemapFilter(1024, 64));

  gzlog << "IrradianceMapPlugin::initialize: complete." << endl;

  return true;
}

void IrradianceMapPlugin::onUpdate()
{
  // Only continue if a second has elapsed
  if (m_timer.GetElapsed().Double() < 2.0)
  {
    return;
  }
  m_timer.Reset();
  m_timer.Start();

  // Cannot do this in constructor or Load function because Heightmap is not yet available.
  if (!initialize())
  {
    return;
  }

  if (m_material.isNull())
  {
    return;
  }

  for (int i = 0; i < 6; i++)
  {
    m_cameras[i]->_renderScene(m_viewports[i], false);
  }

  // Ogre 1.9 does not appear to generate mipmaps for dynamic textures. And
  // trying to generate mipmaps here appears to generate them from the original
  // texture, not the one we just rendered.
  //GLTexture* gltex = static_cast<GLTexture*>(m_texture.getPointer());
  //glGenerateMipmapEXT(gltex->getGLTextureTarget());

  m_cubemap_filter->render();

  TextureUnitState* tus = m_material->getTechnique(0)->getPass(0)->getTextureUnitState("cubemap");
  //tus->setTexture(m_texture);
  tus->setTexture(m_cubemap_filter->getTexture());
}

