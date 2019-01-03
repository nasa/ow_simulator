#ifndef CUBEMAP_FILTER_H
#define CUBEMAP_FILTER_H


#include <OGRE/RenderSystems/GL/OgreGLSupport.h>
#include <OGRE/OgreTexture.h>


class CubemapFilter {
public:
  /**
   * @brief Constructor
   *
   * @param unique_index A unique index to use in global Ogre resource names.
   * @param full_res Full resolution of input texture you will pass to filter().
   * Must be a power of two, such as 1024.
   */
  CubemapFilter(const int unique_index, const Ogre::String& source_cubemap_name);
  ~CubemapFilter();

  void render();

  Ogre::TexturePtr getTexture() { return m_texture; }

private:
  void makeVertexProgram(const Ogre::String& name);
  void makeFragmentProgram(const Ogre::String& name);

  void makeRegularHemisphereSamples(const double min_theta, std::vector<Ogre::Vector3>& v);

  /**
   * @brief Poisson-disc sample generation adapted to a hemisphere
   *
   * Poisson-disc sampling produces points that are tightly-packed, but no closer to each other
   * than a specified minimum distance. This is adapted to a unit hemisphere by using a minimum
   * theta angle instead of a distance.
   *
   * @param min_theta Minimum angle between samples.
   * @param v Storage for sample points on surface of hemisphere.
   */
  void makePoissonHemisphereSamples(const double min_theta, std::vector<Ogre::Vector3>& v);

  Ogre::GLSupport* m_GL_support;

  // A unique index to use in "global" Ogre resources, such as texture, shader, and camera names
  int m_unique_index;

  Ogre::String m_source_cubemap_name;

  Ogre::TexturePtr m_texture;

  // Cameras and viewports for capturing the scene
  Ogre::Camera* m_cameras[6];
  Ogre::Viewport* m_viewports[6];
};


#endif
