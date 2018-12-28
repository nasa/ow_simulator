#ifndef CUBEMAP_FILTER_H
#define CUBEMAP_FILTER_H


#include <OGRE/RenderSystems/GL/OgreGLSupport.h>
#include <OGRE/OgreTexture.h>


class CubemapFilter {
public:
  /**
   * @brief Constructor
   *
   * @param full_res Full resolution of input texture you will pass to filter().
   * Must be a power of two, such as 1024.
   * @param irradiance_res Resolution to use for ideal Lambertian irradiance.
   * Must be be a power of two smaller than full_res, such as 64.
   */
  CubemapFilter(const int full_res, const int irradiance_res);
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

  Ogre::GLSupport* m_GLSupport;

  Ogre::TexturePtr m_texture;

  // Cameras and viewports for capturing the scene
  Ogre::Camera* m_cameras[6];
  Ogre::Viewport* m_viewports[6];
};


#endif
