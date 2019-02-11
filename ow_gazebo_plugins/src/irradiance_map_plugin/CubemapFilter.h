// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#ifndef CUBEMAP_FILTER_H
#define CUBEMAP_FILTER_H


#include <OGRE/RenderSystems/GL/OgreGLSupport.h>
#include <OGRE/OgreTexture.h>


/**
 * @brief The CubemapFilter class
 * Filter a source cubemap to produce an irradiance environment cubemap.
 */
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

  /**
   * @brief Render the final irradiance environment cubemap.
   */
  void render();

  /**
   * @brief Get the final irradiance environment cubemap.
   * @return Pointer to cubemap texture
   */
  Ogre::TexturePtr getTexture() { return m_texture; }

private:
  void makeVertexProgram(const Ogre::String& name);

  /**
   * @brief Create a fragment program that uses many samples from the source
   * cubemap to produce a final filtered cubemap.
   * @param Provide a unique fragment program name.
   */
  void makeFragmentProgram(const Ogre::String& name);

  /**
   * @brief Generate sample positions along regularly spaced latitude lines
   *
   * @param min_theta Minimum angle between samples.
   * @param out_p Storage for sample points on surface of hemisphere.
   */
  void makeRegularHemisphereSamples(const double min_theta, std::vector<Ogre::Vector3>& out_p);

  /**
   * @brief Poisson-disc sample position generation adapted to a hemisphere
   *
   * Poisson-disc sampling produces points that are tightly-packed, but no closer
   * to each other than a specified minimum distance. This is adapted to a unit
   * hemisphere by using a minimum theta angle instead of a distance.
   * Final irradiance maps produced with this method are more splotchy than ones
   * produced with makeRegularHemisphereSamples(), so it is not in use and has
   * not been optimized.
   *
   * @param min_theta Minimum angle between samples.
   * @param out_p Storage for sample points on surface of hemisphere.
   */
  void makePoissonHemisphereSamples(const double min_theta, std::vector<Ogre::Vector3>& out_p);

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
