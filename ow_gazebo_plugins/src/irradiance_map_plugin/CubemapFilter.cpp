// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include "CubemapFilter.h"
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreGpuProgramManager.h>
#include <OGRE/OgreHighLevelGpuProgramManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreSceneManager.h>


using namespace std;
using namespace Ogre;


CubemapFilter::CubemapFilter(const int unique_index, const String& source_cubemap_name) :
  m_unique_index(unique_index),
  m_source_cubemap_name(source_cubemap_name)
{
  //////////////////////////////////////////////////////////////////////////////
  // Create a simple scene of a cube with our dynamically rendered cubemap applied.
  // 6 cameras and a shader will be used to sample that cubemap to create the
  // new filtered cubemap.
  //////////////////////////////////////////////////////////////////////////////

  const String vp_name("make_irradiance_map_vert" + StringConverter::toString(m_unique_index));
  const String fp_name("make_irradiance_map_frag" + StringConverter::toString(m_unique_index));
  makeVertexProgram(vp_name);
  makeFragmentProgram(fp_name);

  // Create a simple material using
  String source_material_name("SourceEnvironmentMaterial" + StringConverter::toString(m_unique_index));
  MaterialPtr material = MaterialManager::getSingleton().create(
    source_material_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->getTechnique(0)->getPass(0)->createTextureUnitState(m_source_cubemap_name);
  material->getTechnique(0)->getPass(0)->setVertexProgram(vp_name);
  material->getTechnique(0)->getPass(0)->setFragmentProgram(fp_name);

  SceneManager* scene_manager = Root::getSingletonPtr()->createSceneManager(ST_GENERIC);

  // get OpenGL support
  GLRenderSystem* gl_render_system = dynamic_cast<GLRenderSystem*>(Root::getSingletonPtr()->getRenderSystem());
  if (gl_render_system != NULL)
  {
    m_GL_support = gl_render_system->getGLSupportRef();
  }

  // Make a cube with inside surfaces visible
  ManualObject* cube = scene_manager->createManualObject("cube");
  cube->begin(source_material_name, RenderOperation::OT_TRIANGLE_LIST);
  cube->position(-1, -1, -1);
  cube->position(1, -1, -1);
  cube->position(-1, 1, -1);
  cube->position(1, 1, -1);
  cube->position(-1, -1, 1);
  cube->position(1, -1, 1);
  cube->position(-1, 1, 1);
  cube->position(1, 1, 1);
  cube->quad(0, 1, 3, 2);
  cube->quad(4, 6, 7, 5);
  cube->quad(0, 2, 6, 4);
  cube->quad(1, 5, 7, 3);
  cube->quad(0, 4, 5, 1);
  cube->quad(2, 3, 7, 6);
  cube->end();

  scene_manager->getRootSceneNode()->attachObject(cube);

  // Just draw something! Anything!!
  //ResourceGroupManager::getSingleton().addResourceLocation("/usr/local/home/twelsh/install/share/OGRE/Media/models", "FileSystem");
  //ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
  //MovableObject* head = scene_manager->createEntity("head", "ogrehead.mesh");
  //scene_manager->getRootSceneNode()->attachObject(head);

  //////////////////////////////////////////////////////////////////////////////
  // Now setup the cameras for rendering the filtered cubemap
  //////////////////////////////////////////////////////////////////////////////

  String irradiance_cubemap_name("IrradianceEnvironmentCubemap" + StringConverter::toString(m_unique_index));
  const int size = 32;
  const int numMipMaps = log2(size);
  m_texture = TextureManager::getSingleton().createManual(
        irradiance_cubemap_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        TEX_TYPE_CUBE_MAP, size, size, 0/*numMipMaps*/, PF_FLOAT32_RGB,
        TU_DYNAMIC_WRITE_ONLY | TU_RENDERTARGET | TU_AUTOMIPMAP);

  for (int i = 0; i < 6; i++)
  {
    // SceneManager destructor is responsible for deleting Cameras
    m_cameras[i] = scene_manager->createCamera("CubemapFilterCamera"
      + StringConverter::toString(m_unique_index) + "_" + StringConverter::toString(i));
    m_cameras[i]->setFOVy(Radian(Math::PI / 2));
    m_cameras[i]->setAspectRatio(1);
    m_cameras[i]->setNearClipDistance(0.01);
    m_cameras[i]->setFarClipDistance(10);
    m_cameras[i]->setPosition(0, 0, 0);

    switch(i)
    {
    case 0: // right
      m_cameras[i]->yaw( Radian( -Math::PI / 2) );
      break;
    case 1: // left
      m_cameras[i]->yaw( Radian( Math::PI / 2) );
      break;
    case 2: // up
      m_cameras[i]->pitch( Radian( Math::PI / 2 ) );
      break;
    case 3: // down
      m_cameras[i]->pitch( Radian( -Math::PI / 2 ) );
      break;
    case 4: // back
      break;
    case 5: // front
      m_cameras[i]->yaw( Radian( Math::PI ) );
      break;
    }

    RenderTarget* renderTarget = m_texture->getBuffer(i)->getRenderTarget();
    // RenderTarget destructor is responsible for deleting Viewports
    m_viewports[i] = renderTarget->addViewport(m_cameras[i]);
    m_viewports[i]->setOverlaysEnabled(false);
    m_viewports[i]->setClearEveryFrame(true);
  }
}

CubemapFilter::~CubemapFilter()
{
}

void CubemapFilter::render()
{
  if (m_GL_support && m_GL_support->checkExtension("GL_ARB_seamless_cube_map"))
  {
    // Enable seamless cube maps
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
  }

  for (int i = 0; i < 6; i++)
  {
    m_cameras[i]->_renderScene(m_viewports[i], false);
  }
}

void CubemapFilter::makeVertexProgram(const String& name)
{
  String code(
    "#version 130\n"
    "// input params bound by Ogre\n"
    "in vec4 position;\n"
    "uniform mat4 worldMatrix;\n"
    "uniform mat4 worldViewProjMatrix;\n"
    "out vec3 wsPos;\n"
    "void main()\n"
    "{\n"
    "  wsPos = vec3(worldMatrix * position);\n"
    "  gl_Position = worldViewProjMatrix * position;\n"
    "}\n"
    );

  HighLevelGpuProgramPtr vp = HighLevelGpuProgramManager::getSingleton().createProgram(name,
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "glsl", GpuProgramType::GPT_VERTEX_PROGRAM);
  vp->setSource(code);

  GpuProgramParametersSharedPtr params = vp->getDefaultParameters();
  params->setNamedAutoConstant("worldMatrix", GpuProgramParameters::ACT_WORLD_MATRIX);
  params->setNamedAutoConstant("worldViewProjMatrix", GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
}

void CubemapFilter::makeFragmentProgram(const String& name)
{
  // Generate sample directions on a hemisphere
  std::vector<Vector3> v;
  // theta = 0.08 between samples yields ~1000 samples. This works reasonably well with
  // texture size = 32, removing most clustering even in low-magnitude irradiance maps.
  const double min_theta = 0.08;
  makeRegularHemisphereSamples(min_theta, v);
  //makePoissonHemisphereSamples(min_theta, v);  // Splotchy at low-magnitudes

  // Convert sample area coverage to mipmap level
  TexturePtr source_tex = TextureManager::getSingleton().getByName(m_source_cubemap_name);
  int num_samples_around_circumference = int(M_PI * 2.0 / min_theta);
  // number of samples across one cubemap face
  double num_cube_samples = double(num_samples_around_circumference) / 4.0;
  double mipmap_level = max(log2(source_tex->getWidth() / num_cube_samples) + 1.0, 0.0);

  String code(
    "#version 130\n"
    "uniform samplerCube cubemap;\n"
    "in vec3 wsPos;\n"
    "out vec4 outputCol;\n"
    "vec3 rotVec(vec3 baseVec, vec3 v)\n"
    "{\n"
    "  vec3 t = baseVec + vec3(0.0, 0.0, 1.0);\n"
    "  vec3 u = v * vec3(-1.0, -1.0, 1.0);\n"
    "  return normalize((t / t.z) * dot(t, u) - u);\n"
    "}\n"
    "void main()\n"
    "{\n"
    "  // OpenGL cubemaps are arranged using RenderMan's left-handed coordinate system\n"
    "  // resulting in the entire map being mirrored when rendered looking out from\n"
    "  // the center, so we negate z to correct our cube texture lookups.\n"
    "  vec3 dir = normalize(vec3(wsPos.x, wsPos.y, -wsPos.z));\n"
    "  vec3 color = vec3(0, 0, 0);\n"
    );

  stringstream ss;
  float divisor = 0.0;
  for (size_t i = 0; i < v.size(); i ++)
  {
    ss << "  color += textureLod(cubemap, rotVec(dir, vec3(" << v[i][0] << ", "
       << v[i][1] << ", " << v[i][2] << ")), " << mipmap_level << ").rgb * "
       << v[i][2] << ";\n";
    divisor += v[i][2];
  }
  ss << "  outputCol = vec4(color / " << divisor << ", 1);\n";

  // draw sampling points for debugging
  /*
  for (size_t i = 0; i < out_p.size(); i ++)
  {
    ss << "  outputCol.g += max(pow(dot(vec3(" << out_p[i][0] << ", " << out_p[i][1]
       << ", " << out_p[i][2] << "), dir), 10000.0), 0.0);\n";
  }
  */

  code += ss.str();
  code += "}\n";

  HighLevelGpuProgramPtr fp = HighLevelGpuProgramManager::getSingleton().createProgram(name,
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, "glsl", GpuProgramType::GPT_FRAGMENT_PROGRAM);
  fp->setSource(code);

  //GpuProgramParametersSharedPtr params = fp->getDefaultParameters();
  //params->setNamedConstant("cubemap", 0);
}

void CubemapFilter::makeRegularHemisphereSamples(const double min_theta, std::vector<Ogre::Vector3>& out_p)
{
  // Want one point at the apex of the hemisphere
  out_p.push_back(Vector3(0, 0, 1));

  // For even hemisphere coverage, samples should be at latitudes >= min_theta
  // apart, and the lowest latitude should be >= min_theta * 0.5 from equator.
  const double half_min_theta = min_theta * 0.5;
  int num_half_lat = int(M_PI * 0.5 / half_min_theta);
  if (num_half_lat % 2 == 0)
    num_half_lat --;
  const double half_lat = M_PI * 0.5 / double(num_half_lat);
  const double lat_step = half_lat * 2.0;
  const int num_lat = (num_half_lat - 1) / 2;

  double lat = 0.0;
  double start_lon = 0.0;

  for (int i = 0; i < num_lat; i++)
  {
    lat += lat_step;

    const double cos_lat = cos(lat);
    const double sin_lat = sin(lat);
    const double circumference = sin_lat * M_PI * 2.0;
    const int num_lon = int(circumference / min_theta);
    const double lon_step = M_PI * 2.0 / double(num_lon);

    // Don't start at longitude = 0 every time. Stagger it to best avoid artifacts.
    start_lon += lon_step * 0.5;
    double lon = start_lon;
    for (int j = 0; j < num_lon; j ++)
    {
      out_p.push_back(Vector3(cos(lon) * sin_lat, sin(lon) * sin_lat, cos_lat));
      lon += lon_step;
    }
  }
}

void CubemapFilter::makePoissonHemisphereSamples(const double min_theta, std::vector<Ogre::Vector3>& out_p)
{
  const double cos_min_theta = cos(min_theta);

  std::vector<Ogre::Vector3> active_points;

  // Want one point at the apex of the hemisphere
  out_p.push_back(Vector3(0, 0, 1));

  int new_points_added = 1;
  while (new_points_added)
  {
    // New points added in the previous loop become active points that generate new points
    active_points.clear();
    for (size_t i = out_p.size() - new_points_added; i < out_p.size(); i ++)
    {
      active_points.push_back(out_p[i]);
    }

    new_points_added = 0;

    for (size_t p = 0; p < active_points.size(); p ++ )
    {
      for (size_t i = 0; i < 15; i++)
      {
        // find a random perpendicular vector
        const double theta = (Math::UnitRandom() * 0.1 + 1.0) * min_theta;
        const double phi = Math::UnitRandom() * 2.0 * M_PI;
        const double sin_theta = sin(theta);
        // Make vector in cone around (0,0,1)
        Vector3 cone_vec(cos(phi) * sin_theta, sin(phi) * sin_theta, cos(theta));
        // Reorient vector using method in Reoriented Normal Mapping: https://blog.selfshadow.com/publications/blending-in-detail
        Vector3 t(active_points[p] + Vector3(0, 0, 1));
        Vector3 u(cone_vec * Vector3(-1, -1, 1));
        const double t_dot_u = t.dotProduct(u);
        Vector3 vec(t / t[2] * t_dot_u - u);

        // Reject point if it is outside of hemisphere or too close to another point
        if (vec[2] < min_theta * 0.5)
        {
          break;
        }
        bool keep = true;
        // This part could be optimized by somehow searching through fewer points
        for (int j = out_p.size() - 1; j >= 0; j --)
        {
          if (vec.dotProduct(out_p[j]) > cos_min_theta)
          {
            keep = false;
            break;
          }
        }

        if (keep)
        {
          out_p.push_back(vec);
          new_points_added ++;
        }
      }
    }
  }
}
