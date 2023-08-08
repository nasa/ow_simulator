// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "DynamicTerrainVisual.h"
#include "TerrainModifier.h"
#include <gazebo/rendering/RenderingIface.hh>

using namespace std;
using namespace gazebo;
using namespace rendering;
using namespace Ogre;

namespace ow_dynamic_terrain
{

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)

DynamicTerrainVisual::DynamicTerrainVisual() :
  DynamicTerrainBase{ "ow_dynamic_terrain", "DynamicTerrainVisual" }
{
}

void DynamicTerrainVisual::Load(VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/)
{
  Initialize("visual");

  m_on_prerender_connection = gazebo::event::Events::ConnectPreRender([this]() {
    initTextureUnits();
  });
}

float DynamicTerrainVisual::getHeightInWorldCoords(const Ogre::Terrain* terrain, int x, int y)
{
  auto value = terrain->getHeightAtPoint(x, y);
  value += terrain->getPosition().z;
  return value;
}

void DynamicTerrainVisual::setHeightFromWorldCoords(Ogre::Terrain* terrain, int x, int y, float value)
{
  value -= terrain->getPosition().z;
  terrain->setHeightAtPoint(x, y, value);
}

void DynamicTerrainVisual::onModifyTerrainCircleMsg(const modify_terrain_circle::ConstPtr& msg)
{
  onModifyTerrainMsg(msg, TerrainModifier::modifyCircle);
}

void DynamicTerrainVisual::onModifyTerrainEllipseMsg(const modify_terrain_ellipse::ConstPtr& msg)
{
  onModifyTerrainMsg(msg, TerrainModifier::modifyEllipse);
}

void DynamicTerrainVisual::onModifyTerrainPatchMsg(const modify_terrain_patch::ConstPtr& msg)
{
  onModifyTerrainMsg(msg, TerrainModifier::modifyPatch);
}

/**
 * Create texture to draw into
 * @returns true if texture already exists or has successfully been created
 */
void DynamicTerrainVisual::initMaskTexture(unsigned int texSize)
{
  // Create a single link track texture for all terrain materials.
  mMaterialMaskTexture = TextureManager::getSingleton().createManual("modifyMaskTexture",
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D,
    texSize, texSize, 0, PF_L8, TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  // Lock the pixel buffer
  HardwarePixelBufferSharedPtr pixelBuffer = mMaterialMaskTexture->getBuffer();
  pixelBuffer->lock(HardwareBuffer::HBL_DISCARD);

  const PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  uint8* pixels = static_cast<uint8*>(pixelBox.data);

  // Paint it white
  memset(pixels, 255, texSize * texSize);

  // Unlock the pixel buffer
  pixelBuffer->unlock();

  gzlog << "DynamicTerrainVisual::initMaskTexture - mask texture was created." << endl;
}

void DynamicTerrainVisual::initTextureUnits()
{
  auto heightmap = getHeightmap(get_scene());
  if (heightmap == nullptr)
  {
    gzerr << m_plugin_name << ": DynamicTerrainVisual::initTextureUnits()"
      " could not acquire heightmap." << endl;
    return;
  }
  auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

  initMaskTexture(heightmap->OgreTerrain()->getTerrainSize() - 1);

  // Override custom material normal map labeled "ow_dynamic_terrain_normal_map"
  // with Ogre::Terrain's internal normal map, and assign a custom mask texture.
  // Ogre::Terrain generates an internal normal map and it is updated by
  // Terrain::updateDerivedData(). But if you use a custom material script
  // with a custom normal map, the internal normal map will never be seen.
  // Note: If you are not using a custom material this code will have no
  // effect, but Ogre should be using the correct normal map and a generated
  // material and shader in that case.
  // Caveat: The internal normal map is always sized to the terrain's DEM
  // resolution, so using a larger texture for more detail is not possible
  // with this plugin.
  ResourceManager::ResourceMapIterator res_it = MaterialManager::getSingleton().getResourceIterator();
  while (res_it.hasMoreElements())
  {
    ResourcePtr resource = res_it.getNext();
    MaterialPtr material = resource.staticCast<Material>();
    Material::TechniqueIterator tech_it = material->getTechniqueIterator();
    while (tech_it.hasMoreElements())
    {
      Technique* technique = tech_it.getNext();
      Technique::PassIterator pass_it = technique->getPassIterator();
      while (pass_it.hasMoreElements())
      {
        Pass* pass = pass_it.getNext();
        // Assign Ogre-generated normal map to the named texture unit if it exists
        TextureUnitState* tus = pass->getTextureUnitState("ow_dynamic_terrain_normal_map");
        if (tus != 0)
        {
          tus->setTexture(terrain->getTerrainNormalMap());
        }
        // Assign mask map to the named texture unit if it exists
        tus = pass->getTextureUnitState("ow_dynamic_terrain_mask_map");
        if (tus != 0)
        {
          tus->setTexture(mMaterialMaskTexture);
        }
      }
    }
  }

  // Disconnect so this method will not be called again.
  m_on_prerender_connection.reset();
}

}  // namespace ow_dynamic_terrain
