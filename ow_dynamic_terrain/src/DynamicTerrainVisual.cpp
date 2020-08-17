// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include "TerrainModifier.h"
#include "ow_dynamic_terrain/modify_terrain_circle.h"
#include "ow_dynamic_terrain/modify_terrain_ellipse.h"
#include "ow_dynamic_terrain/modify_terrain_patch.h"

using namespace std;
using namespace gazebo;
using namespace rendering;

namespace ow_dynamic_terrain
{
class DynamicTerrainVisual : public VisualPlugin
{
public:
  void Load(VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/) override
  {
    if (!ros::isInitialized())
    {
      gzerr << "DynamicTerrainVisual: ROS not initilized!" << endl;
      return;
    }

    m_ros_node.reset(new ros::NodeHandle("dynamic_terrain_visual"));
    m_ros_node->setCallbackQueue(&m_ros_queue);

    auto on_modify_terrain_circle = [this](const modify_terrain_circle::ConstPtr& msg) {
      this->onModifyTerrainCircleMsg(msg);
    };

    m_ros_subscribers.push_back(m_ros_node->subscribe<modify_terrain_circle>(
        "/ow_dynamic_terrain/modify_terrain_circle", 10, on_modify_terrain_circle));

    m_ros_subscribers.push_back(m_ros_node->subscribe<modify_terrain_circle>(
        "/ow_dynamic_terrain/modify_terrain_circle/visual", 10, on_modify_terrain_circle));

    auto on_modify_terrain_ellipse = [this](const modify_terrain_ellipse::ConstPtr& msg) {
      this->onModifyTerrainEllipseMsg(msg);
    };

    m_ros_subscribers.push_back(m_ros_node->subscribe<modify_terrain_ellipse>(
        "/ow_dynamic_terrain/modify_terrain_ellipse", 10, on_modify_terrain_ellipse));

    m_ros_subscribers.push_back(m_ros_node->subscribe<modify_terrain_ellipse>(
        "/ow_dynamic_terrain/modify_terrain_ellipse/visual", 10, on_modify_terrain_ellipse));

    auto on_modify_terrain_patch = [this](const modify_terrain_patch::ConstPtr& msg) {
      this->onModifyTerrainPatchMsg(msg);
    };

    m_ros_subscribers.push_back(m_ros_node->subscribe<modify_terrain_patch>(
        "/ow_dynamic_terrain/modify_terrain_patch", 10, on_modify_terrain_patch));

    m_ros_subscribers.push_back(m_ros_node->subscribe<modify_terrain_patch>(
        "/ow_dynamic_terrain/modify_terrain_patch/visual", 10, on_modify_terrain_patch));

    m_on_update_connection = event::Events::ConnectPostRender([this]() { this->onUpdate(); });

    gzlog << "DynamicTerrainVisual: successfully loaded!" << endl;
  }

private:
  Heightmap* getHeightmap()
  {
    const auto& scene = get_scene();
    if (!scene)
    {
      gzerr << "DynamicTerrainVisual: Couldn't acquire scene!" << endl;
      return nullptr;
    }

    auto heightmap = scene->GetHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainVisual: scene has no heightmap!" << endl;
      return nullptr;
    }

    return heightmap;
  }

private:
  void onUpdate()
  {
    if (m_ros_node->ok())
      m_ros_queue.callAvailable();
  }

private:
  static inline float getHeightInWorldCoords(const Ogre::Terrain* terrain, int x, int y)
  {
    auto value = terrain->getHeightAtPoint(x, y);
    value += terrain->getPosition().z;
    return value;
  }

private:
  static inline void setHeightFromWorldCoords(Ogre::Terrain* terrain, int x, int y, float value)
  {
    value -= terrain->getPosition().z;
    terrain->setHeightAtPoint(x, y, value);
  }

private:
  void onModifyTerrainCircleMsg(const modify_terrain_circle::ConstPtr& msg)
  {
    auto heightmap = getHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainVisual: Couldn't acquire heightmap!" << endl;
      return;
    }

    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
    TerrainModifier::modifyCircle(
        heightmap, msg, [&terrain](int x, int y) { return getHeightInWorldCoords(terrain, x, y); },
        [&terrain](int x, int y, float value) { setHeightFromWorldCoords(terrain, x, y, value); });

    terrain->updateGeometry();
    terrain->updateDerivedData(false, Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);
  }

private:
  void onModifyTerrainEllipseMsg(const modify_terrain_ellipse::ConstPtr& msg)
  {
    auto heightmap = getHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainVisual: Couldn't acquire heightmap!" << endl;
      return;
    }

    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
    TerrainModifier::modifyEllipse(
        heightmap, msg, [&terrain](int x, int y) { return getHeightInWorldCoords(terrain, x, y); },
        [&terrain](int x, int y, float value) { setHeightFromWorldCoords(terrain, x, y, value); });

    terrain->updateGeometry();
    terrain->updateDerivedData(false, Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);
  }

private:
  void onModifyTerrainPatchMsg(const modify_terrain_patch::ConstPtr& msg)
  {
    auto heightmap = getHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainVisual: Couldn't acquire heightmap!" << endl;
      return;
    }

    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
    TerrainModifier::modifyPatch(
        heightmap, msg, [&terrain](int x, int y) { return getHeightInWorldCoords(terrain, x, y); },
        [&terrain](int x, int y, float value) { setHeightFromWorldCoords(terrain, x, y, value); });

    terrain->updateGeometry();
    terrain->updateDerivedData(false, Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);
  }

private:
  event::ConnectionPtr m_on_update_connection;

private:
  unique_ptr<ros::NodeHandle> m_ros_node;

private:
  ros::CallbackQueue m_ros_queue;

private:
  vector<ros::Subscriber> m_ros_subscribers;
};

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)
}  // namespace ow_dynamic_terrain
