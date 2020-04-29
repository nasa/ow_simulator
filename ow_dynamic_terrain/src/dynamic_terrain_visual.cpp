#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <gazebo/common/common.hh>
#include "ow_dynamic_terrain/modify_terrain.h"
#include "terrain_modifier.h"

using namespace gazebo;

class DynamicTerrainVisual : public VisualPlugin
{
public:
  void Load(rendering::VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/)
  {
    gzlog << "DynamicTerrainVisual: successfully loaded!" << std::endl;

    this->on_update_connection_ = event::Events::ConnectPostRender(std::bind(&DynamicTerrainVisual::onUpdate, this));

    if (!ros::isInitialized())
    {
      gzerr << "DynamicTerrainVisual: ROS not initilized!" << std::endl;
    }
    else
    {
      auto so = ros::SubscribeOptions::create<ow_dynamic_terrain::modify_terrain>(
          "/ow_dynamic_terrain/modify_terrain", 1, boost::bind(&DynamicTerrainVisual::onModifyTerrainMsg, this, _1),
          ros::VoidPtr(), &this->ros_queue_);

      ros_node_.reset(new ros::NodeHandle("dynamic_terrain_visual"));
      ros_subscriber_ = ros_node_->subscribe(so);
    }
  }

private:
  rendering::Heightmap* getHeightmap()
  {
    auto scene = rendering::get_scene();
    if (!scene)
    {
      gzerr << "DynamicTerrainVisual: Couldn't acquire scene!" << std::endl;
      return nullptr;
    }

    auto heightmap = scene->GetHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainVisual: scene has no heightmap!" << std::endl;
      return nullptr;
    }

    return heightmap;
  }

private:
  void onUpdate()
  {
    if (ros_node_->ok())
      ros_queue_.callAvailable();
  }

private:
  void onModifyTerrainMsg(const ow_dynamic_terrain::modify_terrainConstPtr mt_msg)
  {
    auto heightmap = getHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainVisual: Couldn't acquire heightmap!" << std::endl;
      return;
    }

    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);
    TerrainModifier::modify(heightmap, mt_msg->operation, mt_msg->position, mt_msg->outer_radius, mt_msg->inner_radius,
                            mt_msg->weight,
                            [&terrain](long x, long y) { return terrain->getHeightAtPoint(x, y); },
                            [&terrain](long x, long y, float value) { terrain->setHeightAtPoint(x, y, value); });

    terrain->updateGeometry();
    terrain->updateDerivedData(false, Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);
  }

private:
  event::ConnectionPtr on_update_connection_;

private:
  std::unique_ptr<ros::NodeHandle> ros_node_;

private:
  ros::Subscriber ros_subscriber_;

private:
  ros::CallbackQueue ros_queue_;
};

GZ_REGISTER_VISUAL_PLUGIN(DynamicTerrainVisual)