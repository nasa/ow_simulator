#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "ow_dynamic_terrain/modify_terrain.h"
#include "TerrainModifier.h"

using namespace gazebo;

class DynamicTerrainModel : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/)
  {
    gzlog << "DynamicTerrainModel: successfully loaded!" << std::endl;

    m_model = model;

    m_on_update_connection = event::Events::ConnectPostRender(std::bind(&DynamicTerrainModel::onUpdate, this));

    if (!ros::isInitialized())
    {
      gzerr << "DynamicTerrainModel: ROS not initilized!" << std::endl;
    }
    else
    {
      auto so = ros::SubscribeOptions::create<ow_dynamic_terrain::modify_terrain>(
          "/ow_dynamic_terrain/modify_terrain", 1, boost::bind(&DynamicTerrainModel::onModifyTerrainMsg, this, _1),
          ros::VoidPtr(), &this->m_ros_queue);

      m_ros_node.reset(new ros::NodeHandle("dynamic_terrain_model"));
      m_ros_subscriber = m_ros_node->subscribe(so);
    }
  }

private:
  rendering::Heightmap* getHeightmap()
  {
    auto scene = rendering::get_scene();
    if (!scene)
    {
      gzerr << "DynamicTerrainModel: Couldn't acquire scene!" << std::endl;
      return nullptr;
    }

    auto heightmap = scene->GetHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainModel: scene has no heightmap!" << std::endl;
      return nullptr;
    }

    return heightmap;
  }

private:
  physics::HeightmapShapePtr getHeightmapShape()
  {
    if (m_model == nullptr)
    {
      gzerr << "DynamicTerrainModel: Couldn't acquire heightmap model!" << std::endl;
      return nullptr;
    }

    auto links = m_model->GetLinks();

    if (links.size() == 0)
    {
      gzerr << "DynamicTerrainModel: Associcated model has no links!" << std::endl;
      return nullptr;
    }

    auto link0 = links[0];

    auto collisions = link0->GetCollisions();

    if (collisions.size() == 0)
    {
      gzerr << "DynamicTerrainModel: Model has no collisions for first link!" << std::endl;
      return nullptr;
    }

    auto collision = collisions[0];
    if (collision == nullptr)
    {
      gzerr << "DynamicTerrainModel: Couldn't acquire heightmap model collision!" << std::endl;
      return nullptr;
    }

    auto shape = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
    if (shape == nullptr)
    {
      gzerr << "DynamicTerrainModel: Couldn't acquire heightmap model collision!" << std::endl;
      return nullptr;
    }

    gzlog << "DynamicTerrainModel: heightmap shape [" << shape->VertexCount().X() << ", " << shape->VertexCount().Y()
          << "]" << std::endl;

    return shape;
  }

private:
  void onUpdate()
  {
    if (m_ros_node->ok())
      m_ros_queue.callAvailable();
  }

  void onModifyTerrainMsg(const ow_dynamic_terrain::modify_terrainConstPtr mt_msg)
  {
    auto heightmap = getHeightmap();
    if (heightmap == nullptr)
    {
      gzerr << "DynamicTerrainModel: Couldn't acquire heightmap!" << std::endl;
      return;
    }

    auto heightmap_shape = getHeightmapShape();
    if (heightmap_shape == nullptr)
    {
      gzerr << "DynamicTerrainModel: Couldn't acquire heightmap shape!" << std::endl;
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 9 && GAZEBO_MINOR_VERSION > 13
    TerrainModifier::modify(heightmap, mt_msg->operation, mt_msg->position, mt_msg->outer_radius, mt_msg->inner_radius,
                            mt_msg->weight,
                            [&heightmap_shape](int x, int y) {
                              return heightmap_shape->GetHeight(x, heightmap_shape->VertexCount().Y() - y - 1);
                            },
                            [&heightmap_shape](int x, int y, float value) {
                              heightmap_shape->SetHeight(x, heightmap_shape->VertexCount().Y() - y - 1, value);
                            });
#endif
  }

private:
  physics::ModelPtr m_model;

private:
  event::ConnectionPtr m_on_update_connection;

private:
  std::unique_ptr<ros::NodeHandle> m_ros_node;

private:
  ros::Subscriber m_ros_subscriber;

private:
  ros::CallbackQueue m_ros_queue;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DynamicTerrainModel)