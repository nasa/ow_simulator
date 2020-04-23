#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include "ow_dynamic_terrain/modify_terrain.h"
#include "terrain_modifier.h"

using namespace gazebo;

class DynamicTerrainModel : public ModelPlugin
{
public: void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/)
    {
        gzlog << "DynamicTerrainModel: successfully loaded!" << std::endl;

        model_ = model;

        on_update_connection_ = event::Events::ConnectPostRender(
            std::bind(&DynamicTerrainModel::onUpdate, this));

        if (!ros::isInitialized())
        {
            gzerr << "DynamicTerrainModel: ROS not initilized!" << std::endl;
        }
        else
        {
            auto so = ros::SubscribeOptions::create<ow_dynamic_terrain::modify_terrain>(
                "/ow_dynamic_terrain/modify_terrain",
                1,
                boost::bind(&DynamicTerrainModel::onModifyTerrainMsg, this, _1),
                ros::VoidPtr(), &this->ros_queue_
            );

            ros_node_.reset(new ros::NodeHandle("dynamic_terrain_model"));
            ros_subscriber_ = ros_node_->subscribe(so);
        }
    }

private: rendering::Heightmap* getHeightmap()
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

private: physics::HeightmapShapePtr getHeightmapShape()
    {
        if (model_ == nullptr)
        {
            gzerr << "DynamicTerrainModel plugin: Couldn't acquire heightmap model!" << std::endl;
            return nullptr;
        }

        auto collision = model_->GetLink("terrain-link")->GetCollision("collision");
        if (collision == nullptr)
        {
            gzerr << "DynamicTerrainModel plugin: Couldn't acquire heightmap model collision!" << std::endl;
            return nullptr;
        }
        
        auto shape = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
        if (shape == nullptr)
        {
            gzerr << "DynamicTerrainModel plugin: Couldn't acquire heightmap model collision!" << std::endl;
            return nullptr;
        }            

        gzlog << "DynamicTerrainModel plugin: heightmap shape ["
            << shape->VertexCount().X() << ", " << shape->VertexCount().Y() << "]" << std::endl;

        return shape;
    }

private: void onUpdate()
    {
        if (ros_node_->ok())
            ros_queue_.callAvailable();
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

        TerrainModifier::modify(heightmap,
            mt_msg->operation,
            mt_msg->position,
            mt_msg->outer_radius,
            mt_msg->inner_radius,
            mt_msg->weight,
            [&heightmap_shape](int x, int y) { return heightmap_shape->GetHeight(x, heightmap_shape->VertexCount().Y() - y - 1); },
            [&heightmap_shape](int x, int y, float value) { heightmap_shape->SetHeight(x, heightmap_shape->VertexCount().Y() - y - 1, value); }
        );
    }

private: physics::ModelPtr model_;
private: event::ConnectionPtr on_update_connection_;
private: std::unique_ptr<ros::NodeHandle> ros_node_;
private: ros::Subscriber ros_subscriber_;
private: ros::CallbackQueue ros_queue_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DynamicTerrainModel)