// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include "ow_dynamic_terrain/modify_terrain_circle.h"
#include "ow_dynamic_terrain/modify_terrain_ellipse.h"
#include "ow_dynamic_terrain/modify_terrain_patch.h"

namespace ow_dynamic_terrain
{
class DynamicTerrainBase
{
protected:
    DynamicTerrainBase(const std::string& package_name, const std::string& plugin_name) :
        m_package_name{package_name},
        m_plugin_name{plugin_name}
    {
    }

protected:
  void Initialize(const std::string& topic_extension);

private:
  template <typename T>
  void subscribe(const std::string& topic,
    const boost::function<void (const boost::shared_ptr<T const>&)>& callback);

protected:
  gazebo::rendering::Heightmap* getHeightmap(gazebo::rendering::ScenePtr scene);

protected:
  virtual void onModifyTerrainCircleMsg(const modify_terrain_circle::ConstPtr& msg) = 0;

protected:
  virtual void onModifyTerrainEllipseMsg(const modify_terrain_ellipse::ConstPtr& msg) = 0;

protected:
  virtual void onModifyTerrainPatchMsg(const modify_terrain_patch::ConstPtr& msg) = 0;

protected:
    std::string m_package_name;

protected:
    std::string m_plugin_name;

protected:
  gazebo::event::ConnectionPtr m_on_update_connection;

protected:
  std::unique_ptr<ros::NodeHandle> m_node_handle;

protected:
  ros::CallbackQueue m_callback_queue;

protected:
  std::vector<ros::Subscriber> m_subscribers;
};

}  // namespace ow_dynamic_terrain
