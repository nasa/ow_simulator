#include "DynamicTerrainBase.h"
#include "memory_ext.h"

using namespace std;
using namespace ow_dynamic_terrain;

void DynamicTerrainBase::Initialize(const std::string& topic_extension)
{
    if (!ros::isInitialized())
    {
        gzerr << m_plugin_name + ": ROS not initilized!" << endl;
        return;
    }

    // m_node_handle.reset(new ros::NodeHandle(m_plugin_name));
    m_node_handle = make_unique<ros::NodeHandle>(m_plugin_name);
    m_node_handle->setCallbackQueue(&m_callback_queue);

    auto on_modify_terrain_circle = [this](const modify_terrain_circle::ConstPtr& msg) {
        this->onModifyTerrainCircleMsg(msg);
    };
    subscribe<modify_terrain_circle>("modify_terrain_circle", on_modify_terrain_circle);
    subscribe<modify_terrain_circle>("modify_terrain_circle/" + topic_extension, on_modify_terrain_circle);

    auto on_modify_terrain_ellipse = [this](const modify_terrain_ellipse::ConstPtr& msg) {
        this->onModifyTerrainEllipseMsg(msg);
    };
    subscribe<modify_terrain_ellipse>("modify_terrain_ellipse", on_modify_terrain_ellipse);
    subscribe<modify_terrain_ellipse>("modify_terrain_ellipse/" + topic_extension, on_modify_terrain_ellipse);

    auto on_modify_terrain_patch = [this](const modify_terrain_patch::ConstPtr& msg) {
        this->onModifyTerrainPatchMsg(msg);
    };
    subscribe<modify_terrain_patch>("modify_terrain_patch", on_modify_terrain_patch);
    subscribe<modify_terrain_patch>("modify_terrain_patch/" + topic_extension, on_modify_terrain_patch);

    m_on_update_connection = gazebo::event::Events::ConnectPostRender([this]() {
        if (m_node_handle->ok())
            m_callback_queue.callAvailable();
    });

    gzlog << m_plugin_name << ": successfully loaded!" << endl;
}

template <class T>
void DynamicTerrainBase::subscribe(const std::string& topic,
    const boost::function<void (const boost::shared_ptr<T const>&)>& callback)
{
    auto topic_fqn = "/" + m_package_name + "/" + topic;
    m_subscribers.push_back(m_node_handle->subscribe<T>(topic_fqn, 10, callback));
}

gazebo::rendering::Heightmap* DynamicTerrainBase::getHeightmap(gazebo::rendering::ScenePtr scene)
{
    if (!scene)
    {
        gzerr << m_plugin_name << ": Couldn't acquire scene!" << endl;
        return nullptr;
    }

    auto heightmap = scene->GetHeightmap();
    if (heightmap == nullptr)
    {
        gzerr << m_plugin_name << ": scene has no heightmap!" << endl;
        return nullptr;
    }

    return heightmap;
}