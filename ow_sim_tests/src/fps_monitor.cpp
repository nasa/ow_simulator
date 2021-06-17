#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <chrono>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>

using namespace std;
using namespace gazebo;
using namespace rendering;

namespace gazebo
{
class FPS_Monitor : public SystemPlugin
{
public:
  virtual ~FPS_Monitor()
  {
    this->m_connections.clear();
    this->m_camera.reset();
  }

public:
  void Load(int /*_argc*/, char** /*_argv*/)
  {
    this->m_connections.push_back(event::Events::ConnectPreRender([this]() { this->onUpdate(); }));
  }

private:
  void Init()
  {
    if (!ros::isInitialized())
    {
      gzerr << "FPS_Monitor: ROS not initilized!" << endl;
      return;
    }

    m_ros_node.reset(new ros::NodeHandle("fps_monitor"));

    m_avg_fps_publisher = m_ros_node->advertise<std_msgs::Float32>("avg_fps", 10);

    last_time = std::chrono::steady_clock::now();
  }

private:
  void onUpdate()
  {
    const auto& scene = rendering::get_scene();

    // Wait until the scene is initialized.
    if (!scene || !scene->Initialized())
      return;

    if (!m_camera)
    {
      // Get a pointer to the active user camera
      m_camera = gui::get_active_camera();

      if (!m_camera)
      {
        ROS_INFO_THROTTLE(1.0, "no active camera found");
        return;
      }
    }

    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() < message_publish_rate_milliseconds)
      return;

    last_time = current_time;
    auto msg = std_msgs::Float32();
    msg.data = m_camera->AvgFPS();
    m_avg_fps_publisher.publish(msg);
  }

private:
  rendering::CameraPtr m_camera;

  /// All the event m_connections.
private:
  std::vector<event::ConnectionPtr> m_connections;

private:
  unique_ptr<ros::NodeHandle> m_ros_node;

private:
  ros::Publisher m_avg_fps_publisher;

private:
  std::chrono::steady_clock::time_point last_time;

private:
  static const int64_t message_publish_rate_milliseconds = 1000; 
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(FPS_Monitor)
}  // namespace gazebo