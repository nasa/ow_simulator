
#include <stdexcept>
#include <sstream>

#include <sdf_utility.h>
#include <ServiceClientFacade.h>

#include <gazebo_msgs/SpawnModel.h>

#include <ModelPool.h>

using namespace ow_regolith;

using namespace gazebo_msgs;
using namespace sdf_utility;

using namespace std;

using tf::pointMsgToTF;
using tf::pointTFToMsg;
using tf::Point;

// service paths used in class
const static string SRV_SPAWN_MODEL     = "/gazebo/spawn_sdf_model";
const static string SRV_DELETE_MODEL    = "/gazebo/delete_model";

const static ros::Duration SERVICE_CONNECT_TIMEOUT = ros::Duration(5.0);

bool ModelPool::setModel(string const &model_uri, string const &model_tag)
{
  // use local variables for data and only set members if all calls succeed
  string model_sdf;
  string model_body_name;

  // load SDF model
  if (!getSdfFromUri(model_uri, model_sdf)) {
    ROS_ERROR("Failed to load SDF for regolith model");
    return false;
  }

  // parse as SDF object to query properties about the model
  auto sdf = parseSdf(model_sdf);
  if (!getModelLinkName(sdf, model_body_name)) {
    ROS_ERROR("Failed to acquire regolith model SDF parameters");
    return false;
  }

  // connect to all ROS services
  // check if already connected in-case this is a re-call of setModel
  if (m_gz_spawn_model.isConnected() || !m_gz_spawn_model.connect<SpawnModel>(
        m_node_handle, SRV_SPAWN_MODEL, SERVICE_CONNECT_TIMEOUT, true))
    return false;

  if (m_gz_delete_model.isConnected() || !m_gz_delete_model.connect<DeleteModel>(
        m_node_handle, SRV_DELETE_MODEL, SERVICE_CONNECT_TIMEOUT, true))
    return false;

  // set the maximum scoop inclination that the psuedo force can counteract
  // NOTE: do not use a value that makes cosine zero!
  // constexpr auto MAX_SCOOP_INCLINATION_DEG = 70.0f; // degrees
  // constexpr auto MAX_SCOOP_INCLINATION_RAD = MAX_SCOOP_INCLINATION_DEG * M_PI / 180.0f; // radians
  // constexpr auto PSUEDO_FORCE_WEIGHT_FACTOR = 1.0f / cos(MAX_SCOOP_INCLINATION_RAD);
  // query gazebo for the gravity vector
  // ServiceClientFacade gz_get_phys_props;
  // GetPhysicsProperties phys_prop_msg;
  // if (!gz_get_phys_props.connect<GetPhysicsProperties>(
  //       m_node_handle, SRV_GET_PHYS_PROPS, SERVICE_CONNECT_TIMEOUT, false) ||
  //     !gz_get_phys_props.call(phys_prop_msg)) {
  //   ROS_ERROR("Failed to connect Gazebo for gravity vector");
  //   return false;
  // }
  // Vector3 gravity;
  // vector3MsgToTF(phys_prop_msg.response.gravity, gravity);
  // // psuedo force magnitude = model's weight X weight factor
  // m_psuedo_force_mag = mass * gravity.length() * PSUEDO_FORCE_WEIGHT_FACTOR;

  m_model_uri = model_uri;
  m_model_sdf = model_sdf;
  m_model_body_name = model_body_name;
  m_model_tag = model_tag;

  return true;
}

bool ModelPool::spawn(Point position, string reference_frame)
{
  ROS_INFO("Spawning regolith");

  static auto spawn_count = 0;
  stringstream model_name, link_name;
  model_name << m_model_tag << "_" << spawn_count++;
  link_name << model_name.str() << "::" << m_model_body_name;

  SpawnModel spawn_msg;

  spawn_msg.request.model_name                  = model_name.str();
  spawn_msg.request.model_xml                   = m_model_sdf;
  spawn_msg.request.reference_frame             = reference_frame;

  pointTFToMsg(position, spawn_msg.request.initial_pose.position);

  if (!m_gz_spawn_model.call(spawn_msg))
    return false;

  m_active_models.emplace(link_name.str(),
    Model{model_name.str(), m_model_body_name, false}
  );

  return true;
}

vector<string> ModelPool::remove(const vector<string> &link_names)
{
  vector<string> not_removed;
  DeleteModel msg;
  if (link_names.empty()) {
    // delete all regolith models
    auto it = begin(m_active_models);
    while (it != end(m_active_models)) {
      msg.request.model_name = it->second.model_name;
      if (removeModel(msg)) {
        it = m_active_models.erase(it);
      } else {
        // do not remove from active list if removal failed
        not_removed.push_back((it++)->first);
      }
    }
  } else {
    // delete one or multiple models provided in link_names
    for (auto const &n : link_names) {
      try {
        msg.request.model_name = m_active_models.at(n).model_name;
      } catch(out_of_range &_err) {
        ROS_WARN("Model %s is not in this pool", n.c_str());
        not_removed.push_back(n);
        continue;
      }
      if (removeModel(msg))
        m_active_models.erase(n);
      else
        not_removed.push_back(n);
    }
  }
  return not_removed;
}

bool ModelPool::removeModel(DeleteModel &msg)
{
  if (!m_gz_delete_model.call(msg)) {
    ROS_WARN("Failed to remove model %s, response: %s",
      msg.request.model_name.c_str(), msg.response.status_message.c_str()
    );
    return false;
  }
  return true;
}
