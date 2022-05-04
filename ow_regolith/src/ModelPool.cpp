
#include <stdexcept>
#include <sstream>

#include <sdf_utility.h>
#include <ServiceClientFacade.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>

#include <ModelPool.h>

using namespace ow_regolith;

using namespace gazebo_msgs;
using namespace sdf_utility;

using std::string, std::vector, std::begin, std::end, std::stringstream,
      std::out_of_range;

using tf::pointMsgToTF, tf::pointTFToMsg, tf::Point, tf::Vector3;

// service paths used in class
const static string SRV_SPAWN_MODEL       = "/gazebo/spawn_sdf_model";
const static string SRV_DELETE_MODEL      = "/gazebo/delete_model";
const static string SRV_APPLY_BODY_WRENCH = "/gazebo/apply_body_wrench";
const static string SRV_CLEAR_BODY_WRENCH = "/gazebo/clear_body_wrenches";

const static ros::Duration SERVICE_CONNECT_TIMEOUT = ros::Duration(5.0);

bool ModelPool::connectServices()
{
  // connect to all ROS services
  return
    m_gz_spawn_model.connect<SpawnModel>(
      m_node_handle, SRV_SPAWN_MODEL, SERVICE_CONNECT_TIMEOUT, true) &&
    m_gz_delete_model.connect<DeleteModel>(
      m_node_handle, SRV_DELETE_MODEL, SERVICE_CONNECT_TIMEOUT, true) &&
    m_gz_apply_wrench.connect<ApplyBodyWrench>(
      m_node_handle, SRV_APPLY_BODY_WRENCH, SERVICE_CONNECT_TIMEOUT, true) &&
    m_gz_clear_wrench.connect<BodyRequest>(
      m_node_handle, SRV_CLEAR_BODY_WRENCH, SERVICE_CONNECT_TIMEOUT, true);
}

bool ModelPool::setModel(const string &model_uri, const string &model_tag)
{
  // use local variables for data and only set members if all calls succeed
  string model_sdf;
  string model_body_name;
  float model_mass;

  // load SDF model
  if (!getSdfFromUri(model_uri, model_sdf)) {
    ROS_ERROR("Failed to load model SDF");
    return false;
  }

  // parse as SDF object to query properties about the model
  auto sdf = parseSdf(model_sdf);
  if (!getModelLinkName(sdf, model_body_name)) {
    ROS_ERROR("Failed to acquire model name");
    return false;
  }
  if (!getModelLinkMass(sdf, model_mass)) {
    ROS_ERROR("Failed to acquire model mass");
    return false;
  }

  m_model_uri = model_uri;
  m_model_sdf = model_sdf;
  m_model_body_name = model_body_name;
  m_model_tag = model_tag;
  m_model_mass = model_mass;

  return true;
}

string ModelPool::spawn(const Point &position, const string &reference_frame)
{
  ROS_INFO("Spawning regolith");

  static auto spawn_count = 0;
  stringstream model_name, link_name;
  model_name << m_model_tag << "_" << spawn_count++;
  link_name << model_name.str() << "::" << m_model_body_name;

  SpawnModel spawn_msg;

  spawn_msg.request.model_name      = model_name.str();
  spawn_msg.request.model_xml       = m_model_sdf;
  spawn_msg.request.reference_frame = reference_frame;

  pointTFToMsg(position, spawn_msg.request.initial_pose.position);

  if (!m_gz_spawn_model.call(spawn_msg))
    return "";

  m_active_models.emplace(
    link_name.str(),
    Model{model_name.str(), m_model_body_name}
  );

  return link_name.str();
}

vector<string> ModelPool::remove(const vector<string> &link_names)
{
  if (link_names.empty())
    return {};
  vector<string> not_removed;
  DeleteModel msg;
  // delete one or multiple models as provided in link_names
  for (auto const &n : link_names) {
    try {
      msg.request.model_name = m_active_models.at(n).model_name;
    } catch(out_of_range &_err) {
      not_removed.push_back(n);
      continue;
    }
    if (removeModel(msg))
      m_active_models.erase(n);
    else
      not_removed.push_back(n);
  }
  return not_removed;
}

vector<string> ModelPool::clear() {
  vector<string> not_removed;
  DeleteModel msg;
  // delete all regolith models
  auto it = begin(m_active_models);
  while (it != end(m_active_models)) {
    msg.request.model_name = it->second.model_name;
    if (removeModel(msg))
      it = m_active_models.erase(it);
    else
      // do not remove from active list if removal failed
      not_removed.push_back((it++)->first);
  }
  return not_removed;
}

bool ModelPool::applyForce(const string &link_name, const Vector3 &force,
                           const ros::Duration &apply_for) {
  ApplyBodyWrench wrench_msg;
  wrench_msg.request.body_name = link_name;
  vector3TFToMsg(force, wrench_msg.request.wrench.force);
  wrench_msg.request.duration = apply_for;

  return m_gz_apply_wrench.call(wrench_msg);
}

bool ModelPool::clearAllForces()
{
  bool service_call_error_occurred = false;
  BodyRequest msg;
  for (const auto &m : m_active_models) {
    msg.request.body_name = m.first;
    if (!m_gz_clear_wrench.call(msg)) {
      ROS_WARN("Failed to clear force on %s", m.first.c_str());
      service_call_error_occurred = true;
    }
  }
  return !service_call_error_occurred;
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
