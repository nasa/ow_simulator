// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <string>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/tf.h>

#include <gazebo_msgs/DeleteModel.h>

#ifndef MODEL_POOL_H
#define MODEL_POOL_H

namespace ow_regolith
{

class ModelPool
{
public:
  ModelPool(std::shared_ptr<ros::NodeHandle> &nh) : m_node_handle(nh) {};

  ModelPool() = delete;
  ModelPool(const ModelPool&) = delete;
  ModelPool& operator=(const ModelPool&) = delete;

  ~ModelPool() = default;

  bool setModel(std::string const &model_uri, std::string const &model_prefix);

  // spawn the regolith model just above the tip of the scoop and apply a force
  // that keeps it in the scoop during the remainder of scooping operation
  bool spawn(tf::Point position, std::string reference_frame);

  std::vector<std::string> remove(const std::vector<std::string> &link_names = {});

private:
  bool removeModel(gazebo_msgs::DeleteModel &msg);

  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ServiceClientFacade m_gz_spawn_model, m_gz_delete_model;

  // regolith model that spawns in the scoop when digging occurs
  std::string m_model_uri;
  std::string m_model_sdf;
  std::string m_model_body_name;
  std::string m_model_tag;

  // keeps track of all regolith models and links present in the simulation
  struct Model {
    std::string model_name;
    std::string body_name;
    bool removal_requested;
  };
  // keys are of the form "model_name::body_name"
  std::unordered_map<std::string, Model> m_active_models;
};

} // namespace ow_regolith

#endif // MODEL_POOL_H
