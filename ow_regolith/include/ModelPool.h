// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef MODEL_POOL_H
#define MODEL_POOL_H

#include <string>
#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/tf.h"

#include "gazebo_msgs/DeleteModel.h"

#include "ow_materials/material_mixing.h"

#include "ServiceClientFacade.h"

namespace ow_regolith
{

// ModelPool enables rapid spawning and removal of multiple instances of a
// model in gazebo. It can also apply forces to the models it has spawned.
class ModelPool
{
public:
  ModelPool()                            = delete;
  ModelPool(const ModelPool&)            = delete;
  ModelPool& operator=(const ModelPool&) = delete;

  ~ModelPool() = default;

  ModelPool(std::shared_ptr<ros::NodeHandle> nh) : m_node_handle(nh) {};

  bool initialize();

  bool setModel(const std::string &model_uri, const std::string &model_prefix);

  float getModelMass() const {return m_model_mass;}

  // spawn a model
  std::string spawn(const tf::Point &position,
                    const std::string &reference_frame,
                    const ow_materials::Bulk &bulk);

  // remove models within the pool by link name
  std::vector<std::string> remove(const std::vector<std::string> &link_names,
                                  bool ingested);

  // remove all models in pool
  std::vector<std::string> clear();

  // applies of force to the specified link
  bool applyForce(const std::string &link_name, const tf::Vector3 &force,
                  const ros::Duration& apply_for);

  // clears all forces acting on all active models
  bool clearAllForces();

private:
  bool removeModel(gazebo_msgs::DeleteModel &msg);

  void onConsolidateIngestedTimeout(const ros::TimerEvent&);

  // make necessary calls to reset the member ROS Timer
  void resetConsolidatedIngestedTimeout();

  ros::Timer m_consolidate_ingested_timeout;

  ow_materials::Bulk m_ingested_bulk;

  std::shared_ptr<ros::NodeHandle> m_node_handle;

  ServiceClientFacade m_gz_spawn_model, m_gz_delete_model,
                      m_gz_apply_wrench, m_gz_clear_wrench;

  ros::Publisher m_pub_material_ingested;

  // regolith model that spawns in the scoop when digging occurs
  std::string m_model_uri;
  std::string m_model_sdf;
  std::string m_model_body_name;
  std::string m_model_tag;
  float m_model_mass;

  // keeps track of all regolith models and links present in the simulation
  struct Model {
    const std::string model_name;
    const std::string body_name;
    const ow_materials::Bulk bulk;
  };
  // keys are of the form "model_name::body_name"
  std::unordered_map<std::string, Model> m_active_models;
};

} // namespace ow_regolith

#endif // MODEL_POOL_H
