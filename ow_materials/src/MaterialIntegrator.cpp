// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <MaterialIntegrator.h>


MaterialIntegrator::MaterialIntegrator(
  std::shared_ptr<ros::NodeHandle> node_handle, const physics::ModelPtr& model)
  : m_node_handle(node_handle), m_model(model)
{
  m_sub_visual_modification = m_node_handle->subscribe(
    "/ow_dynamic_terrain/modification_differential/visual", 10,
    &MaterialIntegrator::onVisualModification, this
  );
  m_sub_collision_modification = m_node_handle->subscribe(
    "/ow_dynamic_terrain/modification_differential/collision", 10,
    &MaterialIntegrator::onCollisionModification, this
  );

  // grab initial terrain shape (as it is without modification)
  m_terrain_shape = getHeightMapShape(m_model);
}

void MaterialIntegrator::onModification(
  const ow_dynamic_terrain::modified_terrain_diff::ConstPtr &msg)
{



  // update heightmap shape
  // TODO: verify this gets the resulting shape after the modification took place
  m_terrain_shape = getHeightMapShape(m_model);
}
