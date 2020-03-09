// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef CosimulationPlugin_h
#define CosimulationPlugin_h

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/OrientedBox.hh>

#include "Cosimulator.h"

namespace gazebo {

class CosimulationPlugin : public ModelPlugin
{
public:
  CosimulationPlugin();
  ~CosimulationPlugin();

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  // Called before each Gazebo physics loop
  void OnUpdate();

private:
  // Cosimulator class that interfaces with external physics simulation
  Cosimulator m_cosim;
  // Timestep the cosimulaiton is iterated at
  double m_timestep;
  // True if Cosimulator has begun updating its physics
  bool m_started;
  // Timepoint when cosimulation started updating its physics
  double m_starttime;
  // Link that will be cosimulated and have resulting forces applied to it
  physics::LinkPtr m_link;
  // World pointer for checking Gazebo simulation clock
  physics::WorldPtr m_world;
  // Region intersecting the heightmap that indicates where link is cosimulated
  ignition::math::OrientedBoxd m_workspace_box;
  // Physics update connection
  event::ConnectionPtr m_phys_connect;
};

// Register this plugin with the Gazebo
GZ_REGISTER_MODEL_PLUGIN(CosimulationPlugin)

}

#endif // CosimulationPlugin_h