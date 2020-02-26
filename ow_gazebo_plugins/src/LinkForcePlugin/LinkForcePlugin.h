/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef LinkForcePlugin_h
#define LinkForcePlugin_h

#include <gazebo/common/Plugin.hh>
#include "ForceRow.h"


namespace gazebo {

class LinkForcePlugin : public ModelPlugin
{
public:
  LinkForcePlugin();
  ~LinkForcePlugin();
    
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  bool LoadLookupTable(std::string filename);

  void OnUpdate();

  bool GetForces(int material, float depth, int pass, float rho,
                 std::vector<float>& out_forces);

  physics::LinkPtr m_link;

  // Connection to the update event
  event::ConnectionPtr m_updateConnection;

  //std::vector<ForceRow> m_forceRows;

  std::map<int, std::map<int, std::map<int, std::map<float, std::vector<float> > > > > m_forcesMap;
};

}

#endif // LinkForcePlugin_h
