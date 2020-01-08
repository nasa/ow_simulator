/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef LinkForcePlugin_h
#define LinkForcePlugin_h

#include <gazebo/common/Plugin.hh>


namespace gazebo {

class LinkForcePlugin : public ModelPlugin
{
public:
  LinkForcePlugin();
  ~LinkForcePlugin();
    
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  bool loadLookupTable(std::string filename);

  void OnUpdate();

  physics::LinkPtr m_link;

  // Connection to the update event
  event::ConnectionPtr m_updateConnection;
};

}

#endif // LinkForcePlugin_h
