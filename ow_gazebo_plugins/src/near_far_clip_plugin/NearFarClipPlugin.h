/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef NearFarClipPlugin_h
#define NearFarClipPlugin_h


#include <gazebo/gui/GuiPlugin.hh>


namespace gazebo {

  class NearFarClipPlugin : public GUIPlugin
  {
  public:
    NearFarClipPlugin();
    ~NearFarClipPlugin();

    virtual void Load(sdf::ElementPtr _sdf);
  };
}

#endif // NearFarClipPlugin_h
