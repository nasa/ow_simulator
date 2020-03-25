/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 ******************************************************************************/

#include "ow_faults/FaultInjector.h"
#include <dynamic_reconfigure/server.h>


using namespace std;


int main(int argc, char **argv) {
  ros::init(argc, argv, "fault");
  ros::NodeHandle node_handle;

  dynamic_reconfigure::Server<ow_faults::FaultsConfig> server;
  dynamic_reconfigure::Server<ow_faults::FaultsConfig>::CallbackType f;

  FaultInjector fault_injector(node_handle);

  f = bind(&FaultInjector::faultsConfigCb, &fault_injector, placeholders::_1, placeholders::_2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
