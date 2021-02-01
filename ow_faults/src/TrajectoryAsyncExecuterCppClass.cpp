// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ow_faults/TrajectoryAsyncExecuterCppClass.h"
#include <algorithm>

using namespace std;
using namespace ow_lander;

TrajectoryAsyncExecuterCppClass::TrajectoryAsyncExecuterCppClass(ros::NodeHandle node_handle)
{
    _connected = false;
    _goal_time_to_tolerance = 0.1;
    _client = NULL;

//   m_arm_state_sub = node_handle.subscribe("/system_faults_status", 10, &TrajectoryAsyncExecuterCppClass::armFailureCb, this);
//   // m_fault_arm_plan_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/fake_arm_plan", 10); 
//   m_fault_arm_plan_pub = node_handle.advertise<ow_faults::SystemFaults>("/fake_arm_plan", 10); 

}

void TrajectoryAsyncExecuterCppClass::connect(type controller){
    _controller = controller;
    _client = 
}

void TrajectoryAsyncExecuterCppClass::execute(trajectory, done_cb=NULL, active_cb=NULL, feedback_cb=NULL){
    if (!_connected) {
        warning(blah)
        return false;
    }
}

void TrajectoryAsyncExecuterCppClass::stop(){
    if (_connected) {
        _client.cancel_goal();
    }
}
void TrajectoryAsyncExecuterCppClass::wait(int timeout){
    if (_connected){
        _client.wait_for_result(timeout = DUration(timeout));
    }
}

type TrajectoryAsyncExecuterCppClass::result(){
    if (!_connected) {
        return NULL;
    }
    return _client.get_result();
}

// void TrajectoryAsyncExecuterCppClass::armFailureCb(const ow_faults::SystemFaults& msg)
// {
//   ow_faults::SystemFaults system_faults_msg;
//   FaultInjector::SystemFaults sf = FaultInjector::ArmExecutionError;
//   if (msg.value == sf) {
//     stopArmMovement();
//     system_faults_msg.value = 100;
//   }
//   m_fault_arm_plan_pub.publish(system_faults_msg);

// }


// Refer to the following website for more information about embedding the
// Python code in C++.
// https://docs.python.org/2/extending/embedding.html
// void TrajectoryAsyncExecuterCppClass::stopArmMovement()
// {
//   // PyObject *module_name, *module, *dict, *python_class, *object;

//   // // Initializes the Python interpreter
//   // Py_Initialize();

//   // module_name = PyBytes_FromString(
//   //     "trajectory_async_execution");

//   // // Load the module object
//   // module = PyImport_Import(module_name);
//   // if (module == nullptr) {
//   //   PyErr_Print();
//   //   std::cerr << "Fails to import the module.\n";
//   //   // return 1;
//   // }
//   // Py_DECREF(module_name);

//   // // dict is a borrowed reference.
//   // dict = PyModule_GetDict(module);
//   // if (dict == nullptr) {
//   //   PyErr_Print();
//   //   std::cerr << "Fails to get the dictionary.\n";
//   //   // return 1;
//   // }
//   // Py_DECREF(module);

//   // // Builds the name of a callable class
//   // python_class = PyDict_GetItemString(dict, "TrajectoryAsyncExecuter");
//   // if (python_class == nullptr) {
//   //   PyErr_Print();
//   //   std::cerr << "Fails to get the Python class.\n";
//   //   // return 1;
//   // }
//   // Py_DECREF(dict);

//   // // Creates an instance of the class
//   // if (PyCallable_Check(python_class)) {
//   //   object = PyObject_CallObject(python_class, nullptr);
//   //   Py_DECREF(python_class);
//   // } else {
//   //   std::cout << "Cannot instantiate the Python class" << std::endl;
//   //   Py_DECREF(python_class);
//   //   // return 1;
//   // }

//   // // int sum = 0;
//   // // int x;

//   // // for (size_t i = 0; i < 5; i++) {
//   // //   x = rand() % 100;
//   // //   sum += x;
//   // //   PyObject *value = PyObject_CallMethod(object, "add", "(i)", x); 
//   // //   if (value)
//   // //     Py_DECREF(value);
//   // //   else
//   // //     PyErr_Print();
//   // // }
//   // PyObject_CallMethod(object, "stop", nullptr);
//   // // std::cout << "the sum via C++ is " << sum << std::endl;

//   // std::getchar();
//   // Py_Finalize();

//   // return (0);
// }



