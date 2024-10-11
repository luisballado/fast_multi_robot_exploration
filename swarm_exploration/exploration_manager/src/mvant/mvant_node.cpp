//mvant_node.cpp

#include <ros/ros.h>
#include <exploration_manager/mvant/mvant_exploration_fsm.h>
#include <plan_manage/backward.hpp> //Backward is a beautiful stack trace pretty printer for C++.

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  
  //ros::init(..) has to be called before calling other ROS functions
  ros::init(argc, argv, "mvant_node");

  //private node handle
  //The node handle is the access point for communications with the ROS system (topics,services,parameters)
  //Private node handle
  ros::NodeHandle nh("~");
  
  MvantExplorationFSM expl_mvant;
  expl_mvant.init(nh);
  
  ros::Duration(1.0).sleep();

  // Usar 4 hilos para manejar callbacks
  // no es necesario
  // ros::MultiThreadedSpinner spinner(4); 
  
  //processes callbacks and will not
  //return until the node has been shutdown 
  ros::spin();

  return 0;
}
