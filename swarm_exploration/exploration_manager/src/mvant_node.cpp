//mvant_node.cpp

#include <ros/ros.h>
#include <exploration_manager/mvant_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  //ros::init(..) has to be called before calling other ROS functions
  ros::init(argc, argv, "mvant_node");

  //private node handle
  ros::NodeHandle nh("~"); //The node handle is the access point for communications with the ROS system (topics,services,parameters)
  
  MvantExplorationFSM expl_mvant;
  expl_mvant.init(nh);
  
  ros::Duration(1.0).sleep();
  ros::spin();
  
  return 0;
}
