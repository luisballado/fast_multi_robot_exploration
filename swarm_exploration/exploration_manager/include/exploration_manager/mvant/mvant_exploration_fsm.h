#ifndef _MVANT_EXPLORATION_MANAGER_H_
#define _MVANT_EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/SearchObstacle.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>
#include <exploration_manager/mvant/collaboration_assigner.h>
#include <bspline/Bspline.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;


namespace fast_planner {
class FastPlannerManager;
class MvantExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

inline Eigen::Vector3d geometryMsgToEigen(const geometry_msgs::Point& point_msg) {
  return Eigen::Vector3d(point_msg.x, point_msg.y, point_msg.z);
}
  
inline geometry_msgs::Point eigenToGeometryMsg(const Eigen::Vector3d& point) {
  geometry_msgs::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  return point_msg;
}

class MvantExplorationFSM {

public:
  MvantExplorationFSM(/* args */) {
  }
  ~MvantExplorationFSM() {
  }
  
  void init(ros::NodeHandle& nh);
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  //calcular distancia euclideana
  double calcularDistancia(double x, double y, double z,
			   double cx, double cy, double cz);
  
  /* helper functions */
  int callExplorationPlanner(); //original implementation
  int callPlannerExploration();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  void sendStopMsg(int code);
  void sendEmergencyMsg(bool emergency);
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void heartbitCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  /* Ejemplos */
  void nearbyObstaclesCallback(const ros::TimerEvent& e);//(const exploration_manager::SearchObstacle::ConstPtr& msg);
  double getDistance(Eigen::Vector3d& cloud_point, Eigen::Vector3d& point);
  //void pruebasCallback(const std_msgs::Empty::ConstPtr& msg);
  void explorationCallback(const ros::TimerEvent& e);

  void pruebaTopicoCallback(const std_msgs::Empty::ConstPtr& msg);
  void pruebaPararCallback(const std_msgs::Empty::ConstPtr& msg);
  void comunicacionCallback(const std_msgs::Int32::ConstPtr& msg);
  
  // Swarm
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<MvantExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;
  shared_ptr<CollaborationAssigner> coll_assigner_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, exploration_timer_;
  ros::Subscriber trigger_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // ******************************
  // *****  nearby obstacles ******
  // ******************************
  ros::Subscriber nearby_obs_sub_;
  ros::Publisher nb_obs_pub_;
  // ******************************
  // ***** Pruebas Dummy **********
  // ******************************
  ros::Publisher test_topico, test_topico2,test_msgs;
  ros::Subscriber topico_sub_, topico_sub_2,test_msgs_sub_;
  // ******************************

  //objetos
  struct Obstacle{
    geometry_msgs::Point position;
    geometry_msgs::Point velocity;
  };
  
  ros::Publisher cloud_pub_;
  ros::Timer timer_;
  std::vector<Obstacle> obstacles_; // Vector de obstaculos
  float max_velocity_;
  float min_velocity_;
  void timerCallback(const ros::TimerEvent& e);
  
  //pruebas
  ros::Publisher test_fronteras;
  ros::Subscriber test_sub_;
  
  // Logging
  ros::Timer heartbit_timer_;
  ros::Publisher stop_pub_, heartbit_pub_;
  
  // Emergency handler
  ros::Publisher emergency_handler_pub_;
  size_t num_fail_;

  // Swarm state
  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_,
      hgrid_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;
  ros::Timer prueba_nb;
};

}  // namespace fast_planner

#endif
