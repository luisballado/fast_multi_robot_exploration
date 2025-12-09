// mvant_exploration_manager.cpp

// #include <fstream>
#include <thread>
#include <iostream>
#include <fstream>

#include <chrono>

#include <exploration_manager/mvant/mvant_exploration_manager.h>
#include <exploration_manager/mvant/expl_data.h>

#include <boost/filesystem.hpp>

#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/hgrid.h>

#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/planner_manager.h>

//#include <lkh_tsp_solver/SolveTSP.h>
//#include <lkh_mtsp_solver/SolveMTSP.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

//Leer
//https://arxiv.org/pdf/2209.10775
//https://arxiv.org/pdf/2409.16972
//https://arxiv.org/pdf/2408.05808
//https://robotfrontier.com/
//http://robotfrontier.com/papers/agents98.pdf
//http://robotfrontier.com/papers/cira97.pdf
//https://luismejias21.wordpress.com/wp-content/uploads/2017/09/inteligencia-artificial-un-enfoque-moderno-stuart-j-russell.pdf
//file:///Users/luisalbertoballadoaradias/Downloads/Artificial-Intelligence-A-Modern-Approach-4th-Edition-1-compressed.pdf

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

MvantExplorationManager::MvantExplorationManager() {
}

MvantExplorationManager::~MvantExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void MvantExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));

  // uniform_grid_.reset(new UniformGrid(edt_environment_, nh));
  hgrid_.reset(new HGrid(edt_environment_, nh));

  //Quitar, dado a que yo no uso este esquema
  //role_assigner_.reset(new RoleAssigner(nh));
  
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));
  
  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  fp_.reset(new FSMParam); //expl_data.h //para tener el comm_range

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  //nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  //nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/drone_num", ep_->drone_num_, 1);
  nh.param("exploration/drone_id", ep_->drone_id_, 1);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);

  // Explorer Parameters
  // Quitar, dado a que yo no uso este esquema
  explorer_params_.reset(new ExplorerParams);
  nh.param("explorer/ftr_max_distance", explorer_params_->ftr_max_distance, 15.0);
  nh.param("explorer/max_ang_dist", explorer_params_->max_ang_dist, M_PI / 4.0);
  nh.param("explorer/label_penalty", explorer_params_->label_penalty, 10.0);
  nh.param("explorer/w_distance", explorer_params_->w_distance, 1.0);
  nh.param("explorer/w_direction", explorer_params_->w_direction, 1.0);
  nh.param("explorer/w_others", explorer_params_->w_others, 1.0);
  nh.param("explorer/w_previous_goal", explorer_params_->w_previous_goal, 1.0);

  // Collector Parameters
  // Quitar, dado a que yo no uso este esquema
  collector_params_.reset(new CollectorParams);
  nh.param("collector/min_vel", collector_params_->min_vel, 1.0);
  nh.param("collector/label_penalty", collector_params_->label_penalty, 15.0);
  nh.param("collector/velocity_factor", collector_params_->velocity_factor, 2.0);
  nh.param("collector/w_distance", collector_params_->w_distance, 1.0);
  nh.param("collector/w_direction", collector_params_->w_direction, 1.0);
  nh.param("collector/w_others", collector_params_->w_others, 1.0);
  nh.param("collector/w_previous_goal", collector_params_->w_previous_goal, 1.0);

  // Use the same parameter as in role assigner
  nh.param("role_assigner/region_size", collector_params_->ftr_max_distance, 8.0);

  // Potential Field Parameters
  pf_params_.reset(new PotentialFieldParams);
  nh.param("potential_field/ka", pf_params_->ka, 1.0);
  nh.param("potential_field/kr", pf_params_->kr, 1.0);
  nh.param("potential_field/d0", pf_params_->d0, 10.0);
  nh.param("potential_field/df", pf_params_->df, 12.0);
  nh.param("potential_field/dc", pf_params_->dc, 5.0);

  nh.param("fsm/communication_range", fp_->communication_range_, std::numeric_limits<double>::max());


  assert(pf_params_->ka >= 0.);
  assert(pf_params_->kr >= 0.);
  
  // Initial role
  // Quitar, dado a que yo no uso este esquema
  role_ = ROLE::UNKNOWN;

  //ed - exploration data
  // se supone que cada nodo se levanta respecto a los num de drones
  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);

  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }

  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);
  
  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  //tsp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
  //acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_acvrp_" + to_string(ep_->drone_id_), true);
  
  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;
  ed_->plan_num_ = 0;

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

// Planificar hacia una nueva frontera 
int MvantExplorationManager::planExploreMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  
  //ROS_WARN_STREAM("start pos: " << pos.transpose() << ", vel: " << vel.transpose() << ", acc: " << acc.transpose());
  //ROS_WARN_STREAM("vel: " << vel.transpose() << ", acc: " << acc.transpose());
  
  Vector3d next_pos;
  double next_yaw;
  bool success;

  //Elegir optimizador basado en el numero de fronteras
  const int ftr_num = frontier_finder_->getFrontiers().size();
  const int drone_num = ed_->swarm_state_.size() - 1; //cero basado

  //TODO
  //cambiar velocidad aqui respecto a algun criterio
  /*
  const ROLE updated_role = role_assigner_->assignRole(
      pos, ep_->drone_id_, ed_->swarm_state_, frontier_finder_->getFrontiers());
  updateRoleAndVelocities(updated_role);
  */
  
  // Aplicar factor de velocidad
  //updateVelocities(0.4);

  //Usar diferentes estrategias basado en el tamaño del problema
  /*
  if (ftr_num < 10 || drone_num == 1) {
    //success = explorerPlan(pos, vel, yaw, next_pos, next_yaw);
    //success = findPathClosestFrontier(pos, vel, yaw, next_pos, next_yaw);
    //ROS_ERROR("**findPathClosestFrontier**");
    success = closestGreedyFrontier(pos, yaw, next_pos, next_yaw);
  } else if (ftr_num < 50){
    success = closestGreedyFrontierUltraFast(pos, yaw, next_pos, next_yaw);
    ROS_ERROR("**UltraFast**");
  } else {
    success = closestGreedyFrontierUltraFast(pos, yaw, next_pos, next_yaw);
    ROS_ERROR("**UltraFast2**");
  }
  */

  success = closestGreedyFrontier(pos, yaw, next_pos, next_yaw); //explorerPlan(pos, vel, yaw, next_pos, next_yaw);

  if (!success) {
    ROS_ERROR("no hay ftr frente, buscar greedy");
    success = closestGreedyFrontier(pos, yaw, next_pos, next_yaw);
  }

  // obtuvimos un nuevo objetivo (posicion de la frontera)
  if (success) {
    
    //ROS_WARN_STREAM("Nueva ubicacion: " << next_pos.transpose() << ", " << next_yaw);
    
    // Check if we have been trying to access them too often
    // evita quedar en un minimo local
    const double kMinDistGoals = 0.2;
    // si el nuevo objetivo esta muy cerca iremos manejando un
    // contador para evitar quedarnos atrapados aqui
    if ((next_pos - ed_->next_pos_).norm() < kMinDistGoals) {
      ++ed_->num_attempts_;
    } else {
      ed_->num_attempts_ = 0;
    }

    //forzar frontera diferente despues de tres intentos
    const size_t kMaxAttempts = 3;
    if (ed_->num_attempts_ > kMaxAttempts) {
      bool force_different = true;
      success = closestGreedyFrontier(pos, yaw, next_pos, next_yaw);
      /**
      if (ftr_num < 10 || drone_num == 1) {
        success = closestGreedyFrontier(pos, yaw, next_pos, next_yaw);
      } else if (ftr_num < 50){
        success = closestGreedyFrontierOptimized(pos, yaw, next_pos, next_yaw);
      } else {
        success = closestGreedyFrontierUltraFast(pos, yaw, next_pos, next_yaw);
      }
      **/
    }

    ed_->next_pos_ = next_pos;
    ed_->next_yaw_ = next_yaw;

  } else {

    //quedarse en hover
    //regreso falla
    next_pos = ed_->next_pos_;
    next_yaw = ed_->next_yaw_;
    return FAIL;
  }

  // planificar la trayectoria que seguira el robot
  // si no se puede llegar regresara fallido
  // Algoritmo A* llega a su fin de explorar y no encuentre nada
  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  double total = (ros::Time::now() - t2).toSec();
  ROS_INFO("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  //aca estoy guardando cosas
  std::string filename = "/home/catkin_ws/logs/file"+ std::to_string(ep_->drone_id_) +".txt";
  //std::string filename = "/home/catkin_ws/logs/file_optimized_" + std::to_string(ep_->drone_id_) + ".txt";
  std::ofstream outfile(filename,std::ios::app);
  outfile << "Total time:" << total << std::endl;
  
  ros::Duration dt = ros::Time::now() - t2;
  double secs = dt.toSec();
  double ms = dt.toSec() * 1e3;
  
  outfile << std::fixed << std::setprecision(3) << ms << " ms" << std::endl;  // ej: 0.004 ms
  double us = dt.toSec() * 1e6;
  
  outfile << std::fixed << std::setprecision(1) << us << " µs" << std::endl;  // ej: 3.8 µs
  
  outfile << "tiempo: " << secs << " s\n" << std::endl;
  outfile.close();

  return SUCCEED;
}

bool MvantExplorationManager::isPositionReachable(const Vector3d& from, const Vector3d& to) const {
  /*if (sdf_map_->getInflateOccupancy(to) != SDFMap::OCCUPANCY::FREE) {
    // End position in occupied or unknown space
    return false;
  } else if (planner_manager_->path_finder_->search(from, to, false) != Astar::REACH_END) {
    // Dynamics-aware path finder doesn't find a solution
    return false;
  }*/
  
  if (!ViewNode::validPathExists(from, to)) {
    // Discreate A* cannot find a solution
    return false;
  } else {
    // Position is cool
    return true;
  }
}

// me regresa un camino hacia un siguiente 
// punto siempre que exista
// se usa para replanificar
int MvantExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = ros::Time::now();
  
  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;
  
  // Generate trajectory of x,y,z
  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  // bool start_unknown = (edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN);
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;

  //reseteo de planner
  planner_manager_->path_finder_->reset();
  
  //busqueda con A* hacia el siguiente objetivo
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END) {
    ROS_ERROR("No path to next viewpoint");
    return FAIL;
  }

  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;

  //distancia
  const double len = Astar::pathLength(ed_->path_next_goal_);
  
  //si la proxima posicion esta cerca - va hacia ella
  //si la proxima posicion esta lejos - usa una posicion del path para lidear con caminos sin salida

  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      ROS_WARN("init plan.");
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    // std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    // std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    ROS_ERROR("Lower bound not satified!");

  double traj_plan_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  
  return SUCCEED;
}

int MvantExplorationManager::updateFrontierStruct(
    const Eigen::Vector3d& pos, double yaw, const Eigen::Vector3d& vel) {

  auto t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  
  // buscar fronteras y agruparlas bfs
  frontier_finder_->searchFrontiers();
  
  // tiempo para calcular fronteras
  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();
  
  // Find viewpoints (x,y,z,yaw) for all clusters;
  // find the informative ones
  frontier_finder_->computeFrontiersToVisit();
  
  // Binary classification of frontiers
  //frontier_finder_->binaryClassify();

  // Get frontiers in front
  frontier_finder_->updateFrontiersInFront(
      pos, vel, yaw, explorer_params_->ftr_max_distance, explorer_params_->max_ang_dist);
  
  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontiersIds(ed_->fronters_ids_);
  //frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  //frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  //frontier_finder_->getLabeledFrontiers(ed_->labeled_frontiers_);
  frontier_finder_->getInFrontFrontiers(ed_->infront_frontiers_);
  
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 1.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  if (ed_->frontiers_.empty()) {
    ROS_WARN("no fronteras");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();
  
  t1 = ros::Time::now();

  //hacer mi updateFrontierCostMatrix
  //aqui debo tener informacion para hacer las comparaciones y llenar la matrix
  //frontier_finder_->updateFrontierCostMatrix();
  
  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  //ROS_WARN_STREAM("Drone: " << ep_->drone_id_<< ", frontier t: " << frontier_time << ", viewpoint t: " << view_time << ", mat: " << mat_time);
  ROS_INFO("Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf", ep_->drone_id_, frontier_time,view_time, mat_time);
  
  ROS_INFO("Total t: %lf", (ros::Time::now() - t2).toSec());
  return ed_->frontiers_.size();
}

void MvantExplorationManager::updateVelocities(const double factor) {

  double velocity_factor = factor;
  
  planner_manager_->pp_.max_vel_ *= velocity_factor;
  double curr_vmax = planner_manager_->getKinodynamicAstarVMax();
  planner_manager_->setKinodynamicAstarVMax(velocity_factor * curr_vmax);
  curr_vmax = planner_manager_->getMaxVelBsplineOpt();
  planner_manager_->setMaxVelBsplineOpt(velocity_factor * curr_vmax);
  
  planner_manager_->pp_.max_acc_ *= velocity_factor;
  double curr_amax = planner_manager_->getKinodynamicAstarAMax();
  planner_manager_->setKinodynamicAstarAMax(velocity_factor * curr_amax);
  curr_amax = planner_manager_->getMaxAccBsplineOpt();
  planner_manager_->setMaxAccBsplineOpt(velocity_factor * curr_amax);
  
  planner_manager_->pp_.accept_vel_ = planner_manager_->pp_.max_vel_ + 0.5;
  planner_manager_->pp_.accept_acc_ = planner_manager_->pp_.max_acc_ + 0.5;
  
  ViewNode::vm_ *= velocity_factor;
  ViewNode::am_ *= velocity_factor;
}
  
//esquema fame
/*
void MvantExplorationManager::updateRoleAndVelocities(const ROLE updated_role) {
  // Update velocities
  const double inv_factor = 1. / collector_params_->velocity_factor;

  if ((role_ == ROLE::EXPLORER || role_ == ROLE::UNKNOWN) &&
      updated_role == ROLE::GARBAGE_COLLECTOR) {
    // Explorer / Unknown -> GC
    planner_manager_->pp_.max_vel_ *= collector_params_->velocity_factor;
    double curr_vmax = planner_manager_->getKinodynamicAstarVMax();
    planner_manager_->setKinodynamicAstarVMax(collector_params_->velocity_factor * curr_vmax);
    curr_vmax = planner_manager_->getMaxVelBsplineOpt();
    planner_manager_->setMaxVelBsplineOpt(collector_params_->velocity_factor * curr_vmax);

    planner_manager_->pp_.max_acc_ *= collector_params_->velocity_factor;
    double curr_amax = planner_manager_->getKinodynamicAstarAMax();
    planner_manager_->setKinodynamicAstarAMax(collector_params_->velocity_factor * curr_amax);
    curr_amax = planner_manager_->getMaxAccBsplineOpt();
    planner_manager_->setMaxAccBsplineOpt(collector_params_->velocity_factor * curr_amax);

    planner_manager_->pp_.accept_vel_ = planner_manager_->pp_.max_vel_ + 0.5;
    planner_manager_->pp_.accept_acc_ = planner_manager_->pp_.max_acc_ + 0.5;

    ViewNode::vm_ *= collector_params_->velocity_factor;
    ViewNode::am_ *= collector_params_->velocity_factor;
  } else if (role_ == ROLE::GARBAGE_COLLECTOR && updated_role == ROLE::EXPLORER) {
    // GC -> Explorer
    planner_manager_->pp_.max_vel_ *= inv_factor;
    double curr_vmax = planner_manager_->getKinodynamicAstarVMax();
    planner_manager_->setKinodynamicAstarVMax(inv_factor * curr_vmax);
    curr_vmax = planner_manager_->getMaxVelBsplineOpt();
    planner_manager_->setMaxVelBsplineOpt(inv_factor * curr_vmax);

    planner_manager_->pp_.max_acc_ *= inv_factor;
    double curr_amax = planner_manager_->getKinodynamicAstarAMax();
    planner_manager_->setKinodynamicAstarAMax(inv_factor * curr_amax);
    curr_amax = planner_manager_->getMaxAccBsplineOpt();
    planner_manager_->setMaxAccBsplineOpt(inv_factor * curr_amax);

    planner_manager_->pp_.accept_vel_ = planner_manager_->pp_.max_vel_ + 0.5;
    planner_manager_->pp_.accept_acc_ = planner_manager_->pp_.max_acc_ + 0.5;

    ViewNode::vm_ *= inv_factor;
    ViewNode::am_ *= inv_factor;
  }

  // Update role
  role_ = updated_role;
}
*/

bool MvantExplorationManager::explorerPlan(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {

  // If don't have frontiers in front, then go the closest frontier
  if (ed_->infront_frontiers_.empty()) {
    ROS_ERROR("No frontal frontiers - back to greedy");
    return findPathClosestFrontier(pos, vel, yaw, next_pos, next_yaw);
  }

  // Iterate over frontal frontiers
  auto distanceCost = [&](const Vector3d& target_pos) {
    std::vector<Vector3d> path;
    return explorer_params_->w_distance * ViewNode::searchPath(pos, target_pos, path);
  };
  
  auto angularCost = [&](const Vector3d& target_pos) {
    Vector3d direction(target_pos - pos);
    return explorer_params_->w_direction *
           vel.head(2).normalized().dot(direction.head(2).normalized());
  };
  
  /**
  auto labelCost = [&](LABEL label) {
    if (role_ == ROLE::EXPLORER && label == LABEL::TRAIL)
      return explorer_params_->label_penalty;
    else if (role_ == ROLE::GARBAGE_COLLECTOR && label == LABEL::FRONTIER)
      return explorer_params_->label_penalty;
    else
      return 0.0;
  };
  */

  auto totalCost = [&](const Vector3d& target_pos, bool verbose = true) {
    double distance_cost = distanceCost(target_pos);
    double angular_cost = angularCost(target_pos);
    //double formation_cost = explorer_params_->w_others * formationCost(target_pos);
    double previous_goal_cost = explorer_params_->w_previous_goal * previousGoalCost(target_pos);

    if (verbose) {
      std::cout << std::endl;
      std::cout << "Distance cost: " << distance_cost << std::endl;
      std::cout << "Angular cost: " << angular_cost << std::endl;
      //std::cout << "Label cost: " << label_cost << std::endl;
      //std::cout << "Formation cost: " << formation_cost << std::endl;
      std::cout << "Previous goal cost: " << previous_goal_cost << std::endl;
      std::cout << std::endl;
    }

    //return distance_cost + angular_cost + label_cost + formation_cost + previous_goal_cost;
    return distance_cost + angular_cost + previous_goal_cost;
  };

  double min_cost = std::numeric_limits<double>::max();
  bool found_goal = false;
  for (const auto& ftr : ed_->infront_frontiers_) {
    // Re-compute centroid
    // FIXME This has already been calculated
    Eigen::Vector3d centroid(Eigen::Vector3d::Zero());
    for (const auto& cell : ftr.second) centroid += cell;
    centroid /= ftr.second.size();

    // Check that the position is valid
    if (!isPositionReachable(pos, centroid)) {
      continue;
    }

    // Total cost
    double total_cost = totalCost(centroid, ftr.first);

    if (total_cost < min_cost) {

      min_cost = total_cost;
      // Compute next position
      Vector3d diff_vec = centroid - pos;
      Vector3d direction = diff_vec.normalized();
      next_yaw = atan2(direction.y(), direction.x());

      if (diff_vec.head(2).norm() < explorer_params_->ftr_max_distance &&
          diff_vec.head(2).norm() >= 0.5) {
        next_pos = centroid;
      } else {

        double range = std::min(explorer_params_->ftr_max_distance, diff_vec.norm());
        next_pos = pos + range * direction;
        while (sdf_map_->getOccupancy(next_pos) != SDFMap::OCCUPANCY::FREE && range >= 0.5) {
          range -= 0.5;
          next_pos = pos + range * direction;
        }
      }

      // Found a path
      found_goal = true;
    }
  }

  return found_goal;

  // If we haven't found a goal, then go to greedy approach
  // if (!found_goal) {
  //   //ROS_ERROR("Goal not found - back to greedy");
  //   return findPathClosestFrontier(pos, vel, yaw, next_pos, next_yaw);
  // } else {
  //   return true;
  // }
}

bool MvantExplorationManager::findPathClosestFrontier(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) const {

  // Iterate over the frontiers, and compute the path to the top viewpoint of
  // each of them. Then grab the best one
  double min_cost = std::numeric_limits<double>::max();
  bool found_ftr = false;
  for (const auto& ftr : frontier_finder_->getFrontiers()) {
    // Get the best viewpoint for the current frontier
    const auto& vp = ftr.viewpoints_.front();

    // Check that the position is valid
    if (!isPositionReachable(pos, vp.pos_)) {
      continue;
    }

    double dist = (pos - vp.pos_).norm();
    bool valid_dist = dist <= explorer_params_->ftr_max_distance && dist >= 0.5;
    if (!valid_dist) {
      continue;
    }

    vector<Vector3d> path;
    double distance = std::max(1., explorer_params_->w_distance) *
                      ViewNode::computeCost(pos, vp.pos_, yaw[0], vp.yaw_, vel, yaw[1], path);
    double formation_cost = explorer_params_->w_others * formationCost(vp.pos_);
    double previous_goal_cost = explorer_params_->w_previous_goal * previousGoalCost(vp.pos_);
    double cost = distance + formation_cost + previous_goal_cost;

    if (cost < min_cost) {
      // Update flag
      found_ftr = true;
      // Target
      min_cost = cost;
      next_pos = vp.pos_;
      next_yaw = vp.yaw_;
    }
  }

  return found_ftr;
}

  /*Exploracion principal*/
  
  class Frontera {
  public:
    int id;   
    float costo;
    Vector3d pos_;
    double yaw_;
    //int edad;
  };
  
  const Frontera& findMinFrontera(const std::list<Frontera>& fronteras){

    auto frontera = fronteras.begin();
    
    for (auto i = fronteras.begin(); i != fronteras.end(); ++i) {
        if (i->costo < frontera->costo) {
            frontera = i;
        }
    }

    /**
      auto it = std::min_element(fronteras.begin(),fronteras.end(),[](const Frontera& a, const Frontera& b){
      return a.distance < b.distance;
    });
    */
    //ROS_WARN_STREAM("DRONE BUSCANDO MINIMO:: ");

    return *frontera; //regresar un apuntador al valor
  }


  std::vector<int> HungarianAlgorithm(Eigen::MatrixXd& costMatrix) {
  
      // Save the original matrix dimensions.
    int origRows = costMatrix.rows();
    int origCols = costMatrix.cols();
    
    const double INF = std::numeric_limits<double>::infinity();
    int n = costMatrix.rows();  // Now 'cost' is an n x n square matrix.
    
    // Step 2: Initialize the dual variables (u and v) and auxiliary arrays (p and way).
    // We use 1-indexed arrays for convenience (index 0 is used as a dummy).
    std::vector<double> u(n + 1, 0), v(n + 1, 0);
    std::vector<int> p(n + 1, 0), way(n + 1, 0);
    
    // Process each row (from 1 to n) to build the matching.
    for (int i = 1; i <= n; i++) {
        p[0] = i;           // Initialize the dummy column with the current row.
        int j0 = 0;         // j0 is the current column in the alternating path.
        std::vector<double> minv(n + 1, INF);  // minv[j] will hold the minimum reduced cost for column j.
        std::vector<bool> used(n + 1, false);  // used[j] indicates whether column j is visited.
        
        // Find an augmenting path.
        do {
            used[j0] = true;
            int i0 = p[j0]; // The row currently matched with column j0.
            int j1 = 0;
            double delta = INF;
            
            // Update the minimum values for all columns not yet visited.
            for (int j = 1; j <= n; j++) {
                if (!used[j]) {
                    // Compute the reduced cost for assigning row i0 to column j.
                    double cur = costMatrix(i0 - 1, j - 1) - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;  // Record the predecessor of column j.
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;  // j1 is the candidate for the next column.
                    }
                }
            }
            
            // Update the dual variables.
            for (int j = 0; j <= n; j++) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;  // Move to the next column.
        } while (p[j0] != 0); // Continue until an unmatched column is found.
        
        // Update the matching along the alternating path.
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    
    // Step 3: Build the assignment result.
    // For each column j (1-indexed), p[j] is the row assigned to j.
    // We only care about assignments for rows in the original matrix.
    std::vector<int> assignment(origRows, -1);
    for (int j = 1; j <= n; j++) {
        int i = p[j]; // i is the row (1-indexed) assigned to column j.
        // Only record the assignment if it is within the original dimensions.
        if (i <= origRows && j <= origCols) {
            assignment[i - 1] = j - 1;  // Convert to 0-indexed.
        }
    }
    
    return assignment;
  }

  //calcular costo yaw
  double compute_yaw_cost(double desired_yaw, double current_yaw) {
  
    double delta_yaw = std::atan2(std::sin(desired_yaw - current_yaw), std::cos(desired_yaw - current_yaw));
    return (1.0 - std::cos(delta_yaw)) / 2.0; //[0,1]

  }
  
  // Compute yaw and pitch from robot to frontier
  std::pair<double, double> compute_yaw_pitch_to_frontier(const Eigen::Vector3d& robot_position, const Eigen::Vector3d& frontier_position) {
    // Compute difference in position (frontier - robot)
    //calculo de direccion
    Eigen::Vector3d delta = frontier_position - robot_position;
    
    // Extract the differences in x, y, z components
    // componentes para calcular angulos
    double dx = delta.x();
    double dy = delta.y();
    double dz = delta.z();
    
    // Calculate yaw (horizontal rotation) in radians
    double yaw_target = std::atan2(dy, dx);  // atan2 gives result in the range [-π, π]
    
    // Calculate pitch (vertical rotation) in radians
    double horizontal_distance = std::sqrt(dx * dx + dy * dy);  // Distance on the XY plane
    double pitch_target = std::atan2(dz, horizontal_distance);  // atan2 gives result in the range [-π/2, π/2]
    
    return std::make_pair(yaw_target, pitch_target);  // Return yaw and pitch as a pair
  }

  double compute_direction_cost(const Eigen::Vector3d& robot_position,const Eigen::Vector3d& robot_vel, const Eigen::Vector3d& frontier_position){

    auto direction = (frontier_position - robot_position).head(2).normalized();
    //double dot = drone_state.vel_.head(2).normalized().dot(direction);
    //double direction_cost = ViewNode::w_dir_ * (1.0 - (drone_state.vel_.head(2).normalized().dot(direction.normalized())));
    double direction_cost = 0.0;
    Eigen::Vector2d vel2d = robot_vel.head(2);
    if (vel2d.norm() < 1e-3) {
        direction_cost = 0.5;  // o máximo, o neutro
    } else {
        double dot = vel2d.normalized().dot(direction);
        dot = std::max(-1.0, std::min(1.0, dot));

        direction_cost = (1.0 - dot) / 2.0;
    }
    return direction_cost;
  }

  //probar con dist eucl directo**
  double compute_distance_cost(const Eigen::Vector3d& robot_position,const Eigen::Vector3d& frontier_position,bool euclidean = true){
    if(!euclidean){
      std::vector<Vector3d> path;
      return ViewNode::searchPath(robot_position, frontier_position, path);
    } else{
      return (robot_position - frontier_position).norm();
    }
  }

  //Funcion principal de exploración con todos los elementos
  bool MvantExplorationManager::closestGreedyFrontier(const Vector3d& pos, const Vector3d& yaw, Vector3d& next_pos, double& next_yaw, bool force_different)  {

    const int drone_num = ed_->swarm_state_.size() - 1;
    const int ftr_num = frontier_finder_->getFrontiers().size();

    //archivo por drone
    std::string filename = "/home/catkin_ws/logs/file"+ std::to_string(ep_->drone_id_) +".txt";
    std::ofstream outfile(filename,std::ios::app);

    outfile << "\n------------------" << std::endl;
    outfile << "Datos para el Drone: " << (ep_->drone_id_) << std::endl;
    outfile << "Cardinalidad VANTS:" << drone_num << std::endl;
    outfile << "numero de fronteras: " << ftr_num << std::endl;

    Frontera front1;
    list<Frontera> fronteras;
    //ed_->fronteras = {};
    
    bool found_ftr = false;
        
    //Inicializar lista de valores de fronteras
    //que esta viendo el dron que esta ejecutando
    //esto para usarla cuando es un solo robot

    for (const auto& ftr : frontier_finder_->getFrontiers()) {
      
      //aqui estoy calculando todas en la iteracion
      Vector3d diff_vec = ftr.average_ - pos;
      Vector3d direction = diff_vec.normalized();
      next_yaw = atan2(direction.y(), direction.x());

      //obtener el view point de la frontera
      Viewpoint vp = ftr.viewpoints_.front();
      double min_dist = std::numeric_limits<double>::max();  
      
      double dist_ftr = 0;

      if (!isPositionReachable(pos,ftr.average_)){
        continue;
      }

      //calcular distancia entre dos posiciones con la norma
      dist_ftr = compute_distance_cost(pos,ftr.average_,true);

      //para quedarme con la mas corta
      if (dist_ftr < min_dist){
        const double kMinDistGoals = 1.0;
        if (force_different && (ftr.average_ - ed_->next_pos_).norm() < kMinDistGoals) {
          continue;
        }
        min_dist = dist_ftr;
      }

      //no brincar fronteras
      //aqui hay infinitos agregar y se deben considerar
      //origen, destino
      if (!isPositionReachable(pos, ftr.average_)) {
        front1.id = ftr.id_;
        front1.costo = 10000.0;
        front1.pos_ = ftr.average_;
        front1.yaw_ = next_yaw;
        //front1.edad = edad_normalizada;  //Insertamos edad
        fronteras.push_back(front1); 
        
        //ed_->fronteras.push_back(ftr.id_);  //<<<<--- en que lo uso?
        continue;
      }
      
      double distance_cost = std::min(min_dist / 20.0, 1.0);
            
      double yaw_cost = compute_yaw_cost(next_yaw,ed_->swarm_state_[ep_->drone_id_].yaw_);
      //outfile << "\ncosto yaw: " << (yaw_cost) << std::endl;

      // Direction
      double direction_cost = compute_direction_cost(pos,ed_->swarm_state_[ep_->drone_id_].vel_, ftr.average_);
      
      double total_cost;// = 0.35 * distance_cost + 0.4 * yaw_cost + 0.25 * direction_cost;
      
      if (min_dist < 5.0) {
          total_cost = 0.6 * distance_cost + 0.25 * yaw_cost + 0.15 * direction_cost;
      } else {
          total_cost = 0.35 * distance_cost + 0.4 * yaw_cost + 0.25 * direction_cost;
      }

      front1.id = ftr.id_;
      front1.costo = total_cost;
      front1.pos_ = ftr.average_;
      front1.yaw_ = next_yaw;
      //front1.edad = edad_normalizada;  //Insertamos edad

      //outfile << "min_dist " << min_dist << std::endl;
      //outfile << "distance cost: " << distance_cost << std::endl;
      //outfile << "yaw cost: " << yaw_cost << std::endl;
      //outfile << "direction_cost: " << direction_cost << std::endl;
      outfile << "costo ftr " << ftr.id_ << " : " << total_cost << std::endl;

      fronteras.push_back(front1);
      
      //no lo estoy usando, pero seria interesante usarlo
      //ed_->fronteras.push_back(ftr.id_);
      
    }

    // se actualizan las edades de las fronteras
    // cada que se un robot calcula una nueva frontera
    //edades_fronteras = nuevas_edades;

    //ROS_WARN_STREAM("[MANAGER] drone ffr: " << ed_->fronteras.size());
    //outfile << "fronteras con dist: " << ed_->fronteras.size() << std::endl;

    //copiar los elementos a un nuevo vector?
    //std::vector<Frontera> fronteras_vector(fronteras.begin(), fronteras.end());

    double min_dist;

    //hacer la matriz cuadrada cuando la cardinalidad de vants sea diferente a la de fronteras
    if(ftr_num >= drone_num && drone_num > 1){      
      
      int nRows = drone_num;
      int nCols = ftr_num;
      
      int n = std::max(nRows, nCols);

      Eigen::MatrixXd mat;
      
      //matriz cuadrada respecto a la cardinalidad mas alta
      mat.resize(n,n);  
    
      //llenar matriz con infinitos
      mat.setConstant(100000.0);

      /*
      for (int i = 0; i < drone_num; ++i) {

        int index = 0;

        const auto& drone_state = ed_->swarm_state_[i]; //estado de los demas vants

        //double rho_k = compute_distance_cost(drone_state.pos_,drone_state.goal_pos_);

        for (const auto& ftr : frontier_finder_->getFrontiers()) {
              
          //no deberia tomar la primera de la lista
          //no tienen un orden
          Viewpoint vj = ftr.viewpoints_.front();
          
          double rho_k = compute_distance_cost(drone_state.pos_,drone_state.goal_pos_);
          //double rho_k = compute_distance_cost(drone_state.pos_,vj.pos_);
      	  double alpha_ki = compute_distance_cost(drone_state.goal_pos_,vj.pos_);
                
      	  double explotacion = rho_k + alpha_ki;

          double direction_cost = compute_direction_cost(drone_state.pos_,drone_state.vel_, vj.pos_);
          auto [yaw, pitch] = compute_yaw_pitch_to_frontier(drone_state.pos_, vj.pos_);
          
          double yaw_cost = compute_yaw_cost(yaw,drone_state.yaw_);

          //enfocarse solamente en distancias
          //es una heuristica, una idea para inicializarla/guiar la exploracion

          double sum = 0.0;
          for (int j = 0; j < drone_num; ++j) {
            if (j == i) continue;  // Excluir el robot evaluador

            //distancia desde el robot j a la frontera candidata
            //double rho_j = compute_distance_cost(ed_->swarm_state_[j].pos_,vj.pos_);
            double rho_j = compute_distance_cost(ed_->swarm_state_[j].pos_,ed_->swarm_state_[j].goal_pos_);


            //distancia desde el objetivo actual del robot j a la frontera 
            double alpha_ji = compute_distance_cost(ed_->swarm_state_[j].goal_pos_,vj.pos_); // O donde almacenes la meta

            //inversa suavizada evitando division por cero
            sum += rho_j + alpha_ji;  
          }

          double exploracion = sum / (drone_num - 1);
          
          //function costo va aqui
          mat(i,index) = explotacion;// - exploracion;
          ++index;

        }

      }
      */

      for (int i = 0; i < drone_num; ++i) {

        int index = 0;

        //const auto& drone_state = ed_->swarm_state_[i]; //estado de los demas vants

        //double rho_k = compute_distance_cost(drone_state.pos_,drone_state.goal_pos_);

        for (const auto& ftr : frontier_finder_->getFrontiers()) {
          
          const auto& drone_state = ed_->swarm_state_[i];
          Viewpoint vj = ftr.viewpoints_.front();
          
          Vector3d diff_vec = ftr.average_ - pos;
          Vector3d direction = diff_vec.normalized();
          next_yaw = atan2(direction.y(), direction.x());

          double rho_k = compute_distance_cost(drone_state.pos_,vj.pos_);
          double alpha_ki = compute_distance_cost(drone_state.goal_pos_,vj.pos_);
          double yaw_cost = compute_yaw_cost(next_yaw,ed_->swarm_state_[ep_->drone_id_].yaw_);
          double direction_cost = compute_direction_cost(pos,ed_->swarm_state_[ep_->drone_id_].vel_, ftr.average_);

          double explotacion = 0.175 * rho_k + 0.175 * alpha_ki + 0.4 * yaw_cost +  0.25 * direction_cost;

          double sum = 0.0;
          
          for (int j = 0; j < drone_num; ++j) {
              if (j == i) continue;  // Excluir el robot evaluador

              //distancia desde el robot j a la frontera candidata
              double rho_j = compute_distance_cost(ed_->swarm_state_[j].pos_,vj.pos_);

              //distancia desde el objetivo actual del robot j a la frontera 
              double alpha_ji = compute_distance_cost(ed_->swarm_state_[j].goal_pos_,vj.pos_); // O donde almacenes la meta

              //inversa suavizada evitando division por cero
              sum += rho_j + alpha_ji;
              
          }

          double exploracion = sum / (drone_num - 1);

          //function costo va aqui
          mat(i,index) = explotacion;//- exploracion;
          ++index;

        }

      }

      const int dimension = mat.rows();

      outfile << "Cardinalidad de VANTS: " << drone_num;
      outfile << " :: Fronteras: " << fronteras.size();
      outfile << " :: dimension matriz a crear: " << dimension << std::endl;
      // Escribit la matriz:
      // Iterar en las filas
      for (int i = 0; i < mat.rows(); i++) {
          // Iterar en cada columna dentro de la fila
          for (int j = 0; j < mat.cols(); j++) {
              // Escibir los elementos
              outfile << mat(i, j);
              if (j < mat.cols() - 1)
                  outfile << " ";
          }
          // salto de linea
          outfile << std::endl;
      }

      
      // for(auto item: fronteras){
      //   outfile << "frt:" << item.id << std::endl;
      //   outfile << "dist:" << item.distance << "\n" << std::endl;
      // }
      
      // calcular tiempo asignacion?
      std::vector<int> assignment = HungarianAlgorithm(mat);

      //ver el resultado de aqui
      outfile << "Asignaciones (robot -> frontera):" << std::endl;
      
      for (size_t i = 0; i < assignment.size(); i++) {
          outfile << "Robot " << i+1 << " asignado columna " << assignment[i] << std::endl;
      }

      outfile << "Robot " << ep_->drone_id_ << " Asignado a " << assignment[ep_->drone_id_-1] << " : " << mat(ep_->drone_id_-1,assignment[ep_->drone_id_-1]) << std::endl;
          

      //asignar frontera    
      // Update flag
      found_ftr = true;  
      
      int item = assignment[ep_->drone_id_-1];  // We want to get the element at index 1.
      
      // Use std::next to get an iterator advanced by 'item' positions.
      auto it = std::next(fronteras.begin(), item);
      outfile << "Frontera " << item  << std::endl;
      if (it != fronteras.end()) {
        min_dist = it->costo;
        //next_pos = it->pos_;
        //next_yaw = it->yaw_;

        // Compute next position
        Vector3d diff_vec = it->pos_ - pos;
        Vector3d direction = diff_vec.normalized();
        next_yaw = atan2(direction.y(), direction.x());

        if (diff_vec.head(2).norm() < explorer_params_->ftr_max_distance &&
            diff_vec.head(2).norm() >= 0.5) {
          next_pos = it->pos_;
        } else {

          double range = std::min(explorer_params_->ftr_max_distance, diff_vec.norm());
          next_pos = pos + range * direction;
          while (sdf_map_->getOccupancy(next_pos) != SDFMap::OCCUPANCY::FREE && range >= 0.5) {
            range -= 0.5;
            next_pos = pos + range * direction;
          }
        }


      }

    } else {

      //outfile << "Estoy donde no frontier>=drones" << std::endl;

      //obtener la frontera con menor distancia
      //auto it = std::min_element(fronteras.begin(), fronteras.end(), [](const Frontera& a, const Frontera& b) {return a.distance < b.distance;});
      
      const Frontera& minFrontera = findMinFrontera(fronteras);
      //Eigen::Vector3d best_f = selectGreedyFrontier(frontier_finder_->getInFrontFrontiers(),frontier_finder_->getFrontiers(),pos);
    
      // asignar frontera    
      // Update flag
      found_ftr = true;
      
      // Compute next position
      Vector3d diff_vec = minFrontera.pos_ - pos;
      Vector3d direction = diff_vec.normalized();
      next_yaw = atan2(direction.y(), direction.x());

      if (diff_vec.head(2).norm() < explorer_params_->ftr_max_distance &&
          diff_vec.head(2).norm() >= 0.5) {
        next_pos = minFrontera.pos_;
      } else {

        double range = std::min(explorer_params_->ftr_max_distance, diff_vec.norm());
        next_pos = pos + range * direction;
        while (sdf_map_->getOccupancy(next_pos) != SDFMap::OCCUPANCY::FREE && range >= 0.5) {
          range -= 0.5;
          next_pos = pos + range * direction;
        }
      }

      // Target
      min_dist = minFrontera.costo; // it->distance; 
      //next_pos = minFrontera.pos_; // it->pos;
      next_yaw = next_yaw;//minFrontera.yaw_; // it->yaw;
      
      //updateVelocities(0.85);   // normal

      outfile << "greedyPlan: " << minFrontera.id << " con costo: " << minFrontera.costo << std::endl;

    }
    
    outfile.close();
    
    return found_ftr;
  
  }

  /*
  bool MvantExplorationManager::closestGreedyFrontier(const Vector3d& pos, const Vector3d& yaw,
    Vector3d& next_pos, double& next_yaw, bool force_different) {
    double min_dist = std::numeric_limits<double>::max();
  bool found_ftr = false;
  for (const auto& ftr : frontier_finder_->getFrontiers()) {
    // Find the viewpoint that will be evaluated by iterating over viewpoints (sorted)
    for (const auto& vp : ftr.viewpoints_) {
      // Check that the position is valid
      if (!isPositionReachable(pos, vp.pos_)) {
        continue;
      }

      std::vector<Vector3d> path;
      double distance = ViewNode::searchPath(pos, vp.pos_, path);
      if (distance < min_dist) {
        // Check if we need to force a new goal
        const double kMinDistGoals = 1.0;
        if (force_different && (vp.pos_ - ed_->next_pos_).norm() < kMinDistGoals) {
          continue;
        }

        // Update flag
        found_ftr = true;
        // Target
        min_dist = distance;
        next_pos = vp.pos_;
        next_yaw = vp.yaw_;
      }
    }
  }

  return found_ftr;
  }
  */

  
  bool MvantExplorationManager::closestGreedyFrontierOptimized(const Vector3d& pos, const Vector3d& yaw, 
                                                           Vector3d& next_pos, double& next_yaw, 
                                                           bool force_different) {
  
  const int drone_num = ed_->swarm_state_.size() - 1;
  const int ftr_num = frontier_finder_->getFrontiers().size();
  
  // Performance parameters - tune these for your setup
  const int MAX_CANDIDATES_PER_DRONE = 10;  // Limit candidates per drone
  const double MAX_DISTANCE_RADIUS = 25.0;  // Max distance to consider frontiers
  const double INF_COST = 1e5;              // Large cost for invalid assignments
  
  // Logging setup
  std::time_t now = std::time(nullptr);
  std::string filename = "/home/catkin_ws/logs/file_optimized_" + std::to_string(ep_->drone_id_) + ".txt";
  std::ofstream outfile(filename, std::ios::app);
  
  outfile << "\n------------------" << std::endl;
  outfile << "Optimized Frontier Assignment - Drone: " << ep_->drone_id_ << std::endl;
  outfile << "Drones: " << drone_num << ", Frontiers: " << ftr_num << std::endl;
  
  bool found_ftr = false;
  
  // Step 1: Pre-filter frontiers for each drone using fast distance check
  std::vector<std::vector<int>> drone_candidates(drone_num);
  std::vector<std::vector<double>> drone_costs(drone_num);
  
  // Get all frontier centroids for fast distance computation
  std::vector<Vector3d> frontier_centroids;
  frontier_centroids.reserve(ftr_num);
  for (const auto& ftr : frontier_finder_->getFrontiers()) {
    frontier_centroids.push_back(ftr.average_);
  }
  
  // For each drone, find top-K closest frontiers within radius
  for (int i = 0; i < drone_num; ++i) {
    const auto& drone_pos = ed_->swarm_state_[i].pos_;
    std::vector<std::pair<double, int>> distance_pairs;
    
    // Fast distance filtering using XY distance only
    for (int j = 0; j < ftr_num; ++j) {
      double dist_xy = (drone_pos - frontier_centroids[j]).head<2>().norm();
      if (dist_xy <= MAX_DISTANCE_RADIUS) {
        distance_pairs.emplace_back(dist_xy, j);
      }
    }
    
    // Sort by distance and keep only top-K
    std::sort(distance_pairs.begin(), distance_pairs.end());
    int keep_count = std::min(MAX_CANDIDATES_PER_DRONE, (int)distance_pairs.size());
    
    drone_candidates[i].reserve(keep_count);
    drone_costs[i].reserve(keep_count);
    
    for (int k = 0; k < keep_count; ++k) {
      drone_candidates[i].push_back(distance_pairs[k].second);
      drone_costs[i].push_back(distance_pairs[k].first);
    }
    
    outfile << "Drone " << i+1 << " candidates: " << drone_candidates[i].size() << std::endl;
  }
  
  // Step 2: Build reduced cost matrix only for selected candidates
  int max_candidates = 0;
  for (const auto& candidates : drone_candidates) {
    max_candidates = std::max(max_candidates, (int)candidates.size());
  }
  
  if (max_candidates == 0) {
    outfile << "No valid candidates found!" << std::endl;
    outfile.close();
    return false;
  }
  
  // Create reduced matrix: drone_num × max_candidates
  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Constant(drone_num, max_candidates, INF_COST);
  
  // Fill cost matrix with detailed costs only for selected candidates
  for (int i = 0; i < drone_num; ++i) {
    const auto& drone_state = ed_->swarm_state_[i];
    
    for (int k = 0; k < (int)drone_candidates[i].size(); ++k) {
      int ftr_idx = drone_candidates[i][k];
      const auto& ftr = *std::next(frontier_finder_->getFrontiers().begin(), ftr_idx);
      
      // Get best viewpoint for this frontier (same logic as original)
      Viewpoint best_vp;
      double min_vp_cost = std::numeric_limits<double>::max();
      
      for (const auto& vp : ftr.viewpoints_) {
        if (!isPositionReachable(drone_state.pos_, vp.pos_)) continue;
        
        // Compute detailed cost for this viewpoint
        double dist_cost = compute_distance_cost(drone_state.pos_, vp.pos_, true);
        double yaw_cost = compute_yaw_cost(vp.yaw_, drone_state.yaw_);
        double direction_cost = compute_direction_cost(drone_state.pos_, drone_state.vel_, vp.pos_);
        
        double total_cost = 0.35 * dist_cost + 0.4 * yaw_cost + 0.25 * direction_cost;
        
        if (total_cost < min_vp_cost) {
          min_vp_cost = total_cost;
          best_vp = vp;
        }
      }
      
      // Add swarm coordination cost (simplified version)
      double swarm_cost = 0.0;
      for (int j = 0; j < drone_num; ++j) {
        if (j == i) continue;
        double other_dist = (ed_->swarm_state_[j].pos_ - best_vp.pos_).head<2>().norm();
        swarm_cost += 1.0 / (1.0 + other_dist);  // Penalty for being close to other drones
      }
      
      cost_matrix(i, k) = min_vp_cost + 0.1 * swarm_cost;
    }
  }
  
  // Step 3: Run Hungarian algorithm on reduced matrix
  std::vector<int> assignment = HungarianAlgorithm(cost_matrix);
  
  // Step 4: Extract result for current drone
  if (ep_->drone_id_ > 0 && ep_->drone_id_ <= drone_num) {
    int assigned_idx = assignment[ep_->drone_id_ - 1];
    
    if (assigned_idx >= 0 && assigned_idx < (int)drone_candidates[ep_->drone_id_ - 1].size()) {
      int ftr_idx = drone_candidates[ep_->drone_id_ - 1][assigned_idx];
      const auto& ftr = *std::next(frontier_finder_->getFrontiers().begin(), ftr_idx);
      
      // Get the best viewpoint for the assigned frontier
      Viewpoint best_vp;
      double min_cost = std::numeric_limits<double>::max();
      
      for (const auto& vp : ftr.viewpoints_) {
        if (!isPositionReachable(pos, vp.pos_)) continue;
        
        // Check force_different constraint
        if (force_different && (vp.pos_ - ed_->next_pos_).norm() < 1.0) {
          continue;
        }
        
        double dist_cost = compute_distance_cost(pos, vp.pos_, true);
        double yaw_cost = compute_yaw_cost(vp.yaw_, yaw[0]);
        double direction_cost = compute_direction_cost(pos, ed_->swarm_state_[ep_->drone_id_].vel_, vp.pos_);
        
        double total_cost = 0.35 * dist_cost + 0.4 * yaw_cost + 0.25 * direction_cost;
        
        if (total_cost < min_cost) {
          min_cost = total_cost;
          best_vp = vp;
        }
      }
      
      if (min_cost < std::numeric_limits<double>::max()) {
        found_ftr = true;
        next_pos = best_vp.pos_;
        next_yaw = best_vp.yaw_;
        
        outfile << "Assigned frontier " << ftr_idx << " with cost " << min_cost << std::endl;
        //updateVelocities(1.0);
      }
    }
  }
  
  // Fallback: if no assignment found, use greedy selection
  if (!found_ftr) {
    outfile << "Hungarian failed, falling back to greedy selection" << std::endl;
    
    double min_cost = std::numeric_limits<double>::max();
    for (int k = 0; k < (int)drone_candidates[ep_->drone_id_ - 1].size(); ++k) {
      int ftr_idx = drone_candidates[ep_->drone_id_ - 1][k];
      const auto& ftr = *std::next(frontier_finder_->getFrontiers().begin(), ftr_idx);
      
      for (const auto& vp : ftr.viewpoints_) {
        if (!isPositionReachable(pos, vp.pos_)) continue;
        
        double dist_cost = compute_distance_cost(pos, vp.pos_, true);
        double yaw_cost = compute_yaw_cost(vp.yaw_, yaw[0]);
        double direction_cost = compute_direction_cost(pos, ed_->swarm_state_[ep_->drone_id_].vel_, vp.pos_);
        
        double total_cost = 0.35 * dist_cost + 0.4 * yaw_cost + 0.25 * direction_cost;
        
        if (total_cost < min_cost) {
          min_cost = total_cost;
          next_pos = vp.pos_;
          next_yaw = vp.yaw_;
          found_ftr = true;
        }
      }
    }
  }
  
  outfile << "Result: " << (found_ftr ? "SUCCESS" : "FAILED") << std::endl;
  outfile.close();
  
  return found_ftr;
}

// Alternative version with even more aggressive filtering
bool MvantExplorationManager::closestGreedyFrontierUltraFast(const Vector3d& pos, const Vector3d& yaw,
                                                           Vector3d& next_pos, double& next_yaw,
                                                           bool force_different) {
  
  const int drone_num = ed_->swarm_state_.size() - 1;
  const int ftr_num = frontier_finder_->getFrontiers().size();
  
  // Ultra-aggressive parameters
  const int MAX_CANDIDATES = 8;           // Very small candidate set
  const double MAX_DISTANCE = 20.0;       // Smaller radius
  const double SWARM_SPACING = 5.0;       // Minimum spacing between drone targets
  
  bool found_ftr = false;
  double min_cost = std::numeric_limits<double>::max();
  
  // Step 1: Get all frontier centroids for fast filtering
  std::vector<std::pair<double, int>> candidate_pairs;
  
  for (const auto& ftr : frontier_finder_->getFrontiers()) {
  //for (int i = 0; i < ftr_num; ++i) {
    //const auto& ftr = *std::next(frontier_finder_->getFrontiers().begin(), i);
    double dist = (pos - ftr.average_).head<2>().norm();
    
    if (dist <= MAX_DISTANCE) {
      // Check if other drones are already targeting nearby frontiers
      bool too_close_to_others = false;
      for (int j = 0; j < drone_num; ++j) {
        if (j == ep_->drone_id_ - 1) continue;
        double other_dist = (ed_->swarm_state_[j].goal_pos_ - ftr.average_).head<2>().norm();
        if (other_dist < SWARM_SPACING) {
          too_close_to_others = true;
          break;
        }
      }
      
      if (!too_close_to_others) {
        candidate_pairs.emplace_back(dist, ftr.id_);
      }
    }
  }
  
  // Step 2: Sort and keep only top candidates
  std::sort(candidate_pairs.begin(), candidate_pairs.end());
  int keep_count = std::min(MAX_CANDIDATES, (int)candidate_pairs.size());
  
  // Step 3: Evaluate only the top candidates
  for (int k = 0; k < keep_count; ++k) {
    int ftr_idx = candidate_pairs[k].second;
    const auto& ftr = *std::next(frontier_finder_->getFrontiers().begin(), ftr_idx);
    
    for (const auto& vp : ftr.viewpoints_) {
      if (!isPositionReachable(pos, vp.pos_)) continue;
      
      // Check force_different constraint
      if (force_different && (vp.pos_ - ed_->next_pos_).norm() < 1.0) {
        continue;
      }
      
      // Simple cost function for speed
      double dist_cost = (pos - vp.pos_).head<2>().norm() / 20.0;
      double yaw_cost = std::abs(vp.yaw_ - yaw[0]) / M_PI;
      if (yaw_cost > 1.0) yaw_cost = 2.0 - yaw_cost;
      
      double total_cost = 0.6 * dist_cost + 0.4 * yaw_cost;
      
      if (total_cost < min_cost) {
        min_cost = total_cost;
        next_pos = vp.pos_;
        next_yaw = vp.yaw_;
        found_ftr = true;
      }
    }
  }
  
  return found_ftr;
}

  //Calcular un campo de potencial atractivo basado en la distancia a
  //un objetivo. Devuelve un valor negativo que representa que tan
  //atractivo es un punto a una cierta distancia.
  //Cuanto más negativo, más atractivo
  double MvantExplorationManager::attractivePotentialField(double distance) const {
    // NOTE: Here function returns a negative value (the lower the distance, the better),
    // since we are selecting the target with LOWEST cost
    if (distance >= 0 && distance <= pf_params_->d0) {
      return 0.0;
    } else if (distance > pf_params_->d0 && distance <= pf_params_->df) {
      //cuanto más lejos, mayor atracción
      return -(distance - pf_params_->d0) * (distance - pf_params_->d0);
    } else {
      //Cuando la distancia ya es muy grande, se cambia la forma a una función
      //arco tangente suaviza la pendiente para evitar que la atracción crezca indefinidamente
      double lambda = (pf_params_->df - pf_params_->d0) * (pf_params_->df - pf_params_->d0) /
                      atan(0.1 * pf_params_->df);
      return -lambda * atan(distance - 0.9 * pf_params_->df);
    }
  }

  double MvantExplorationManager::repulsivePotentialField(double distance) const {
    if (distance <= pf_params_->dc) {
      double lambda0 =
          pow((pf_params_->dc - pf_params_->d0), 2.) * sqrt(pf_params_->dc * pf_params_->d0);
      lambda0 *= 1. / (sqrt(pf_params_->d0) - sqrt(pf_params_->dc));
      return lambda0 * (1. / sqrt(distance) - 1. / sqrt(pf_params_->d0));
    } else if (distance >= pf_params_->dc && distance <= pf_params_->d0) {
      return (distance - pf_params_->d0) * (distance - pf_params_->d0);
    } else {
      return 0.;
    }
  }

  double MvantExplorationManager::previousGoalCost(const Eigen::Vector3d& target_pos) const {
    double distance = (ed_->next_pos_ - target_pos).norm();
    return attractivePotentialField(distance);
  }

  double MvantExplorationManager::formationCost(const Eigen::Vector3d& target_pos) const {
    double formation_cost = 0.0;

    // Iterate over the drones
    for (int i = 0; i < ed_->swarm_state_.size(); ++i) {
      // Skip oneself
      if (i == ep_->drone_id_ - 1) continue;

      const auto& drone_state = ed_->swarm_state_[i];

      double distance_goals = (target_pos - drone_state.goal_pos_).norm();
      formation_cost += pf_params_->kr * repulsivePotentialField(distance_goals);

      double distance_pos = (target_pos - drone_state.pos_).norm();
      formation_cost += pf_params_->kr * repulsivePotentialField(distance_pos);
    }

    // Push drone towards useful area for collaboration
    if (ed_->swarm_state_.size() > 1) {
      double distance = (target_pos - ed_->next_pos_).norm();
      formation_cost += pf_params_->ka * attractivePotentialField(distance);
    }

    return formation_cost;
  }

  /*bool MvantExplorationManager::collectorPlan(const Vector3d& pos, const Vector3d& vel,
      const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {

    if (!findTourOfTrails(pos, vel, yaw, next_pos, next_yaw)) {
      return greedyPlan(pos, vel, yaw, next_pos, next_yaw);
    } else {
      return true;
    }

    // bool success_line_path = linePlan(pos, vel, yaw, next_pos, next_yaw);
    // if (!success_line_path) {
    //   return greedyPlan(pos, vel, yaw, next_pos, next_yaw);
    // } else {
    //   return true;
    // }
  }*/

  /*bool MvantExplorationManager::linePlan(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw,
    Vector3d& next_pos, double& next_yaw) {

    bool success = false;

    // Get all the most promising viewpoint for each frontier
    vector<pair<LABEL, Vector3d>> vp_pos;
    vector<Vector3d> clusters_avg;
    vector<double> vp_yaws;
    vector<int> ids;
    frontier_finder_->getTopViewpointsInfo(pos, vp_pos, vp_yaws, clusters_avg, ids);

    auto areaAlreadyAssigned = [&](const Eigen::Vector3d& candidate_pos) {
      for (int i = 0; i < ed_->swarm_state_.size(); ++i) {
        if (i == ep_->drone_id_ - 1) {
          continue;
        } else if ((candidate_pos - ed_->swarm_state_[i].goal_pos_).norm() < 8.) {
          return true;
        }
      }
      return false;
    };

    double min_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < vp_pos.size(); ++i) {
      Vector3d view_pos = vp_pos[i].second;
      LABEL ftr_label = vp_pos[i].first;
      bool valid_ftr = (ftr_label == LABEL::TRAIL && role_ == ROLE::GARBAGE_COLLECTOR) ||
                       (ftr_label == LABEL::FRONTIER && role_ == ROLE::EXPLORER);

      if (valid_ftr && ViewNode::straightLineFeasible(pos, view_pos) &&
          !areaAlreadyAssigned(view_pos)) {

        // Check that the position is valid
        if (!isPositionReachable(pos, view_pos)) {
          continue;
        }

        // Distance
        double distance_cost = (view_pos - pos).head(2).norm();

        // Direction
        auto direction = (view_pos - pos).head(2).normalized();
        double direction_cost = ViewNode::w_dir_ * acos(vel.head(2).normalized().dot(direction));

        // Other agents
        double others_cost = collector_params_->w_others * formationCost(view_pos);

        // Label penalty
        double label_cost = 0.0;
        if (role_ == ROLE::GARBAGE_COLLECTOR && ftr_label == LABEL::FRONTIER) {
          label_cost = collector_params_->label_penalty;
        }

        double total_cost = distance_cost + direction_cost + others_cost + label_cost;
        if (total_cost < min_cost) {
          success = true;
          next_pos = view_pos;
          next_yaw = vp_yaws[i];
        }
      }
    }

    return success;
  }*/

  bool MvantExplorationManager::greedyPlan(const Vector3d& pos, const Vector3d& vel,
      const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {

    ed_->trails_tour_.clear();

    // Get all the most promising viewpoint for each frontier
    vector<pair<LABEL, Vector3d>> vp_pos;
    vector<Vector3d> clusters_avg;
    vector<double> vp_yaws;
    vector<int> ids;
    frontier_finder_->getTopViewpointsInfo(pos, vp_pos, vp_yaws, clusters_avg, ids);

    // Calculate the cost for each viewpoint
    // Iterate over viewpoints
    double min_cost = std::numeric_limits<double>::max();
    bool goal_found = false;
    for (size_t i = 0; i < vp_pos.size(); ++i) {
      Vector3d vp_position = vp_pos[i].second;
      LABEL vp_label = vp_pos[i].first;

      // Check that the position is valid
      if (!isPositionReachable(pos, vp_position)) {
        continue;
      }

      // Calculate path from current position to viewpoint
      double path_length;
      if (!ViewNode::validPathExists(pos, vp_position, path_length)) {
        continue;
      }
      double time_to_dest = collector_params_->w_distance * path_length / ViewNode::vm_;

      // Calculate velocity change
      double vel_change = 0.0;
      if (vel.norm() > collector_params_->min_vel) {
        Vector3d dir = (vp_position - pos).normalized();
        double diff = acos(vel.normalized().dot(dir));
        vel_change = ViewNode::w_dir_ * diff;
      } else {
        // Calculate yaw change
        double diff = fabs(vp_yaws[i] - yaw[0]);
        vel_change = min(diff, 2 * M_PI - diff) / ViewNode::yd_;
      }
      vel_change *= collector_params_->w_direction;

      // Iterate over others
      double formation_cost = collector_params_->w_others * formationCost(vp_position);

      // Label associated cost (incentivate to explore trails)
      double label_penalty = 0.0;
      if (role_ == ROLE::EXPLORER && vp_label == LABEL::TRAIL) {
        label_penalty = collector_params_->label_penalty;
      } else if (role_ == ROLE::GARBAGE_COLLECTOR && vp_label == LABEL::FRONTIER) {
        label_penalty = collector_params_->label_penalty;
      }

      // Previous goal influence
      double previous_goal_cost = collector_params_->w_previous_goal * previousGoalCost(vp_position);

      // Compute final cost and update
      double total_cost =
          time_to_dest + vel_change + formation_cost + label_penalty + previous_goal_cost;

      if (total_cost < min_cost) {
        min_cost = total_cost;
        next_pos = vp_position;
        next_yaw = vp_yaws[i];
        goal_found = true;
      }
    }

    return goal_found;
  }

  void MvantExplorationManager::shortenPath(vector<Vector3d>& path) {
    if (path.empty()) {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = { path.front() };
    for (int i = 1; i < path.size() - 1; ++i) {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else {
        // Add waypoints to shorten path only to avoid collision
        ViewNode::caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (ViewNode::caster_->nextId(idx) && ros::ok()) {
          if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
              edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
  }

  /*void MvantExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
      const Vector3d cur_yaw, vector<int>& indices) {
    auto t1 = ros::Time::now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    const int dimension = cost_mat.rows();
    // std::cout << "mat:   " << cost_mat.rows() << std::endl;

    double mat_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Initialize TSP par file
    ofstream par_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par");
    par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE ="
             << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) +
                    ".tou"
                    "r\n";
    par_file << "RUNS = 1\n";
    par_file.close();

    // Write params and cost matrix to problem file
    ofstream prob_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp");
    // Problem specification part, follow the format of TSPLIB
    string prob_spec;
    prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                "\nEDGE_WEIGHT_TYPE : "
                "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 100;
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
    prob_file << "EOF";
    prob_file.close();

    // solveTSPLKH((ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par").c_str());
    lkh_tsp_solver::SolveTSP srv;
    if (!tsp_client_.call(srv)) {
      ROS_ERROR("Fail to solve TSP.");
      return;
    }

    // Read optimal tour from the tour section of result file
    ifstream res_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    while (getline(res_file, res)) {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0) break;
    }

    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }

    res_file.close();

    std::cout << "Tour " << ep_->drone_id_ << ": ";
    for (auto id : indices) std::cout << id << ", ";
    std::cout << "" << std::endl;

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

    double tsp_time = (ros::Time::now() - t1).toSec();
    ROS_INFO("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);

    // if (tsp_time > 0.1) ROS_BREAK();
  }*/

  /*void MvantExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
      const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
      const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
      vector<double>& refined_yaws) {
    double create_time, search_time, parse_time;
    auto t1 = ros::Time::now();
    
    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    std::cout << "Local refine graph size: 1, ";
    for (int i = 0; i < n_points.size(); ++i) {
      // Create nodes for viewpoints of one frontier
      for (int j = 0; j < n_points[i].size(); ++j) {
        ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
        g_search.addNode(node);
        // Connect a node to nodes in last group
        for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
        cur_group.push_back(node);

        // Only keep the first viewpoint of the last local frontier
        if (i == n_points.size() - 1) {
          final_node = node;
          break;
        }
      }
      // Store nodes for this group for connecting edges
      std::cout << cur_group.size() << ", ";
      last_group = cur_group;
      cur_group.clear();
    }
    std::cout << "" << std::endl;
    create_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i) {
      refined_pts.push_back(path[i]->pos_);
      refined_yaws.push_back(path[i]->yaw_);
    }

    // Extract optimal local tour (for visualization)
    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.2);
    for (auto pt : refined_pts) {
      vector<Vector3d> path;
      if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
        ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
      else
        ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
  }*/

  /*void MvantExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
      const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
      vector<int>& other_ids) {
    // ROS_INFO("Allocate grid.");

    auto t1 = ros::Time::now();
    auto t2 = t1;

    if (grid_ids.size() == 1) {  // Only one grid, no need to run ACVRP
      auto pt = hgrid_->getCenter(grid_ids.front());
      // double d1 = (positions[0] - pt).norm();
      // double d2 = (positions[1] - pt).norm();
      vector<Eigen::Vector3d> path;
      double d1 = ViewNode::computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      double d2 = ViewNode::computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (d1 < d2) {
        ego_ids = grid_ids;
        other_ids = {};
      } else {
        ego_ids = {};
        other_ids = grid_ids;
      }
      return;
    }

    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
    hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

    // int unknown = hgrid_->getTotalUnknwon();
    int unknown;

    double mat_time = (ros::Time::now() - t1).toSec();

    // Find optimal path through AmTSP
    t1 = ros::Time::now();
    const int dimension = mat.rows();
    const int drone_num = positions.size();

    vector<int> unknown_nums;
    int capacity = 0;
    for (int i = 0; i < grid_ids.size(); ++i) {
      int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
      unknown_nums.push_back(unum);
      capacity += unum;
      // std::cout << "Grid " << i << ": " << unum << std::endl;
    }
    // std::cout << "Total: " << capacity << std::endl;
    capacity = capacity * 0.75 * 0.1;

    // int prob_type;
    // if (grid_ids.size() >= 3)
    //   prob_type = 2;  // Use ACVRP
    // else
    //   prob_type = 1;  // Use AmTSP

    const int prob_type = 2;

    // Create problem file--------------------------
    ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : pairopt\n";

    if (prob_type == 1)
      file << "TYPE : ATSP\n";
    else if (prob_type == 2)
      file << "TYPE : ACVRP\n";

    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

    if (prob_type == 2) {
      file << "CAPACITY : " + to_string(capacity) + "\n";   // ACVRP
      file << "VEHICLES : " + to_string(drone_num) + "\n";  // ACVRP
    }

    // Cost matrix
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
      }
      file << "\n";
    }

    if (prob_type == 2) {  // Demand section, ACVRP only
      file << "DEMAND_SECTION\n";
      file << "1 0\n";
      for (int i = 0; i < drone_num; ++i) {
        file << to_string(i + 2) + " 0\n";
      }
      for (int i = 0; i < grid_ids.size(); ++i) {
        int grid_unknown = unknown_nums[i] * 0.1;
        file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
      }
      file << "DEPOT_SECTION\n";
      file << "1\n";
      file << "EOF";
    }

    file.close();

    // Create par file------------------------------------------
    int min_size = int(grid_ids.size()) / 2;
    int max_size = ceil(int(grid_ids.size()) / 2.0);
    file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
    if (prob_type == 1) {
      file << "SALESMEN = " << to_string(drone_num) << "\n";
      file << "MTSP_OBJECTIVE = MINSUM\n";
      // file << "MTSP_OBJECTIVE = MINMAX\n";
      file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
      file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
      file << "TRACE_LEVEL = 0\n";
    } else if (prob_type == 2) {
      file << "TRACE_LEVEL = 1\n";  // ACVRP
      file << "SEED = 0\n";         // ACVRP
    }
    file << "RUNS = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 3;
    // if (!tsp_client_.call(srv)) {
    if (!acvrp_client_.call(srv)) {
      ROS_ERROR("Fail to solve ACVRP.");
      return;
    }
    // system("/home/boboyu/software/LKH-3.0.6/LKH
    // /home/boboyu/workspaces/hkust_swarm_ws/src/swarm_exploration/utils/lkh_mtsp_solver/resource/amtsp3_1.par");

    double mtsp_time = (ros::Time::now() - t1).toSec();
    std::cout << "Allocation time: " << mtsp_time << std::endl;

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
      if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
      int id = stoi(res);
      ids.push_back(id - 1);
      if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour of grid
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
      if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
      } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
      } else {
        tour.push_back(id);
      }
    }
    // // Print tour ids
    // for (auto tr : tours) {
    //   std::cout << "tour: ";
    //   for (auto id : tr) std::cout << id << ", ";
    //   std::cout << "" << std::endl;
    // }

    for (int i = 1; i < tours.size(); ++i) {
      if (tours[i][0] == 1) {
        ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
      } else {
        other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
      }
    }
    for (auto& id : ego_ids) {
      id = grid_ids[id - 1 - drone_num];
    }
    for (auto& id : other_ids) {
      id = grid_ids[id - 1 - drone_num];
    }
    // // Remove repeated grid
    // unordered_map<int, int> ego_map, other_map;
    // for (auto id : ego_ids) ego_map[id] = 1;
    // for (auto id : other_ids) other_map[id] = 1;

    // ego_ids.clear();
    // other_ids.clear();
    // for (auto p : ego_map) ego_ids.push_back(p.first);
    // for (auto p : other_map) other_ids.push_back(p.first);

    // sort(ego_ids.begin(), ego_ids.end());
    // sort(other_ids.begin(), other_ids.end());
  }*/

  /*double MvantExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos,
      const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
      const vector<vector<int>>& seconds, const double& w_f) {
    if (grid_ids.empty()) return 0.0;

    double cost = 0.0;
    vector<Eigen::Vector3d> path;
    cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
    for (int i = 0; i < grid_ids.size() - 1; ++i) {
      cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
    }
    return cost;
  }*/

  /*bool MvantExplorationManager::findTourOfTrails(const Vector3d& cur_pos,
      const Eigen::Vector3d& cur_yaw, const Vector3d& cur_vel, Eigen::Vector3d& next_pos,
      double& next_yaw) {
    // 1. Get all trails within an area
    list<Frontier> close_by_trails;
    frontier_finder_->getTrailCentroidsAroundPosition(
        close_by_trails, cur_pos, collector_params_->ftr_max_distance);

    // Limit case: we don't have trails around or we have only one
    const size_t num_trails = close_by_trails.size();
    if (num_trails == 0) {
      return false;
    } else if (num_trails == 1) {
      // Target pose
      next_pos = close_by_trails.front().viewpoints_.front().pos_;
      next_yaw = close_by_trails.front().viewpoints_.front().yaw_;

      // For visualization
      ed_->trails_tour_ = { { cur_pos, cur_yaw[0] }, { next_pos, next_yaw } };

      return true;
    }

    // Create cost matrix
    const auto dimension = num_trails + 1;
    Eigen::MatrixXd cost_mat = Eigen::MatrixXd::Zero(dimension, dimension);

    auto updateCostFrontiers = [&](const list<Frontier>::iterator& it1,
                                   const list<Frontier>::iterator& it2) {
      // Search path from old cluster's top viewpoint to new cluster'
      const Viewpoint& vui = it1->viewpoints_.front();
      const Viewpoint& vuj = it2->viewpoints_.front();
      vector<Vector3d> path_ij;
      return collector_params_->w_distance * ViewNode::searchPath(vui.pos_, vuj.pos_, path_ij) +
             collector_params_->w_others * (formationCost(vui.pos_) - formationCost(vuj.pos_));
    };

    auto updateCostFromState = [&](const list<Frontier>::iterator& it) {
      // Search path from old cluster's top viewpoint to new cluster'
      const Viewpoint& v = it->viewpoints_.front();
      vector<Vector3d> path;
      return collector_params_->w_distance * ViewNode::searchPath(cur_pos, v.pos_, path) +
             collector_params_->w_others * formationCost(v.pos_);  // +
      // collector_params_->w_previous_goal * previousGoalCost(v.pos_);
    };

    auto start_time = ros::Time::now();

    // cost between trails
    int i = 1, j = 1;
    for (auto it1 = close_by_trails.begin(); it1 != close_by_trails.end(); ++it1) {
      for (auto it2 = close_by_trails.begin(); it2 != close_by_trails.end(); ++it2) {
        cost_mat(i, j++) = it1 == it2 ? 0. : updateCostFrontiers(it1, it2);
      }
      ++i;
      j = 1;
    }

    // cost between current position and trails
    j = 1;
    for (auto it = close_by_trails.begin(); it != close_by_trails.end(); ++it) {
      cost_mat(0, j++) = updateCostFromState(it);
    }

    // ROS_WARN_STREAM("Time to build problem: " << (ros::Time::now() - start_time).toSec() << " s");

    // Create problem to solve
    start_time = ros::Time::now();

    const int scale = 100;
    ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = scale * cost_mat(i, j);
        file << int_cost << " ";
      }
      file << "\n";
    }
    file.close();

    // Create par file
    const int drone_num = 1;
    
    file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "MTSP_MIN_SIZE = " << to_string(min(int(close_by_trails.size()) / drone_num, 4)) << "\n";
    file << "MTSP_MAX_SIZE = "
         << to_string(max(1, int(close_by_trails.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 1;
    if (!tsp_client_.call(srv)) {
      ROS_ERROR("Fail to solve ATSP.");
      return false;
    }

    // ROS_WARN_STREAM("Time to solve problem: " << (ros::Time::now() - start_time).toSec() << " s");

    // Read results
    ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
      if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
      int id = stoi(res);
      ids.push_back(id - 1);
      if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour
    ed_->trails_tour_ = { { cur_pos, cur_yaw[0] } };
    for (auto id : ids) {
      if (id <= 0) continue;
      
      auto it = close_by_trails.begin();
      std::advance(it, id - 1);
      ed_->trails_tour_.push_back({ it->viewpoints_.front().pos_, it->viewpoints_.front().yaw_ });
    }

    // Target pose
    next_pos = ed_->trails_tour_[1].first;
    next_yaw = ed_->trails_tour_[1].second;
    return true;
  }*/

  /*bool MvantExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
      bool init) {

    ROS_INFO("Find grid tour---------------");

    auto t1 = ros::Time::now();

    auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

    // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

    vector<int> first_ids, second_ids;
    hgrid_->inputFrontiers(ed_->averages_);

    hgrid_->updateGridData(
        ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);

    if (grid_ids.empty()) {
      ROS_WARN("Empty dominance.");
      ed_->grid_tour_.clear();
      return false;
    }

    // std::cout << "Allocated grid: ";
    // for (auto id : grid_ids) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
    if (!init)
      hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
    else
      hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

    double mat_time = (ros::Time::now() - t1).toSec();

    // Find optimal path through ATSP
    t1 = ros::Time::now();
    const int dimension = mat.rows();
    const int drone_num = 1;

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
      }
      file << "\n";
    }
    file.close();

    // Create par file
    file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
    // "\n"; file << "MTSP_MAX_SIZE = "
    //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 2;
    if (!tsp_client_.call(srv)) {
      ROS_ERROR("Fail to solve ATSP.");
      return false;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    // std::cout << "AmTSP time: " << mtsp_time << std::endl;

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
      if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
      int id = stoi(res);
      ids.push_back(id - 1);
      if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour of grid
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
      if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
      } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
      } else {
        tour.push_back(id);
      }
    }

    // for (auto tr : tours) {
    //   std::cout << "tour: ";
    //   for (auto id : tr) std::cout << id << ", ";
    //   std::cout << "" << std::endl;
    // }
    others.resize(drone_num - 1);
    for (int i = 1; i < tours.size(); ++i) {
      if (tours[i][0] == 1) {
        indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
      } else {
        others[tours[i][0] - 2].insert(
            others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
      }
    }
    for (auto& id : indices) {
      id -= 1 + drone_num;
    }
    for (auto& other : others) {
      for (auto& id : other) id -= 1 + drone_num;
    }
    // std::cout << "Grid tour: ";
    for (auto& id : indices) {
      id = grid_ids[id];
      // std::cout << id << ", ";
    }
    // std::cout << "" << std::endl;

    // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
    grid_ids = indices;
    hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

    ed_->last_grid_ids_ = grid_ids;
    ed_->reallocated_ = false;

    // hgrid_->checkFirstGrid(grid_ids.front());

    return true;
  }*/

  /*void MvantExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
      const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
      vector<int>& indices) {

    auto t1 = ros::Time::now();

    vector<Eigen::Vector3d> positions = { cur_pos };
    vector<Eigen::Vector3d> velocities = { cur_vel };
    vector<double> yaws = { cur_yaw[0] };

    // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
    Eigen::MatrixXd mat;
    frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
    const int dimension = mat.rows();
    // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

    double mat_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("mat time: %lf", mat_time);

    // Find optimal allocation through AmTSP
    t1 = ros::Time::now();

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
      }
      file << "\n";
    }
    file.close();

    // Create par file
    const int drone_num = 1;

    file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
    file << "MTSP_MAX_SIZE = "
         << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 1;
    if (!tsp_client_.call(srv)) {
      ROS_ERROR("Fail to solve ATSP.");
      return;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("AmTSP time: %lf", mtsp_time);

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
      if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
      int id = stoi(res);
      ids.push_back(id - 1);
      if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
      if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
      } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
      } else {
        tour.push_back(id);
      }
    }

    vector<vector<int>> others(drone_num - 1);
    for (int i = 1; i < tours.size(); ++i) {
      if (tours[i][0] == 1) {
        indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
      }
      // else {
      //   others[tours[i][0] - 2].insert(
      //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
      // }
    }
    for (auto& id : indices) {
      id -= 1 + drone_num;
    }
    // for (auto& other : others) {
    //   for (auto& id : other)
    //     id -= 1 + drone_num;
    // }

    if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
      indices.pop_back();
    }
    // Subset of frontier inside first grid
    for (int i = 0; i < indices.size(); ++i) {
      indices[i] = ftr_ids[indices[i]];
    }

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
    if (!grid_pos.empty()) {
      ed_->frontier_tour_.push_back(grid_pos[0]);
    }

    // ed_->other_tours_.clear();
    // for (int i = 1; i < positions.size(); ++i) {
    //   ed_->other_tours_.push_back({});
    //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
    // }

    double parse_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
    //     parse_time, indices.size());
  }*/

}  // namespace fast_planner
