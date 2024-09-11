//mvant_exploration_fsm.cpp
//FSM

#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

//exploration_manager/include
#include <exploration_manager/mvant_exploration_manager.h>
#include <exploration_manager/mvant_exploration_fsm.h>
#include <exploration_manager/expl_data.h>

#include <exploration_manager/HGrid.h>
#include <exploration_manager/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <fstream>

using Eigen::Vector4d;

namespace fast_planner {
  
  /**
     INIT
  */
  void MvantExplorationFSM::init(ros::NodeHandle& nh) {
    fp_.reset(new FSMParam); //expl_data.h
    fd_.reset(new FSMData);  //expl_data.h
    
    /*  Fsm param  */
    // Nodes use the parameter server to store
    // and retrieve parameters at runtime
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
    nh.param("fsm/pair_opt_interval", fp_->pair_opt_interval_, 1.0);
    nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);
    
    nh.param("fsm/communication_range", fp_->communication_range_, std::numeric_limits<double>::max());
    
    // *****************************************************
    // ******** PASO DE PARAM, DEFINIR COORDINACION ******** 
    // *****************************************************
    nh.param("exploration/coordination/type", fp_->coordination_type, string("null"));
    
    /* Initialize main modules */
    expl_manager_.reset(new MvantExplorationManager);
    expl_manager_->initialize(nh);
    
    visualization_.reset(new PlanningVisualization(nh));
    coll_assigner_.reset(new CollaborationAssigner(nh));
    
    planner_manager_ = expl_manager_->planner_manager_;
    
    state_ = EXPL_STATE::INIT; //estado inicial
    
    fd_->have_odom_ = false;
    
    /* Estados VANT FSM*/
    fd_->state_str_ = {
		       "INIT", "WAIT_TRIGGER",
		       "PLAN_TRAJ", "PUB_TRAJ",
		       "EXEC_TRAJ", "FINISH",
		       "IDLE"
    };
    
    fd_->static_state_ = true;
    fd_->trigger_ = false;
    fd_->avoid_collision_ = false;
    fd_->go_back_ = false;
    
    num_fail_ = 0;
    
    // PROGRAMAS EN PARALELO CORRIENDO CADA CIERTO TIEMPO
    /* Ros sub, pub and timer */

    //FSM principal
    exec_timer_     = nh.createTimer(ros::Duration(0.01), &MvantExplorationFSM::FSMCallback, this);

    //revisar colisiones
    safety_timer_   = nh.createTimer(ros::Duration(0.05), &MvantExplorationFSM::safetyCallback, this);
    
    //actualiza la frontera
    frontier_timer_ = nh.createTimer(ros::Duration(0.1), &MvantExplorationFSM::frontierCallback, this);

    //mandar mensaje de vacio estar vivo
    heartbit_timer_ = nh.createTimer(ros::Duration(1.0), &MvantExplorationFSM::heartbitCallback, this);
    
    // ******************************************************
    // ************** Exploración fronteras *****************
    // ******************************************************
    // exploration_timer_ = nh.createTimer(ros::Duration(0.005), &MvantExplorationFSM::explorationCallback, this);
    
    // ******************************************************
    // ************ Trigger para lanzar FSM *****************
    // ******************************************************
    // Se puede invocar desde terminal
    // rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
    trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MvantExplorationFSM::triggerCallback, this);

    // ******************************************************
    // **************** Datos de odometria ******************
    // ******************************************************
    odom_sub_    = nh.subscribe("/odom_world", 1, &MvantExplorationFSM::odometryCallback, this);
    
    //funcion que se suscribe de prueba
    //test_sub_ = nh.subscribe("/pruebas", 1, &MvantExplorationFSM::pruebasCallback, this);
    
    //subscriber llevarlo a nuevo archivo
    //nearby_obs_sub_ = nh.subscribe("/nearby_obstacles", 1000, &MvantExplorationFSM::nearbyObstaclesCallback, this);
    
    //-------------------------------------------------------------------------------------------------
    // ******************************************************
    // ****************** TOPICOS  **************************
    // ******************************************************
    replan_pub_   = nh.advertise<std_msgs::Empty>("/planning/replan", 10);  // Replanear trayectoria
    new_pub_      = nh.advertise<std_msgs::Empty>("/planning/new", 10);     // Publicar
    bspline_pub_  = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
    stop_pub_     = nh.advertise<std_msgs::Int32>("/stop", 1000);
    heartbit_pub_ = nh.advertise<std_msgs::Empty>("/heartbit", 100);
    
    emergency_handler_pub_ = nh.advertise<std_msgs::Bool>("/trigger_emergency", 10);
    
    //prueba un nuevo publicador
    //nearby_obs_pub_ = nh.advertise<exploration_manager::SearchObstacle>("/nearby_obstacles", 10);

    //prueba de fronteras
    //test_fronteras = nh.advertise<std_msgs::Empty>("/pruebas", 100);

    //prueba de topico dummy
    test_topico = nh.advertise<std_msgs::Empty>("/test_topic", 10);
    topico_sub_ = nh.subscribe("/test_topic", 10, &MvantExplorationFSM::pruebaTopicoCallback, this);
    
    // si lo utilizo para la evaluacion de los K-vecinos respecto al VANT
    // el topico se renombra haciendo un remap dentro del archivo
    // single_drone_planner_mvant.xml
    nb_obs_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/obs", 10);
    
    //se ejecuta cada medio segundo
    prueba_nb = nh.createTimer(ros::Duration(0.2), &MvantExplorationFSM::nearbyObstaclesCallback, this);
    
    //-------------------------------------------------------------------------------------------------
    // Swarm, timer, pub and sub
    drone_state_timer_ = nh.createTimer(ros::Duration(0.04), &MvantExplorationFSM::droneStateTimerCallback, this);
    drone_state_pub_   = nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
    drone_state_sub_   = nh.subscribe("/swarm_expl/drone_state_recv", 10, &MvantExplorationFSM::droneStateMsgCallback, this);
    
    opt_timer_ = nh.createTimer(ros::Duration(0.05), &MvantExplorationFSM::optTimerCallback, this);
    opt_pub_   = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
    opt_sub_   = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &MvantExplorationFSM::optMsgCallback,
			    this, ros::TransportHints().tcpNoDelay());
    
    opt_res_pub_ = nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
    opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100, &MvantExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());
    
    swarm_traj_pub_   = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
    swarm_traj_sub_   = nh.subscribe("/planning/swarm_traj_recv", 100, &MvantExplorationFSM::swarmTrajCallback, this);
    swarm_traj_timer_ = nh.createTimer(ros::Duration(0.1), &MvantExplorationFSM::swarmTrajTimerCallback, this);
    
    hgrid_pub_     = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
    grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);
  }
  
  int MvantExplorationFSM::getId() {
    return expl_manager_->ep_->drone_id_;
  }
  
  double MvantExplorationFSM::calcularDistancia(double x, double y, double z, double cx, double cy, double cz){
    return std::sqrt(std::pow(x - cx, 2) + std::pow(y - cy, 2) + std::pow(z - cz, 2)); 
  }
  
  void MvantExplorationFSM::sendStopMsg(int code) {
    std_msgs::Int32 msg;
    msg.data = code;
    
    const size_t kNumAttempts = 5;
    for (size_t i = 0; i < kNumAttempts; ++i) {
      stop_pub_.publish(msg);
      ros::Duration(0.01).sleep();
    }
  }
  
  void MvantExplorationFSM::sendEmergencyMsg(bool emergency) {
    std_msgs::Bool msg;
    msg.data = emergency;
    emergency_handler_pub_.publish(msg);
  }
  
  /*FINIT STATE CALLBACK*/
  void MvantExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
    
    //ROS_WARN_STREAM_THROTTLE(1.0, "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);
    
    switch (state_) {
      
      // Estado inicial,
      // espera a tener valores de odometria
    case INIT: {
      // Wait for odometry ready
      // La odometria la recibe del callback de odometria
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "-- no datos odometria --");
        return;
      }
      if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "-- esperar para inicializar --");
	return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }
      
      //Esperando el lanzador
    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_ONCE(" -- esperando lanzador -- ");
      ROS_WARN_ONCE("Exploracion: %s", "mvant");
      ROS_WARN_ONCE("Tipo coordinacion: %s", fp_->coordination_type.c_str());
            
      break;
    }
      
    case PLAN_TRAJ: {
      
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
	//LocalTrajData esta dentro de plan_container.hpp
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }
      
      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      
      //llamar a planificador original
      //int res = callExplorationPlanner();

      //poner aqui mi logica
      //buscar una frontera acudir y su trayectoria hacia ella en el grid
      int res = callPlannerExploration();
      
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
        // Emergency
        num_fail_ = 0;
        sendEmergencyMsg(false);
	
      } else if (res == FAIL) {  // Keep trying to replan
	fd_->static_state_ = true;
	ROS_WARN_THROTTLE(1.0, "-- Plan fail (drone %d) --", getId());
	// Check if we need to send a message
	if (num_fail_ > 10) {
          sendEmergencyMsg(true);
          num_fail_ = 0;
        } else {
          ++num_fail_;
        }
	
      } else if (res == NO_GRID) {
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = ros::Time::now();
        // ROS_WARN("No grid (drone %d)", getId());
        transitState(IDLE, "FSM");
	
        // Emergency
        num_fail_ = 0;
        sendEmergencyMsg(false);
      }
      
      //visualize(1);
      clearVisMarker();
      break;
    }
      
    case PUB_TRAJ: {
      
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;
	
        // fd_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);
	
        thread vis_thread(&MvantExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }
      
    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      // Check whether replan is needed
      //revisar aqui, si ya llego a la frontera
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();
      
      //Aqui va si necesita replanificar si la frontera fue cubierta, hacer topico
      
      if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          // ROS_WARN("Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          // ROS_WARN("Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          // Replan after some time
          // ROS_WARN("Replan: periodic call=======================================");
          need_replan = true;
        }
	
        if (need_replan) {
          if (expl_manager_->updateFrontierStruct(fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_) != 0) {
            // Update frontier and plan new motion
            //thread vis_thread(&MvantExplorationFSM::visualize, this, 1);
            //vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, finish exploration
            fd_->last_check_frontier_time_ = ros::Time::now();
            transitState(IDLE, "FSM");
            ROS_WARN_THROTTLE(1., "Idle since no frontier is detected");
            fd_->static_state_ = true;
            replan_pub_.publish(std_msgs::Empty());
            sendStopMsg(1);
          }
          clearVisMarker();
          //visualize(1);
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          //thread vis_thread(&MvantExplorationFSM::visualize, this, 1);
          //vis_thread.detach();
        }
      }
      
      break;
    }

         //Estado final
    case FINISH: {
      sendStopMsg(1);
      ROS_INFO_THROTTLE(1.0, "-- exploracion terminada --");
      
      break;
    }
      
      //Estado Inactivo
    case IDLE: {
      double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
      
      // Check: if we don't have any frontier, then stop
      if (expl_manager_->updateFrontierStruct(fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_) <= 1) {
	
	ROS_WARN_THROTTLE(1.0, "-- No fronteras para agente %d", getId());
	
	sendStopMsg(1);
	
	//transitState(FINISH, "FSM");
		
	ros::Duration(1).sleep();
	
	break;
	
      }

      if (check_interval > 100.0) {
        // if (!expl_manager_->updateFrontierStruct(fd_->odom_pos_)) {
        ROS_WARN("Go back to (0,0,1)");
        // if (getId() == 1) {
        //   expl_manager_->ed_->next_pos_ = Eigen::Vector3d(-3, 1.9, 1);
        // } else
        // expl_manager_->ed_->next_pos_ = Eigen::Vector3d(0, 0.0, 1);
        // Eigen::Vector3d dir = (fd_->start_pos_ - fd_->odom_pos_);
        // expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);
	
        expl_manager_->ed_->next_pos_ = fd_->start_pos_;
        expl_manager_->ed_->next_yaw_ = 0.0;
	
        fd_->go_back_ = true;
        transitState(PLAN_TRAJ, "FSM");
        // } else {
        //   fd_->last_check_frontier_time_ = ros::Time::now();
        // }
      }
      break;
    }
      
    }
  }

  // original implementation
  int MvantExplorationFSM::callExplorationPlanner() {
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
    
    int res;
    
    //TODO revisar expl_manager_->ed_->next_pos_
    
    if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
      res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
      fd_->avoid_collision_ = false;
    } else {  // Do full planning normally
      res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
    }
    
    if (res == SUCCEED) {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
      
      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i) {
	geometry_msgs::Point pt;
	pt.x = pos_pts(i, 0);
	pt.y = pos_pts(i, 1);
	pt.z = pos_pts(i, 2);
	bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
	bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i) {
	double yaw = yaw_pts(i, 0);
	bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
      fd_->newest_traj_ = bspline;
    }
    return res;
  }
    
  int MvantExplorationFSM::callPlannerExploration() {
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
    
    int res;
    
    //TODO revisar expl_manager_->ed_->next_pos_
    
    //que significa el avoid collision ??
    //que significa el go back         ??
    
    if (fd_->avoid_collision_ || fd_->go_back_) {  // Only replan trajectory
      ROS_WARN_STREAM("planTrajToView");
      res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
      fd_->avoid_collision_ = false;
    } else {  // Do full planning normally
      ROS_WARN_STREAM("planExploreMotion");
      res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
    }
    
    if (res == SUCCEED) {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
      
      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i) {
	geometry_msgs::Point pt;
	pt.x = pos_pts(i, 0);
	pt.y = pos_pts(i, 1);
	pt.z = pos_pts(i, 2);
	bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
	bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i) {
	double yaw = yaw_pts(i, 0);
	bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
      fd_->newest_traj_ = bspline;
    }
    return res;
  }
  
  void MvantExplorationFSM::visualize(int content) {
    
    // content 1: frontier; 2 paths & trajs
    auto info = &planner_manager_->local_data_;
    auto plan_data = &planner_manager_->plan_data_;
    auto ed_ptr = expl_manager_->ed_;
    
    //Obtencion de colores
    auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
			 double a = (drone_id - 1) / double(num + 1);
			 double b = 1 / double(num + 1);
			 return a + b * double(id) / ed_ptr->frontiers_.size();
		       };
    
    if (content == 1) {
      // Draw frontier
      auto res = expl_manager_->sdf_map_->getResolution();
      static int last_ftr_num = 0;

      for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {

	auto color = visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 1);

	visualization_->drawCubes(ed_ptr->frontiers_[i], res, color, "frontier", i, 0.9);

	// getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
	// double(i) / ed_ptr->frontiers_.size()
	
	// visualization_->drawBox(ed_ptr->frontier_boxes_[i].first,
	// ed_ptr->frontier_boxes_[i].second,
	//   color, "frontier_boxes", i, 4);

	//obtener centroide
	//puntos
       	Vector3d centroid(0.0, 0.0, 0.0);
	int count = 0;
	
	vector<Vector3d> points = ed_ptr->frontiers_[i];
	
	for (const auto& point : points) {
	  centroid += point;
	  count++;
	}
	
	if (count > 0) {
	  centroid /= count;
	}
	
	//Mostrar el numero de frontera
	auto id_str = std::to_string(ed_ptr->fronters_ids_[i]);
	ROS_WARN_STREAM("Frontera:: " << id_str);
	//ROS_WARN_STREAM("Frontera:: " << ed_ptr->frontiers_[i][0]);
	ROS_WARN_STREAM("Frontera:: " << centroid.transpose());
	//visualization_->drawText(ed_ptr->frontiers_[i][0] - Eigen::Vector3d(0., 0., 0.), id_str, 0.5, Eigen::Vector4d::Ones(), "id", ed_ptr->frontiers_.size() + i, 4);
	visualization_->drawText(centroid - Eigen::Vector3d(0., 0., 0.), id_str, 0.5, Eigen::Vector4d::Ones(), "id", ed_ptr->frontiers_.size() + i, 4);
      }
      
      for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
	visualization_->drawCubes({}, res, Vector4d(0, 0, 0, 1), "frontier", i, 4);
	// visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
	//   "frontier_boxes", i, 4);
	// visualization_->drawText(ed_ptr->frontiers_[i][0] - Eigen::Vector3d(0., 0., 0.), "", 0.5,
	// Eigen::Vector4d::Ones(), "id", i + 1, 4);
      }
      
      last_ftr_num = ed_ptr->frontiers_.size();
      
      // Draw labeled frontier
      static size_t last_ftr_num_labeled = 0;
      
      for (size_t i = 0; i < ed_ptr->labeled_frontiers_.size(); ++i) {
	Eigen::Vector4d color(0, 0, 0, 0.5);
	if (ed_ptr->labeled_frontiers_[i].first == 0) {
	  color[0] = 1.;
	} else if (ed_ptr->labeled_frontiers_[i].first == LABEL::FRONTIER) {
	  color[1] = 1.;
	} else {
	  color[2] = 1.;
	}
	visualization_->drawCubes(
				  ed_ptr->labeled_frontiers_[i].second, res, color, "labeled_frontier", i, 7);
      }
      
      for (size_t i = ed_ptr->labeled_frontiers_.size(); i < last_ftr_num_labeled; ++i) {
	visualization_->drawCubes({}, res, Eigen::Vector4d(0, 0, 0, 1), "labeled_frontier", i, 7);
      }
      
      last_ftr_num_labeled = ed_ptr->labeled_frontiers_.size();
      
      // Draw frontiers in front
      static size_t last_ftr_num_infront = 0;
      
      for (size_t i = 0; i < ed_ptr->infront_frontiers_.size(); ++i) {
	Eigen::Vector4d color(1, 1, 0, 0.5);
	visualization_->drawCubes(
				  ed_ptr->infront_frontiers_[i].second, res, color, "infront_frontiers", i, 10);
      }
      
      for (size_t i = ed_ptr->infront_frontiers_.size(); i < last_ftr_num_labeled; ++i) {
	visualization_->drawCubes({}, res, Eigen::Vector4d(0, 0, 0, 1), "infront_frontiers", i, 10);
      }
      
      last_ftr_num_infront = ed_ptr->infront_frontiers_.size();
      
      // Publish role
      const std::string role_str = roleToString(expl_manager_->role_);
      visualization_->drawText(fd_->odom_pos_ - Eigen::Vector3d(1., 0., 0.), role_str, 0.4,
			       Eigen::Vector4d::Ones(), "role", 0, 8);
      
      // Publish current goal
      auto color =
	PlanningVisualization::getColor((getId() - 1) / double(expl_manager_->ep_->drone_num_));
      visualization_->drawArrow(expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_,
				Eigen::Vector3d(0.75, 0.3, 0.75), color, "goal", 0, 9);
      
      // Publish trails tour
      if (expl_manager_->role_ == ROLE::GARBAGE_COLLECTOR) {
	std::vector<Eigen::Vector3d> tour;
	for (const auto& p : expl_manager_->ed_->trails_tour_) tour.push_back(p.first);
	visualization_->drawLines(tour, 0.07, color, "trails_tour", 0, 11);
      } else {
	visualization_->drawLines({}, 0.07, Eigen::Vector4d::Zero(), "trails_tour", 0, 11);
      }
      
    } else if (content == 2) {
      
      // Hierarchical grid and global tour --------------------------------
      // vector<Eigen::Vector3d> pts1, pts2;
      // expl_manager_->uniform_grid_->getPath(pts1, pts2);
      // visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.3, 0, 1), "partition", 0,
      // 6);
      
      if (expl_manager_->ep_->drone_id_ == 1) {
	vector<Eigen::Vector3d> pts1, pts2;
	expl_manager_->hgrid_->getGridMarker(pts1, pts2);
	visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);
	
	vector<Eigen::Vector3d> pts;
	vector<string> texts;
	expl_manager_->hgrid_->getGridMarker2(pts, texts);
	static int last_text_num = 0;

	for (int i = 0; i < pts.size(); ++i) {
	  visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
	}

	for (int i = pts.size(); i < last_text_num; ++i) {
	  visualization_->drawText(
				   Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
	}

	last_text_num = pts.size();
	
	// // Pub hgrid to ground node
	// exploration_manager::HGrid hgrid;
	// hgrid.stamp = ros::Time::now().toSec();
	// for (int i = 0; i < pts1.size(); ++i) {
	//   geometry_msgs::Point pt1, pt2;
	//   pt1.x = pts1[i][0];
	//   pt1.y = pts1[i][1];
	//   pt1.z = pts1[i][2];
	//   hgrid.points1.push_back(pt1);
	//   pt2.x = pts2[i][0];
	//   pt2.y = pts2[i][1];
	//   pt2.z = pts2[i][2];
	//   hgrid.points2.push_back(pt2);
	// }
	// hgrid_pub_.publish(hgrid);
      }
      
      auto grid_tour = expl_manager_->ed_->grid_tour_;
      // auto grid_tour = expl_manager_->ed_->grid_tour2_;
      // for (auto& pt : grid_tour) pt = pt + trans;
      
      visualization_->drawLines(grid_tour, 0.05,PlanningVisualization::getColor((expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),"grid_tour", 0, 6);
      
      // Publish grid tour to ground node
      exploration_manager::GridTour tour;

      for (int i = 0; i < grid_tour.size(); ++i) {
	geometry_msgs::Point point;
	point.x = grid_tour[i][0];
	point.y = grid_tour[i][1];
	point.z = grid_tour[i][2];
	tour.points.push_back(point);
      }

      tour.drone_id = expl_manager_->ep_->drone_id_;
      tour.stamp = ros::Time::now().toSec();
      grid_tour_pub_.publish(tour);
      
      visualization_->drawBspline(info->position_traj_, 0.1,PlanningVisualization::getColor((expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),false, 0.15, Vector4d(1, 1, 0, 1));
      
    }
  }
  
  void MvantExplorationFSM::clearVisMarker() {
    
    for (int i = 0; i < 10; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      //visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      //   "frontier_boxes", i, 4);
    }
    
    for (int i = 0; i < 10; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "labeled_frontier", i, 7);
      // visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "dead_frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      //   "frontier_boxes", i, 4);
    }
    // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
    visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
    // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
    // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
    visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);
    
    // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
  }
  
  void MvantExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
    
    if (state_ != WAIT_TRIGGER) {
      
      auto ft = expl_manager_->frontier_finder_;
      auto ed = expl_manager_->ed_;
      
      /*
	auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
	double a = (drone_id - 1) / double(num + 1);
	double b = 1 / double(num + 1);
	return a + b * double(id) / ed->frontiers_.size();
	};
      */
      
      // ft->searchFrontiers();
      // ft->computeFrontiersToVisit();
      // ft->updateFrontierCostMatrix();
      
      // ft->getFrontiers(ed->frontiers_);
      // ft->getFrontierBoxes(ed->frontier_boxes_);
      
      expl_manager_->updateFrontierStruct(fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_);
      
      // cout << "odom: " << fd_->odom_pos_.transpose() << endl;
      // vector<int> tmp_id1;
      // vector<vector<int>> tmp_id2;
      // bool status = expl_manager_->findGlobalTourOfGrid(
      //     { fd_->odom_pos_ }, { fd_->odom_vel_ }, tmp_id1, tmp_id2, true);
      
      // Draw frontier and bounding box
      auto res = expl_manager_->sdf_map_->getResolution();

      for (int i = 0; i < ed->frontiers_.size(); ++i) {
	auto color = visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4);
	visualization_->drawCubes(ed->frontiers_[i], res, color, "frontier", i, 4);
	// getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
	// double(i) / ed->frontiers_.size()
	// visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
	//   color, "frontier_boxes", i, 4);

	Vector3d centroid(0.0, 0.0, 0.0);
	int count = 0;
	
	centroid = ed->views_[i];
	/* ########################################### */
	
	// mostrar el numero de frontera
	auto id_str = std::to_string(ed->fronters_ids_[i]);
	//ROS_WARN_STREAM("Frontera:: " << id_str);
	//ROS_WARN_STREAM("Frontera:: " << centroid.transpose());
	
	visualization_->drawText(centroid - Eigen::Vector3d(0., 0., 0.), "F-"+id_str, 0.8, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), "id", ed->frontiers_.size() + i, 4);
	
	
      }
      
      for (int i = ed->frontiers_.size(); i < 50; ++i) {
	visualization_->drawCubes({}, res, Vector4d(0, 0, 0, 1), "frontier", i, 4);
	visualization_->drawText({}, "", 0.8, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), "id", ed->frontiers_.size() + i, 4);
	// visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
	//   "frontier_boxes", i, 4);

	
      }
      
      // Draw labeled frontier
      /**
      for (size_t i = 0; i < ed->labeled_frontiers_.size(); ++i) {
	Eigen::Vector4d color(0, 0, 0, 0.5);
	if (ed->labeled_frontiers_[i].first == LABEL::TRAIL) {
	  color[0] = 1.;
	} else if (ed->labeled_frontiers_[i].first == LABEL::FRONTIER) {
	  color[1] = 1.;
	} else {
	  color[2] = 1.;
	}
	visualization_->drawCubes(
				  ed->labeled_frontiers_[i].second, res, color, "labeled_frontier", i, 7);
      }
      for (int i = ed->frontiers_.size(); i < 50; ++i) {
	visualization_->drawCubes({}, res, Vector4d(0, 0, 0, 1), "labeled_frontier", i, 7);
      }
      **/
      
      visualize(2);
      // if (status)
      //   visualize(2);
      // else
      //   visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
      
      // Draw grid tour
    }
    
  }
  
  /**
     Función para calcular la distancia euclidiana en 3D
  **/
  double MvantExplorationFSM::getDistance(Eigen::Vector3d& cloud_point, Eigen::Vector3d& point) {
    return std::sqrt(std::pow(cloud_point(0) - point(0), 2) + std::pow(cloud_point(1) - point(1), 2) + std::pow(cloud_point(2) - point(2), 2));
  }

  /**
   * Ejemplo de un topico
   */

  void MvantExplorationFSM::pruebaTopicoCallback(const std_msgs::Empty::ConstPtr& msg){
    ROS_WARN_ONCE("Tipo coordinacion: %s", fp_->coordination_type.c_str());
  }
  
  /**
     Dummy Explorarcion
   **/
  void MvantExplorationFSM::explorationCallback(const ros::TimerEvent& e) {
    //const std_msgs::Empty::ConstPtr& msg){
    
    if (state_ == EXEC_TRAJ || state_ == WAIT_TRIGGER || state_ == INIT || state_ == PUB_TRAJ) return;
    
    auto ft = expl_manager_->frontier_finder_; //cosas de frontier finder
    auto ed = expl_manager_->ed_;              //cosas de exploration data
    
    // Actualizar fonteras
    // respecto a la posicion del vant (pos,yaw,vel)
    auto fronteras_num = expl_manager_->updateFrontierStruct(fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_);
    
    // Resolucion de los voxels (0.15)
    auto res = expl_manager_->sdf_map_->getResolution();
    
    // almacenar distancias
    std::multimap<double,Eigen::Vector3d> distancias;
    
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      
      /* ########################################### */
      // obtener centroide para cada frontera
      Vector3d centroid(0.0, 0.0, 0.0);
      //en views_[] <- viene el centroide de la frontera
      centroid = ed->views_[i];
      /* ########################################### */
      
      // calcular distancia y meterlo a un arreglo
      double _distancia_ = getDistance(fd_->odom_pos_,centroid);
      distancias.emplace(_distancia_,centroid);
            
    }

    auto it = distancias.begin();

    double dis = it->first;
    ROS_WARN_STREAM("DISTANCIA::" << dis);
    if (dis < 3.0) {
      advance(it, 1); // mover el iterador
    }
        
    // multimap los ordena ascendente
    //la posicion del vector views_[] funciona, pero el punto lo marca atras en ciertas condiciones
    Eigen::Vector3d posicion = it->second; // ed->views_[0]; // 

    //ROS_WARN_STREAM("VALOR::" << posicion.transpose());
    
    Eigen::Vector3d pos;
    pos << posicion(0), posicion(1), 1;

    Eigen::Vector3d dir = pos - fd_->odom_pos_;
    
    ed->next_pos_ = pos;
    ed->next_yaw_ = atan2(dir[1], dir[0]);
        
    //pintar un punto - que es el objetivo
    auto _col_ = visualization_->getColor(2 / 20, 1); //RED
    visualization_->drawGoal(pos,0.30,_col_,1);
    
    transitState(PLAN_TRAJ, "pruebasCallback");
    
    // imprimir num de fronteras
    ROS_WARN_STREAM("Num::" << ed->frontiers_.size());
    //visualize(1);

    // transitar a IDLE cuando no existan mas fronteras por descubrir
    if (ed->frontiers_.size() == 0) {
      transitState(IDLE, "pruebasCallback");
    }
    
  }
  
  /***
      Entrada k, di
      Salida Vector con los k cercanos -- talvez su indice
      --
      Obtener los obstaculos cercanos respecto a la posicion del VANT
      debo pasarle el parametro de k
      regresar los vecinos k vecinos mas cercanos
      respecto a una distancia
      - deberia regresar un vector con los vecinos cercanos - los indices -
      
  ***/
  
  
  void MvantExplorationFSM::nearbyObstaclesCallback(const ros::TimerEvent& e) {
    //si tuviera un msg
    //(const exploration_manager::SearchObstacle::ConstPtr& msg) {

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    int _di_ = 10;
    int _k_  = 20;
    
    //ROS_WARN_STREAM("isPositionReachable : " << (expl_manager_->isPositionReachable(Eigen::Vector3d(fd_->odom_pos_[0],fd_->odom_pos_[1],fd_->odom_pos_[2]), Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z)) ? "true" : "false"));
    
    Eigen::Vector3d central_point;
    central_point << fd_->odom_pos_[0], fd_->odom_pos_[1], fd_->odom_pos_[2];
    
    // Los indices si son seguidos
    // ix,iy,iz -> aumentan con incrementos de la resolucion (0.15)
    
    //posToIndex
    Eigen::Vector3i index_pos;
    expl_manager_->sdf_map_->posToIndex(central_point,index_pos);
    //ROS_WARN_STREAM("posToIndex - index0: " << index_pos(0) << ", index1: " << index_pos(1) << ", index3: " << index_pos(2));
    
    // distancia de interes
    double di = _di_ * (1 / expl_manager_->sdf_map_->getResolution());
    //ROS_WARN_STREAM("di:: " << di);
    
    // Definir los limites del cubo
    double minX = index_pos(0)-di, maxX = index_pos(0)+di;
    double minY = index_pos(1)-di, maxY = index_pos(1)+di;
    double minZ = index_pos(2)-1 , maxZ = index_pos(2)+2;
    
    // guardar distancia
    double distancia;
    
    Eigen::Vector3d cloud_points;
    
    int state;
    
    std::multimap<double, std::pair<Eigen::Vector3d, int>> neighborhood; 
    
    // Iterar sobre el cubo 3D con los valores del voxel, entonces la sumatoria debe ser la resolucion del mapa
    for (int x = minX; x < maxX; ++x) {
      for (int y = minY; y < maxY; ++y) {
	for (int z = minZ; z < maxZ; ++z) {
	  
	  // Calcular la distancia de todos y guardar datos en un vector
	  // en el vector guardar x,y,z ; distancia ; occupancy
	  //son los puntos en la iter
	  
	  expl_manager_->sdf_map_->indexToPos(Eigen::Vector3i(x,y,z),cloud_points);
	  
	  distancia = getDistance(cloud_points,central_point);
	  //ROS_WARN_STREAM("Distancia:: " << distancia);
	  //0 - DESCONOCIDO | 1 - LIBRE | 2 - OCUPADO
	  state = expl_manager_->sdf_map_->getOccupancy(Eigen::Vector3i(x,y,z));
	  
	  //guardar en vector
	  //distancia | x,y.z
	  
	  //neighborhood.emplace(distancia,std::make_pair(cloud_points,state));
	  neighborhood.emplace(distancia,std::make_pair(cloud_points,state));
	  
	  //ROS_WARN_STREAM("Estado" << state);
	  
	}
      }
    }
    
    int count = 0;

    //tienen que ser los cercanos ocupados
    //multimap los ordena ascendente
    for (auto it = neighborhood.begin(); it != neighborhood.end() && count < _k_; ++it) {
      
      //si no esta ocupada saltarlo
      
      double magnitud = it->first;
      Eigen::Vector3d vector = it->second.first;
      int _estado_ = it->second.second;
      
      if(_estado_==2){
	
	//ROS_WARN_STREAM("Magnitud: " << magnitud << ", Vector: (" << vector.x() << "," << vector.y() << "," << vector.z() << " - " << _estado_ << ")");
	
	pt.x = vector.x();
	pt.y = vector.y();
	pt.z = vector.z();
	
	cloud.push_back(pt);
	
	++count;
	
      }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    cloud.header.stamp = ros::Time::now().toNSec() / 1e3;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);
    nb_obs_pub_.publish(cloud_msg);
    
  }

  //heartbit que se almacena en algun logger
  void MvantExplorationFSM::heartbitCallback(const ros::TimerEvent& e) {
    heartbit_pub_.publish(std_msgs::Empty());
  }
  
  /**
     Callback para comenzar el programa
     maneja estados para evolucionar el comportamiento del código
  **/
  void MvantExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        
    //----------------------------------------------
    // // Debug traj planner
    //----------------------------------------------
    /*
    Eigen::Vector3d pos;
    pos << msg->pose.position.x, msg->pose.position.y, 1;
    expl_manager_->ed_->next_pos_ = pos;
    
    Eigen::Vector3d dir = pos - fd_->odom_pos_;
    
    expl_manager_->ed_->next_yaw_ = atan2(dir[1], dir[0]);
    fd_->go_back_ = true;
    
    ROS_WARN_STREAM("ODOM: " << fd_->odom_pos_.transpose());
    ROS_WARN_STREAM("atan2: " << atan2(dir[1], dir[0]));
    
    //pintar un punto - que es el objetivo
    auto colors = visualization_->getColor(2 / 20, 1);
    visualization_->drawGoal(pos,expl_manager_->sdf_map_->getResolution(),colors,1);
    
    transitState(PLAN_TRAJ, "triggerCallback");
    return;
    */
    //----------------------------------------------
    
    //Solo se hace cuando el estado es WAIT_TRIGGER
    if (state_ != WAIT_TRIGGER) return;
    
    //obtener la posicion para ir hacia ella
    ROS_WARN_STREAM("Start expl pos: " << fd_->odom_pos_);
    ROS_WARN_STREAM("Start expl pos transpose: " << fd_->start_pos_.transpose());
    
    fd_->trigger_ = true;
    ROS_WARN_STREAM("Triggered!");
    //cout << "Triggered!" << endl;
    fd_->start_pos_ = fd_->odom_pos_;
    
    ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());

    //verificar si aun existen fronteras que atender
    if (expl_manager_->updateFrontierStruct(fd_->odom_pos_, fd_->odom_yaw_, fd_->odom_vel_) != 0) {
      transitState(PLAN_TRAJ, "triggerCallback");
    } else
      transitState(FINISH, "triggerCallback");
  }
  
  void MvantExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
    if (state_ == EXPL_STATE::EXEC_TRAJ) {
      // Check safety and trigger replan if necessary
      double dist;
      bool safe = planner_manager_->checkTrajCollision(dist);
      //si no es seguro
      if (!safe) {
	ROS_WARN_STREAM("Replan: collision detected=======================");
	fd_->avoid_collision_ = true;
	transitState(PLAN_TRAJ, "safetyCallback");
      }
      
      static auto time_check = ros::Time::now();
      if (expl_manager_->sdf_map_->getOccupancy(fd_->odom_pos_) != SDFMap::OCCUPANCY::FREE) {
	if ((ros::Time::now() - time_check).toSec() > 20.) {
	  sendStopMsg(-1);
	}
      } else {
	time_check = ros::Time::now();
      }
    }
  }
  
  void MvantExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;
    
    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;
    
    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;
    
    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

    if (!fd_->have_odom_) {
      fd_->have_odom_ = true;
      fd_->fsm_init_time_ = ros::Time::now();
    }
  }
  
  /***
   * Transitar estado FSM
   **/
  void MvantExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
    
    int pre_s = int(state_);
    state_ = new_state;
    
    ROS_WARN_STREAM("[" + pos_call + "]: Drone "
		    << getId()
		    << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
    
  }
  
  void MvantExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {
    // Broadcast own state periodically
    exploration_manager::DroneState msg;
    msg.drone_id = getId();

    //modificar el vector que almacena la estructura de datos tipo DroneState
    // por que es -1?? --> creo por ser cero basado
    auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];
    
    if (fd_->static_state_) {
      state.pos_ = fd_->odom_pos_;
      state.vel_ = fd_->odom_vel_;
      state.yaw_ = fd_->odom_yaw_;
    } else {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_r = (ros::Time::now() - info->start_time_).toSec();
      state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
      state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
      state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
    }
    state.stamp_ = ros::Time::now().toSec();
    msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
    msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
    msg.yaw = state.yaw_;
    //creo que aqui plancha el mapa
    for (auto id : state.grid_ids_) msg.grid_ids.push_back(id);
    msg.recent_attempt_time = state.recent_attempt_time_;
    msg.stamp = state.stamp_;
    msg.goal_posit = eigenToGeometryMsg(expl_manager_->ed_->next_pos_);
    msg.role = int(expl_manager_->role_);
    
    drone_state_pub_.publish(msg);
  }

  /**
   *recibo un mensaje DroneState con la informacion de un Drone
   */
  void MvantExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
    
    // Update other drones' states
    if (msg->drone_id == getId()) return;
    
    // Skip update if collaboration is inactive
    if (!coll_assigner_->isActive()) return;
    
    // Simulate swarm communication loss
    const Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
    if ((msg_pos - fd_->odom_pos_).norm() > fp_->communication_range_) {
      return;
    }
    
    auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
    if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg
    
    drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
    drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
    drone_state.yaw_ = msg->yaw;
    drone_state.grid_ids_.clear();
    for (auto id : msg->grid_ids) drone_state.grid_ids_.push_back(id);
    drone_state.stamp_ = msg->stamp;
    drone_state.recent_attempt_time_ = msg->recent_attempt_time;
    drone_state.goal_pos_ = geometryMsgToEigen(msg->goal_posit);
    drone_state.role_ = ROLE(msg->role);
    
    //std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
    //std::endl; std::cout << drone_state.pos_.transpose() << std::endl;
    //ROS_WARN_STREAM_THROTTLE(1.0, "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" << drone_state.pos_.transpose());
  }
  
  void MvantExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
    if (state_ == INIT) return;
    
    // Select nearby drone not interacting with recently
    auto& states = expl_manager_->ed_->swarm_state_;
    auto& state1 = states[getId() - 1];
    // bool urgent = (state1.grid_ids_.size() <= 1 /* && !state1.grid_ids_.empty() */);
    bool urgent = state1.grid_ids_.empty();
    auto tn = ros::Time::now().toSec();
    
    // Avoid frequent attempt
    if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;
    
    int select_id = -1;
    double max_interval = -1.0;
    for (int i = 0; i < states.size(); ++i) {
      if (i + 1 <= getId()) continue;
      // Check if have communication recently
      // or the drone just experience another opt
      // or the drone is interacted with recently /* !urgent &&  */
      if (tn - states[i].stamp_ > 0.2) continue;
      if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) continue;
      if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) continue;
      
      double interval = tn - states[i].recent_interact_time_;
      if (interval <= max_interval) continue;
      select_id = i + 1;
      max_interval = interval;
    }
    if (select_id == -1) return;
    
    std::cout << "\nSelect: " << select_id << std::endl;
    // ROS_WARN("Pair opt %d & %d", getId(), select_id);
    
    // Do pairwise optimization with selected drone, allocate goal poses
    auto ego_goal = expl_manager_->ed_->next_pos_;
    auto& state2 = states[select_id - 1];
    auto other_goal = state2.goal_pos_;
    
    auto t1 = ros::Time::now();
    Eigen::Vector3d ego_opt, other_opt;
    const bool res = coll_assigner_->optimizePositions(ego_goal, other_goal, ego_opt, other_opt);
    double alloc_time = (ros::Time::now() - t1).toSec();
    if (!res) {
      ROS_ERROR("Opt failed - keep same assignment");
    }
    
    // Send the result to selected drone and wait for confirmation
    exploration_manager::PairOpt opt;
    opt.from_drone_id = getId();
    opt.to_drone_id = select_id;
    // opt.msg_type = 1;
    opt.stamp = tn;
    // for (auto id : ego_ids) opt.ego_ids.push_back(id);
    // for (auto id : other_ids) opt.other_ids.push_back(id);
    
    for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_pub_.publish(opt);

    // ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf", getId(),
    // select_id,
    //     ros::Time::now().toSec() - tn, alloc_time);
    
    // Reserve the result and wait...
    auto ed = expl_manager_->ed_;
    // ed->ego_ids_ = ego_ids;
    // ed->other_ids_ = other_ids;
    ed->next_pos_ = ego_opt;
    ed->swarm_state_[select_id - 1].goal_pos_ = other_opt;
    ed->pair_opt_stamp_ = opt.stamp;
    ed->wait_response_ = true;
    state1.recent_attempt_time_ = tn;
  }
  
  void MvantExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
    // Create map of all active
    unordered_map<int, char> active_map;
    for (auto ativ : actives) {
      active_map[ativ] = 1;
    }
    
    // Remove allocated ones
    for (auto state : expl_manager_->ed_->swarm_state_) {
      for (auto id : state.grid_ids_) {
	if (active_map.find(id) != active_map.end()) {
	  active_map.erase(id);
	} else {
	  // ROS_ERROR("Inactive grid %d is allocated.", id);
	}
      }
    }
    
    missed.clear();
    for (auto p : active_map) {
      missed.push_back(p.first);
    }
  }
  
  void MvantExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr& msg) {
    if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;
    
    // Check stamp to avoid unordered/repeated msg
    if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4) return;
    expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;
    
    auto& state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
    auto& state2 = expl_manager_->ed_->swarm_state_[getId() - 1];
    
    // auto tn = ros::Time::now().toSec();
    exploration_manager::PairOptResponse response;
    response.from_drone_id = msg->to_drone_id;
    response.to_drone_id = msg->from_drone_id;
    response.stamp = msg->stamp;  // reply with the same stamp for verificaiton
    
    if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
      // Just made another pair opt attempt, should reject this attempt to avoid frequent changes
      // ROS_WARN("Reject frequent attempt");
      response.status = 2;
    } else {
      // No opt attempt recently, and the grid info between drones are consistent, the pair opt
      // request can be accepted
      response.status = 1;
      
      // Update from the opt result
      state1.grid_ids_.clear();
      state2.grid_ids_.clear();
      for (auto id : msg->ego_ids) state1.grid_ids_.push_back(id);
      for (auto id : msg->other_ids) state2.grid_ids_.push_back(id);
      
      state1.goal_pos_ = geometryMsgToEigen(msg->ego_posit);
      state2.goal_pos_ = geometryMsgToEigen(msg->other_posit);
      
      state1.recent_interact_time_ = msg->stamp;
      state2.recent_attempt_time_ = ros::Time::now().toSec();
      expl_manager_->ed_->reallocated_ = true;
      
      if (state_ == IDLE && !state2.grid_ids_.empty()) {
	transitState(PLAN_TRAJ, "optMsgCallback");
	// ROS_WARN("Restart after opt!");
      }
      
      // if (!check_consistency(tmp1, tmp2)) {
      //   response.status = 2;
      //   ROS_WARN("Inconsistent grid info, reject pair opt");
      // } else {
      // }
    }
    for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_res_pub_.publish(response);
  }
  
  void MvantExplorationFSM::optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg) {
    if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;
    
    // Check stamp to avoid unordered/repeated msg
    if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4) return;
    expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;
    
    auto ed = expl_manager_->ed_;
    // Verify the consistency of pair opt via time stamp
    if (!ed->wait_response_ || fabs(ed->pair_opt_stamp_ - msg->stamp) > 1e-5) return;
    
    ed->wait_response_ = false;
    // ROS_WARN("get response %d", int(msg->status));
    
    if (msg->status != 1) return;  // Receive 1 for valid opt
    
    auto& state1 = ed->swarm_state_[getId() - 1];
    auto& state2 = ed->swarm_state_[msg->from_drone_id - 1];
    state1.grid_ids_ = ed->ego_ids_;
    state2.grid_ids_ = ed->other_ids_;
    // FIXME goal here?
    state2.recent_interact_time_ = ros::Time::now().toSec();
    ed->reallocated_ = true;
    
    if (state_ == IDLE && !state1.grid_ids_.empty()) {
      transitState(PLAN_TRAJ, "optResMsgCallback");
      // ROS_WARN("Restart after opt!");
    }
  }

  /**
   *recibe trajectorias del grupo de drones
   *
   */
  void MvantExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
    // Get newest trajs from other drones, for inter-drone collision avoidance
    auto& sdat = planner_manager_->swarm_traj_data_;
    
    // Ignore self trajectory
    if (msg->drone_id == sdat.drone_id_) return;
    
    // Ignore outdated trajectory
    if (sdat.receive_flags_[msg->drone_id - 1] == true &&
	msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
      return;
    
    // Convert the msg to B-spline
    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
    Eigen::VectorXd knots(msg->knots.size());
    for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    
    for (int i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(i, 0) = msg->pos_pts[i].x;
      pos_pts(i, 1) = msg->pos_pts[i].y;
      pos_pts(i, 2) = msg->pos_pts[i].z;
    }
    
    // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
    // Eigen::Vector4d tf;
    // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
    // double yaw = tf[3];
    // Eigen::Matrix3d rot;
    // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    // Eigen::Vector3d trans = tf.head<3>();
    // for (int i = 0; i < pos_pts.rows(); ++i) {
    //   Eigen::Vector3d tmp = pos_pts.row(i);
    //   tmp = rot * tmp + trans;
    //   pos_pts.row(i) = tmp;
    // }
    
    sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
    sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
    sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
    sdat.receive_flags_[msg->drone_id - 1] = true;
    
    if (state_ == EXEC_TRAJ) {
      // Check collision with received trajectory
      if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
	ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
	fd_->avoid_collision_ = true;
	//forzar a volver a planear una trajectoria
	transitState(PLAN_TRAJ, "swarmTrajCallback");
      }
    }
  }
  
  void MvantExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
    // Broadcast newest traj of this drone to others
    if (state_ == EXEC_TRAJ) {
      swarm_traj_pub_.publish(fd_->newest_traj_);
      
    } else if (state_ == WAIT_TRIGGER) {
      // Publish a virtual traj at current pose, to avoid collision
      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = ros::Time::now();
      bspline.traj_id = planner_manager_->local_data_.traj_id_;
      
      Eigen::MatrixXd pos_pts(4, 3);
      for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();
      
      for (int i = 0; i < pos_pts.rows(); ++i) {
	geometry_msgs::Point pt;
	pt.x = pos_pts(i, 0);
	pt.y = pos_pts(i, 1);
	pt.z = pos_pts(i, 2);
	bspline.pos_pts.push_back(pt);
      }
      
      NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
      Eigen::VectorXd knots = tmp.getKnot();
      for (int i = 0; i < knots.rows(); ++i) {
	bspline.knots.push_back(knots(i));
      }
      bspline.drone_id = expl_manager_->ep_->drone_id_;
      swarm_traj_pub_.publish(bspline);
    }
  }
}  // namespace fast_planner
