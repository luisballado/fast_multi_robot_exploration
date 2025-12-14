#!bash/usr/bin/env

# Script para correr un experimento,
# <num_drones> 
# <planificador> 
# <ruta global a carpeta para guardar los datos> 
# <modelo pcd> 
# <activar vulcan>
# <visualizar>
# <comm_range> 
# modelos disponibles:
# office, office2, office3
# forest_50x50_01_200
# forest_50x50_01_300
# forest_50x50_100_denser_3
# forest_4_densities_50_40_20_10
# bash single_run.sh 10 mvant /home/logg office2 false

# User input
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Input arguments
# no igual a 5
# 
if [ "$#" -ne 7 ]; then
  echo -e "${RED}Wrong number of input arguments${NC}"
  return
else

  drone_num=${1}
  planner_type=${2}
  log_folder=${3}
  model=${4}
  vulkan_renderer=${5}
  
  if [[ "${planner_type}" != "racer" && "${planner_type}" != "fame" && "${planner_type}" != "mvant" ]]; then
    echo -e "${RED}Unknown planner type: ${planner_type}${NC}"
    return
  fi

  # Parameters
  visualize=${6}
  communication_range=${7} #1000000.0  # Basically infinite communication

  if [ ${vulkan_renderer} == true ]; then
    waiting_time=60.0
  else
    waiting_time=15.0
  fi

  # Roscore
  roscore &
  sleep 3s

  # Visualization
  if [ ${visualize} == true ]; then
    roslaunch exploration_manager rviz.launch &
    sleep 2s
  fi
  
  # Planner
  cmd_line_args="drone_num:=${drone_num} log_folder:=${log_folder} vulkan_renderer:=${vulkan_renderer} model:=${model} communication_range:=${communication_range}"
  
  if [ ${planner_type} == "explorer" ]; then
    cmd_line_args="${cmd_line_args} role_assigner_fixed:=true planner_type:=fame"
  elif [ ${planner_type} == "no_coll" ]; then
    cmd_line_args="${cmd_line_args} active_collaboration:=false planner_type:=fame"
  else
    cmd_line_args="${cmd_line_args} planner_type:=${planner_type}"
  fi

  roslaunch exploration_manager swarm_exploration.launch ${cmd_line_args} &
  sleep 2s

  # Automatic trigger
  roslaunch logging_utils trigger_node.launch waiting_time:=${waiting_time} &
  sleep 1s

  # Logging
  roslaunch logging_utils logging_node.launch drone_num:=${drone_num} log_folder:=${log_folder} model:=${model}
  sleep 1s

  # Kill all
  kill $(jobs -p)
  rosnode kill -a && killall rosmaster

  # Backup
  # cp src/fast_multi_robot_exploration/swarm_exploration/exploration_manager/launch/single_drone_planner_${planner_type}.xml ${log_folder}

fi
