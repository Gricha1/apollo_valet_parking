#!/usr/bin/env bash


TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/apollo_base.sh"

#${TOP_DIR}/bazel-bin/modules/tools/perception/publishing_obstacles "$@"
${TOP_DIR}/bazel-bin/modules/tools/perception/publishing_obstacles_many_obstacles_same_speed "$@"





#dynamic obstacle in Borregas ave
#python3 PythonAPI/scripts/replay_perception.py PythonAPI/scripts/demo1_onroad_vehicle.json  $@


#my code for publishing 
#python3 PythonAPI/scripts/publishing_random_obstacles.py $@
#python3 added_scripts/publishing_obstacles.py $@






