#!/usr/bin/env bash
set -euo pipefail

backup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${backup_dir}/../.." && pwd)"

cp -a -- \
  "${backup_dir}/ros2_ws/src/livox_sim_gpu_ros2/include/livox_sim_plugins/livox_sim_gpu_laser.h" \
  "${workspace_root}/ros2_ws/src/livox_sim_gpu_ros2/include/livox_sim_plugins/livox_sim_gpu_laser.h"
cp -a -- \
  "${backup_dir}/ros2_ws/src/livox_sim_gpu_ros2/src/livox_sim_gpu_laser.cpp" \
  "${workspace_root}/ros2_ws/src/livox_sim_gpu_ros2/src/livox_sim_gpu_laser.cpp"
cp -a -- \
  "${backup_dir}/ros2_ws/src/livox_sim_gpu_ros2/CMakeLists.txt" \
  "${workspace_root}/ros2_ws/src/livox_sim_gpu_ros2/CMakeLists.txt"
cp -a -- \
  "${backup_dir}/ros2_ws/src/livox_sim_gpu_ros2/package.xml" \
  "${workspace_root}/ros2_ws/src/livox_sim_gpu_ros2/package.xml"
cp -a -- \
  "${backup_dir}/ros2_ws/src/livox_sim_gpu_ros2/livox_sim_gpu_laser.xml" \
  "${workspace_root}/ros2_ws/src/livox_sim_gpu_ros2/livox_sim_gpu_laser.xml"
cp -a -- \
  "${backup_dir}/ros2_ws/src/livox_sim_gpu_ros2/urdf/mid360.xacro" \
  "${workspace_root}/ros2_ws/src/livox_sim_gpu_ros2/urdf/mid360.xacro"
cp -a -- \
  "${backup_dir}/ros2_ws/src/px4_mid360_sim/models/iris_mid360/model.sdf" \
  "${workspace_root}/ros2_ws/src/px4_mid360_sim/models/iris_mid360/model.sdf"

echo "Restored Livox simulation sources to the state captured before this change."
