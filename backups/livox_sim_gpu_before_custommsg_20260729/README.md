# Livox GPU simulation backup

This directory contains the exact source files captured before adding Livox
`CustomMsg` output and the configurable output type.

Restore them from the workspace root with:

```bash
bash backups/livox_sim_gpu_before_custommsg_20260729/restore.sh
```

After restoring, rebuild the package if the installed workspace should also
use the restored plugin:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select livox_gazebo_ros2_gpu_simulation
```
