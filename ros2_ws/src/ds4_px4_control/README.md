# DS4 MAVROS position control

This ROS 2 node uses the American-hand/Mode 2 stick layout and sends PX4
Offboard position targets through MAVROS. It does not publish directly to
PX4 `/fmu/in/*` topics.

## Default controls (Mode 2)

| DS4 control | Action |
|---|---|
| Left stick X | Yaw left/right |
| Left stick Y | Climb/descend |
| Right stick X | Move left/right |
| Right stick Y | Move forward/backward |
| CROSS | Pre-stream setpoints, request OFFBOARD, then arm through MAVROS |
| Hold L1 | Enable stick motion |
| TRIANGLE | Reset the target to the measured current position |
| CIRCLE | Request `AUTO.LAND` through MAVROS |

All four stick axes are read and integrated in the same 20 Hz control cycle,
so lateral, vertical, and yaw commands work simultaneously. Releasing L1 or
losing `/joy` input holds the last target.

MAVROS provides ROS ENU coordinates on `/mavros/local_position/pose`; this
node publishes ENU targets to `/mavros/setpoint_position/local`. MAVROS
performs the conversion to PX4's NED frame.

## Build

```bash
cd /home/lucy/uav_sim_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ds4_px4_control
source install/setup.bash
```

The required ROS packages are `joy`, `mavros`, and `mavros_msgs`.

## Start MAVROS and run

For a common PX4 SITL UDP connection:

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

Then start the controller in another terminal:

```bash
source /home/lucy/uav_sim_ws/ros2_ws/install/setup.bash
ros2 launch ds4_px4_control ds4_px4_control.launch.py
```

For `/dev/input/js1`, add `device_id:=1` to the controller launch command.

Before pressing CROSS, verify the connection, pose, and DS4 mapping:

```bash
ros2 topic echo /mavros/state
ros2 topic hz /mavros/local_position/pose
ros2 topic echo /joy
```

The default axis/button numbering in `config/ds4.yaml` is common but can vary
by Linux kernel and DS4 driver. Change the `axis_*`, `sign_*`, and `button_*`
parameters to match the observed `/joy` data.

Test in SITL first and remove propellers when testing with real hardware. PX4
preflight and safety checks remain active and may reject OFFBOARD or arming.
