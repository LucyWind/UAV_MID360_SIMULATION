# FAST-LIO 与 Livox CustomMsg 仿真测试报告

## 1. 测试目的

本次测试用于验证修改后的 Gazebo Livox 仿真插件所发布的
`livox_ros_driver2/msg/CustomMsg` 是否能够被 ROS 2 版 FAST-LIO 正确接收和处理。

测试重点包括：

1. 雷达消息类型是否为 Livox `CustomMsg`。
2. 雷达与 IMU 是否按预期频率发布。
3. `CustomMsg` 中关键字段是否自洽。
4. FAST-LIO 是否能够完成 IMU 初始化和地图初始化。
5. FAST-LIO 是否能够持续发布里程计、配准点云和地图。
6. RViz 是否能够订阅并显示 FAST-LIO 的结果。

本次测试使用的是静止传感器和简单几何场景，因此主要验证消息兼容性与完整数据链路，
不等价于动态飞行精度、运动畸变补偿效果或真实 Mid-360 扫描规律验证。

## 2. 测试环境

- 工作区：`/home/lucy/uav_sim_ws/ros2_ws`
- ROS 2：Humble
- 仿真器：Gazebo Classic 11
- LIO 算法包：`fast_lio`
- 雷达仿真包：`livox_gazebo_ros2_gpu_simulation`
- 点云消息类型：`livox_ros_driver2/msg/CustomMsg`
- 测试世界：`/usr/share/gazebo-11/worlds/shapes.world`
- 雷达话题：`/mid360`
- IMU 话题：`/imu/data`
- FAST-LIO 固定坐标系：`camera_init`

`shapes.world` 中包含地面、方块、球体和圆柱，用于给静止雷达提供可辨识的几何点云。

## 3. 测试前检查

### 3.1 检查 FAST-LIO 的输入支持

检查 FAST-LIO 源码后确认：

- 当 `preprocess.lidar_type: 1` 时，FAST-LIO 创建
  `livox_ros_driver2/msg/CustomMsg` 订阅器。
- 其他类型走 `sensor_msgs/msg/PointCloud2` 订阅器。
- Livox 点的 `offset_time` 会被 FAST-LIO 转换为点的相对时间，用于点云预处理和去畸变。

因此，原仿真配置中的 `lidar_type: 4` 必须改成 `1`，才能真正进入 Livox
`CustomMsg` 回调，而不是继续等待 `PointCloud2`。

### 3.2 检查测试模型坐标关系

插件自带的独立 Mid-360 测试模型中：

- 雷达和 IMU 都安装在同一个 `mid360` link 上。
- 两者之间没有平移和旋转。

因此本次独立测试使用单位外参：

```yaml
extrinsic_T: [0.0, 0.0, 0.0]
extrinsic_R: [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
```

该外参只适用于本次独立 Mid-360 测试模型，不应直接用于通过 MAVROS
提供机体 IMU 的无人机配置。

## 4. 备份工作

修改 FAST-LIO 配置前，原文件已备份到：

```text
/home/lucy/uav_sim_ws/backups/fast_lio_before_custommsg_validation_20260729/mid360_sim.yaml
```

需要恢复时可以执行：

```bash
cp \
  /home/lucy/uav_sim_ws/backups/fast_lio_before_custommsg_validation_20260729/mid360_sim.yaml \
  /home/lucy/uav_sim_ws/ros2_ws/src/FAST_LIO/config/mid360_sim.yaml
```

## 5. FAST-LIO 配置修改

修改文件：

```text
/home/lucy/uav_sim_ws/ros2_ws/src/FAST_LIO/config/mid360_sim.yaml
```

### 5.1 输入话题

将 IMU 输入从 MAVROS 话题：

```yaml
imu_topic: "/mavros/imu/data_raw"
```

改为独立 Gazebo Mid-360 模型直接发布的话题：

```yaml
imu_topic: "/imu/data"
```

雷达输入保持：

```yaml
lid_topic: "/mid360"
```

这样雷达与 IMU 都使用同一个 Gazebo 仿真时钟，避免在本次独立测试中引入
MAVROS 转换链路。

### 5.2 点云消息分支

将：

```yaml
lidar_type: 4
```

改为：

```yaml
lidar_type: 1
```

FAST-LIO 内部将类型 `1` 命名为 AVIA，但该分支本质上用于订阅
`livox_ros_driver2/msg/CustomMsg`，同样可以用于本次 Mid-360 CustomMsg 测试。

### 5.3 扫描频率

将配置中的扫描频率设为：

```yaml
scan_rate: 10
```

使其与 Gazebo 雷达传感器的 10 Hz 更新频率一致。

### 5.4 外参

独立模型中雷达和 IMU 共坐标系，因此设置为：

```yaml
extrinsic_est_en: false
extrinsic_T: [0.0, 0.0, 0.0]
extrinsic_R: [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
```

关闭在线外参估计，避免静止、简单场景下外参发生不必要的漂移。

## 6. 启动过程中发现并修复的问题

### 6.1 `robot_description` 被错误地当作 YAML 解析

第一次启动测试 launch 文件时出现：

```text
Unable to parse the value of parameter robot_description as yaml
```

原因是 ROS 2 Humble 的 launch 系统将 Xacro 命令输出尝试按 YAML 参数解析。

修改文件：

```text
/home/lucy/uav_sim_ws/ros2_ws/src/livox_sim_gpu_ros2/launch/test_gpu_laser.launch.py
```

将 Xacro 命令输出明确声明为字符串：

```python
robot_description = {
    'robot_description': ParameterValue(
        Command(['xacro ', model_file]),
        value_type=str,
    )
}
```

同时增加：

```python
from launch_ros.parameter_descriptions import ParameterValue
```

修复后，测试模型能够被 `spawn_entity.py` 正常生成到 Gazebo 中。

### 6.2 独立 Xacro 中没有正确发布 IMU 话题

点云发布成功后检查发现：

```text
Unknown topic '/imu/data'
```

原因有两个：

1. `gazebo_ros_imu_sensor` 使用了不正确的 `<topic_name>` 写法。
2. IMU 的 `<imu>` 噪声配置错误地放在 `<plugin>` 内部，而不是直接放在
   `<sensor>` 内部。

修改文件：

```text
/home/lucy/uav_sim_ws/ros2_ws/src/livox_sim_gpu_ros2/urdf/mid360.xacro
```

将话题配置改为 Gazebo ROS 2 IMU 插件使用的 remap 形式：

```xml
<plugin name="${name}_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
  <ros>
    <namespace>/</namespace>
    <remapping>~/out:=${imu_topic}</remapping>
  </ros>
  <frame_name>${name}</frame_name>
</plugin>
```

同时把 `<imu>` 噪声块移动到 `<sensor type="imu">` 下。

修复后成功出现：

```text
/imu/data [sensor_msgs/msg/Imu]
```

## 7. 构建工作

重新构建了雷达插件和 FAST-LIO：

```bash
cd /home/lucy/uav_sim_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select livox_gazebo_ros2_gpu_simulation fast_lio \
  --symlink-install
```

构建结果：

```text
Summary: 2 packages finished
```

FAST-LIO 构建过程中只有 PCL CMake 策略和 Boost 占位符弃用提示，没有编译错误。

修复测试 launch 后，还单独重新构建过雷达仿真包：

```bash
colcon build \
  --packages-select livox_gazebo_ros2_gpu_simulation \
  --symlink-install
```

## 8. Gazebo 测试启动

测试使用以下命令启动：

```bash
cd /home/lucy/uav_sim_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_gazebo_ros2_gpu_simulation \
  test_gpu_laser.launch.py \
  world_file:=/usr/share/gazebo-11/worlds/shapes.world
```

启动后确认插件参数为：

```text
Output type: livox_custom
LiDAR id: 0
Samples per update: 20000
Topic name: mid360
Frame name: mid360
```

扫描模式文件成功加载：

```text
Successfully loaded 800000 scan pattern points from CSV
```

GPU 雷达射线网格为：

```text
1440 × 252 = 362880 rays
```

每次更新从扫描模式中尝试采样 20000 条射线。

## 9. 原始数据检查

### 9.1 话题类型

雷达话题：

```text
/mid360 [livox_ros_driver2/msg/CustomMsg]
```

IMU 话题：

```text
/imu/data [sensor_msgs/msg/Imu]
```

雷达发布端使用可靠 QoS：

```text
Reliability: RELIABLE
Durability: VOLATILE
```

### 9.2 发布频率

实测结果：

| 话题 | 实测频率 |
|---|---:|
| `/mid360` | 约 10.00 Hz |
| `/imu/data` | 约 99.5 Hz |

雷达实测区间大致为：

```text
min: 0.088 s
max: 0.108 s
average rate: 10.001 Hz
```

### 9.3 IMU 静止重力数据

读取一帧 IMU 线加速度：

```text
x: -0.0091
y: -0.0122
z:  9.7862
```

静止状态下 Z 轴接近 `9.81 m/s²`，说明 IMU 数据不是零值占位。

### 9.4 Livox CustomMsg 完整字段检查

对同一帧完整数组进行统计，得到：

```text
header_ns=204374000000
timebase=204374000000
point_num=3030
points_entries=3030
offset_time=[0,0]
line=[0,3]
tag=[0,0]
```

该结果说明：

1. `header.stamp` 转换成纳秒后与 `timebase` 完全相等。
2. `point_num` 与 `points` 数组实际长度一致。
3. 所有点的 `offset_time` 都是 0，符合本次“全域同时快照”定义。
4. `line` 覆盖 0～3。
5. `tag` 为 0，能够通过 FAST-LIO 对有效 Livox 点标签的判断。

### 9.5 为什么不是固定 20000 个输出点

插件每帧尝试发射 20000 条射线，但只把实际命中场景的射线写入消息。

在 `shapes.world` 中，大约有 84.6% 的射线没有命中有效物体，因此一帧通常只有
约 3000 个有效点，例如：

```text
point_num=3030
```

没有命中的射线没有用零坐标、重复点或伪造距离填充到 20000 个点。

所以：

- `20000` 表示每帧尝试的射线/采样数量。
- `point_num` 表示这一帧真实命中的有效点数量。
- 当前消息满足“数据真实，不为了凑点数而填充”的要求。

## 10. FAST-LIO 启动

使用以下命令启动 FAST-LIO 和 RViz：

```bash
cd /home/lucy/uav_sim_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360_sim.yaml \
  use_sim_time:=true \
  rviz:=true
```

FAST-LIO 启动日志确认：

```text
p_pre->lidar_type 1
Multi thread started
Node init finished.
IMU Initial Done
Initialize the map kdtree
```

其中 `p_pre->lidar_type 1` 证明 FAST-LIO 使用的是 Livox `CustomMsg`
订阅分支。

初始化早期曾出现一次：

```text
No point, skip this scan!
```

随后完成地图初始化并持续输出。这属于 IMU 初始化与首帧数据对齐阶段的一次跳帧，
不是持续性错误。

## 11. FAST-LIO 输出验证

实测输出：

| FAST-LIO 输出 | 验证结果 |
|---|---|
| `/Odometry` | 持续发布，约 10 Hz |
| `/cloud_registered` | 持续发布，约 10 Hz |
| `/Laser_map` | 能够接收到累计地图 |
| `/path` | 持续发布，约 1 Hz |
| TF `camera_init -> body` | 由 FAST-LIO 发布 |

一帧配准点云的宽度示例：

```text
cloud_registered.width=3039
```

累计地图宽度示例：

```text
Laser_map.width=484189
```

这表明 FAST-LIO 已经完成以下数据链路：

```text
Livox CustomMsg
        ↓
FAST-LIO Livox 预处理
        ↓
IMU 初始化与状态传播
        ↓
点云配准与地图更新
        ↓
Odometry / Path / Registered Cloud / Laser Map
```

## 12. RViz 可视化

FAST-LIO 的 launch 文件同时启动了 RViz：

```text
rviz2
```

使用的 RViz 配置为：

```text
/home/lucy/uav_sim_ws/ros2_ws/src/FAST_LIO/rviz/fastlio.rviz
```

主要显示项：

- 固定坐标系：`camera_init`
- 里程计：`/Odometry`
- 轨迹：`/path`
- 累计地图：`/Laser_map`
- 可选配准点云：`/cloud_registered`
- TF：`camera_init -> body`

测试时 Gazebo 和 RViz 图形窗口均已正常启动。

## 13. 测试结束与进程清理

测试完成后向两个 ROS 2 launch 会话发送了 `Ctrl+C`，关闭：

- Gazebo `gzserver`
- Gazebo `gzclient`
- FAST-LIO
- RViz
- `laser_listener`
- `robot_state_publisher`
- Livox 仿真插件节点

随后使用绕过 ROS 2 daemon 缓存的节点检查确认，没有相关测试节点残留。

Gazebo 在卸载 GPU 雷达插件时报告了一次 `exit code -11`，但进程已经终止，
没有遗留运行中的 Gazebo 或 ROS 2 测试节点。该退出码发生在测试结束的插件卸载阶段，
不影响前面的数据和算法验证结果，但后续可以单独排查插件析构阶段的问题。

## 14. 测试结论

### 14.1 已验证通过

1. 仿真插件能够发布真正的 `livox_ros_driver2/msg/CustomMsg`。
2. FAST-LIO 能够进入 Livox CustomMsg 订阅和预处理分支。
3. 点云与 IMU 频率符合 10 Hz 和约 100 Hz 的配置。
4. `point_num`、数组长度、时间戳、线号和标签字段自洽。
5. 全域快照点的 `offset_time` 全部为 0。
6. 无效射线被丢弃，没有伪造点填充到 20000。
7. FAST-LIO 能完成 IMU 初始化、地图初始化和持续状态估计。
8. 里程计、路径、配准点云和累计地图均能输出。
9. RViz 能订阅 FAST-LIO 输出进行可视化。

因此，从消息格式兼容性和算法数据链路角度看，该 CustomMsg 点云可以供
FAST-LIO 使用。

### 14.2 尚未验证

本次测试不能证明以下内容：

1. 无人机动态飞行时的里程计精度。
2. 快速旋转、快速平移时的稳定性。
3. 真实 Mid-360 非重复扫描随时间展开的效果。
4. 逐点运动畸变与 FAST-LIO 去畸变效果。
5. MAVROS IMU 坐标系、时间戳和实际雷达外参是否正确。
6. 大型、复杂场景下的实时性能和长期内存占用。

由于所有点的 `offset_time=0`，FAST-LIO 会把每帧理解为同一时刻的全域快照，
不会对帧内点执行有意义的逐点运动补偿。这符合当前插件的快照语义，但不等价于真实
Livox 扫描。

### 14.3 静止测试中的漂移

静止测试中观察到数厘米位置变化和一定姿态变化。测试场景较简单、有效点约为每帧
3000 个，几何约束有限，因此本次结果只判定为“消息可用、算法链路可运行”，
不判定为“里程计精度已经合格”。

## 15. 本次涉及的文件

### FAST-LIO 配置

```text
/home/lucy/uav_sim_ws/ros2_ws/src/FAST_LIO/config/mid360_sim.yaml
```

### Gazebo 测试启动文件

```text
/home/lucy/uav_sim_ws/ros2_ws/src/livox_sim_gpu_ros2/launch/test_gpu_laser.launch.py
```

### Mid-360 Xacro

```text
/home/lucy/uav_sim_ws/ros2_ws/src/livox_sim_gpu_ros2/urdf/mid360.xacro
```

### 配置备份

```text
/home/lucy/uav_sim_ws/backups/fast_lio_before_custommsg_validation_20260729/mid360_sim.yaml
```

