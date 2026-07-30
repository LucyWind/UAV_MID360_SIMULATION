# Livox GPU 仿真插件修改说明

## 1. 修改目标

本次修改将 MID-360 GPU 仿真插件设置为：

- 雷达更新频率：10 Hz
- 每次快照尝试采样：20,000 条 Livox 扫描方向
- 默认输出类型：`livox_ros_driver2/msg/CustomMsg`
- 可通过 XML/SDF 参数切换为 `sensor_msgs/msg/PointCloud2`
- 一帧内所有点来自同一个 Gazebo GPU 深度快照
- 一帧内所有 Livox 点的 `offset_time` 均为 0
- 无效或没有返回的射线直接丢弃，不使用零点凑数

这里的“20,000 点”准确含义是每个周期尝试处理 20,000 条射线。最终消息中的
`point_num` 是实际有效返回点数量；如果部分射线没有击中物体，消息点数会小于
20,000。

## 2. 修改的文件

### 2.1 插件头文件

文件：

`ros2_ws/src/livox_sim_gpu_ros2/include/livox_sim_plugins/livox_sim_gpu_laser.h`

修改内容：

- 引入 `livox_ros_driver2/msg/custom_msg.hpp`
- 增加 `OutputType` 枚举：
  - `PointCloud2`
  - `LivoxCustom`
- 将原来的单一 PointCloud2 发布器拆分为：
  - `pointcloud2_pub_`
  - `custom_pub_`
- 增加：
  - `output_type_`
  - `lidar_id_`
- 将默认每周期处理点数对应的语义改为“每次快照处理的射线数量”

## 2.2 插件核心实现

文件：

`ros2_ws/src/livox_sim_gpu_ros2/src/livox_sim_gpu_laser.cpp`

### 参数读取

新增两个 SDF 参数：

```xml
<output_type>livox_custom</output_type>
<lidar_id>0</lidar_id>
```

`output_type` 支持：

- `pointcloud2`
- `livox_custom`

另外兼容 `custom` 和 `custommsg` 写法，并且不区分大小写。

如果没有设置 `samples`，插件默认使用：

```text
samples = 20000
```

如果 `output_type` 不合法，或者 `lidar_id` 不在 0～255 范围内，插件会输出错误并
停止初始化。

### 发布器选择

插件启动时只创建所选消息类型的发布器：

```text
output_type=livox_custom
    -> livox_ros_driver2/msg/CustomMsg

output_type=pointcloud2
    -> sensor_msgs/msg/PointCloud2
```

两个发布器使用可靠 QoS、队列深度 10，以便兼容 Point-LIO 中常见的 CustomMsg
订阅方式。

### 全域快照时间模型

每次 Gazebo 激光回调只使用该回调携带的一个仿真时间：

```cpp
scan_stamp = Gazebo LaserScanStamped time
timebase = scan_stamp 转换成纳秒
```

由于所有点来自同一张 GPU 深度图，因此：

```cpp
custom_point.offset_time = 0;
```

没有使用 CSV 时间列伪造逐点时间，也没有使用 CPU 处理耗时作为点时间。

### Livox CustomMsg 字段来源

| 字段 | 数据来源 |
|---|---|
| `header.stamp` | 当前 Gazebo 激光快照的仿真时间 |
| `header.frame_id` | XML/SDF 中的 `frameName` |
| `timebase` | 与 `header.stamp` 相同的时间，单位为纳秒 |
| `point_num` | 实际有效点数量，等于 `points.size()` |
| `lidar_id` | XML/SDF 中的 `lidar_id` |
| `rsvd` | 按 Livox 协议规定全部置 0 |
| `offset_time` | 全部为 0，因为整帧来自同一时刻 |
| `x/y/z` | GPU 深度图距离与 CSV 扫描角度计算结果 |
| `reflectivity` | Gazebo `LaserScan` 中对应射线的 intensity |
| `tag` | `0x00`，表示正常、有效的单回波点 |
| `line` | 原始 MID-360 扫描序号对 4 取模 |

反射率会限制在 Livox 规定的 0～255 范围内。如果 Gazebo 场景没有配置
`laser_retro`，Gazebo intensity 会是 0，此时发布的 reflectivity 也会真实地保持为
0，不会随机生成。

### 无效射线处理

以下射线不会加入消息：

- 角度超过传感器视场
- 深度图索引越界
- 距离不是有限数值
- 距离小于最小量程
- 距离达到或超过最大量程

因此不会加入 `(0, 0, 0)` 点来凑满 20,000。

### PointCloud2 输出

选择 `pointcloud2` 时，保留以下字段：

```text
x
y
z
reflectivity
tag
line
```

其中 reflectivity 也改为使用 Gazebo 的实际 intensity，而不是原来的固定 0。

### 插件退出稳定性

原插件的延迟加载线程在析构时没有执行 `join()`，Gazebo 关闭时可能触发：

```text
terminate called without an active exception
```

现在析构函数会先检查并回收 `deferred_load_thread_`，再释放 ROS 节点。运行测试
确认关闭时不再出现上述 `std::terminate`。

## 2.3 CMake 构建配置

文件：

`ros2_ws/src/livox_sim_gpu_ros2/CMakeLists.txt`

修改内容：

```cmake
find_package(livox_ros_driver2 REQUIRED)
```

并将 `livox_ros_driver2` 加入插件目标的 `ament_target_dependencies`。

Gazebo 依赖在 Livox 依赖之前发现，以避免 PCL/JsonCpp 与 Gazebo 重复创建
`JsonCpp::JsonCpp` 目标。

## 2.4 ROS 包依赖

文件：

`ros2_ws/src/livox_sim_gpu_ros2/package.xml`

新增：

```xml
<depend>livox_ros_driver2</depend>
```

## 2.5 MID-360 Xacro

文件：

`ros2_ws/src/livox_sim_gpu_ros2/urdf/mid360.xacro`

宏新增参数：

```xml
output_type:='livox_custom'
lidar_id:='0'
```

雷达更新率设置为：

```xml
<update_rate>10</update_rate>
```

每周期处理数量为：

```xml
<samples>20000</samples>
```

调用宏时可以这样选择输出类型：

```xml
<xacro:mid360
  name="mid360"
  parent="base_link"
  topic="mid360"
  imu_topic="imu/data"
  output_type="livox_custom"
  lidar_id="0">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:mid360>
```

切换为 PointCloud2：

```xml
output_type="pointcloud2"
```

## 2.6 PX4 MID-360 模型

文件：

`ros2_ws/src/px4_mid360_sim/models/iris_mid360/model.sdf`

雷达更新率从本轮开始前的 100 Hz 改为：

```xml
<update_rate>10</update_rate>
```

插件配置增加：

```xml
<samples>20000</samples>
<output_type>livox_custom</output_type>
<lidar_id>0</lidar_id>
```

如果需要改回 PointCloud2，只需修改为：

```xml
<output_type>pointcloud2</output_type>
```

## 2.7 插件描述 XML

文件：

`ros2_ws/src/livox_sim_gpu_ros2/livox_sim_gpu_laser.xml`

在插件描述中注明 `output_type` 支持：

- `pointcloud2`
- `livox_custom`

实际运行参数仍然应写在 Xacro/SDF 的 `<plugin>` 节点中。

## 3. 构建与验证

构建命令：

```bash
cd /home/lucy/uav_sim_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select livox_gazebo_ros2_gpu_simulation
```

构建结果：

```text
Summary: 1 package finished
```

无界面 Gazebo 运行验证结果：

### Livox CustomMsg 模式

```text
话题类型：livox_ros_driver2/msg/CustomMsg
实测频率：约 9.94～10.00 Hz
point_num == points.size()
timebase == header.stamp 对应的纳秒时间
全部 offset_time == 0
line 值包含 0、1、2、3
```

空世界测试中，每周期尝试 20,000 条射线，约 2,500 条射线击中地面，因此消息中
约有 2,500 个有效点。这证明插件没有使用无效点填充消息。

### PointCloud2 模式

```text
话题类型：sensor_msgs/msg/PointCloud2
实测频率：约 10.00 Hz
```

两种 `output_type` 均已实际启动验证。

## 4. 备份与恢复

修改前的文件保存在：

```text
/home/lucy/uav_sim_ws/backups/livox_sim_gpu_before_custommsg_20260729
```

该备份包含本轮开始时已有的 `model.sdf` 100 Hz 版本。

一键恢复：

```bash
cd /home/lucy/uav_sim_ws
bash backups/livox_sim_gpu_before_custommsg_20260729/restore.sh
```

恢复后重新构建：

```bash
cd /home/lucy/uav_sim_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select livox_gazebo_ros2_gpu_simulation
```

备份目录包含 `COLCON_IGNORE`，不会被 `colcon` 错误识别为重复 ROS 包。

## 5. 当前未修改的部分

本次只修改了 Livox GPU 仿真插件及 MID-360 模型配置，没有修改 Point-LIO。

当前仓库中的 Point-LIO 仍然存在以下情况：

- Livox `CustomMsg` include 被注释
- Livox `CustomMsg` 订阅器被注释
- Livox/AVIA 预处理函数被注释

因此插件现在已经可以正确发布快照式 Livox `CustomMsg`，但还需要单独恢复
Point-LIO 的 Livox CustomMsg 订阅与预处理后，Point-LIO 才能直接使用该消息。

