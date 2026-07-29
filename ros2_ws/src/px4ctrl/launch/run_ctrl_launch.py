# launch文件所必要的东西
from launch import LaunchDescription
from launch_ros.actions import Node

# 找到share包的位置，便于读取config文件
from launch_ros.substitutions import FindPackageShare

# 后面在终端中对Launch进行操作的时候会用上
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 载入一下系统
import os

def generate_launch_description():
    # 获取包的路径
    pkg_share = FindPackageShare('px4ctrl').find('px4ctrl')

    # 获取config文件路径
    config_path = os.path.join(pkg_share, 'config', 'ctrl_param_fpv.yaml')

    # 创建节点
    px4ctrl_node = Node(
        package='px4ctrl',
        executable='px4ctrl_node',
        name='px4ctrl_node',
        output='screen',
        parameters=[config_path],
        remappings=[
                    ('odom', '/imu_propagate'),
                    ('cmd', '/position_cmd')]
    )

    # 向launch文件返回一下对应的东西
    return LaunchDescription([px4ctrl_node])