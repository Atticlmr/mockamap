import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('mockamap')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz.rviz')
    
    return LaunchDescription([
        # 地图生成节点 - 随机障碍物
        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[
                {'seed': 511},
                {'update_freq': 1.0},
                # box edge length, unit meter
                {'resolution': 0.1},
                # map size unit meter
                {'x_length': 10},
                {'y_length': 10},
                {'z_length': 4},
                # type 2: random obstacles
                {'type': 2},
                {'width_min': 0.6},
                {'width_max': 1.5},
                {'obstacle_number': 50},
            ]
        ),
        # RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
