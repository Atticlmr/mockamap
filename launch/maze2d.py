import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('mockamap')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz.rviz')
    
    return LaunchDescription([
        # 地图生成节点 - 2D迷宫
        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[
                {'seed': 510},
                {'update_freq': 1.0},
                # box edge length, unit meter
                {'resolution': 0.1},
                # map size unit meter
                {'x_length': 20},
                {'y_length': 20},
                {'z_length': 2},
                # type 3: 2D maze
                {'type': 3},
                {'road_width': 0.5},
                {'add_wall_x': 0},
                {'add_wall_y': 0},
                # maze type: 1 recursive division maze
                {'maze_type': 1},
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
