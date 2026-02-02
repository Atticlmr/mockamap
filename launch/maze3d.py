import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('mockamap')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz.rviz')
    
    return LaunchDescription([
        # 地图生成节点 - 3D迷宫
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
                {'x_length': 20},
                {'y_length': 20},
                {'z_length': 20},
                # type 4: 3D maze
                {'type': 4},
                {'numNodes': 64},
                {'connectivity': 0.5},
                {'roadRad': 4},
                {'nodeRad': 3},
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
