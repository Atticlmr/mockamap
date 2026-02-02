import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('mockamap')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz.rviz')
    
    return LaunchDescription([
        # 地图生成节点 - Perlin噪声3D
        Node(
            package='mockamap',
            executable='mockamap_node',
            output='screen',
            parameters=[
                {'seed': 511},
                {'update_freq': 1.0},
                # box edge length, unit meter
                {'resolution': 0.25},
                # map size unit meter
                {'x_length': 50},
                {'y_length': 50},
                {'z_length': 5},
                # 1 perlin noise 3D, 2 perlin box random map, 3 2d maze
                {'type': 1},
                # Perlin noise parameters
                # complexity: base noise frequency, large value will be complex (0.0 ~ 0.5)
                # fill: infill percentage (0.4 ~ 0.0)
                # fractal: large value will have more detail
                # attenuation: for fractal attenuation (0.0 ~ 0.5)
                {'complexity': 0.035},
                {'fill': 0.4},
                {'fractal': 1},
                {'attenuation': 0.1}
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
