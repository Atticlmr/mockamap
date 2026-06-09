from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    mockamap_node = launch_ros.actions.Node(
        package='mockamap',
        executable='mockamap_node',
        output='screen',
        parameters=[
            {'seed': 511},
            {'update_freq': 1.0},
            {'resolution': 0.1},
            {'x_length': 25},
            {'y_length': 10},
            {'z_length': 3},
            {'type': 5},
            {'cylinder_number': 70},
            {'cylinder_radius_min': 0.5},
            {'cylinder_radius_max': 0.5},
            {'cylinder_height_min': 2.0},
            {'cylinder_height_max': 3.0},
            {'min_distance': 1.4},
            {'ring_number': 0},
            {'ring_radius_min': 0.7},
            {'ring_radius_max': 1.2},
            {'ring_z_min': 0.7},
            {'ring_z_max': 0.8},
            {'ring_max_yaw': 0.5},
        ])

    return LaunchDescription([mockamap_node])
