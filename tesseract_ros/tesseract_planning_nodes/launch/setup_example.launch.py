import launch
import launch.actions
import launch_ros.actions

import os
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('tesseract_support'), 'urdf', 'abb_irb2400.urdf')
    srdf_path = os.path.join(get_package_share_directory('tesseract_support'), 'urdf', 'abb_irb2400.srdf')

    manager = launch_ros.actions.Node(
        node_name='manager_node',
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_planning_manager_node',
        output='screen',
    )

    worker1 = launch_ros.actions.Node(
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_planning_worker_node',
        output='screen',
    )

    worker2 = launch_ros.actions.Node(
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_planning_worker_node',
        output='screen',
    )

    toplevel = launch_ros.actions.Node(
        node_name='motion_node',
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_motion_manager_node',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'srdf_path': srdf_path,
                     }]
    )

    return launch.LaunchDescription([
        manager,
        worker1,
        worker2,
        toplevel,
    ])
