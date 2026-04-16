from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from launch_ros.actions import Node
import os



def generate_launch_description():


      # --- 1. Gazebo simulation ---
    #gazebo_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(
    #            get_package_share_directory('bme_gazebo_sensors'),
    #            'launch',
    #            'spawn_robot_ex.launch.py'
    #        )
    #    )
    #) 

    ekf_config = os.path.join(
        get_package_share_directory('bme_gazebo_sensors'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )
    # 2. Container
    container = ComposableNodeContainer(
        name='assignment_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
    
            ComposableNode(
                package='assignment',
                plugin='assignment::NavServer',
                name='nav_server',
                extra_arguments=[{'use_intra_process_comms': False}],  # ← obbligatorio
            ),
            ComposableNode(
                package='assignment',
                plugin='assignment::NavClient',
                name='nav_client',
                extra_arguments=[{'use_intra_process_comms': False}],  # ← obbligatorio
            ),
        ],
    )

    # --- 3. Customer interface in a separate terminal ---
    interface_script = os.path.join(
        get_package_prefix('assignment'),
        'lib',
        'assignment',
        'nav_interface.py'
    )

    interface_terminal = ExecuteProcess(
        cmd=["xterm", "-hold", "-e", f"python3 {interface_script}"],
        output="screen"
    )

    return LaunchDescription([
        #gazebo_launch,
        ekf_node,
        container,
        interface_terminal
    ])
