import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    num_vehicles = int(context.launch_configurations['num_vehicles'])
    config_file = context.launch_configurations['config_file']
    launch_metrics = context.launch_configurations.get('launch_metrics', 'true') == 'true'
    
    # Load config file parameters dynamically to bypass namespace mismatch issues
    import yaml
    config_data = {}
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f) or {}
        except Exception as e:
            print(f"Warning: Failed to parse config file {config_file}: {e}")

    def get_params(node_name):
        return config_data.get(node_name, {}).get('ros__parameters', {})

    thermal_params = get_params('thermal_field_node')
    event_params = get_params('ground_event_node')
    metrics_params = get_params('metrics_collector_node')
    fov_params = get_params('fov_detection_node')
    batt_params = get_params('battery_estimator_node')
    fsm_params = get_params('fsm_node')

    # Global environment nodes
    nodes = [
        Node(
            package='soarer_env',
            executable='thermal_field_node',
            name='thermal_field_node',
            output='screen',
            parameters=[thermal_params, {'num_vehicles': num_vehicles}]
        ),
        Node(
            package='soarer_env',
            executable='ground_event_node',
            name='ground_event_node',
            output='screen',
            parameters=[event_params]
        )
    ]
    
    if launch_metrics:
        nodes.append(
            Node(
                package='soarer_env',
                executable='metrics_collector_node',
                name='metrics_collector_node',
                output='screen',
                parameters=[metrics_params, {'num_vehicles': num_vehicles}]
            )
        )
    
    # Per-UAV sensing nodes
    for i in range(1, num_vehicles + 1):
        nodes.append(
            Node(
                package='soarer_env',
                executable='fov_detection_node',
                name='fov_detection_node',
                namespace=f'px4_{i}',
                output='screen',
                parameters=[fov_params, {'vehicle_id': i}]
            )
        )
        nodes.append(
            Node(
                package='soarer_env',
                executable='battery_estimator_node',
                name='battery_estimator_node',
                namespace=f'px4_{i}',
                output='screen',
                parameters=[batt_params, {'vehicle_id': i}]
            )
        )
        nodes.append(
            Node(
                package='soarer_env',
                executable='fsm_node',
                name='fsm_node',
                namespace=f'px4_{i}',
                output='screen',
                parameters=[fsm_params, {'vehicle_id': i}]
            )
        )
        
    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory('soarer_env')
    default_config = os.path.join(pkg_share, 'config', 'config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_vehicles',
            default_value='2',
            description='Number of vehicles in the swarm'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to YAML config file'
        ),
        DeclareLaunchArgument(
            'launch_metrics',
            default_value='true',
            description='Whether to launch the metrics collector node'
        ),
        OpaqueFunction(function=launch_setup)
    ])
