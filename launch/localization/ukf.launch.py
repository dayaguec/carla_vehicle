from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import (LaunchConfiguration, TextSubstitution, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  ukf_config = LaunchConfiguration('ukf_config')
  ukf_config_arg = DeclareLaunchArgument(name='ukf_config', default_value='ukf_ego.yaml',
    description='Configuration file for the UKF')

  filtered_odometry_topic = LaunchConfiguration('filtered_odometry_topic', default='filtered_odometry')

  # UKF Node
  ukf_node = Node(
    package='robot_localization',
    executable='ukf_node',
    name='ukf_filter_node',
    output='screen',
    remappings=[
      ('odometry/filtered', filtered_odometry_topic)
    ],
    parameters=[
      PathJoinSubstitution([ukf_config])
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
    ukf_config_arg,
    ukf_node
  ])
