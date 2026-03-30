from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node

def generate_launch_description():
  selected_odometry_topic = LaunchConfiguration('selected_odometry_topic', default='filtered_odometry_front')
  filtered_odometry_topic = LaunchConfiguration('filtered_odometry_topic', default='filtered_odometry')
  front_wheel_frame = LaunchConfiguration('front_wheel_frame', default='front_wheel_axis')
  odom_frame = LaunchConfiguration('odom_frame', default='odom')

  # Geo converter node for lat/lon to local
  front_axis_transform_node = Node(
    package='carla_vehicle',
    executable='front_axis_transform_node',
    name='front_axis_transform_node',
    output='screen',
    parameters=[
      # "Global" params for this node
      {'selected_odometry_topic': selected_odometry_topic},
      {'filtered_odometry_topic': filtered_odometry_topic},
      {'front_wheel_frame': front_wheel_frame},
      {'odom_frame': odom_frame}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
    front_axis_transform_node
  ])
