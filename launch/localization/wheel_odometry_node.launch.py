from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node

def generate_launch_description():

  node_rate = LaunchConfiguration('node_rate')
  node_rate_arg = DeclareLaunchArgument(name='node_rate', default_value=TextSubstitution(text='50.0'),
    description='Node rate for this node')
  wheel_base = LaunchConfiguration('wheel_base')
  wheel_base_arg = DeclareLaunchArgument(name='wheel_base', default_value=TextSubstitution(text='2.84'),
    description='Vehicle wheel base')
  rot_covariance = LaunchConfiguration('rot_covariance')
  rot_covariance_arg = DeclareLaunchArgument(name='rot_covariance', default_value=TextSubstitution(text='0.05'),
    description='Covariance for twist')

  vehicle_control_topic = LaunchConfiguration('vehicle_control_topic', default='vehicle_control')
  wheel_odometry_topic = LaunchConfiguration('wheel_odometry_topic', default='wheel_odometry')
  wheel_distance_topic = LaunchConfiguration('wheel_distance_topic', default='wheel_distance')
  footprint_frame = LaunchConfiguration('footprint_frame', default='base_footprint')
  odom_frame = LaunchConfiguration('odom_frame', default='odom')

  # Wheel odometry for UKF
  ackermann_wheel_odometry_node = Node(
    package='carla_vehicle',
    executable='ackermann_wheel_odometry_node',
    name='ackermann_wheel_odometry_node',
    output='screen',
    parameters=[
      # 'Global' params for this node
      {'vehicle_control_topic': vehicle_control_topic},
      {'wheel_odometry_topic': wheel_odometry_topic},
      {'wheel_distance_topic': wheel_distance_topic},
      {'footprint_frame': footprint_frame},
      {'odom_frame': odom_frame},
      # Private params
      {'node_rate': node_rate},
      {'wheel_base': wheel_base},
      {'root_covariance': rot_covariance}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
    node_rate_arg,
    wheel_base_arg,
    rot_covariance_arg,
    ackermann_wheel_odometry_node
  ])
