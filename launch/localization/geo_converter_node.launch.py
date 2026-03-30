from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import (LaunchConfiguration, TextSubstitution)
from launch_ros.actions import Node

def generate_launch_description():

  geo_projection = LaunchConfiguration('geo_projection')
  geo_projection_arg = DeclareLaunchArgument(name='geo_projection', default_value=TextSubstitution(
    text='+proj=tmerc +lat_0=40.354550084445 +lon_0=-3.7463664011244586 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs'),
    description='Geo projection for nav_sat_fix conversions')
  first_altitude = LaunchConfiguration('first_altitude')
  first_altitude_arg = DeclareLaunchArgument(name='first_altitude', default_value=TextSubstitution(text='0.0'),
    description='Altitude at origin')
  rot_covariance = LaunchConfiguration('rot_covariance')
  rot_covariance_arg = DeclareLaunchArgument(name='rot_covariance', default_value=TextSubstitution(text='99999.0'),
    description='Covariance for twist')

  gps_topic = LaunchConfiguration('gps_topic', default='fix')
  gps_local_odometry_topic = LaunchConfiguration('gps_local_odometry_topic', default='gps_local_odometry')
  footprint_frame = LaunchConfiguration('footprint_frame', default='base_footprint')
  odom_frame = LaunchConfiguration('odom_frame', default='odom')

  # Geo converter node for lat/lon to local
  geo_converter_node = Node(
    package='carla_vehicle',
    executable='geo_converter_node',
    name='geo_converter_node',
    output='screen',
    parameters=[
      # "Global" params for this node
      {'gps_topic': gps_topic},
      {'gps_local_odometry_topic': gps_local_odometry_topic},
      {'footprint_frame': footprint_frame},
      {'odom_frame': odom_frame},
      # Private params
      {'geo_projection': geo_projection},
      {'first_altitude': first_altitude},
      {'rot_covariance': rot_covariance}
    ],
    on_exit=Shutdown()
  )

  return LaunchDescription([
    geo_projection_arg,
    first_altitude_arg,
    rot_covariance_arg,
    geo_converter_node
  ])
