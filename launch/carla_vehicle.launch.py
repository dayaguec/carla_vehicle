from ament_index_python.packages import get_package_share_path

from launch_ros.actions import (PushRosNamespace, Node)
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
  GroupAction, TimerAction, Shutdown, OpaqueFunction, EmitEvent)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
  TextSubstitution)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.events import Shutdown as ShutdownEvent

import yaml
import os
import collada

def launch_setup(context, *args, **kwargs):

  prefix = LaunchConfiguration('prefix')
  vehicle = LaunchConfiguration('vehicle')
  spawn_point = LaunchConfiguration('spawn_point')
  vehicle_color = LaunchConfiguration('vehicle_color')
  rviz_on = LaunchConfiguration('rviz_on')
  vehicle_type = LaunchConfiguration('vehicle_type')
  meshes_directory = LaunchConfiguration('meshes_directory')
  vehicle_namespace = vehicle.perform(context)

  # Read config file and replace vehicle prefix with the one required in this launch file
  global_params_yaml = get_package_share_path('carla_vehicle') / 'config/global_params.yaml'
  params_file = open(global_params_yaml,'r')
  original_params = params_file.read()
  params_with_prefix = original_params.replace('ego', vehicle_namespace)
  # Parse global yaml config file
  global_params = yaml.safe_load(params_with_prefix)

  ############################################################
  ########################### CARLA ##########################
  ############################################################

  # Read sensor config file and replace vehicle prefix with the one required in this launch file
  sensor_params_yaml = get_package_share_path('carla_vehicle') / 'config/sensor_params.yaml'
  sensors_params_file = open(sensor_params_yaml, "r")
  sensors_original_params = sensors_params_file.read()
  sensors_params_with_prefix = sensors_original_params.replace('ego', vehicle_namespace)
  sensors_global_params = yaml.safe_load(sensors_params_with_prefix)
  # Write tmp sensor param file with current namespace
  tmp_yaml_path = '/tmp/' + vehicle_namespace + '_sensor_params.yaml'
  with open(tmp_yaml_path, 'w') as file:
    yaml.dump(sensors_global_params, file)

  # Read meshes description config file and replace vehicle prefix with the one required in this launch file
  type_v = vehicle_type.perform(context).split('.')[0]
  class_type = "_" + type_v
  r,g,b = vehicle_color.perform(context).split(',')

  path = os.path.join('/tmp', 'Meshes/')
  os.makedirs(path, exist_ok=True) 

  try:
    fairing_mesh = collada.Collada(meshes_directory.perform(context) + '/ego' + class_type + ".dae",
      ignore=[collada.common.DaeUnsupportedError, collada.common.DaeBrokenRefError])
    wheel_mesh = collada.Collada(meshes_directory.perform(context) + '/ego' + class_type + "_wheel.dae",
      ignore=[collada.common.DaeUnsupportedError, collada.common.DaeBrokenRefError])  
    for effect in fairing_mesh.effects:
      if(effect.id == "Fairing-effect"):
        effect.diffuse = (int(r)/256, int(g)/256, int(b)/256, 1)
    fairing_mesh.write(path + '/' + vehicle_namespace + class_type + '.dae')
    wheel_mesh.write(path + '/' + vehicle_namespace + class_type + "_wheel.dae")
  except Exception as e:
    print(f"[ERROR] Exception caught when parsing meshes: {e}")
    return [EmitEvent(event=ShutdownEvent(reason=f"Exception caught: {e}"))]

  # Vehicle description with Carla ROS bridge and Ackermann controller for vehicle
  carla_vehicle_launch = GroupAction(
    actions = [
      PushRosNamespace(vehicle),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_vehicle'), 'launch', 'vehicle_node.launch.py'
            ])
          ]),
          launch_arguments = {
            # Launch parameters
            'vehicle' : vehicle,
            'prefix' : prefix,
            'semantic_on': 'false',
            'v2x_on': 'false',
            'camera_on' : 'true',
            'lidar_on' : 'false',
            'lidars_setup': 'setup_1',
            'joint_states' : 'false',
            'rviz_on': 'false',
            'vehicle_type' : vehicle_type.perform(context),
            'role_name': 'hero',
            'carla_world': global_params.get('hd_map', 'Town10HD_Opt'),
            'sync' : '--sync', # '--sync' or ''
            'target_fps' : str(global_params.get('target_fps', 60.0)),
            'standalone' : '--standalone', # '--standalone' or ''
            'render' : '--render', # '--no-render' or '--render'
            'enable_tm' : '', # '--enable-tm' or ''
            'image_transport_on' : 'false',
            'spawn_point' : spawn_point,
            'vehicle_color' : vehicle_color,
            'teleop_on' : 'false',
            'sensor_params_path' : tmp_yaml_path,
            'meshes_directory' : path,
            'meshes_type' : type_v,
            # Global parameters from yaml
            'vehicle_cmd_topic' : global_params.get('vehicle_cmd_topic', 'vehicle_cmd'),
            'vehicle_control_topic' : global_params.get('vehicle_control_topic', 'vehicle_control'),
            'carla_info_topic' : global_params.get('carla_info_topic', 'carla_info'),
            'carla_actor_id_topic' : global_params.get('carla_actor_id_topic', 'carla_actor_id'),
            'quit_simulation_topic' : global_params.get('quit_simulation_topic', 'quit_simulation'),
            'perception_event_topic' : global_params.get('perception_event_topic', 'perception_event'),
            'footprint_frame' : global_params.get('footprint_frame', vehicle_namespace + '/base_footprint'),
            'map_frame': global_params.get('map_frame', 'map'),
            # This is passed as a whole string to avoid ROS2 concatenations
            'wheel_joint_names' : str(global_params.get('wheel_joint_names', 'wheel_joint_names')),
            'ackermann_cmd_topic' : global_params.get('ackermann_cmd_topic', 'ackermann_cmd'),
            'ackermann_joy_topic' : global_params.get('ackermann_joy_topic', 'ackermann_joy')
          }.items()
      )
    ]
  )

  ############################################################
  ####################### LOCALIZATION #######################
  ############################################################
  
  # Wheel odometry for ackermann steering geometry
  wheel_odometry_launch = GroupAction(
    actions = [
      PushRosNamespace(vehicle),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_vehicle'), 'launch/localization', 'wheel_odometry_node.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'vehicle_control_topic' : global_params.get('vehicle_control_topic', 'vehicle_control'),
            'wheel_odometry_topic' : global_params.get('wheel_odometry_topic', 'wheel_odometry'),
            'wheel_distance_topic' : global_params.get('wheel_distance_topic', 'wheel_distance'),
            'footprint_frame' : global_params.get('footprint_frame', vehicle_namespace + '/base_footprint'),
            'odom_frame' : global_params.get('odom_frame', vehicle_namespace + '/odom'),
            # Private params
            'node_rate': '30.0',
            'wheel_base' : str(2 * global_params.get('vehicle_lf', 1.42)),
            'rot_covariance' : '0.05'
          }.items()
      )
    ]
  )

  # Geo converter node to convert between Fix and ENU
  geo_converter_launch = GroupAction(
    actions = [
      PushRosNamespace(vehicle),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_vehicle'), 'launch/localization', 'geo_converter_node.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'gps_topic' : global_params.get('gps_topic', 'gps/fix'),
            'gps_local_odometry_topic' : global_params.get('gps_local_odometry_topic', 'gps_local_odometry'),
            'footprint_frame' : global_params.get('footprint_frame', vehicle_namespace + '/base_footprint'),
            'odom_frame' : global_params.get('odom_frame', vehicle_namespace + '/odom'),
            # Private params
            # If Carla, remember to change in /CarlaUE4/Content/Carla/Maps/OpenDrive the starting GPS location -->
            'geo_projection' : global_params.get('geo_projection',
              '+proj=tmerc +lat_0=40.354550084445 +lon_0=-3.7463664011244586 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs'),
            'first_altitude' : '0.0',
            'rot_covariance' : '99999.9'
          }.items()
      )
    ]
  )

  # Read sensor config file and replace vehicle prefix with the one required in this launch file
  ukf_params_yaml = get_package_share_path('carla_vehicle') / 'config/localization/ukf_vehicle.yaml'
  ukf_params_file = open(ukf_params_yaml, "r")
  ukf_original_params = ukf_params_file.read()
  ukf_params_with_prefix = ukf_original_params.replace('ego', vehicle_namespace)
  ukf_global_params = yaml.safe_load(ukf_params_with_prefix)
  # Write tmp sensor param file with current namespace
  ukf_tmp_yaml_path = '/tmp/' + 'ukf_' + vehicle_namespace + '.yaml'
  with open(ukf_tmp_yaml_path, 'w') as file:
    yaml.dump(ukf_global_params, file)

  # UKF
  ukf_launch = GroupAction(
    actions = [
      PushRosNamespace(vehicle),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_vehicle'), 'launch/localization', 'ukf.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'ukf_config' : ukf_tmp_yaml_path,
            'filtered_odometry_topic' : global_params.get('filtered_odometry_topic', 'filtered_odometry')
          }.items()
      )
    ]
  )

  # Transform filtered odometry from base_footprint to front wheel ackermann axis
  front_axis_transform_launch = GroupAction(
    actions = [
      PushRosNamespace(vehicle),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_vehicle'), 'launch/localization', 'front_axis_transform_node.launch.py'
            ])
          ]),
          launch_arguments = {
            # Global params
            'filtered_odometry_topic': global_params.get('filtered_odometry_topic', 'filtered_odometry'),
            'selected_odometry_topic': global_params.get('selected_odometry_topic', 'filtered_odometry_front'),
            'front_wheel_frame': global_params.get('front_wheel_frame', vehicle_namespace + '/front_wheel_axis'),
            'odom_frame' : global_params.get('odom_frame', vehicle_namespace + '/odom')
          }.items()
      )
    ]
  )

  map_to_odom_tf = TimerAction(
    period = 5.0,
    actions = [
      Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'map', '--child-frame-id', vehicle_namespace + '/odom'],
        on_exit=Shutdown()
      )
    ]
  )

  ##################################################################
  ########################## VISUALIZATION #########################
  ##################################################################

  rviz_config_path = PathJoinSubstitution([
    FindPackageShare('carla_vehicle'), 'rviz', 'carla_vehicle.rviz'
  ])
  rviz_node = TimerAction(
    period = 2.0,
    actions = [
      Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        condition=IfCondition(rviz_on.perform(context)),
        arguments=['-d', rviz_config_path]
      )
    ]
  )

  return [
    carla_vehicle_launch,
    geo_converter_launch,
    wheel_odometry_launch,
    ukf_launch,
    map_to_odom_tf,
    front_axis_transform_launch,
    rviz_node
  ]

def generate_launch_description():

  vehicle_arg = DeclareLaunchArgument(name='vehicle', default_value=TextSubstitution(text='hero'),
    description='Vehicle name for namespace')
  prefix_arg = DeclareLaunchArgument(name='prefix', default_value=TextSubstitution(text='hero_'),
    description='Prefix of vehicle for tfs and link')
  spawn_point_arg = DeclareLaunchArgument(name='spawn_point',
    default_value=TextSubstitution(text='-64.1225 -27.9658 0.0 0.0 0.0 0.0'),
    description='World coordinates (x y z roll pitch yaw) to spawn vehicle,\
                 location in meters, orientation in radians as ROS standard')
  # Toyota: 116,0,3 | Mercedes: 0,62,150 | Tesla: 46,46,46
  vehicle_color_arg = DeclareLaunchArgument(name='vehicle_color',
    default_value=TextSubstitution(text='23,51,236'),
    description='Color scheme for the vehicle spawned (r,g,b)')
  rviz_on_arg = DeclareLaunchArgument(name='rviz_on', default_value='true', choices=['true', 'false'],
    description='Wether to display description Rviz or not')
  # toyota.prius | tesla.model3 | mercedes.coupe
  vehicle_type_arg = DeclareLaunchArgument(name='vehicle_type',
    default_value=TextSubstitution(text='toyota.prius'),
    description='Vehicle type to Spawn')
  default_meshes_directory = PathJoinSubstitution([
      FindPackageShare('ego_vehicle_description'), 'meshes'
  ])
  meshes_directory_arg = DeclareLaunchArgument(name='meshes_directory',
    default_value=default_meshes_directory,
    description='Where to find the vehicle mesh to spawn in rviz')

  return LaunchDescription([
    vehicle_arg,
    prefix_arg,
    spawn_point_arg,
    vehicle_color_arg,
    vehicle_type_arg,
    meshes_directory_arg,
    rviz_on_arg,
    OpaqueFunction(function=launch_setup)
  ])
