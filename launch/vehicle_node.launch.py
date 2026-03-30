from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
  GroupAction, LogInfo, RegisterEventHandler, EmitEvent, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
  TextSubstitution, LocalSubstitution)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import (OnProcessExit, OnShutdown)
from launch.events import Shutdown
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_path

import yaml

vehicle_namespace = "hero"

def namespace_to_str(context, conf):
  vehicle_namespace = conf.perform(context)

def generate_launch_description():
  # Arguments for this launch file
  prefix = LaunchConfiguration('prefix')
  prefix_arg = DeclareLaunchArgument(name='prefix', default_value=TextSubstitution(text='hero_'),
    description='Prefix of vehicle for tfs and link')
  vehicle = LaunchConfiguration('vehicle')
  vehicle_arg = DeclareLaunchArgument(name='vehicle', default_value=TextSubstitution(text='hero'),
    description='Vehicle name for namespace')
  output = LaunchConfiguration('output')
  output_arg = DeclareLaunchArgument(name='output', default_value=TextSubstitution(text='log'),
    description='Forces a custom output for nodes')
  vehicle_type = LaunchConfiguration('vehicle_type')
  vehicle_type_arg = DeclareLaunchArgument(name='vehicle_type', default_value=TextSubstitution(text='toyota.prius'),
    description='Type of Carla vehicle. Available options: random, tesla.model3, toyota.prius, mercedes.coupe')
  rolename = LaunchConfiguration('role_name')
  role_name_arg = DeclareLaunchArgument(name='role_name', default_value=TextSubstitution(text='hero'),
    description='Role name of the vehicle inside Carla simulation')
  host = LaunchConfiguration('host')
  host_arg = DeclareLaunchArgument(name='host', default_value=TextSubstitution(text='localhost'),
    description='Network location of the machine where Carla server is running')
  port = LaunchConfiguration('port')
  port_arg = DeclareLaunchArgument(name='port', default_value=TextSubstitution(text='2000'),
    description='Port for the communication with Carla server')
  timeout = LaunchConfiguration('timeout') 
  timeout_arg = DeclareLaunchArgument(name='timeout', default_value=TextSubstitution(text='20.0'),
    description='Time to wait for Carla server response to the client')
  target_fps = LaunchConfiguration('target_fps') 
  target_fps_arg = DeclareLaunchArgument(name='target_fps', default_value=TextSubstitution(text='60.0'),
    description='Target FPS for the simulation')
  carla_world = LaunchConfiguration('carla_world') 
  carla_world_arg = DeclareLaunchArgument(name='carla_world', default_value=TextSubstitution(text='Town10_Opt'),
    description='Carla world to generate, available worlds: Town01, Town02, Town03, Town04, Town05, Town06 \
    Town07, Town10. Use "_Opt" for Layered version')
  rviz_on = LaunchConfiguration('rviz_on')
  rviz_on_arg = DeclareLaunchArgument(name='rviz_on', default_value='false', choices=['true', 'false'],
    description='Wether to display description Rviz or not')
  sync = LaunchConfiguration('sync')
  sync_arg = DeclareLaunchArgument(name='sync', default_value='', choices=['--sync', ''],
    description='Run Simulation on sync mode')
  standalone = LaunchConfiguration('standalone')
  standalone_arg = DeclareLaunchArgument(name='standalone', default_value='', choices=['--standalone', ''],
    description='Run Simulation standalone')
  render = LaunchConfiguration('render')
  render_arg = DeclareLaunchArgument(name='render', default_value='--no-render', choices=['--render', '--no-render'],
    description='Render graphics on Carla server side, this is overrided if a GPU sensor is spawned')
  image_transport_on = LaunchConfiguration('image_transport_on') 
  image_transport_on_arg = DeclareLaunchArgument(name='image_transport_on', default_value='true',
    choices=['true', 'false'],
    description='Enable Image Transport for RGB Images')
  spawn_point = LaunchConfiguration('spawn_point')
  spawn_point_arg = DeclareLaunchArgument(name='spawn_point',
    default_value=TextSubstitution(text='random'),
    description='World coordinates to spawn vehicle, location in meters, orientation in degrees as Carla standard')
  teleop_on = LaunchConfiguration('teleop_on')
  teleop_arg =  DeclareLaunchArgument(name='teleop_on', default_value='false', choices=['true', 'false'],
    description='Wether to launch teleop control or not')
  semantic_on = LaunchConfiguration('semantic_on')
  semantic_on_arg = DeclareLaunchArgument(name='semantic_on', default_value='true', choices=['true', 'false'],
    description='Flag to enable Semantic cameras')
  v2x_on = LaunchConfiguration('v2x_on')
  v2x_on_arg = DeclareLaunchArgument(name='v2x_on', default_value='true', choices=['true', 'false'],
    description='Flag to enable OBU sensors for V2X')
  camera_on = LaunchConfiguration('camera_on')
  camera_on_arg = DeclareLaunchArgument(name='camera_on', default_value='true', choices=['true', 'false'],
    description='Flag to enable RGB cameras')
  lidar_on = LaunchConfiguration('lidar_on')
  lidar_on_arg = DeclareLaunchArgument(name='lidar_on', default_value='false', choices=['true', 'false'],
    description='Flag to enable lidars')
  lidars_setup = LaunchConfiguration('lidars_setup')
  lidars_setup_arg = DeclareLaunchArgument(name='lidars_setup', default_value='setup_1', choices=['setup_1', 'setup_2', 'setup_3'],
    description='Configuration for the lidars setup')
  vehicle_color = LaunchConfiguration('vehicle_color')
  vehicle_color_arg = DeclareLaunchArgument(name='vehicle_color',
    default_value=TextSubstitution(text='116,0,30'),
    description='Color scheme for the vehicle spawned')
  enable_tm = LaunchConfiguration('enable_tm')
  enable_tm_arg = DeclareLaunchArgument(name='enable_tm', default_value='', choices=['--enable-tm', ''],
    description='Connect ROS Node with Traffic Manager')
  default_meshes_directory = PathJoinSubstitution([
      FindPackageShare('ego_vehicle_description'), 'meshes'
  ])
  meshes_directory = LaunchConfiguration('meshes_directory')
  meshes_directory_arg = DeclareLaunchArgument(name='meshes_directory',
      default_value=default_meshes_directory,
      description='Directory to spawn vehicle meshes')
  meshes_type = LaunchConfiguration('meshes_type')
  meshes_type_arg = DeclareLaunchArgument(name='meshes_type',
      default_value='toyota',
      description='Type of vehicle to extract mesh name')

  description_launch = GroupAction(
    actions = [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_vehicle'), 'launch/visualization', 'carla_vehicle_description.launch.py'
            ])
          ]),
          launch_arguments = {
            'rviz_on': rviz_on,
            'gui' : 'false',
            'vehicle' : vehicle,
            'prefix' : prefix,
            'semantic_on' : semantic_on,
            'v2x_on' : v2x_on,
            'camera_on' : camera_on,
            'lidar_on' : lidar_on,
            'lidars_setup' : lidars_setup,
            'meshes_directory' : meshes_directory,
            'vehicle_type' : meshes_type,
            'joint_states': 'false'
          }.items()
      )
    ]
  )

  # Global parameters from top level launch
  vehicle_cmd_topic = LaunchConfiguration('vehicle_cmd_topic', default='vehicle_cmd')
  vehicle_control_topic = LaunchConfiguration('vehicle_control_topic', default='vehicle_control')
  carla_info_topic = LaunchConfiguration('carla_info_topic', default='carla_info')
  carla_actor_id_topic = LaunchConfiguration('carla_actor_id_topic', default='carla_actor_id')
  quit_simulation_topic = LaunchConfiguration('quit_simulation_topic', default='quit_simulation')
  perception_event_topic = LaunchConfiguration('perception_event_topic', default='perception_event')
  footprint_frame = LaunchConfiguration('footprint_frame', default='base_footprint')
  map_frame = LaunchConfiguration('map_frame', default='world')
  wheel_joint_names = LaunchConfiguration('wheel_joint_names', default='wheel_joint_names')
  ackermann_cmd_topic = LaunchConfiguration('ackermann_cmd_topic', default='ackermann_cmd')
  ackermann_joy_topic = LaunchConfiguration('ackermann_joy_topic', default='ackermann_joy')
  sensor_params_path = LaunchConfiguration('sensor_params_path',
    default=(get_package_share_path('carla_vehicle') / 'config/sensor_params.yaml'))

  # CARLA ROS Bridge
  vehicle_node = Node(
    package='carla_vehicle',
    executable='vehicle_node.py',
    name='vehicle_node',
    output='screen',
    arguments=['--spawn', spawn_point, '--host', host, '--port', port,
      '--timeout', timeout, '--rolename', rolename, '--vehicle', vehicle_type,
      '--vehicle_color', vehicle_color, '--world', carla_world, '--fps', target_fps,
      sync, standalone, render, enable_tm],
    parameters=[
      {'vehicle_cmd_topic': vehicle_cmd_topic},
      {'vehicle_control_topic': vehicle_control_topic},
      {'carla_info_topic': carla_info_topic},
      {'carla_actor_id_topic': carla_actor_id_topic},
      {'quit_simulation_topic': quit_simulation_topic},
      {'perception_event_topic': perception_event_topic},
      {'footprint_frame': footprint_frame},
      {'map_frame': map_frame},
      {'wheel_joint_names': wheel_joint_names},
      {'sensor_params': sensor_params_path},
      {'cmd_timeout': 10.0}
    ]
  )

  # ROS Akermann commands conversions to CARLA throttle/brake
  carla_ackermann_node = Node(
    package='carla_vehicle',
    executable='carla_ackermann_node.py',
    name='carla_ackermann_node',
    output='screen',
    parameters=[
      # Private params
      PathJoinSubstitution([
        FindPackageShare('carla_vehicle'), 'config/control', 'carla_ackermann_params.yaml'
      ]),
      # 'Global' params for this node
      {'vehicle_control_topic': vehicle_control_topic},
      {'ackermann_cmd_topic': ackermann_cmd_topic},
      {'vehicle_cmd_topic': vehicle_cmd_topic}
    ]
  )

  # ROS Akermann commands front teleop node
  carla_keyboard_control_node = Node(
    package='carla_vehicle',
    executable='carla_keyboard_control_node.py',
    name='carla_keyboard_control_node',
    output='screen',
    prefix=['gnome-terminal --'],
    condition=IfCondition(teleop_on),
    parameters=[
      # Private params
      PathJoinSubstitution([
        FindPackageShare('carla_vehicle'), 'config/control', 'carla_vehicle_teleop.yaml'
      ]),
      # 'Global' params for this node
      {'ackermann_joy_topic': ackermann_joy_topic}
    ]
  )

  # Image transport for RGB cameras compression
  image_transport_node = Node(
    package='carla_vehicle',
    executable='image_transport_node',
    name='image_transport_node',
    output='screen',
    condition=IfCondition(image_transport_on),
    parameters=[
      # Private params
      {'sensor_params': sensor_params_path},
      {'in_transport': "raw"}
    ]
  )

  # Handlers for shutdown
  vehicle_node_handler = RegisterEventHandler(
    OnProcessExit(
      target_action=vehicle_node,
        on_exit=[
          LogInfo(msg='Carla Vehicle ROS node is required!'),
            EmitEvent(event=Shutdown(
              reason='Carla Vehicle ROS node exited'))
      ]
    )
  )

  handler_on_shutdown = RegisterEventHandler(
    OnShutdown(
      on_shutdown=[LogInfo(
          msg=['Launch was asked to shutdown: ',
            LocalSubstitution('event.reason')]
      )]
    )
  )

  return LaunchDescription([
    prefix_arg,
    vehicle_arg,
    semantic_on_arg,
    v2x_on_arg,
    camera_on_arg,
    lidar_on_arg,
    lidars_setup_arg,
    output_arg,
    vehicle_type_arg,
    role_name_arg,
    host_arg,
    port_arg,
    timeout_arg,
    carla_world_arg,
    rviz_on_arg,
    sync_arg,
    target_fps_arg,
    standalone_arg,
    render_arg,
    spawn_point_arg,
    vehicle_color_arg,
    enable_tm_arg,
    teleop_arg,
    image_transport_on_arg,
    meshes_directory_arg,
    meshes_type_arg,
    description_launch,
    vehicle_node,
    carla_ackermann_node,
    carla_keyboard_control_node,
    image_transport_node,
    vehicle_node_handler,
    handler_on_shutdown
  ])
