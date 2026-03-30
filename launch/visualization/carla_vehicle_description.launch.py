from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
  LogInfo, RegisterEventHandler)
from launch.conditions import (IfCondition, UnlessCondition)
from launch.substitutions import (Command, LaunchConfiguration,
  TextSubstitution, LocalSubstitution, PathJoinSubstitution)
from launch.event_handlers import (OnProcessExit, OnShutdown)
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    model_path = PathJoinSubstitution([
      FindPackageShare('ego_vehicle_description'), 'urdf/ego_vehicle', 'ego_vehicle.xacro'
    ])
    rviz_config_path = PathJoinSubstitution([
      FindPackageShare('ego_vehicle_description'), 'rviz', 'ego_description.rviz'
    ])
    default_meshes_directory = PathJoinSubstitution([
      FindPackageShare('ego_vehicle_description'), 'meshes'
    ])

    output = LaunchConfiguration('output')
    output_arg = DeclareLaunchArgument(name='output', default_value=TextSubstitution(text='log'),
      description='Forces a custom output for nodes')
    joint_states = LaunchConfiguration('joint_states')
    joint_states_arg = DeclareLaunchArgument(name='joint_states', default_value='true',
      choices=['true', 'false'],
      description='Flag to enable joint_state_publisher_gui')
    vehicle = LaunchConfiguration('vehicle')
    vehicle_arg = DeclareLaunchArgument(name='vehicle', default_value=TextSubstitution(text='ego'),
      description='Name for the vehicle')
    prefix = LaunchConfiguration('prefix')
    prefix_arg = DeclareLaunchArgument(name='prefix', default_value=TextSubstitution(text='ego_'),
      description='Vehicle prefix')
    semantic_on = LaunchConfiguration('semantic_on')
    semantic_on_arg = DeclareLaunchArgument(name='semantic_on', default_value='true', choices=['true', 'false'],
      description='Flag to enable Semantic cameras')
    camera_on = LaunchConfiguration('camera_on')
    camera_on_arg = DeclareLaunchArgument(name='camera_on', default_value='true', choices=['true', 'false'],
      description='Flag to enable RGB cameras')
    lidar_on = LaunchConfiguration('lidar_on')
    lidar_on_arg = DeclareLaunchArgument(name='lidar_on', default_value='false', choices=['true', 'false'],
      description='Flag to enable lidars')
    lidars_setup = LaunchConfiguration('lidars_setup')
    lidars_setup_arg = DeclareLaunchArgument(name='lidars_setup', default_value='setup_1', choices=['setup_1', 'setup_2', 'setup_3'],
      description='Configuration for the lidars setup')
    model = LaunchConfiguration('model')
    model_arg = DeclareLaunchArgument(name='model', default_value=model_path,
      description='Absolute path to robot urdf file')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_arg = DeclareLaunchArgument(name='rviz_config', default_value=rviz_config_path,
                                     description='Absolute path to rviz config file')
    rviz_on = LaunchConfiguration('rviz_on')
    rviz_on_arg = DeclareLaunchArgument(name='rviz_on', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable rviz display')
    meshes_directory = LaunchConfiguration('meshes_directory')
    meshes_directory_arg = DeclareLaunchArgument(name='meshes_directory',
      default_value=default_meshes_directory,
      description='Directory to spawn vehicle meshes')
    vehicle_type = LaunchConfiguration('vehicle_type')
    vehicle_type_arg = DeclareLaunchArgument(name='vehicle_type',
      default_value=TextSubstitution(text='toyota'),
      description='toyota, tesla or mercedes')
    
    robot_description = ParameterValue(Command(['xacro ', model,
                                       ' meshes_directory:=', meshes_directory,
                                       ' vehicle_type:=', vehicle_type,
                                       ' vehicle:=', vehicle,
                                       ' prefix:=', prefix,      
                                       ' semantic_on:=', semantic_on,  
                                       ' camera_on:=', camera_on,
                                       ' lidar_on:=', lidar_on,
                                       ' lidars_setup:=', lidars_setup]),
                                       value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output=output,
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output=output,
        condition=IfCondition(joint_states)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz_on),
        name='rviz2',
        output=output,
        arguments=['-d', rviz_config]
    )

    rviz_event_handler = RegisterEventHandler(
      OnProcessExit(
        target_action=rviz_node,
        on_exit=[
          LogInfo(msg="Rviz node required!"),
            EmitEvent(event=Shutdown(
              reason='Rviz window closed'))
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
        output_arg,
        meshes_directory_arg,
        vehicle_type_arg,
        joint_states_arg,
        vehicle_arg,
        prefix_arg,
        semantic_on_arg,
        camera_on_arg,
        lidar_on_arg,
        lidars_setup_arg,
        model_arg,
        rviz_on_arg,
        rviz_config_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        rviz_event_handler,
        handler_on_shutdown
    ])
