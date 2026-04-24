import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler

import yaml
from yaml.loader import SafeLoader

import numpy as np
import carla
import datetime
import threading

from std_msgs.msg import (Empty, Header, String)
from geometry_msgs.msg import (Pose, Point, Quaternion)
from sensor_msgs.msg import (Imu, NavSatFix, JointState,
  CameraInfo, Image, PointCloud2)
from perception_interfaces.msg import PerceptionEvent
from carla_interfaces.msg import (CarlaVehicleControl, CarlaSimulationInfo, CarlaActorId)
from carla_interfaces.srv import (CarlaSpawnTraffic, CarlaCleanTraffic,
  CarlaChangeLayer, CarlaChangeWeather, CarlaResetVehicle)
from vehicle_interfaces.msg import VehicleControl

from carla_vehicle.localization.transform import (ros_transform_to_carla_transform,
  carla_rotation_to_RPY)
from carla_vehicle.utils import get_actor_display_name

class VehicleROSNode(Node):
  def __init__(self, world, is_sync_mode, standalone=True):
    """Initialize the ROS Node in charge of generating a bridge between Carla and ROS"""
    super().__init__('vehicle_ros_node')

    self._carla_traffic_manager = None
    self._carla_lock = threading.Lock()

    # Carla simulator world
    self._carla_world = world
    self._client_fps = .0
    self._world_tick_id = None
    self._is_sync_mode = is_sync_mode
    self._is_game_quit = False
    self._last_client_time = self.get_clock().now()

    # Veicle control inputs
    self._vehicle_control = carla.VehicleControl()
    self._lights = carla.VehicleLightState.NONE
    self._vehicle_control_manual_override = True
    self._carla_world.player.set_autopilot(False)
    self._carla_world.player.set_light_state(self._lights)

    # Declare this node params to use later
    self.declare_parameters(
      namespace='',
      parameters=[
        ('vehicle_cmd_topic', 'carla_vehicle_cmd'),
        ('joint_states_topic', 'joint_states'),
        ('vehicle_control_topic', 'vehicle_control'),
        ('carla_info_topic', 'carla_info'),
        ('carla_actor_id_topic', 'carla_actor_id'),
        ('quit_simulation_topic', 'quit_simulation'),
        ('perception_event_topic', 'peception_event'),
        ('footprint_frame', 'base_footprint'),
        ('map_frame', 'world'),
        ('sensor_params', ''),
        ('cmd_timeout', 10.0),
        ('wheel_joint_names', [''])
      ]
    )
    self._vehicle_base_footprint = \
      self.get_parameter('footprint_frame').get_parameter_value().string_value
    self._map_frame = \
      self.get_parameter('map_frame').get_parameter_value().string_value

    # Parse sensor yaml file for sensors configuration
    try:
      sensor_config_file_path = self.get_parameter('sensor_params').get_parameter_value().string_value
      self._sensors_data = None
      with open(sensor_config_file_path) as f:
        self._sensors_data = yaml.load(f, Loader=SafeLoader)
        self._sensors_data = self._sensors_data['sensors']
    except FileNotFoundError:
      self.get_logger().error(
        'Sensor config file provided: {} does not exist, exiting...'.format(sensor_config_file_path))
      self._is_game_quit = True

    # Parse frames, topics, types and params. If key sensor is not found in yalm,
    # it generates an empty vector
    # GPS
    try:
      gps_frames, self._gps_topics, gps_parameters = \
        [gps_item['frame_id'] for gps_item in self._sensors_data['gps']],\
        [gps_item['topic'] for gps_item in self._sensors_data['gps']],\
        [gps_item['params'] for gps_item in self._sensors_data['gps']]
    except KeyError:
      gps_frames, self._gps_topics, gps_parameters = [], [], []
      self.get_logger().warn(
        'GPS sensor configuration not found, GPS will not spawn; localization may not work properly!')

    # IMU
    try:
      imu_frames, self._imu_topics, imu_parameters = \
        [imu_item['frame_id'] for imu_item in self._sensors_data['imu']],\
        [imu_item['topic'] for imu_item in self._sensors_data['imu']],\
        [imu_item['params'] for imu_item in self._sensors_data['imu']]
    except KeyError:
      imu_frames, self._imu_topics, imu_parameters = [], [], []
      self.get_logger().warn(
        'IMU sensor configuration not found, IMU will not spawn; localization may not work properly!')

    # Radar
    try:
      radar_frames, self._radar_topics, radar_parameters = \
        [radar_item['frame_id'] for radar_item in self._sensors_data['radar']],\
        [radar_item['topic'] for radar_item in self._sensors_data['radar']], \
        [radar_item['params'] for radar_item in self._sensors_data['radar']]
    except KeyError:
      radar_frames, self._radar_topics, radar_parameters = [], [], []
      self.get_logger().warn(
        'Radar sensor configuration not found, Radar will not spawn!')

    # Lidars
    try:
      lidar_frames, self._lidar_topics, lidar_parameters = \
        [lidar_item['frame_id'] for lidar_item in self._sensors_data['lidar']],\
        [lidar_item['topic'] for lidar_item in self._sensors_data['lidar']],\
        [lidar_item['params'] for lidar_item in self._sensors_data['lidar']]
    except KeyError:
      lidar_frames, self._lidar_topics, lidar_parameters = [], [], []
      self.get_logger().warn(
        'Lidar sensor configuration not found, Lidar will not spawn!')

    # Semantic LiDARs
    try:
      sem_lidar_frames, self._sem_lidar_topics, sem_lidar_parameters = \
        [sem_lidar_item["frame_id"] for sem_lidar_item in self._sensors_data["semantic_lidar"]],\
        [sem_lidar_item["topic"] for sem_lidar_item in self._sensors_data["semantic_lidar"]],\
        [sem_lidar_item["params"] for sem_lidar_item in self._sensors_data["semantic_lidar"]]
    except KeyError:
      sem_lidar_frames, self._sem_lidar_topics, sem_lidar_parameters = [], [], []
      self.get_logger().warn(
        "Semantic LiDAR sensor configuration not found, Semantic LiDAR will not spawn!")

    # RGBCamera
    try:
      rgb_camera_frames, self._rgb_camera_topics, rgb_camera_parameters = \
        [rgb_camera_item['frame_id'] for rgb_camera_item in self._sensors_data['rgb_camera']],\
        [rgb_camera_item['topic'] for rgb_camera_item in self._sensors_data['rgb_camera']],\
        [rgb_camera_item['params'] for rgb_camera_item in self._sensors_data['rgb_camera']]
    except KeyError:
      rgb_camera_frames, self._rgb_camera_topics, rgb_camera_parameters = [], [], []
      self.get_logger().warn(
        'RBG Camera sensor configuration not found, RBG Camera will not spawn!')

    # Semantic camera
    try:
      sem_camera_frames, self._sem_camera_topics, sem_camera_parameters = \
        [sem_camera_item["frame_id"] for sem_camera_item in self._sensors_data["semantic_camera"]],\
        [sem_camera_item["topic"] for sem_camera_item in self._sensors_data["semantic_camera"]],\
        [sem_camera_item["params"] for sem_camera_item in self._sensors_data["semantic_camera"]]
    except KeyError:
      sem_camera_frames, self._sem_camera_topics, sem_camera_parameters = [], [], []
      self.get_logger().warn("Semantic camera sensor configuration not found, Semantic Camera will not spawn!")

    # V2X Sensor
    try:
      obu_frames, self._obu_topics, obu_parameters = \
        [sem_camera_item["frame_id"] for sem_camera_item in self._sensors_data["obu"]],\
        [sem_camera_item["topic"] for sem_camera_item in self._sensors_data["obu"]],\
        [sem_camera_item["params"] for sem_camera_item in self._sensors_data["obu"]]
    except KeyError:
      obu_frames, self._obu_topics, obu_parameters = [], [], []
      self.get_logger().warn("OBU configuration not found, OBU will not spawn!")

    # @todo: change to StaticTransformListener once implemented (in humble)
    tf_buffer = Buffer()
    tf_listener = TransformListener(
      buffer=tf_buffer, node=self, spin_thread=True)

    self._gps_tf, self._imu_tf, self._radar_tf, self._lidar_tf, self._sem_lidar_tf, self._rgb_camera_tf, self._sem_camera_tf, self._obu_tf = \
      [], [], [], [], [], [], [], []
    self._gps_msgs, self._imu_msgs, self._radar_msgs, self._lidar_msgs, self._sem_lidar_msgs, self._rgb_camera_msgs, self._sem_camera_msgs, self._obu_msgs = \
      [], [], [], [], [], [], [], []

    # Read all tfs to proper transform to Carla world
    for trf in gps_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._gps_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        self._gps_msgs.append(NavSatFix(header=Header(frame_id=trf)))
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._gps_tf.append(None)

    for trf in imu_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._imu_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        self._imu_msgs.append(Imu(header=Header(frame_id=trf)))
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._imu_tf.append(None)

    for trf in radar_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._radar_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        # TODO: Change to radar msgs when using Radar, now is PointCloud2...
        self._radar_msgs.append(PointCloud2(header=Header(frame_id=trf)))
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._radar_tf.append(None)

    for trf in lidar_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._lidar_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        self._lidar_msgs.append(PointCloud2(header=Header(frame_id=trf)))
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._lidar_tf.append(None)

    for trf in sem_lidar_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._sem_lidar_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        self._sem_lidar_msgs.append(PointCloud2(header=Header(frame_id=trf)))
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._sem_lidar_tf.append(None)

    for trf in rgb_camera_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._rgb_camera_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        self._rgb_camera_msgs.append([CameraInfo(header=Header(frame_id=trf)), Image(header=Header(frame_id=trf))])
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._rgb_camera_tf.append(None)

    for trf in sem_camera_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._sem_camera_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        self._sem_camera_msgs.append([CameraInfo(header=Header(frame_id=trf)), Image(header=Header(frame_id=trf))])
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._sem_camera_tf.append(None)

    for trf in obu_frames:
      try:
        self.get_logger().info('Looking at tf: {} -> {}'.format(self._vehicle_base_footprint, trf))
        self._obu_tf.append(tf_buffer.lookup_transform(
          self._vehicle_base_footprint,
          trf,
          rclpy.time.Time(), Duration(seconds=3)).transform)
        # @todo: update with CAM, DENM information
        self._obu_msgs.append(String(data=''))
      except TransformException as ex:
        self.get_logger().warn('Could not transform {} to {}: {}'.format(
          self._vehicle_base_footprint, trf, ex))
        self._obu_tf.append(None)

    # Stop listening to transforms and remove references to garbage collect
    tf_listener.executor.shutdown()
    tf_listener.dedicated_listener_thread.join()
    tf_listener = None
    tf_buffer = None

    # Parse sensors tfs and parameters
    gps_tf_carla, gps_sensor_params = [], []
    for tf_item, s_param in zip(self._gps_tf, gps_parameters):
      if tf_item is not None:
        gps_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        gps_sensor_params.append(s_param)

    imu_tf_carla, imu_sensor_params = [], []
    for tf_item, s_param in zip(self._imu_tf, imu_parameters):
      if tf_item is not None:
        imu_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        imu_sensor_params.append(s_param)

    radar_tf_carla, radar_sensor_params = [], []
    for tf_item, s_param in zip(self._radar_tf, radar_parameters):
      if tf_item is not None:
        radar_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        radar_sensor_params.append(s_param)

    lidar_tf_carla, lidar_sensor_params = [], []
    for tf_item, s_param in zip(self._lidar_tf, lidar_parameters):
      if tf_item is not None:
        lidar_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        lidar_sensor_params.append(s_param)

    sem_lidar_tf_carla, sem_lidar_sensor_params = [], []
    for tf_item, s_param in zip(self._sem_lidar_tf, sem_lidar_parameters):
      if tf_item is not None:
        sem_lidar_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        sem_lidar_sensor_params.append(s_param)

    rgb_camera_tf_carla, rgb_camera_sensor_params = [], []
    for tf_item, s_param in zip(self._rgb_camera_tf, rgb_camera_parameters):
      if tf_item is not None:
        rgb_camera_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        rgb_camera_sensor_params.append(s_param)
    
    sem_camera_tf_carla, sem_camera_sensor_params = [], []
    for tf_item, s_param in zip(self._sem_camera_tf, sem_camera_parameters):
      if tf_item is not None:
        sem_camera_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        sem_camera_sensor_params.append(s_param)

    obu_tf_carla, obu_params = [], []
    for tf_item, s_param in zip(self._obu_tf, obu_parameters):
      if tf_item is not None:
        obu_tf_carla.append(ros_transform_to_carla_transform(tf_item))
        obu_params.append(s_param)

    # Aggregate data for easy processing
    sensor_params = {
      "gps" : (gps_tf_carla, gps_sensor_params),
      "imu" : (imu_tf_carla, imu_sensor_params),
      "radar" : (radar_tf_carla, radar_sensor_params),
      "lidar" : (lidar_tf_carla, lidar_sensor_params),
      "semantic_lidar" : (sem_lidar_tf_carla, sem_lidar_sensor_params),
      "rgb_camera": (rgb_camera_tf_carla, rgb_camera_sensor_params),
      "sem_camera": (sem_camera_tf_carla, sem_camera_sensor_params),
      "obu": (obu_tf_carla, obu_params)
    }

    # Spawn sensors proccesed (only those implemented in the description) in Carla world
    self._carla_world.spawn_sensors(sensor_params)

    # Max physical steering angle (in radians) of the vehicle
    # front right wheel 0: Front right, 1: Front left, 2: Back right, 3: Back left
    self._vehicle_max_steering_angle = np.radians(
      self._carla_world.player.get_physics_control().wheels[0].max_steer_angle)
    # Vehicle wheel diameter in meters (thus divided by 100) 
    self._wheel_diameter = self._carla_world.player.get_physics_control().wheels[0].radius/100 * 2

    # Distanced moved from start (0) to update wheel joint position
    self._distance_moved = [0.0, 0.0, 0.0, 0.0]

    # Individual messages types
    self._joints_state_msg = JointState()
    self._vehicle_control_msg = VehicleControl()
    self._vehicle_control_msg.header.frame_id = self._vehicle_base_footprint

    # Joint states variables and messages
    wheel_joint_names = self.get_parameter('wheel_joint_names').get_parameter_value().string_array_value
    for name in wheel_joint_names:
      self._joints_state_msg.name.append(name)

    # Just report position, do not have specific velocity and effort from Carla
    num_joints = len(wheel_joint_names)
    self._joints_state_msg.position = num_joints * [0.0]
    # self._joints_state_msg.velocity = num_joints * [0.0]
    # self._joints_state_msg.effort = num_joints * [0.0]

    # ROS comunication
    self._cb_group = MutuallyExclusiveCallbackGroup()
    self.create_subcribers()
    self.create_publishers()
    if standalone:
      self.create_services()

    # Carla vehicle control variables for ackermann to throttle/brake conversion
    self._vehicle_control_throttle = 0.0
    self._vehicle_control_steer = 0.0
    self._vehicle_control_brake = 0.0
    self._vehicle_control_hand_brake = False
    self._vehicle_control_reverse = False
    self._vehicle_control_gear = 0
    self._vehicle_control_manual_gear_shift = False
    self._controller_type = 'NA'

    # Control simulation msgs
    self._carla_info_msg = CarlaSimulationInfo(header=Header(), server_fps=0.0, client_fps=0.0,
      simulation_time='00:00:00', vehicle_name='NA', map_name='NA', carla_location=Pose(),
      throttle=0.0, brake=0.0, speed=0.0, steering=0.0, gear=0, hand_brake=False, controller_type='NA')
    self._carla_actor_id_msg = CarlaActorId(id=0)

    # Last time for control loop
    self._cmd_timeout = Duration(
      seconds=self.get_parameter('cmd_timeout').get_parameter_value().double_value)
    self._last_time = self.get_clock().now()
    self._last_cmd_time = self.get_clock().now()

    if not (self._is_sync_mode and standalone):
      self._world_tick_id = self._carla_world.world.on_tick(self.on_tick)

    self._cache_timer = self.create_timer(2.0, self.update_vehicle_cache, self._cb_group)
    self.update_vehicle_cache()

    mode_str = "Synchronous" if self._is_sync_mode else "Asynchronous"
    standalone_str = "STANDALONE" if standalone else "SUBORDINATE"
    self.get_logger().info("ROS 2 Carla Bridge started in {} {} mode. Waiting for stop signal or Ctrl+C.".format(mode_str, standalone_str))

  def create_services(self):
    """Create ROS Services."""
    self._reset_vehicle_service = self.create_service(
      CarlaResetVehicle, 'carla_reset_vehicle', self.reset_vehicle,
      callback_group=self._cb_group)
    self._traffic_manager_spawn_service = self.create_service(
      CarlaSpawnTraffic, 'carla_spawn_traffic', self.spawn_traffic,
      callback_group=self._cb_group)
    self._traffic_manager_clean_service = self.create_service(
      CarlaCleanTraffic, 'carla_clean_traffic', self.clean_traffic,
      callback_group=self._cb_group)
    self._carla_world_change_weather = self.create_service(
      CarlaChangeWeather, 'carla_change_weather', self.change_wheather,
      callback_group=self._cb_group)
    self._carla_world_change_layer = self.create_service(
      CarlaChangeLayer, 'carla_change_map_layer', self.change_layer,
      callback_group=self._cb_group)

  def create_subcribers(self):
    """Create ROS Subscribers."""
    topic = self.get_parameter('vehicle_cmd_topic').get_parameter_value().string_value
    self._vehicle_cmd_sub = self.create_subscription(CarlaVehicleControl, topic,
      self.carla_vehicle_control_callback, 1, callback_group=self._cb_group)
    topic = self.get_parameter('quit_simulation_topic').get_parameter_value().string_value
    self._quit_simulation_sub = self.create_subscription(Empty, topic,
      self.quit_simulation_callback, 1, callback_group=self._cb_group)

  def create_publishers(self):
    """Create ROS Publishers."""
    # Individual topic publishers
    topic = self.get_parameter('carla_info_topic').get_parameter_value().string_value
    self._carla_info_pub = self.create_publisher(CarlaSimulationInfo, topic, 1)
    topic = self.get_parameter('vehicle_control_topic').get_parameter_value().string_value
    self._vehicle_control_pub = self.create_publisher(VehicleControl, topic, 1)
    topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
    self._joints_state_pub = self.create_publisher(JointState, topic, 1)
    topic = self.get_parameter('perception_event_topic').get_parameter_value().string_value
    self._detections_pub = self.create_publisher(PerceptionEvent, topic, 1)
    topic = self.get_parameter('carla_actor_id_topic').get_parameter_value().string_value
    self._carla_actor_id_pub = self.create_publisher(CarlaActorId, topic, 1)

    # List sensor data publishers
    self._gps_publishers = [self.create_publisher(NavSatFix, topic[0], 1)\
      for topic in zip(self._gps_topics, self._gps_tf) if topic[1] is not None]
    self._imu_publishers = [self.create_publisher(Imu, topic[0], 1)\
      for topic in zip(self._imu_topics, self._imu_tf) if topic[1] is not None]
    self._radar_publishers = [self.create_publisher(PointCloud2, topic[0], 1)\
      for topic in zip(self._radar_topics, self._radar_tf) if topic[1] is not None]
    self._lidar_publishers = [self.create_publisher(PointCloud2, topic[0], 1)\
      for topic in zip(self._lidar_topics, self._lidar_tf) if topic[1] is not None]
    self._sem_lidar_publishers = [self.create_publisher(PointCloud2, topic, 1)\
      for topic in self._sem_lidar_topics]
    self._rgb_camera_publishers = [(self.create_publisher(CameraInfo, topic[0] + '/camera_info', 1),\
      self.create_publisher(Image, topic[0] + '/image', 1)) \
      for topic in zip(self._rgb_camera_topics, self._rgb_camera_tf) if topic[1] is not None]
    self._sem_camera_publishers = [(self.create_publisher(CameraInfo, topic[0] + '/camera_info', 1),\
      self.create_publisher(Image, topic[0] + '/image', 1)) \
      for topic in zip(self._sem_camera_topics, self._sem_camera_tf) if topic[1] is not None]
    self._obu_publishers = [self.create_publisher(String, topic, 1)\
      for topic in self._obu_topics]

  def update_vehicle_cache(self):
    """Use Carla world to update nearby vehicle cache with a low rate RPC Call."""
    self._carla_world.update_vehicle_cache()

  def publish_messages(self):
    """Publish every ROS Message."""
    self._vehicle_control_pub.publish(self._vehicle_control_msg) # Steer from vehicle
    self._joints_state_pub.publish(self._joints_state_msg) # Joint states for robot_description
    self._carla_info_pub.publish(self._carla_info_msg) # Carla simulation info
    self._carla_actor_id_pub.publish(self._carla_actor_id_msg) # Carla actor id
    self._detections_pub.publish(self._detection_msg) # Detections

    # List publishers (GPS, IMU, LiDAR and RGB_Camera)
    for gps in zip(self._gps_publishers, self._gps_msgs):
      gps[1].header.stamp = self.get_clock().now().to_msg()
      gps[0].publish(gps[1])
    for imu in zip(self._imu_publishers, self._imu_msgs):
      imu[1].header.stamp = self.get_clock().now().to_msg()
      imu[0].publish(imu[1])
    for radar in zip(self._radar_publishers, self._radar_msgs):
      radar[1].header.stamp = self.get_clock().now().to_msg()
      radar[0].publish(radar[1])
    for lidar in zip(self._lidar_publishers, self._lidar_msgs):
      lidar[1].header.stamp = self.get_clock().now().to_msg()
      lidar[0].publish(lidar[1])
    for sem_lidar in zip(self._sem_lidar_publishers, self._sem_lidar_msgs):
      sem_lidar[1].header.stamp = self.get_clock().now().to_msg()
      sem_lidar[0].publish(sem_lidar[1])
    for rgb_camera in zip(self._rgb_camera_publishers, self._rgb_camera_msgs):
      rgb_camera[1][0].header.stamp = self.get_clock().now().to_msg()
      rgb_camera[1][1].header.stamp = self.get_clock().now().to_msg()
      rgb_camera[0][0].publish(rgb_camera[1][0])
      rgb_camera[0][1].publish(rgb_camera[1][1])
    for sem_camera in zip(self._sem_camera_publishers, self._sem_camera_msgs):
      sem_camera[1][0].header.stamp = self.get_clock().now().to_msg()
      sem_camera[1][1].header.stamp = self.get_clock().now().to_msg()
      sem_camera[0][0].publish(sem_camera[1][0])
      sem_camera[0][1].publish(sem_camera[1][1])
    for obu in zip(self._obu_publishers, self._obu_msgs):
      obu[0].publish(obu[1])

  def on_tick(self, snapshot):
    """Callback to tick in a constant rate."""
    if self._is_game_quit:
      return

    # Times for the control loop
    self._t = self.get_clock().now()
    self._delta_t = self._t - self._last_time
    self._last_time = self._t

    # Speed and Steer raw data from Carla
    v = self._carla_world.player.get_velocity()
    c = self._carla_world.player.get_control()
    s = c.steer
    t = self._carla_world.player.get_transform()

    # Speed data conversion from [0,1] to [min_speed, max_speed] (m/s)
    vehicle_speed = np.sqrt(v.x**2 + v.y**2 + v.z**2)
    # Steer data conversion: from [-1,1] to [min_steer, max_steer] (rad)
    s_max = self._vehicle_max_steering_angle
    steer = s * s_max
    self._vehicle_control_msg.vehicle_speed.speed = vehicle_speed
    self._vehicle_control_msg.vehicle_steering.steering = -steer
    self._vehicle_control_msg.header.stamp = self.get_clock().now().to_msg()

    # Populate joint states message
    sign = -1 if self._carla_world.player.get_control().reverse else 1
    # Derive movement from vehicle naive cinematic model
    for ii in range(len(self._distance_moved)):
      self._distance_moved[ii] += \
        (self._wheel_diameter * np.pi * sign * vehicle_speed * (self._delta_t.nanoseconds * 1e-9))

    # Joint states for ROS Description
    self._joints_state_msg.header.stamp = self.get_clock().now().to_msg()
    self._joints_state_msg.position[0] = -steer                  # Must be ego_front_right_wheel_steer_joint
    self._joints_state_msg.position[1] = self._distance_moved[1] # Must be ego_front_right_wheel_joint
    self._joints_state_msg.position[2] = -steer                  # Must be ego_front_left_wheel_steer_joint
    self._joints_state_msg.position[3] = self._distance_moved[0] # Must be ego_front_left_wheel_joint
    self._joints_state_msg.position[4] = self._distance_moved[3] # Must be ego_back_right_wheel_joint
    self._joints_state_msg.position[5] = self._distance_moved[2] # Must be ego_back_left_wheel_joint

    # To get the current steering angle of both front wheels properly, the physics control engine must be
    # called recursively in each tick but this actually adds a lot of overhead and slows too much the simulation
    # thus, assume the global steering in both wheel just for visual consistency.
    # -1 * np.radians(self._carla_world.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel))
    # -1 * np.radians(self._carla_world.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))

    # Populate carla info msg for rviz
    try:
      roll, pitch, yaw = carla_rotation_to_RPY(self._carla_world.imu_sensors[0].transform.rotation)
    except (IndexError, AttributeError) as e:
      return
    quat = quaternion_from_euler(roll, pitch, yaw)
    
    # Server FPS
    server_delta_time = snapshot.timestamp.delta_seconds
    if server_delta_time > 0.0:
        server_fps = 1.0 / server_delta_time
    else:
        server_fps = 0.0

    # Client FPS
    current_client_time = self.get_clock().now()
    client_delta_time = current_client_time - self._last_client_time
    self._last_client_time = current_client_time
    if (client_delta_time.nanoseconds * 1e-9) > 0.0:
        client_fps = 1.0 / (client_delta_time.nanoseconds * 1e-9)
    else:
        client_fps = 0.0

    self._carla_info_msg = CarlaSimulationInfo(header=Header(frame_id='carla_info', stamp=self.get_clock().now().to_msg()),
      server_fps=np.trunc(round(server_fps)), client_fps=np.trunc(round(client_fps)),
      simulation_time=str(datetime.timedelta(seconds=int(snapshot.timestamp.elapsed_seconds))),
      vehicle_name=get_actor_display_name(self._carla_world.player, truncate=20),
      map_name=self._carla_world.map.name.split('/')[-1],
      carla_location=Pose(position=Point(x=t.location.x, y=t.location.y, z=t.location.z),
          orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])),
      throttle=c.throttle, brake=c.brake, speed=vehicle_speed, steering=s,
      gear=c.gear, hand_brake=c.hand_brake, controller_type=self._controller_type)
    self._carla_actor_id_msg = CarlaActorId(id=self._carla_world.player.id)

    # Sensor data conversions
    # IMU
    for ii in range(len(self._imu_msgs)):
      # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
      # Here, these measurements are converted to the right-handed ROS convention
      #  (X forward, Y left, Z up).
      imu_object = self._carla_world.imu_sensors[ii]
      imu_ros_msg = self._imu_msgs[ii]

      imu_ros_msg.angular_velocity.x = -imu_object.gyroscope[0]
      imu_ros_msg.angular_velocity.y = imu_object.gyroscope[1]
      imu_ros_msg.angular_velocity.z = -imu_object.gyroscope[2]

      imu_ros_msg.linear_acceleration.x = imu_object.accelerometer[0]
      imu_ros_msg.linear_acceleration.y = -imu_object.accelerometer[1]
      imu_ros_msg.linear_acceleration.z = imu_object.accelerometer[2]

      roll, pitch, yaw = carla_rotation_to_RPY(self._carla_world.imu_sensors[ii].transform.rotation)
      quat = quaternion_from_euler(roll, pitch, yaw)
      imu_ros_msg.orientation.x = quat[0]
      imu_ros_msg.orientation.y = quat[1]
      imu_ros_msg.orientation.z = quat[2]
      imu_ros_msg.orientation.w = quat[3]

      # Add sensor covariance extracted from sensor noise model
      noise_gyro_x = 0.00019  # float(imu_object.sensor.attributes['noise_accel_stddev_x'])**2
      noise_gyro_y = 0.00019  # float(imu_object.sensor.attributes['noise_accel_stddev_y'])**2
      noise_gyro_z = 0.00019  # float(imu_object.sensor.attributes['noise_accel_stddev_z'])**2
      noise_accel_x = 0.00019 # float(imu_object.sensor.attributes['noise_gyro_stddev_x'])**2
      noise_accel_y = 0.00019 # float(imu_object.sensor.attributes['noise_gyro_stddev_y'])**2
      noise_accel_z = 0.00019 # float(imu_object.sensor.attributes['noise_gyro_stddev_z'])**2
      imu_ros_msg.orientation_covariance[0] = noise_gyro_x
      imu_ros_msg.orientation_covariance[4] = noise_gyro_y
      imu_ros_msg.orientation_covariance[8] = noise_gyro_z
      imu_ros_msg.angular_velocity_covariance[0] = noise_accel_x
      imu_ros_msg.angular_velocity_covariance[4] = noise_accel_y
      imu_ros_msg.angular_velocity_covariance[8] = noise_accel_z
      imu_ros_msg.linear_acceleration_covariance[0] = noise_accel_x
      imu_ros_msg.linear_acceleration_covariance[4] = noise_accel_y
      imu_ros_msg.linear_acceleration_covariance[8] = noise_accel_z

    # GPS Data conversion
    for ii in range(len(self._gps_msgs)):
      gps_object = self._carla_world.gnss_sensors[ii]
      gps_ros_msg = self._gps_msgs[ii]

      gps_ros_msg.latitude = gps_object.lat
      gps_ros_msg.longitude = gps_object.lon
      gps_ros_msg.altitude = gps_object.alt

      # Add sensor covariance extracted from sensor noise model
      gps_ros_msg.position_covariance_type = 2 # COVARIANCE_TYPE_DIAGONAL_KNOWN
      gps_ros_msg.position_covariance[0] = 0.00019 # float(gps_object.sensor.attributes['noise_lat_stddev'])**2
      gps_ros_msg.position_covariance[4] = 0.00019 # float(gps_object.sensor.attributes['noise_lon_stddev'])**2
      gps_ros_msg.position_covariance[8] = 0.00019 # float(gps_object.sensor.attributes['noise_alt_stddev'])**2

      gps_ros_msg.status.status = 2  # With augmentated fix
      gps_ros_msg.status.service = 1 # GPS signal normal

    # Lidar Data conversion
    for ii in range(len(self._lidar_msgs)):
      last_header = self._lidar_msgs[ii].header
      self._lidar_msgs[ii] = self._carla_world.lidar_sensors[ii].get_ros_pointcloud()
      self._lidar_msgs[ii].header = last_header

    # Semantic Lidar Data conversion
    for ii in range(len(self._sem_lidar_msgs)):
      last_header = self._sem_lidar_msgs[ii].header
      self._sem_lidar_msgs[ii] = self._carla_world.sem_lidar_sensors[ii].get_ros_pointcloud()
      self._sem_lidar_msgs[ii].header = last_header

    # Radar Data conversion, @todo: adjust this with proper message generation
    for ii in range(len(self._radar_msgs)):
      last_header = self._radar_msgs[ii].header
      self._radar_msgs[ii] = self._carla_world.radar_sensors[ii]._ros_pointcloud
      self._radar_msgs[ii].header = last_header

    # RGB Camera data conversion, assume camera info header is the same as camera RGB data
    for ii in range(len(self._rgb_camera_msgs)):
      last_header = self._rgb_camera_msgs[ii][0].header
      self._rgb_camera_msgs[ii][0] = self._carla_world.rgb_camera_sensors[ii]._camera_info
      self._rgb_camera_msgs[ii][1] = self._carla_world.rgb_camera_sensors[ii]._ros_data
      self._rgb_camera_msgs[ii][0].header = last_header
      self._rgb_camera_msgs[ii][1].header = last_header

    # Semantic Camera data conversion, assume camera info header is the same as camera Semantic data
    for ii in range(len(self._sem_camera_msgs)):
      last_header = self._sem_camera_msgs[ii][0].header
      self._sem_camera_msgs[ii][0] = self._carla_world.sem_camera_sensors[ii].get_camera_info()
      self._sem_camera_msgs[ii][1] = self._carla_world.sem_camera_sensors[ii].get_ros_image()
      self._sem_camera_msgs[ii][0].header = last_header
      self._sem_camera_msgs[ii][1].header = last_header

    # Obu data conversion, it only sends CAM data @todo: sensor is only for sending not reception
    for ii in range(len(self._obu_msgs)):
      obu_object = self._carla_world.obu_sensors[ii]
      obu_ros_msg = self._obu_msgs[ii]
      obu_ros_msg.data = obu_object.data

    # Near player vehicles for cooperation behaviours
    self._detection_msg = PerceptionEvent()
    self._carla_world.get_nearby_vehicles(self._detection_msg, 50.0, snapshot)
    # self._carla_world.get_near_player_walkers(self._detection_msg)
    # self._carla_world.get_near_player_landmarks(self._detection_msg)
    # self._carla_world.get_near_player_infrastructure(self._detection_msg)
    self._detection_msg.header = Header(frame_id=self._map_frame, stamp=self.get_clock().now().to_msg())

    self.publish_messages()
    self.parse_vehicle_ackermann_cmd()

  def quit_simulation_callback(self, msg):
    """Check if the high level interface wants to finish the simulation."""
    self.get_logger().info("Stop signal received! Initiating shutdown...")
    self._is_game_quit = True

  def reset_vehicle(self, request, response):
    """ROS service callback to reset the vehicle to the original position."""
    self._carla_world.reset_vehicle_position()
    response.result = True
    return response

  def cleanup(self):
    """Remove the tick callback if aynch mode is on"""
    if self._world_tick_id:
      self._carla_world.world.remove_on_tick(self._world_tick_id)

  def change_wheather(self, request, response):
    """ROS service callback to change Carla world weather."""
    with self._carla_lock:
      response.result, response.message = self._carla_world.change_wheather(request.weather)
    return response

  def change_layer(self, request, response):
    """ROS service callback to change a Carla Map layer."""
    with self._carla_lock:
      if request.action == CarlaChangeLayer.Request.LOAD: # Load Layer
        response.result, response.message = self._carla_world.change_map_layer(request.layer)
      else: # Unload Layer
        response.result, response.message = self._carla_world.change_map_layer(request.layer, False)
    return response

  def spawn_traffic(self, request, response):
    """ROS service callback to spawn traffic controller with Carla Traffic Manager."""
    if self._carla_traffic_manager is None:
      response.result = False
      response.message = "Traffic Manager is not enabled in this client, cannot spawn traffic..."
      return response

    with self._carla_lock:
      self._carla_traffic_manager.setup_parameters(
        request.n_vehicles, request.n_walkers,
        request.random_seed, request.hybrid_mode, request.hybrid_radius)

      response.result, response.message = self._carla_traffic_manager.spawn_traffic()

    return response

  def clean_traffic(self, request, response):
    """ROS service callback to remove every Carla Actor related with traffic."""
    if self._carla_traffic_manager is None:
      response.result = False
      response.message = "Traffic Manager is not enabled in this client, cannot clean traffic..."
      return response

    with self._carla_lock:
      response.result, response.message = self._carla_traffic_manager.clean_traffic()

    return response

  def set_traffic_manager(self, traffic_manager):
    """Just set the traffic manager object to deal with traffic."""
    self._carla_traffic_manager = traffic_manager

  def carla_vehicle_control_callback(self, carla_vehicle_control_msg):
    """ROS topic ackermann control commands callback."""
    self._last_cmd_time = self.get_clock().now()

    self._controller_type = carla_vehicle_control_msg.header.frame_id

    self._vehicle_control_throttle = carla_vehicle_control_msg.throttle
    self._vehicle_control_steer = carla_vehicle_control_msg.steer
    self._vehicle_control_brake = carla_vehicle_control_msg.brake
    self._vehicle_control_hand_brake = carla_vehicle_control_msg.hand_brake
    self._vehicle_control_reverse = carla_vehicle_control_msg.reverse
    self._vehicle_control_gear = carla_vehicle_control_msg.gear
    self._vehicle_control_manual_gear_shift = carla_vehicle_control_msg.manual_gear_shift

  def parse_vehicle_ackermann_cmd(self):
    """Convert from ackermann to throttle/brake"""
    if ((self._cmd_timeout.nanoseconds * 1e-9 > 0.0) \
      and self._t - self._last_cmd_time > self._cmd_timeout):
      self._vehicle_control.throttle = 0.0
      self._vehicle_control.steer = 0.0
      self._vehicle_control.brake = 1.0
      self._vehicle_control.hand_brake = True
      self._vehicle_control.reverse = False
      # self._vehicle_control._manual_gear_shift = True
      # self._vehicle_control._gear = 0
      self.get_logger().warn(
        "Too much time since last CarlaVehicleControl message. Stopping vehicle...")
    else:
      self._vehicle_control.throttle = self._vehicle_control_throttle
      self._vehicle_control.steer = -self._vehicle_control_steer
      self._vehicle_control.brake = self._vehicle_control_brake
      self._vehicle_control.hand_brake = self._vehicle_control_hand_brake
      self._vehicle_control.reverse = self._vehicle_control_reverse
      # self._vehicle_control._manual_gear_shift = self._vehicle_control_gear
      # self._vehicle_control._gear = self._vehicle_control_manual_gear_shift
    self._carla_world.player.apply_control(self._vehicle_control)

  def is_game_quit(self):
    """Checks if the simulation finished."""
    return self._is_game_quit

  def set_client_fps(self, fps):
    """Gets the client fps."""
    self._client_fps = fps

class VehicleROSBridge(object):
  def __init__(self, world, is_sync_mode, standalone=True):
    """Initilize the ROS Bridge Node and execution mode."""
    self._carla_node = VehicleROSNode(world, is_sync_mode, standalone)

    self._executor = MultiThreadedExecutor(num_threads=4)
    self._executor.add_node(self._carla_node)

    self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
    self._executor_thread.start()

  def on_tick(self, snapshot):
    """Calback for every Carla World tick. Ensure adquire lock if sync mode"""
    self._carla_node.on_tick(snapshot)

  def is_game_quit(self):
    """Returns to the high level if the simulation needs to finish."""
    return self._carla_node.is_game_quit()

  def set_traffic_manager(self, traffic_manager):
    """Just set the traffic manager object to deal with traffic at Node level."""
    self._carla_node.set_traffic_manager(traffic_manager)

  def get_lock(self):
    """Just return the lock to avoid concurrency problems in synch simulations."""
    return self._carla_node._carla_lock

  def destroy(self):
    """Destroy every object created."""
    self._executor.shutdown()
    self._carla_node.cleanup()
    self._executor_thread.join(timeout=1.0)
    self._carla_node.destroy_node()
