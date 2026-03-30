from carla_vehicle.sensor.gnss_sensor import GnssSensor
from carla_vehicle.sensor.imu_sensor import IMUSensor
from carla_vehicle.sensor.radar_sensor import RadarSensor
from carla_vehicle.sensor.lidar_sensor import LidarSensor
from carla_vehicle.sensor.semantic_lidar_sensor import SemanticLidarSensor
from carla_vehicle.sensor.rgb_camera_sensor import RGBCameraSensor
from carla_vehicle.sensor.semantic_camera_sensor import SemanticCameraSensor
from carla_vehicle.sensor.obu_sensor import OBUSensor

from carla_vehicle.localization.transform import (ros_point_to_carla_location,
  RPY_to_carla_rotation, carla_transform_to_ros_pose)

from perception_interfaces.msg import (Detection, ClassType, DetectionHypothesis,
  VelocityHypothesis, ClassTypeHypothesis, BoundingBox3DHypothesis)
from geometry_msgs.msg import Point

from carla_vehicle.utils import find_weather_presets

import numpy as np
from dataclasses import dataclass
import carla
import random

@dataclass
class VehicleProfile:
  """Stores static data that doesn't change during the simulation."""
  extent: carla.Vector3D
  speed: carla.Vector3D
  type_id: str

class VehicleWorld(object):
  def __init__(self, carla_world, args):
    """Initialize World to hold Carla World and Map."""
    self.world = carla_world
    self.sync = args.sync

    try:
      self.map = self.world.get_map()
    except RuntimeError as error:
      print('RuntimeError: {}'.format(error))
      print('  The server could not send the OpenDRIVE (.xodr) file:')
      print('  Make sure it exists, has the same name of your town, and is correct.')
      sys.exit(1)

    self.gnss_sensors = None
    self.imu_sensors = None
    self.lidar_sensors = None
    self.sem_lidar_sensors = None
    self.radar_sensors = None
    self.rgb_camera_sensors = None
    self.sem_camera_sensors = None
    self.obu_sensors = None

    self.player = None
    self._weather_presets = find_weather_presets()
    self._map_layers = [
      carla.MapLayer.NONE,
      carla.MapLayer.Buildings,
      carla.MapLayer.Decals,
      carla.MapLayer.Foliage,
      carla.MapLayer.Ground,
      carla.MapLayer.ParkedVehicles,
      carla.MapLayer.Particles,
      carla.MapLayer.Props,
      carla.MapLayer.StreetLights,
      carla.MapLayer.Walls,
      carla.MapLayer.All
    ]
    self._loaded_map_layers = set(self._map_layers)

    self._near_vehicles_cache = {}
    self._vehicle_id_list = []

    self._confusion_matrix = {
      ClassType.CAR: {ClassType.CAR: 0.90, ClassType.TRUCK: 0.08, ClassType.UNKNOWN: 0.02},
      ClassType.WALKER: {ClassType.WALKER: 0.85, ClassType.BIKE: 0.10, ClassType.UNKNOWN: 0.05},
      ClassType.BIKE: {ClassType.BIKE: 0.80, ClassType.WALKER: 0.15, ClassType.UNKNOWN: 0.05},
      ClassType.MOTORCYCLE: {ClassType.MOTORCYCLE: 0.80, ClassType.WALKER: 0.15, ClassType.UNKNOWN: 0.05},
      ClassType.TRUCK: {ClassType.TRUCK: 0.85, ClassType.VAN: 0.10, ClassType.UNKNOWN: 0.05},
      ClassType.VAN: {ClassType.VAN: 0.85, ClassType.TRUCK: 0.10, ClassType.UNKNOWN: 0.05}
    }

    actor_role_name = args.rolename
    vehicle_type = args.vehicle
    vehicle_color = args.vehicle_color
    # Get a custom vehicle.
    blueprint = self.world.get_blueprint_library().find('vehicle.' + vehicle_type)
    blueprint.set_attribute('role_name', actor_role_name)
    if blueprint.has_attribute('color'):
      color = vehicle_color
      if vehicle_color == 'random':
        color = random.choice(blueprint.get_attribute('color').recommended_values)
      blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
      driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
      blueprint.set_attribute('driver_id', driver_id)
    if blueprint.has_attribute('is_invincible'):
      blueprint.set_attribute('is_invincible', 'true')

    # Spawn the player
    spawn_pos = args.spawn
    self.last_transform = None
    while self.player is None:
      if not self.map.get_spawn_points():
        print('There are no spawn points available in your map/town.')
        print('Please add some Vehicle Spawn Point to your UE4 scene.')
        sys.exit(1)
      if spawn_pos == "random":
        spawn_points = self.map.get_spawn_points()
        self.last_transform = random.choice(spawn_points) if spawn_points else carla.Transform()
        self.player = self.world.try_spawn_actor(blueprint, self.last_transform)
        self.modify_vehicle_physics(self.player)
      else:
        x, y, z, roll, pitch, yaw = [float(ii) for ii in spawn_pos.split()]
        z+=0.2
        self.last_transform = carla.Transform(
          ros_point_to_carla_location(Point(x=x, y=y, z=z)),
          RPY_to_carla_rotation(roll, pitch, yaw))
        self.player = self.world.try_spawn_actor(blueprint, self.last_transform)
        self.last_transform2 = carla.Transform(
          ros_point_to_carla_location(Point(x=x-10, y=y, z=z)),
          RPY_to_carla_rotation(roll, pitch, yaw))
        self.player2 = self.world.try_spawn_actor(blueprint, self.last_transform2)
        if self.player is None:
          print("Unable to spawn vehicle in point (x={0}, y={1}, z={2}, roll={3}, pitch={4}, yaw={5}), will select a random position...".format(x, y, z, roll, pitch, yaw))
          spawn_pos = "random"
        else:
          self.modify_vehicle_physics(self.player)

    if self.sync:
      # This is a workaround to avoid the time needed until the car reacts to throttle
      self.player.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
      self.world.tick()
      self.player.apply_control(carla.VehicleControl(manual_gear_shift=False))
    else:
      self.world.wait_for_tick()

  def spawn_sensors(self, sensor_params):
    """Spawn every sensor attached to the Robot description"""
    self.gnss_sensors = [GnssSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["gps"][0], sensor_params["gps"][1])]
    self.imu_sensors = [IMUSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["imu"][0], sensor_params["imu"][1])]
    self.lidar_sensors = [LidarSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["lidar"][0], sensor_params["lidar"][1])]
    self.sem_lidar_sensors = [SemanticLidarSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["semantic_lidar"][0], sensor_params["semantic_lidar"][1])]
    self.radar_sensors = [RadarSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["radar"][0], sensor_params["radar"][1])]
    self.rgb_camera_sensors = [RGBCameraSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["rgb_camera"][0], sensor_params["rgb_camera"][1])]
    self.sem_camera_sensors = [SemanticCameraSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["sem_camera"][0], sensor_params["sem_camera"][1])]
    self.obu_sensors = [OBUSensor(self.player, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["obu"][0], sensor_params["obu"][1])]

  def reset_vehicle_position(self):
    """Reset vehicle to original position."""
    self.player.set_transform(self.last_transform)

  def change_wheather(self, weather_index):
    """Query Carla World to change current weather with a preset."""
    try:
      preset = self._weather_presets[weather_index]
    except IndexError:
      return False, "Weather not recognized, out of bounds!"

    if self.world.get_weather() != preset[0]:
      self.world.set_weather(preset[0])
      return True, "Weather changed succesfully!"
    return False, "Unable to change Weather, already in that preset!"

  def change_map_layer(self, layer_index, load=True):
    """Query Carla World to load/unload the current map layer in the simulation."""
    map_name = self.world.get_map().name
    if "_Opt" in map_name: # Only available in layered maps
      try:
        target_layer = self._map_layers[layer_index]
      except IndexError:
        return False, "Layer not recognized, out of bounds!"

      if load:
        if target_layer in self._loaded_map_layers:
          return True, "Layer {} already loaded. Skipping.".format(target_layer)
        else:
          self.world.load_map_layer(target_layer)
          self._loaded_map_layers.add(target_layer)
          return True, "Successfully loaded {}.".format(target_layer)
      else:
        if target_layer not in self._loaded_map_layers:
          return True, "Layer {} already unloaded. Skipping.".format(target_layer)
        else:
          self.world.unload_map_layer(target_layer)
          self._loaded_map_layers.discard(target_layer)
          return True, "Successfully unloaded {}.".format(target_layer)

    return False, "Unable to load Layer, map is not layered!"

  def modify_vehicle_physics(self, actor):
    """Modify some vehicle physics."""
    physics_control = actor.get_physics_control()
    physics_control.use_sweep_wheel_collision = True
    physics_control.use_gear_autobox = True
    physics_control.gear_switch_time = 0.001
    actor.apply_physics_control(physics_control)

  def get_class(self, base_type):
    """Get the detection Class given an Carla actor_id."""
    if base_type == 'car':
      return ClassType.VEHICLE, ClassType.CAR
    elif base_type == 'van':
      return ClassType.VEHICLE, ClassType.VAN
    elif base_type == 'truck':
      return ClassType.VEHICLE, ClassType.TRUCK
    elif base_type == 'bus':
      return ClassType.VEHICLE, ClassType.BUS
    elif base_type == 'motorcycle':
      return ClassType.VEHICLE, ClassType.MOTORCYCLE
    elif base_type == 'bicycle':
      return ClassType.VEHICLE, ClassType.BIKE
    elif base_type == 'walker':
      return ClassType.PEDESTRIAN, ClassType.WALKER
    else:
      return ClassType.UNKNOWN, ClassType.UNKNOWN

  def update_vehicle_cache(self):
    """Low-frequency RPC call to get all vehicle in the map."""
    vehicles = self.world.get_actors().filter('vehicle.*')
    current_ids = [v.id for v in vehicles if v.id != self.player.id]

    # Clean up bbox cache for vehicles that no longer exist
    self._near_vehicles_cache = {id: profile for id, profile in self._near_vehicles_cache.items() if id in current_ids}

    # Add new vehicles to the cache
    for v in vehicles:
      if v.id not in self._near_vehicles_cache:
        self._near_vehicles_cache[v.id] = VehicleProfile(
          extent=v.bounding_box.extent,
          speed=v.get_velocity(),
          # Walker actors do not have 'base_type' attribute so if not found just set as 'walker'
          type_id=v.attributes.get('base_type', 'walker')
        )

    self._vehicle_id_list = current_ids

  # @todo: do the same with infrastructure and walkers!
  def get_nearby_vehicles(self, detection_msg, radius, snapshot, num_hypotheses=2):
    """
    High-frequency, ZERO-RPC call. 
    Calculates nearby vehicles using local snapshot memory.
    
    :param radius: float (search radius in meters)
    :param snapshot: carla.WorldSnapshot (from world.get_snapshot())
    :return: list of actor IDs within the radius
    """
    radius_sq = radius ** 2  # Squared radius for faster math
    target_location = self.player.get_transform().location

    # Define standard deviations for the physical noise (meters and m/s)
    std_pos = 0.2
    std_size = 0.1
    std_speed = 0.2

    for actor_id in self._vehicle_id_list:
      # Fetch the actor's state directly from the local snapshot
      actor_snapshot = snapshot.find(actor_id)
      
      # If it returns None, the vehicle was destroyed since our last cache update
      if actor_snapshot is None:
        continue 

      transform = actor_snapshot.get_transform()
      loc = transform.location

      dist_sq = (loc.x - target_location.x)**2 + (loc.y - target_location.y)**2

      if dist_sq <= radius_sq:
        profile = self._near_vehicles_cache.get(actor_id)
        if not profile:
          continue

        detection = Detection()
        detection.id = actor_id

        v_type, v_class = self.get_class(profile.type_id)
        original_pose = carla_transform_to_ros_pose(transform)

        new_hypothesis = DetectionHypothesis()
        decay = 0.05
        for _ in range(num_hypotheses):
          vel_hypoth = VelocityHypothesis()
          t_hypoth = ClassTypeHypothesis()
          t_hypoth.type.type = v_type
          bbox_hypoth = BoundingBox3DHypothesis()

          # Add Gaussian noise to Pose Center
          bbox_hypoth.bounding_box.center.pose.position.x = original_pose.position.x + np.random.normal(0.0, std_pos)
          bbox_hypoth.bounding_box.center.pose.position.y = original_pose.position.y + np.random.normal(0.0, std_pos)
          bbox_hypoth.bounding_box.center.pose.position.z = original_pose.position.z + np.random.normal(0.0, std_pos)
          # Add Gaussian noise to Extent/Size
          bbox_hypoth.bounding_box.size.x = max(0.01, (profile.extent.x * 2.0) + np.random.normal(0.0, std_size))
          bbox_hypoth.bounding_box.size.y = max(0.01, (profile.extent.y * 2.0) + np.random.normal(0.0, std_size))
          bbox_hypoth.bounding_box.size.z = max(0.01, (profile.extent.z * 2.0) + np.random.normal(0.0, std_size))
          size_error = np.sqrt(((profile.extent.x * 2.0) - bbox_hypoth.bounding_box.size.x)**2 +
                               ((profile.extent.y * 2.0) - bbox_hypoth.bounding_box.size.y)**2 +
                               ((profile.extent.z * 2.0) - bbox_hypoth.bounding_box.size.z)**2)
          pose_error = np.sqrt((original_pose.position.x - bbox_hypoth.bounding_box.center.pose.position.x)**2 +
                               (original_pose.position.y - bbox_hypoth.bounding_box.center.pose.position.y)**2 +
                               (original_pose.position.z - bbox_hypoth.bounding_box.center.pose.position.z)**2)
          size_score = np.exp(-decay * size_error)
          pose_score = np.exp(-decay * pose_error)
          bbox_hypoth.score = (size_score + pose_score)/2
          std_sq = bbox_hypoth.score**2
          bbox_hypoth.bounding_box.center.covariance = [std_sq, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, std_sq, 0.0, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, std_sq, 0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, std_sq, 0.0, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, std_sq, 0.0,
                                                        0.0, 0.0, 0.0, 0.0, 0.0, std_sq]

          # Add Gaussian noise to Speed
          vel_hypoth.velocity.x = profile.speed.x + np.random.normal(0.0, std_speed)
          vel_hypoth.velocity.y = profile.speed.y + np.random.normal(0.0, std_speed)
          vel_hypoth.velocity.z = profile.speed.z + np.random.normal(0.0, std_speed)
          speed_error = np.sqrt((profile.speed.x - vel_hypoth.velocity.x)**2 +
                                (profile.speed.y - vel_hypoth.velocity.y)**2 +
                                (profile.speed.z - vel_hypoth.velocity.z)**2)
          vel_hypoth.score = np.exp(-decay * speed_error)

          # Apply Classification Noise using the Confusion Matrix
          if v_class in self._confusion_matrix:
            possible_classes = list(self._confusion_matrix[v_class].keys())
            probabilities = list(self._confusion_matrix[v_class].values())
            
            # Pick a new class based on the defined probabilities
            t_hypoth.type.class_detection = int(np.random.choice(possible_classes, p=probabilities))
            t_hypoth.score = self._confusion_matrix[v_class][t_hypoth.type.class_detection]
          else:
            # Safe fallback: if the GT type isn't in our matrix, trust the GT
            t_hypoth.type.class_detection = v_class
            t_hypoth.score = 1.0

          new_hypothesis.type_hypothesis.append(t_hypoth)
          new_hypothesis.bbox_hypothesis.append(bbox_hypoth)
          new_hypothesis.velocity_hypothesis.append(vel_hypoth)
          decay+=0.07

        new_hypothesis.type_hypothesis.sort(reverse=True, key=lambda x: x.score)
        new_hypothesis.bbox_hypothesis.sort(reverse=True, key=lambda x: x.score)
        new_hypothesis.velocity_hypothesis.sort(reverse=True, key=lambda x: x.score)

        # Add Ground truth in last item with negative score to easy detect
        gt_vel_hypoth = VelocityHypothesis()
        gt_vel_hypoth.velocity.x = profile.speed.x
        gt_vel_hypoth.velocity.y = profile.speed.y
        gt_vel_hypoth.velocity.z = profile.speed.z
        gt_vel_hypoth.score = -1.0
        gt_type_hypoth = ClassTypeHypothesis()
        gt_type_hypoth.type.type = v_type
        gt_type_hypoth.type.class_detection = v_class
        gt_type_hypoth.score = -1.0
        gt_bbox_hypoth = BoundingBox3DHypothesis()
        gt_bbox_hypoth.bounding_box.center.pose.position.x = original_pose.position.x
        gt_bbox_hypoth.bounding_box.center.pose.position.y = original_pose.position.y
        gt_bbox_hypoth.bounding_box.center.pose.position.z = original_pose.position.z
        gt_bbox_hypoth.bounding_box.size.x = profile.extent.x * 2.0
        gt_bbox_hypoth.bounding_box.size.y = profile.extent.y * 2.0
        gt_bbox_hypoth.bounding_box.size.z = profile.extent.z * 2.0
        gt_bbox_hypoth.score = -1.0

        new_hypothesis.type_hypothesis.append(gt_type_hypoth)
        new_hypothesis.bbox_hypothesis.append(gt_bbox_hypoth)
        new_hypothesis.velocity_hypothesis.append(gt_vel_hypoth)

        detection.hypothesis = new_hypothesis
        detection_msg.detections.append(detection)

  def destroy(self):
    """Once the simulation finish destroy every object spawned."""
    # Destroy all world items
    sensor_lists = [
      self.gnss_sensors,
      self.imu_sensors,
      self.obu_sensors,
      self.lidar_sensors,
      self.sem_lidar_sensors,
      self.radar_sensors,
      self.rgb_camera_sensors,
      self.sem_camera_sensors
    ]

    for sensor_list in sensor_lists:
      if sensor_list is not None:
        for sensor in sensor_list:
            sensor.destroy()

    if self.player is not None:
      self.player.destroy()
      self.player2.destroy()
