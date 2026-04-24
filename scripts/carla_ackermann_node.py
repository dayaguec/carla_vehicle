#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import math
import threading

from carla_interfaces.msg import CarlaVehicleControl
from simple_pid import PID
from ackermann_msgs.msg import AckermannDriveStamped
from vehicle_interfaces.msg import VehicleControl

from rcl_interfaces.msg import SetParametersResult

from rclpy.executors import ExternalShutdownException

class CarlaAckermannNode(Node):
  def __init__(self):
    super().__init__('carla_ackermann_node')

    # Declare this node params to use later
    self.declare_parameters(
      namespace='',
      parameters=[
        ('cmd_timeout', 10.0),
        ('pub_frequency', 30.0),
        ('steer_limit', 0.31),
        ('speed_limit', 10.0),
        ('max_throttle', 0.4),
        ('max_brake', 0.4),
        ('accel_limit', 1.5),
        ('steer_vel_limit', 1.0),
        ('vehicle_max_steering_angle', math.radians(70)),
        ('controller_rate', 10.0),
        ('ackermann_cmd_topic', 'ackermann_cmd'),
        ('vehicle_cmd_topic', 'carla_vehicle_cmd'),
        ('vehicle_control_topic', 'vehicle_control'),
        ('speed_pid.kp', 1.0),
        ('speed_pid.ki', 0.0),
        ('speed_pid.kd', 0.0),
        ('speed_pid.sample_time', 0.05),
        ('speed_pid.auto_mode', True),
        ('speed_pid.proportional_on_measurement', False),
        ('brake_pid.kp', 1.0),
        ('brake_pid.ki', 0.0),
        ('brake_pid.kd', 0.0),
        ('brake_pid.sample_time', 0.05),
        ('brake_pid.auto_mode', True),
        ('brake_pid.proportional_on_measurement', False),
        ('brake_epsilon', 0.4),
        ('stop_epsilon', 0.05)
      ]
    )

    self._cmd_timeout = Duration(
      seconds=self.get_parameter('cmd_timeout').get_parameter_value().double_value)
    self._pub_frequency = self.get_parameter('pub_frequency').get_parameter_value().double_value
    self._steer_limit = self.get_parameter('steer_limit').get_parameter_value().double_value
    self._speed_limit = self.get_parameter('speed_limit').get_parameter_value().double_value
    self._max_throttle = self.get_parameter('max_throttle').get_parameter_value().double_value
    self._max_brake = self.get_parameter('max_brake').get_parameter_value().double_value
    self._accel_limit = self.get_parameter('accel_limit').get_parameter_value().double_value
    self._steer_vel_limit = self.get_parameter('steer_vel_limit').get_parameter_value().double_value
    self._vehicle_max_steering_angle = self.get_parameter('vehicle_max_steering_angle').get_parameter_value().double_value
    self._control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value

    # Time at which the most recent Ackermann command was received.
    self._last_cmd_time = self.get_clock().now()

    # _ackermann_cmd_lock is used to control access to _steer_ang,
    # _steer_ang_vel, _speed, _accel, and _jerk.
    self._ackermann_cmd_lock = threading.Lock()
    self._steer_ang = .0      # Steering angle (rad)
    self._steer_ang_vel = .0  # Steering angle velocity (rad/s)
    self._speed = .0          # Speed (m/s)
    self._accel = .0          # Acceleration (m/s2)
    self._jerk = .0           # Jerk (m/s3)

    self._last_steer_ang = .0  # Last steering angle

    self._last_speed = .0
    self._last_accel_limit = .0  # Last acceleration limit

    # Control variables
    self._current_vehicle_speed = .0
    # This control variable is always positive
    self._control_steer = .0
    # This control variable can be both negative: backward movement or positive: forward movement
    self._control_speed = .0
    # This control variable can be positive or negative, in the case of negative (or 0)
    # vel the controller will assume max steer vel
    self._control_steer_vel = .0

    self._final_throttle_cmd = .0
    self._final_steering_cmd = .0
    self._final_brake_cmd = .0
    self._final_handbrake_status = True

    # ROS Comunication
    topic = self.get_parameter('vehicle_cmd_topic').get_parameter_value().string_value
    self._vehicle_cmd_publisher_ = self.create_publisher(CarlaVehicleControl, topic, 1)
    topic = self.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value
    self._ackermann_cmd_sub = self.create_subscription(AckermannDriveStamped, topic,
      self.ackermann_cmd_cb, 1)
    topic = self.get_parameter('vehicle_control_topic').get_parameter_value().string_value
    self._vehicle_control_sub = self.create_subscription(VehicleControl, topic,
      self.control_cb, 1)

    # Control PIDs
    self._speed_pid_controller = PID(Kp=self.get_parameter('speed_pid.kp').get_parameter_value().double_value,
                                     Ki=self.get_parameter('speed_pid.ki').get_parameter_value().double_value,
                                     Kd=self.get_parameter('speed_pid.kd').get_parameter_value().double_value,
                                     sample_time=self.get_parameter('speed_pid.sample_time').get_parameter_value().double_value,
                                     output_limits=(0.0, 1.0))

    self._brake_pid_controller = PID(Kp=self.get_parameter('brake_pid.kp').get_parameter_value().double_value,
                                     Ki=self.get_parameter('brake_pid.ki').get_parameter_value().double_value,
                                     Kd=self.get_parameter('brake_pid.kd').get_parameter_value().double_value,
                                     sample_time=self.get_parameter('brake_pid.sample_time').get_parameter_value().double_value,
                                     output_limits=(-1.0, 0.0))
    self._brake_epsilon = self.get_parameter('brake_epsilon').get_parameter_value().double_value
    self._full_stop_epsilon = self.get_parameter('stop_epsilon').get_parameter_value().double_value

    self.add_on_set_parameters_callback(self.reconfigure_pid_parameters)

    self._last_time = self.get_clock().now()
    self._control_timer = self.create_timer(1.0/self._control_rate, self.pid_control_loop)

  def reconfigure_pid_parameters(self, params):
    param_values = {p.name: p.value for p in params}

    pid_param_names = {
      'speed_pid.kp',
      'speed_pid.ki',
      'speed_pid.kd',
      'speed_pid.sample_time',
      'speed_pid.auto_mode',
      'speed_pid.proportional_on_measurement',
      'brake_pid.kp',
      'brake_pid.ki',
      'brake_pid.kd',
      'brake_pid.sample_time',
      'brake_pid.auto_mode',
      'brake_pid.proportional_on_measurement',
      'brake_epsilon',
      'stop_epsilon'
    }
    common_names = pid_param_names.intersection(param_values.keys())
    if not common_names:
      return SetParametersResult(successful=True)

    if any(p.value is None for p in params):
      return SetParametersResult(
        successful=False, reason="Parameter must have a value assigned")

    self._speed_pid_controller.tunings = (
      param_values.get("speed_pid.kp", self._speed_pid_controller.Kp),
      param_values.get("speed_pid.ki", self._speed_pid_controller.Ki),
      param_values.get("speed_pid.kd", self._speed_pid_controller.Kd)
    )
    self._speed_pid_controller.sample_time = param_values.get("speed_pid.sample_time",
      self._speed_pid_controller.sample_time)
    self._speed_pid_controller.auto_mode = param_values.get("speed_pid.auto_mode",
      self._speed_pid_controller.auto_mode)
    self._speed_pid_controller.proportional_on_measurement = param_values.get("speed_pid.proportional_on_measurement",
      self._speed_pid_controller.proportional_on_measurement)
    
    self._brake_pid_controller.tunings = (
      param_values.get("brake_pid.kp", self._brake_pid_controller.Kp),
      param_values.get("brake_pid.ki", self._brake_pid_controller.Ki),
      param_values.get("brake_pid.kd", self._brake_pid_controller.Kd)
    )
    self._brake_pid_controller.sample_time = param_values.get("brake_pid.sample_time",
      self._brake_pid_controller.sample_time)
    self._brake_pid_controller.auto_mode = param_values.get("brake_pid.auto_mode",
      self._brake_pid_controller.auto_mode)
    self._brake_pid_controller.proportional_on_measurement = param_values.get("brake_pid.proportional_on_measurement",
      self._brake_pid_controller.proportional_on_measurement)

    self.get_logger().info(
      "Reconfigure Request:\nspeed_pid (kp: {}, kd: {}, ki: {}), sample_time: {}, auto_mode: {}, proportional_on_measurement: {}\n"\
      "brake_pid (kp: {}, kd: {}, ki: {}), sample_time: {}, auto_mode: {}, proportional_on_measurement: {}".format(
      self._speed_pid_controller.tunings[0], self._speed_pid_controller.tunings[1], self._speed_pid_controller.tunings[2],
      self._speed_pid_controller.sample_time, self._speed_pid_controller.auto_mode, self._speed_pid_controller.proportional_on_measurement,
      self._brake_pid_controller.tunings[0], self._brake_pid_controller.tunings[1], self._brake_pid_controller.tunings[2],
      self._brake_pid_controller.sample_time, self._brake_pid_controller.auto_mode, self._brake_pid_controller.proportional_on_measurement))

    return SetParametersResult(successful=True)

  def pid_control_loop(self):
    """ROS Timer callback"""

    t = self.get_clock().now()
    delta_t = (t - self._last_time).nanoseconds * 1e-9
    self._last_time = t

    # Too much time has elapsed since the last command. Stop the vehicle.
    if ((self._cmd_timeout.nanoseconds * 1e-9 > 0.0)\
        and t - self._last_cmd_time > self._cmd_timeout):
      self.get_logger().warn("Too much time has elapsed since the last command. Stop the vehicle.", \
        throttle_duration_sec=5.0)
      steer_ang_changed = self.ctrl_steering(self._last_steer_ang, 0.0, 0.001)
      self.ctrl_axles(0.0, 0.0, 0.0, 0.001, steer_ang_changed)
    elif delta_t > 0.0:
      with self._ackermann_cmd_lock:
        steer_ang = self._steer_ang
        steer_ang_vel = self._steer_ang_vel
        speed = self._speed
        accel = self._accel
        jerk = self._jerk
      steer_ang_changed = self.ctrl_steering(steer_ang, steer_ang_vel, delta_t)
      self.ctrl_axles(speed, accel, jerk, delta_t, steer_ang_changed)

    # Control steering
    self._final_steering_cmd = self._control_steer / self._vehicle_max_steering_angle

    self._brake_pid_controller.setpoint = abs(self._control_speed)
    brake_cmd = self._brake_pid_controller(abs(self._current_vehicle_speed))
    brake_cmd = self.clamp(brake_cmd, -self._max_brake, .0)

    self._speed_pid_controller.setpoint = abs(self._control_speed)
    throttle_cmd = self._speed_pid_controller(abs(self._current_vehicle_speed))
    throttle_cmd = self.clamp(throttle_cmd, .0, self._max_throttle)

    # Full stop: Full brake the vehicle
    if (abs(self._current_vehicle_speed) <= self._full_stop_epsilon\
        and abs(self._control_speed) <= self._full_stop_epsilon):
      self._final_throttle_cmd = 0.0
      # Can force to steer 0.0 in the control when vehicle is stopped.
      # self._final_steering_cmd = 0.0
      self._final_brake_cmd = 1.0
      self._final_handbrake_status = False
      self.get_logger().debug("Vehicle is stopping!!!!\nControl Speed: {}\nCurrent Speed: {}".format(
        self._control_speed, self._current_vehicle_speed))
    # Vehicle needs to brake when the current speed is higher than a threshold + the commanded speed...
    elif (abs(self._current_vehicle_speed) >= abs(self._control_speed) + self._brake_epsilon):
      self._final_throttle_cmd = 0.0
      # This is just a workaround to avoid low speed bug in Carla.
      # See: https://github.com/carla-simulator/carla/issues/3906
      if abs(self._current_vehicle_speed) >= 1.2:
        self._final_brake_cmd = abs(brake_cmd)
      else:
        self._final_brake_cmd = 0.0
      self._final_handbrake_status = False
      self.get_logger().debug("Vehicle needs to brake!!!!\nControl Speed: {}\n Current Speed: {}"\
        "\nCommand: {}\n Final: {}".format(
        self._control_speed, self._current_vehicle_speed, brake_cmd, self._final_brake_cmd))
    # Vehicle needs to accelerate to reach commanded speed...
    elif (abs(self._speed) > 0):
      self._final_throttle_cmd = throttle_cmd
      self._final_brake_cmd = 0.0
      self._final_handbrake_status = False
      self.get_logger().debug("Vehicle is accelerating!!!!\n Control Speed: {}\n Current Speed: {}"\
        "Final: {}".format(self._control_speed, self._current_vehicle_speed, self._final_throttle_cmd))
    else:
      # Vehicle needs to coast...
      self._final_throttle_cmd = 0.0
      self._final_brake_cmd = 0.01
      self._final_handbrake_status = False
      self.get_logger().debug("Vehicle is coasting!!!!\nControl Speed: {}\n Current Speed: ".format(
        self._control_speed, self._current_vehicle_speed))

    # Publish command
    control_cmd = CarlaVehicleControl()
    control_cmd.header.stamp = self.get_clock().now().to_msg()
    control_cmd.header.frame_id = self.get_name()

    control_cmd.throttle = self._final_throttle_cmd
    control_cmd.steer = self._final_steering_cmd
    control_cmd.brake = self._final_brake_cmd
    control_cmd.hand_brake = self._final_handbrake_status
    control_cmd.reverse = True if self._control_speed < 0 else False
    # These commands are supposed to be automatically set by Carla
    # control_cmd.gear = True
    # control_cmd.manual_gear_shift = 0

    self._vehicle_cmd_publisher_.publish(control_cmd)

  def ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
    """Control steering wheel angles

    :Parameters:
      steer_ang: Real steering angle
      steer_ang_vel_limit: Steering angle velocity
      delta_t: Time rate
    """

    # Limit steering angle
    steer_ang = self.clamp(steer_ang, -self._steer_limit, self._steer_limit)

    # Compute theta, the virtual front wheel desired steering angle.
    ang_vel = 0.0
    if steer_ang_vel_limit > 0.0:
      # Limit the steering velocity.
      ang_vel = (steer_ang - self._last_steer_ang) / delta_t
      ang_vel = max(-self._steer_vel_limit,
                min(ang_vel, self._steer_vel_limit))
      theta = self._last_steer_ang + ang_vel * delta_t
    else:
      theta = steer_ang

    self._control_steer = theta
    self._control_steer_vel = ang_vel

    # Compute the desired steering angles for the left and right front
    # wheels.
    steer_ang_changed = theta != self._last_steer_ang
    if steer_ang_changed:
      self._last_steer_ang = theta

    return steer_ang_changed

  def ctrl_axles(self, speed, accel_limit, jerk_limit, delta_t, steer_ang_changed):
    """Control drive wheel speed

    :Parameters:
      speed: Real speed to achieve (m/s)
      accel_limit: Acceleration limit (m/s2)
      jerk_limit: Jeck limit (m/s3)
      delta_t: Time rate
      steer_ang_changed: Rate of steering angle change
    """

    # Limit speed
    speed = self.clamp(speed, -self._speed_limit, self._speed_limit)

    # Compute veh_speed, the vehicle desired speed.
    if accel_limit > 0.0:
      # Limit the vehicle acceleration.
      if jerk_limit > 0.0:
        if self._last_accel_limit > 0.0:
          jerk = (accel_limit - self._last_accel_limit) / delta_t
          jerk = max(-jerk_limit, min(jerk, jerk_limit))
          accel_limit_2 = self._last_accel_limit + jerk * delta_t
        else:
          accel_limit_2 = accel_limit
      else:
        accel_limit_2 = accel_limit
      self._last_accel_limit = accel_limit_2

      accel = (speed - self._last_speed) / delta_t
      accel = max(-accel_limit_2, min(accel, accel_limit_2))
      veh_speed = self._last_speed + accel * delta_t
    else:
      self._last_accel_limit = accel_limit
      veh_speed = speed

    self._control_speed = veh_speed

    # Compute the desired angular velocities of the wheels.
    if veh_speed != self._last_speed or steer_ang_changed:
      self._last_speed = veh_speed

  def clamp(self, num, min_value, max_value):
    """Clamp a number between a range

    :Parameters:
      num: Number to clamp
      min_value: Min number on range
      max_value: Max number on range
    """
    return max(min(num, max_value), min_value)

  def ackermann_cmd_cb(self, ackermann_cmd):
    """Ackermann driving command callback

    :Parameters:
      ackermann_cmd : ackermann_msgs.msg.AckermannDriveStamped
        Ackermann driving command.
    """
    self._last_cmd_time = self.get_clock().now()
    with self._ackermann_cmd_lock:
      self._steer_ang = ackermann_cmd.drive.steering_angle
      self._steer_ang_vel = ackermann_cmd.drive.steering_angle_velocity
      self._speed = ackermann_cmd.drive.speed
      self._accel = self.clamp(abs(ackermann_cmd.drive.acceleration), 0.0, self._accel_limit)
      self._jerk = ackermann_cmd.drive.jerk

  def control_cb(self, control_msg):
    """Current vehicle control msg

    :Parameters:
      control_msg: vehicle_interfaces.msg.VehicleControl
        VehicleControl with current vehicle speed
    """
    self._current_vehicle_speed = control_msg.vehicle_speed.speed

def main(args=None):
  rclpy.init(args=args)
  try:
    ackermann_controller_node = CarlaAckermannNode()
    rclpy.spin(ackermann_controller_node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass

if __name__ == '__main__':
  main()
