#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import sys
import select
import termios
import tty
import os

from numpy import clip
from time import sleep

from ackermann_msgs.msg import AckermannDriveStamped

import threading

class KeyboardThread(threading.Thread):
  """Class thread to read from keyboard without blocking"""
  def __init__(self, input_cbk = None, name='keyboard_input_thread'):
    self.input_cbk = input_cbk
    super(KeyboardThread, self).__init__(name=name)
    self.settings = termios.tcgetattr(sys.stdin)
    self.quit = False
    self.start()
    
  def get_key(self):
    """Get key from standard input"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    if key == '\x03' or key == '\x71':
      self.quit = True
    return key

  def run(self):
    """Run override method"""
    while not self.quit:
      self.input_cbk(self.get_key())

class CarlaKeyboardControlNode(Node):
  """Keyboard control node to publish Ackermann joy commands"""
  def __init__(self):
    super().__init__('carla_keyboard_control_node')
    # Keyboard inputs
    self._control_keys = {
      'up'         : '\x41',
      'down'       : '\x42',
      'right'      : '\x43',
      'left'       : '\x44',
      'steer_inc'  : '\x65', # e
      'steer_dec'  : '\x64', # d
      'acc_inc'    : '\x77', # w
      'acc_dec'    : '\x73', # s
      'space'      : '\x20',
      'tab'        : '\x09'
    }
    # Bindings
    self._key_bindings = {
      '\x41' : ( 1.0 , 0.0),
      '\x42' : (-1.0 , 0.0),
      '\x43' : ( 0.0 ,-1.0),
      '\x44' : ( 0.0 , 1.0),
      '\x20' : ( 0.0 , 0.0),
      '\x09' : ( 0.0 , 0.0),
      '\x65' : ( 0.0 , 0.0),
      '\x64' : ( 0.0 , 0.0),
      '\x77' : ( 0.0 , 0.0),
      '\x73' : ( 0.0 , 0.0)
    }

    # Declare this node params to use later
    self.declare_parameters(
      namespace='',
      parameters=[
        ('steer_teleop_limit', 1.22),
        ('speed_teleop_limit', 10.0),
        ('accel_teleop_limit', 1.5),
        ('angle_vel_teleop_limit', 0.5),
        ('ackermann_joy_topic', 'ackermann_joy')
      ]
    )

    max_steering_angle = self.get_parameter('steer_teleop_limit').get_parameter_value().double_value
    max_speed = self.get_parameter('speed_teleop_limit').get_parameter_value().double_value
    accel_limit = self.get_parameter('accel_teleop_limit').get_parameter_value().double_value
    ang_vel_limit = self.get_parameter('angle_vel_teleop_limit').get_parameter_value().double_value

    # Stablish variables, ranges and key control
    self.speed_range = [-float(max_speed), float(max_speed)]
    self.accel_range = [0.0, float(accel_limit)]
    self.ang_vel_range = [0.0, float(ang_vel_limit)]
    self.steering_angle_range = [-float(max_steering_angle), float(max_steering_angle)]
    for key in self._key_bindings:
      self._key_bindings[key] = \
        (self._key_bindings[key][0] * float(max_speed) / 10.0,
          self._key_bindings[key][1] * float(max_steering_angle) / 30.0)

    self.speed = .0
    self.steering_angle = .0
    self.accel = 0.3
    self.ang_vel = 0.1
    self.quit = False

    # ROS communication
    cmd_topic = self.get_parameter('ackermann_joy_topic').get_parameter_value().string_value
    self._vehicle_cmd_publisher = self.create_publisher(AckermannDriveStamped, cmd_topic, 1)

    self._kthread = KeyboardThread(self.parse_key)

  def parse_key(self, key):
    """Key selection to modify command outputs"""
    if key in self._key_bindings.keys():
      if key == self._control_keys['space']:
        self.speed = 0.0
        self.accel = self.accel_range[1]
      elif key == self._control_keys['tab']:
        self.steering_angle = 0.0
      elif key == self._control_keys['acc_inc']:
        self.accel += 0.1
        self.accel = clip(
          self.accel, self.accel_range[0], self.accel_range[1])
      elif key == self._control_keys['acc_dec']:
        self.accel -= 0.1
        self.accel = clip(
          self.accel, self.accel_range[0], self.accel_range[1])
      elif key == self._control_keys['steer_inc']:
        self.ang_vel +=0.1
        self.ang_vel = clip(
          self.ang_vel, self.ang_vel_range[0], self.ang_vel_range[1])
      elif key == self._control_keys['steer_dec']:
        self.ang_vel -=0.1
        self.ang_vel = clip(
          self.ang_vel, self.ang_vel_range[0], self.ang_vel_range[1])
      else:
        self.speed = self.speed + self._key_bindings[key][0]
        self.steering_angle = \
          self.steering_angle + self._key_bindings[key][1]
        self.speed = clip(
          self.speed, self.speed_range[0], self.speed_range[1])
        self.steering_angle = clip(
          self.steering_angle,
          self.steering_angle_range[0],
          self.steering_angle_range[1])
      self.print_state()
    elif key == '\x03' or key == '\x71':  # ctr-c or q
      self.quit = True
    else:
      pass

  def print_state(self):
    """Print status of the control every cycle."""
    os.system('clear')
    self.get_logger().info('\x1b[1M\r************' + self.get_name() + '************')
    self.get_logger().info('\x1b[1M\rUse arrows to change speed and steering angle')
    self.get_logger().info('\x1b[1M\rUse space to brake and tab to align wheels')
    self.get_logger().info('\x1b[1M\rUse w/s to increase/decrease acceleration')
    self.get_logger().info('\x1b[1M\rUse e/d to increase/decrease angular velocity')
    self.get_logger().info('\x1b[1M\rPress <ctrl-c> or <q> to exit')
    self.get_logger().info('\x1b[1M\r************' + self.get_name() + '************')
    self.get_logger().info('\x1b[1M\r\033[34;1mSpeed: \033[32;1m {:.2f} m/s, ' \
                           '\033[34;1mSteer Angle: \033[32;1m {:.2f} rad\033[0m'.format(
                           self.speed, self.steering_angle))
    self.get_logger().info('\x1b[1M\r'
                           '\033[34;1mAcceleration: \033[32;1m {:.2f} m/s2, '
                           '\033[34;1mSteer Angle Velocity: \033[32;1m {:.2f} rad/s\033[0m'.format(
                           self.accel, self.ang_vel)) 

  def finalize(self):
    """Final steps when shutting down the teleop controller"""
    self.get_logger().info('Halting motors, aligning wheels and exiting...')
    self.settings = termios.tcgetattr(sys.stdin)
    ackermann_cmd_msg = AckermannDriveStamped()
    ackermann_cmd_msg.drive.speed = .0
    ackermann_cmd_msg.drive.steering_angle = .0
    ackermann_cmd_msg.drive.steering_angle_velocity = .0
    ackermann_cmd_msg.drive.acceleration = .0
    ackermann_cmd_msg.drive.jerk = .0
    self._vehicle_cmd_publisher.publish(ackermann_cmd_msg)
    self._kthread.join()

  def spin(self):
    """ROS Spinning old school style"""
    self.print_state()
    while not self.quit:
      ackermann_cmd_msg = AckermannDriveStamped()
      ackermann_cmd_msg.header.stamp = self.get_clock().now().to_msg()
      ackermann_cmd_msg.header.frame_id = "joy_controller"
      ackermann_cmd_msg.drive.speed = self.speed
      ackermann_cmd_msg.drive.steering_angle = self.steering_angle
      ackermann_cmd_msg.drive.steering_angle_velocity = self.ang_vel
      ackermann_cmd_msg.drive.acceleration = self.accel
      ackermann_cmd_msg.drive.jerk = 0.1
      self._vehicle_cmd_publisher.publish(ackermann_cmd_msg)
      sleep(0.05)

    self.finalize()

def main(args=None):
  rclpy.init(args=args)

  keyboard_controller_node = CarlaKeyboardControlNode()
  keyboard_controller_node.spin()

  keyboard_controller_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
