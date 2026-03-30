import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from awareness_interfaces.msg import (AwarenessEvent, SystemStatus)
from std_msgs.msg import Header

class AwarenessSimNode(Node):

  def __init__(self):
    super().__init__('awareness_sim_node')
    # Declare this node params to use later
    self.declare_parameters(
      namespace='',
      parameters=[
        ('sensor_status', SystemStatus.SUCCESS),
        ('localization_status', SystemStatus.SUCCESS),
        ('perception_status', SystemStatus.SUCCESS),
        ('planning_status', SystemStatus.SUCCESS),
        ('lowlevel_status', SystemStatus.SUCCESS),
        ('awareness_topic', 'self_awareness')
      ]
    )

    awareness_topic = self.get_parameter('awareness_topic').get_parameter_value().string_value
    self._awareness_pub = self.create_publisher(AwarenessEvent, awareness_topic, 1)

    self._sensor_status = self.get_parameter('sensor_status').get_parameter_value().integer_value
    self._localization_status = self.get_parameter('localization_status').get_parameter_value().integer_value
    self._perception_status = self.get_parameter('perception_status').get_parameter_value().integer_value
    self._planning_status = self.get_parameter('planning_status').get_parameter_value().integer_value
    self._lowlevel_status = self.get_parameter('lowlevel_status').get_parameter_value().integer_value

    self._awareness_msg = AwarenessEvent(header=Header(frame_id="map", stamp=self.get_clock().now().to_msg()))
    for ii in range(5):
      sys_status = SystemStatus()
      sys_status.status = SystemStatus.SUCCESS
      sys_status.system_involved = self.get_system(ii)
      self._awareness_msg.status.append(sys_status)

    self.add_on_set_parameters_callback(self.cb_params)

    self.timer = self.create_timer(0.2, self.timer_callback)

  def cb_params(self, data):
    for parameter in data:
      if parameter.name == "sensor_status":
        if parameter.type_ == Parameter.Type.INTEGER:
          self._sensor_status = parameter.value
          self._awareness_msg.status[0].status = self.get_status(self._sensor_status)
          self.get_logger().info("[SENSOR] parameter changed... {}".format(
            self.get_string_status(self._sensor_status)))
      elif parameter.name == "localization_status": 
        if parameter.type_ == Parameter.Type.INTEGER:
          self._localization_status = parameter.value
          self._awareness_msg.status[1].status = self.get_status(self._localization_status)
          self.get_logger().info("[LOCALIZATION] parameter changed... {}".format(
            self.get_string_status(self._localization_status)))
      elif parameter.name == "perception_status": 
        if parameter.type_ == Parameter.Type.INTEGER:
          self._perception_status = parameter.value
          self._awareness_msg.status[2].status = self.get_status(self._perception_status)
          self.get_logger().info("[PERCEPTION] parameter changed... {}".format(
            self.get_string_status(self._perception_status)))
      elif parameter.name == "planning_status": 
        if parameter.type_ == Parameter.Type.INTEGER:
          self._planning_status = parameter.value
          self._awareness_msg.status[3].status = self.get_status(self._planning_status)
          self.get_logger().info("[PLANNING] parameter changed... {}".format(
            self.get_string_status(self._planning_status)))
      elif parameter.name == "lowlevel_status": 
        if parameter.type_ == Parameter.Type.INTEGER:
          self._lowlevel_status = parameter.value
          self._awareness_msg.status[4].status = self.get_status(self._lowlevel_status)
          self.get_logger().info("[LOWLEVEL] parameter changed... {}".format(
            self.get_string_status(self._lowlevel_status)))
    for item in self._awareness_msg.status:
      if item.status == SystemStatus.WAITING\
         or item.status == SystemStatus.IDLE\
         or item.status == SystemStatus.FAILURE:
        item.nodes_involved = ["node_placeholder_1", "node_placeholder_2"]
      else:
        item.nodes_involved = []
    return SetParametersResult(successful=True)

  def get_system(self, system):
    if system == 0:
      return SystemStatus.SENSOR
    elif system == 1:
      return SystemStatus.LOCALIZATION
    elif system == 2:
      return SystemStatus.PERCEPTION
    elif system == 3:
      return SystemStatus.PLANNING
    elif system == 4:
      return SystemStatus.LOWLEVEL
    else:
      return SystemStatus.UNKNOWN

  def get_status(self, status):
    if status == 0:
      return SystemStatus.FAILURE
    elif status == 1:
      return SystemStatus.SUCCESS
    elif status == 2:
      return SystemStatus.WAITING
    elif status == 3:
      return SystemStatus.IDLE
    else:
      return SystemStatus.UNKNOWN

  def get_string_status(self, status):
    if status == SystemStatus.FAILURE:
      return "FAILURE"
    elif status == SystemStatus.SUCCESS:
      return "SUCCESS"
    elif status == SystemStatus.WAITING:
      return "WAITING"
    elif status == SystemStatus.IDLE:
      return "IDLE"
    else:
      return "UNKNOWN"

  def timer_callback(self):
    self._awareness_msg.header.stamp = self.get_clock().now().to_msg()
    self._awareness_pub.publish(self._awareness_msg)

def main(args=None):
  rclpy.init(args=args)
  awareness_sim_node = AwarenessSimNode()
  rclpy.spin(awareness_sim_node)
  awareness_sim_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
