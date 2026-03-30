#include <carla_vehicle/ackermann_wheel_odometry_node.hpp>

AckermannWheelOdometryNode::AckermannWheelOdometryNode ()
  : Node("ackermann_wheel_odometry_node"),
    vehicle_steering_(0.0),
    vehicle_speed_(0.0),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    longitudinal_acc_dist_(0.0)
{
  this->declare_parameter<std::string>("vehicle_control_topic", "vehicle_control");
  this->declare_parameter<std::string>("wheel_odometry_topic", "wheel_odometry");
  this->declare_parameter<std::string>("wheel_distance_topic", "wheel_distance");
  this->declare_parameter<std::string>("footprint_frame", "front_wheel_axis");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<double>("node_rate", 50.0);
  this->declare_parameter<double>("wheel_base", 1.42);
  this->declare_parameter<double>("root_covariance", 0.05);

  std::string vehicle_control_topic = this->get_parameter("vehicle_control_topic").as_string();
  std::string wheel_odometry_topic = this->get_parameter("wheel_odometry_topic").as_string();
  std::string wheel_distance_topic = this->get_parameter("wheel_distance_topic").as_string();

  odom_frame_ = this->get_parameter("odom_frame").as_string();
  footprint_frame_ = this->get_parameter("footprint_frame").as_string();

  double node_rate = this->get_parameter("node_rate").as_double();
  wheel_base_ = this->get_parameter("wheel_base").as_double();
  covariance_ = this->get_parameter("root_covariance").as_double();

  using std::placeholders::_1;

  current_control_sub_ = this->create_subscription<vehicle_interfaces::msg::VehicleControl>(
    vehicle_control_topic, 1, std::bind(&AckermannWheelOdometryNode::current_control_callback, this, _1));

  wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odometry_topic, 1);
  wheel_dist_pub_ = this->create_publisher<vehicle_interfaces::msg::VehicleWheelDistance>(wheel_distance_topic, 1);

  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/node_rate),
    std::bind(&AckermannWheelOdometryNode::timer_callback, this));

  time_ = this->now();
  last_time_ = this->now();
}

void AckermannWheelOdometryNode::timer_callback()
{
  auto dt = (time_ - last_time_).seconds();
  double vth = (vehicle_speed_ / wheel_base_) * std::tan(vehicle_steering_);
  yaw_ += vth * dt;
  x_ += vehicle_speed_ * std::cos(yaw_) * dt;
  y_ += vehicle_speed_ * std::sin(yaw_) * dt;

  last_time_ = time_;

  nav_msgs::msg::Odometry wheel_odometry;
  wheel_odometry.header.stamp = this->now();
  wheel_odometry.header.frame_id = odom_frame_;
  wheel_odometry.child_frame_id = footprint_frame_;
  wheel_odometry.pose.pose.position.x = x_;
  wheel_odometry.pose.pose.position.y = y_;
  wheel_odometry.pose.pose.position.z = 0.0;
  wheel_odometry.twist.twist.linear.x = vehicle_speed_;
  wheel_odometry.twist.twist.angular.z = vth;
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, yaw_);
  wheel_odometry.pose.pose.orientation = tf2::toMsg(tf2_quat);

  wheel_odometry.pose.covariance = {covariance_, 0, 0, 0, 0, 0,
                                    0, covariance_, 0, 0, 0, 0,
                                    0, 0, covariance_, 0, 0, 0,
                                    0, 0, 0, covariance_, 0, 0,
                                    0, 0, 0, 0, covariance_, 0,
                                    0, 0, 0, 0, 0, covariance_};

  wheel_odometry.twist.covariance = {0.001, 0, 0, 0, 0, 0,
                                     0, 0.0001, 0, 0, 0, 0,
                                     0, 0, 0.001, 0, 0, 0,
                                     0, 0, 0, 0.01, 0, 0,
                                     0, 0, 0, 0, 0.01, 0,
                                     0, 0, 0, 0, 0, 0.03};
  longitudinal_acc_dist_ = longitudinal_acc_dist_ + (vehicle_speed_ * dt);

  vehicle_interfaces::msg::VehicleWheelDistance wheel_distance;
  wheel_distance.header = wheel_odometry.header;
  wheel_distance.distance = longitudinal_acc_dist_;

  wheel_odom_pub_->publish(wheel_odometry);
  wheel_dist_pub_->publish(wheel_distance);
}

/**
 * @brief Current Vehicle control callback provided by the low level
 * @param current_control_msg: VehicleControl containing the speed in m/s
 */
void AckermannWheelOdometryNode::current_control_callback(
  const vehicle_interfaces::msg::VehicleControl::SharedPtr current_control_msg)
{
  vehicle_steering_ = current_control_msg->vehicle_steering.steering;
  vehicle_speed_ = current_control_msg->vehicle_speed.speed;
  time_ = current_control_msg->header.stamp;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannWheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}