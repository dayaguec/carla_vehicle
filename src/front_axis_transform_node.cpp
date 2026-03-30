#include <carla_vehicle/front_axis_transform_node.hpp>

FrontAxisTransformerNode::FrontAxisTransformerNode ()
  : Node("front_axis_transform_node")
{
  this->declare_parameter<std::string>("selected_odometry_topic", "filtered_odometry_front");
  this->declare_parameter<std::string>("filtered_odometry_topic", "filtered_odometry");
  this->declare_parameter<std::string>("odom_frame", "ego/odom");
  this->declare_parameter<std::string>("front_wheel_frame", "ego/front_wheel_axis");

  std::string selected_odometry_topic = this->get_parameter("selected_odometry_topic").as_string();
  std::string filtered_odometry_topic = this->get_parameter("filtered_odometry_topic").as_string();
  base_frame_id_ = this->get_parameter("odom_frame").as_string();
  front_frame_id_ = this->get_parameter("front_wheel_frame").as_string();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  filtered_odometry_front_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    selected_odometry_topic, 1);
  filtered_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    filtered_odometry_topic, 1, std::bind(
      &FrontAxisTransformerNode::filtered_odometry_callback, this, _1));
}

void FrontAxisTransformerNode::filtered_odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr odometry_msg)
{
  try
  {
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(
        base_frame_id_, front_frame_id_, tf2::TimePointZero);
    nav_msgs::msg::Odometry odometry_front = *odometry_msg;
    odometry_front.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
    odometry_front.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
    odometry_front.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
    odometry_front.pose.pose.orientation.w = transform_stamped.transform.rotation.w;
    odometry_front.pose.pose.position.x = transform_stamped.transform.translation.x;
    odometry_front.pose.pose.position.y = transform_stamped.transform.translation.y;
    odometry_front.pose.pose.position.z = transform_stamped.transform.translation.z;
    odometry_front.header.stamp = this->now();
    odometry_front.header.frame_id = base_frame_id_;
    odometry_front.child_frame_id = front_frame_id_;

    RCLCPP_DEBUG(this->get_logger(), "Received transform from %s to %s.",
      base_frame_id_.c_str(), front_frame_id_.c_str());

    filtered_odometry_front_pub_->publish(odometry_front);
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      base_frame_id_.c_str(), front_frame_id_.c_str(), ex.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontAxisTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
