#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class FrontAxisTransformerNode : public rclcpp::Node
{
  public:
    FrontAxisTransformerNode();

   private:
    void filtered_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);

    std::string base_frame_id_, front_frame_id_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odometry_front_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odometry_sub_;
    
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
