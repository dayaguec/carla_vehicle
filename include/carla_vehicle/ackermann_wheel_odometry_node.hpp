#include <rclcpp/rclcpp.hpp>
#include <vehicle_interfaces/msg/vehicle_control.hpp>
#include <vehicle_interfaces/msg/vehicle_wheel_distance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class AckermannWheelOdometryNode : public rclcpp::Node
{
  public:
    AckermannWheelOdometryNode();

   private:
    void timer_callback();

    rclcpp::Subscription<vehicle_interfaces::msg::VehicleControl>::SharedPtr current_control_sub_;
    void current_control_callback(
      const vehicle_interfaces::msg::VehicleControl::SharedPtr current_control_msg);

    std::string odom_frame_, footprint_frame_;
    double wheel_base_, vehicle_steering_, vehicle_speed_, x_, y_, yaw_, longitudinal_acc_dist_,
           covariance_;

    rclcpp::Time time_, last_time_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
    rclcpp::Publisher<vehicle_interfaces::msg::VehicleWheelDistance>::SharedPtr wheel_dist_pub_;
};
