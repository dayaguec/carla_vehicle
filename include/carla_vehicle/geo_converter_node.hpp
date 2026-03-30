#include <ad/map/point/Operation.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using namespace ::ad;
using namespace ::ad::map;

class GeoConverterNode : public rclcpp::Node
{
  public:
    GeoConverterNode();

   private:
    void gps_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix) const;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odometry_pub_;

    std::string tf_parent_, tf_child_;
    double first_altitude_, rot_cov_;

    point::CoordinateTransform coordinate_transform_;
};