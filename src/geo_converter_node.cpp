#include <carla_vehicle/geo_converter_node.hpp>

GeoConverterNode::GeoConverterNode ()
  : Node("geo_converter_node")
{
  this->declare_parameter<std::string>("odom_frame", "ego/odom");
  this->declare_parameter<std::string>("footprint_frame", "ego/base_footprint");
  this->declare_parameter<std::string>("geo_projection", "+proj=tmerc +lat_0=40.354550084445 +lon_0=-3.7463664011244586 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs");
  this->declare_parameter<std::string>("gps_topic", "ego/gps/fix");
  this->declare_parameter<std::string>("gps_local_odometry_topic", "ego/gps_local_odometry");
  this->declare_parameter<double>("first_altitude", 0.0);
  this->declare_parameter<double>("rot_covariance", 99999.0);
    
  tf_parent_ = this->get_parameter("odom_frame").as_string();
  tf_child_ = this->get_parameter("footprint_frame").as_string();
  std::string geo_projection = this->get_parameter("geo_projection").as_string();
  std::string gps_topic = this->get_parameter("gps_topic").as_string();
  std::string local_odometry_topic = this->get_parameter("gps_local_odometry_topic").as_string();

  first_altitude_ = this->get_parameter("first_altitude").as_double();
  rot_cov_ = this->get_parameter("rot_covariance").as_double();

  if(!coordinate_transform_.setGeoProjection(geo_projection))
  {
    RCLCPP_ERROR(this->get_logger(), "Projection string is not valid...");
    rclcpp::shutdown();
  }

  using std::placeholders::_1;
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, 1, std::bind(&GeoConverterNode::gps_fix_callback, this, _1));
  local_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(local_odometry_topic, 1);
}

void GeoConverterNode::gps_fix_callback(
  const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix) const
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = gps_fix->header.stamp;
  odom_msg.header.frame_id = tf_parent_;
  odom_msg.child_frame_id = tf_child_;

  double altitude = gps_fix->altitude - first_altitude_;
  auto geo = point::createGeoPoint(point::Longitude(gps_fix->longitude),
    point::Latitude(gps_fix->latitude),
    point::Altitude(altitude));

  auto ENU_pt = coordinate_transform_.Geo2ENU(geo);

  odom_msg.pose.pose.position.x = ENU_pt.x;
  odom_msg.pose.pose.position.y = -ENU_pt.y;
  odom_msg.pose.pose.position.z = ENU_pt.z;
  
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = 0;
  odom_msg.pose.pose.orientation.w = 1;

  odom_msg.pose.covariance = {gps_fix->position_covariance[0], gps_fix->position_covariance[1], gps_fix->position_covariance[2],
                              0, 0, 0,
                              gps_fix->position_covariance[3],gps_fix->position_covariance[4], gps_fix->position_covariance[5],
                              0, 0, 0,
                              gps_fix->position_covariance[6], gps_fix->position_covariance[7], gps_fix->position_covariance[8],
                              0, 0, 0,
                              0, 0, 0, rot_cov_, 0, 0,
                              0, 0, 0, 0, rot_cov_, 0,
                              0, 0, 0, 0, 0, rot_cov_};
  local_odometry_pub_->publish(odom_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoConverterNode>());
  rclcpp::shutdown();
  return 0;
}