/**
 * odom_tf_broadcaster
 *
 * The Go2 X onboard publishes /utlidar/robot_odom (odom -> base_link) as a
 * nav_msgs/Odometry *message* but does not broadcast the TF tree. RViz (and
 * anything doing TF lookups) needs the odom -> base_link transform.
 *
 * This node re-broadcasts that odometry as a TF so the point cloud
 * (/utlidar/cloud_base, frame base_link) can be displayed in the odom frame.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTfBroadcaster : public rclcpp::Node
{
public:
  OdomTfBroadcaster() : Node("odom_tf_broadcaster")
  {
    declare_parameter("odom_topic", "/utlidar/robot_odom");
    declare_parameter("odom_frame", "odom");
    declare_parameter("child_frame", "base_link");
    declare_parameter("publish_rate", 50.0);

    odom_frame_ = get_parameter("odom_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { latest_ = msg; });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / get_parameter("publish_rate").as_double()),
        std::bind(&OdomTfBroadcaster::tick, this));

    RCLCPP_INFO(get_logger(), "broadcasting TF %s -> %s from %s",
                odom_frame_.c_str(), child_frame_.c_str(),
                get_parameter("odom_topic").as_string().c_str());
  }

private:
  void tick()
  {
    if (!latest_)
      return;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = latest_->header.stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = child_frame_;

    const auto &p = latest_->pose.pose.position;
    const auto &o = latest_->pose.pose.orientation;
    t.transform.translation.x = p.x;
    t.transform.translation.y = p.y;
    t.transform.translation.z = p.z;
    t.transform.rotation = o;

    tf_broadcaster_->sendTransform(t);
  }

  std::string odom_frame_;
  std::string child_frame_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry::SharedPtr latest_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
