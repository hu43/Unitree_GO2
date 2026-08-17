/**
 * go2_cmd_vel_bridge_node
 *
 * Subscribes to geometry_msgs/Twist (e.g. the closed_loop_controller's /cmd_vel)
 * and forwards it to the Unitree Go2 via the official ROS 2 sport client
 * (publishes on /api/sport/request, pure rclcpp - no bundled DDS).
 *
 * Safety:
 *  - If no /cmd_vel is received within `cmd_timeout`, the robot is commanded to stop.
 *  - Velocities are clamped to [max_vx, max_vy, max_vyaw].
 *  - Motion is only active while `enable` is true (default true; use enable:=false
 *    for dry-run bring-up before hardware).
 *  - On shutdown the robot is put into Damp (or StandDown if stand_down_on_exit).
 */

#include <algorithm>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_api/msg/request.hpp>

#include "ros2_sport_client.h"

static std::atomic<bool> g_stop{false};
static void signal_handler(int) { g_stop.store(true); }

namespace
{

constexpr double kDefaultRate = 50.0;   // Hz
constexpr double kDefaultTimeout = 0.3; // s

class CmdVelBridge : public rclcpp::Node
{
public:
  CmdVelBridge() : Node("go2_cmd_vel_bridge"), sport_client_(this)
  {
    declare_parameter("enable", true);
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("cmd_timeout", kDefaultTimeout);
    declare_parameter("control_rate", kDefaultRate);
    declare_parameter("max_vx", 0.8);
    declare_parameter("max_vy", 0.4);
    declare_parameter("max_vyaw", 1.2);
    declare_parameter("stand_mode", 0);        // 0 = StandUp, 1 = BalanceStand
    declare_parameter("stand_down_on_exit", false);

    enable_ = get_parameter("enable").as_bool();
    timeout_ = get_parameter("cmd_timeout").as_double();
    rate_hz_ = get_parameter("control_rate").as_double();
    max_vx_ = get_parameter("max_vx").as_double();
    max_vy_ = get_parameter("max_vy").as_double();
    max_vyaw_ = get_parameter("max_vyaw").as_double();
    stand_mode_ = get_parameter("stand_mode").as_int();
    stand_down_ = get_parameter("stand_down_on_exit").as_bool();

    if (enable_)
    {
      // Take joystick control away so SDK commands are authoritative.
      sport_client_.SwitchJoystick(req_, false);

      if (stand_mode_ == 1)
      {
        sport_client_.BalanceStand(req_);
        RCLCPP_INFO(get_logger(), "BalanceStand issued");
      }
      else
      {
        sport_client_.StandUp(req_);
        RCLCPP_INFO(get_logger(), "StandUp issued");
      }
    }
    else
    {
      RCLCPP_WARN(get_logger(),
                  "enable=false: dry-run mode, connecting but not issuing any robot commands");
    }

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_topic").as_string(), rclcpp::QoS(1),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          last_cmd_ = *msg;
          last_cmd_time_ = now();
          has_cmd_ = true;
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        std::bind(&CmdVelBridge::tick, this));

    RCLCPP_INFO(get_logger(), "bridge ready: enable=%s timeout=%.2fs rate=%.1fHz",
                enable_ ? "true" : "false", timeout_, rate_hz_);
  }

  ~CmdVelBridge() override
  {
    if (stand_down_)
    {
      sport_client_.StandDown(req_);
    }
    else
    {
      sport_client_.Damp(req_);
    }
  }

private:
  void tick()
  {
    if (g_stop.load())
    {
      return;
    }

    // Dry-run: fully passive, never touch the robot.
    if (!enable_)
    {
      return;
    }

    double vx = 0.0, vy = 0.0, vyaw = 0.0;
    const bool fresh = has_cmd_ && (now() - last_cmd_time_).seconds() < timeout_;

    if (fresh)
    {
      vx = clamp(last_cmd_.linear.x, -max_vx_, max_vx_);
      vy = clamp(last_cmd_.linear.y, -max_vy_, max_vy_);
      vyaw = clamp(last_cmd_.angular.z, -max_vyaw_, max_vyaw_);
    }

    sport_client_.Move(req_, static_cast<float>(vx),
                       static_cast<float>(vy),
                       static_cast<float>(vyaw));

    if (!fresh)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "cmd_vel stale for %.1fs -> holding zero", timeout_);
    }
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  bool enable_ = true;
  bool stand_down_ = false;
  double timeout_ = kDefaultTimeout;
  double rate_hz_ = kDefaultRate;
  double max_vx_ = 0.8, max_vy_ = 0.4, max_vyaw_ = 1.2;
  int stand_mode_ = 0;

  SportClient sport_client_;
  unitree_api::msg::Request req_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  bool has_cmd_ = false;
};

} // namespace

int main(int argc, char **argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelBridge>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
