#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
    // 1) Parameters: name + default value, stored in member variables
    linear_speed_ = declare_parameter("linear_speed", 0.1);
    angular_gain_ = declare_parameter("angular_gain", 1.0);
    wheel_track_ = declare_parameter("wheel_track", 0.287);
    max_wheel_speed_ = declare_parameter("max_wheel_speed", 0.26);
    line_timeout_ = declare_parameter("line_timeout", 0.5);
    // 2) Publisher: Twist -> /cmd_vel
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // 3) Subscription: Float32 <- /line_error (only remember value + time)
    error_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/line_error", 10,
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        last_error_ = msg->data;
        last_error_time_ = now();
        has_error_ = true;
      });
    // 4) Timer: run control_loop() every 50 ms (20 Hz)
    timer_ = create_wall_timer(50ms, [this]() {control_loop();});
    RCLCPP_INFO(get_logger(), "control_node started");
  }
private:
  void control_loop()
  {
    // Empty Twist = all zeros = stop
    geometry_msgs::msg::Twist twist;
    // Line lost: never received, or last value too old -> stop
    const bool line_lost =
      !has_error_ || (now() - last_error_time_).seconds() > line_timeout_;
    if (line_lost) {
      cmd_pub_->publish(twist);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "line lost, stopping");
      return;
    }
    // Line on the right (error > 0) -> turn right (omega < 0)
    const double v = linear_speed_;
    const double omega = -angular_gain_ * last_error_;
    // Differential-drive: wheel speeds
    double v_right = v + (omega * wheel_track_) / 2.0;
    double v_left = v - (omega * wheel_track_) / 2.0;

    // Scale both wheels by the same factor to keep the turning radius
    const double fastest = std::max(std::abs(v_right), std::abs(v_left));
    if (fastest > max_wheel_speed_) {
      const double scale = max_wheel_speed_ / fastest;
      v_right *= scale;
      v_left *= scale;
    }
    // Back to (v, omega) for /cmd_vel
    twist.linear.x = (v_right + v_left) / 2.0;
    twist.angular.z = (v_right - v_left) / wheel_track_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "error=%.3f omega=%.3f v_L=%.3f v_R=%.3f",
      last_error_, omega, v_left, v_right);

    cmd_pub_->publish(twist);  // step 4a: numbers only, robot does not move
  }

  // Parameters
  double linear_speed_;
  double angular_gain_;
  double wheel_track_;
  double max_wheel_speed_;
  double line_timeout_;
  // Latest data from vision_node
  double last_error_{0.0};
  rclcpp::Time last_error_time_;
  bool has_error_{false};
  // ROS handles
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr error_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
  

