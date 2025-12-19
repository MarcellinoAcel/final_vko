#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include "icecream.hpp"

class OdomSubscriber : public rclcpp::Node
{
public:
  OdomSubscriber() : Node("odom_subscriber")
  {
    smoothvel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_joy", 10);
    joy_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "vel_to_smooth", 10,
        std::bind(&OdomSubscriber::cmd_joy_callback, this, std::placeholders::_1));
  }

  struct accel
  {
    double prev_vel;

    double convert2accel(double velocity, double deltaT)
    {
      double acceleration = 0.0;

      double deltaV = velocity - prev_vel;
      if (deltaT != 0)
      {
        acceleration = deltaV / deltaT;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("accel_calculator"), "Warning: Time interval is zero!");
      }

      prev_vel = velocity;

      return acceleration;
    }
  };

  void calculate_smooth_vel(double &current_output, double input_value, double deltaT)
  {
    double target = input_value;

    if (input_value == 0.0 && std::abs(current_output) > 1e-6)
    {
      target = 0.0;
    }
    double max_delta = max_acceleration_ * deltaT;

    double delta_target = target - current_output;

    if (std::abs(delta_target) > max_delta)
    {
      delta_target = (delta_target > 0.0) ? max_delta : -max_delta;
    }

    current_output += delta_target;
  }
  void cmd_joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    joy.x = msg->linear.x;
    joy.y = msg->linear.y;
    joy.head = msg->angular.z;
    double current_time = this->now().seconds();
    double deltaT = current_time - prev_time;
    prev_time = current_time;

    if (deltaT <= 1e-6)
    {
      return;
    }

    // auto vel = msg->twist.twist.linear;
    // double vel_xy = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    // double vel_head = atan2(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    float xy = sqrt(pow(joy.x, 2) + pow(joy.y, 2));
    float head = atan2(joy.x, joy.y);

    calculate_smooth_vel(outspeed_x, joy.x, deltaT);
    calculate_smooth_vel(outspeed_y, joy.y, deltaT);

    calculate_smooth_vel(out_xy, xy, deltaT);
    calculate_smooth_vel(out_head, head, deltaT);
    geometry_msgs::msg::Twist smoothed_vel;
    smoothed_vel.linear.x = outspeed_x;
    smoothed_vel.linear.y = outspeed_y;
    ;
    smoothed_vel.angular.z = joy.head;

    smoothvel_pub->publish(smoothed_vel);
    IC(smoothed_vel.linear.x, smoothed_vel.linear.y, smoothed_vel.angular.z, out_xy, out_head, joy.x, joy.y);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr smoothvel_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub;
  accel accel_x_calculator_;
  accel accel_y_calculator_;

  double prev_time = 0.0;
  double outspeed_x = 0.0;
  double outspeed_y = 0.0;
  double out_xy = 0.0;
  double out_head = 0.0;
  double max_acceleration_ = 1.5;
  struct joystick
  {
    double x = 0.0;
    double y = 0.0;
    double head = 0.0;
  } joy;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}