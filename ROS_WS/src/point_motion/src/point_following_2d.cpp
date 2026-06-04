#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class ServoController : public rclcpp::Node
{
public:
  ServoController() : Node("servo_controller")
  {
    // ---------------- PARAMETERS ----------------
    this->declare_parameter<double>("max_vel", 0.2);
    this->declare_parameter<double>("gain", 0.5);

    max_vel_ = this->get_parameter("max_vel").as_double();
    K_ = this->get_parameter("gain").as_double();

    // ---------------- SUB / PUB ----------------
    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/object_pixel", 10,
      std::bind(&ServoController::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ServoController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Normalized visual servo controller started");
  }

private:

  void callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    u_ = msg->x;
    v_ = msg->y;
    confidence_ = msg->z;
    has_data_ = true;
  }

  void controlLoop()
  {
    if (!has_data_)
      return;

    // ---------------- NORMALIZED ERROR ----------------
    // image center assumed = (0,0)
    double ex = u_;   // already centered in vision node OR normalized
    double ey = v_;

    // dead zone
    if (std::abs(ex) < 0.05 && std::abs(ey) < 0.05)
      return;

    // confidence gate
    if (confidence_ < 5.0)
      return;

    // ---------------- CONTROL LAW ----------------
    double vx = -K_ * ex;
    double vy = -K_ * ey;

    // clamp
    vx = std::clamp(vx, -max_vel_, max_vel_);
    vy = std::clamp(vy, -max_vel_, max_vel_);

    // ---------------- COMMAND ----------------
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = 0.0;

    pub_->publish(cmd);
  }

  // ---------------- ROS ----------------
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- STATE ----------------
  double u_ = 0.0;
  double v_ = 0.0;
  double confidence_ = 0.0;

  bool has_data_ = false;

  double K_;
  double max_vel_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoController>());
  rclcpp::shutdown();
  return 0;
}