#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

using std::placeholders::_1;

class VisualServo : public rclcpp::Node
{
public:
  VisualServo()
  : Node("visual_servo")
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/object_pixel", 10,
      std::bind(&VisualServo::callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VisualServo::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Node started.");
  }

  void initMoveIt()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");

    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setEndEffectorLink("end_effector_link");

    move_group_->setPlanningTime(0.5);
    move_group_->setNumPlanningAttempts(2);

    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.1);

    RCLCPP_INFO(this->get_logger(), "MoveIt initialized.");
  }

private:

  void callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    x_ = msg->x;
    y_ = msg->y;
    conf_ = msg->z;
    has_data_ = true;
  }

  void controlLoop()
  {
    if (!move_group_) {
      // initialize once when TF + MoveIt ready
      initMoveIt();
      return;
    }

    if (!has_data_ || conf_ < 5.0)
      return;

    if (std::abs(x_) < 0.05 && std::abs(y_) < 0.05)
      return;

    RCLCPP_INFO(this->get_logger(),
      "x=%.3f y=%.3f conf=%.2f",
      x_, y_, conf_);

    auto pose = move_group_->getCurrentPose().pose;

    const double gain = 0.05;

    pose.position.y -= x_ * gain;
    pose.position.z += y_ * gain;

    move_group_->stop();
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group_->asyncExecute(plan);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Planning failed");
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  double x_{0.0}, y_{0.0}, conf_{0.0};
  bool has_data_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VisualServo>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
