#include <memory>
#include <iostream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "reactive_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("reactive_moveit");

  using moveit::planning_interface::MoveGroupInterface;

  MoveGroupInterface move_group(node, "arm");

  move_group.setPoseReferenceFrame("base_link");
  move_group.setEndEffectorLink("end_effector_link");

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);

  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.1);

  RCLCPP_INFO(logger, "Reactive mode enabled.");
  RCLCPP_INFO(logger, "Enter x y z. Robot will interrupt current motion and go to new target.");
  RCLCPP_INFO(logger, "Type 'q' to quit.");

  std::string line;

  while (rclcpp::ok())
  {
    std::cout << "\nTarget (x y z): ";
    std::getline(std::cin, line);

    if (!std::cin || line == "q" || line == "quit")
      break;

    std::stringstream ss(line);

    double x, y, z;
    if (!(ss >> x >> y >> z))
    {
      std::cout << "Invalid input\n";
      continue;
    }

    // 🚨 CRITICAL: cancel current motion immediately
    move_group.stop();

    // remove old goals
    move_group.clearPoseTargets();

    // set new target
    move_group.setPositionTarget(x, y, z);

    MoveGroupInterface::Plan plan;

    auto result = move_group.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "New path computed. Executing...");

      // async = allows interruption by next input
      move_group.asyncExecute(plan);
    }
    else
    {
      RCLCPP_WARN(logger, "No path possible.");
    }
  }

  // stop robot on exit
  move_group.stop();

  rclcpp::shutdown();
  return 0;
}