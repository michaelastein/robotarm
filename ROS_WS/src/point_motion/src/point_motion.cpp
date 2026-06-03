#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // =========================
  // BASIC SETUP
  // =========================
  move_group_interface.setPoseReferenceFrame("base_link");
  move_group_interface.setEndEffectorLink("end_effector_link");

  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setNumPlanningAttempts(20);

  // =========================
  // POSITION-ONLY TARGET (IMPORTANT FIX)
  // =========================
  double x = 0.199;
  double y = 0.034;
  double z = 0.304;

  move_group_interface.setPositionTarget(x, y, z);

  // relax constraints (important for 4DOF arms)
  move_group_interface.setGoalPositionTolerance(0.01);
  move_group_interface.setGoalOrientationTolerance(3.14);

  // =========================
  // PLANNING
  // =========================
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success =
    (move_group_interface.plan(plan)
     == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful. Executing...");
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}