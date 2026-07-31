#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


using namespace std::chrono_literals;


/*
 * These names must match robotarm_moveit_config.
 */
static constexpr char PLANNING_GROUP[] = "arm";
static constexpr char TOOL_LINK[] = "tool_tip_link";
static constexpr char HOTSPOT_TOPIC[] = "/hotspot/target";


class HotspotPathPlanner : public rclcpp::Node
{
public:
  HotspotPathPlanner()
  : Node(
      "hotspot_path_planner",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true))
  {
    hotspot_subscription_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
        HOTSPOT_TOPIC,
        10,
        std::bind(
          &HotspotPathPlanner::hotspot_callback,
          this,
          std::placeholders::_1));

    /*
     * Do not plan at camera frequency.
     * Check for a new correction twice per second.
     */
    planning_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    planning_timer_ = create_wall_timer(
      500ms,
      std::bind(
        &HotspotPathPlanner::planning_timer_callback,
        this),
      planning_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Hotspot path planner node created");

    RCLCPP_INFO(
      get_logger(),
      "Listening to %s",
      HOTSPOT_TOPIC);
  }


  void initialize_move_group()
  {
    move_group_ =
      std::make_shared<
        moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        PLANNING_GROUP);

    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(5);

    /*
     * Position-only goal tolerance.
     */
    move_group_->setGoalPositionTolerance(0.008);

    /*
     * Start carefully on the physical robot.
     */
    move_group_->setMaxVelocityScalingFactor(0.08);
    move_group_->setMaxAccelerationScalingFactor(0.05);

    const std::string detected_tool_link =
      move_group_->getEndEffectorLink();

    RCLCPP_INFO(
      get_logger(),
      "MoveIt planning group: %s",
      PLANNING_GROUP);

    RCLCPP_INFO(
      get_logger(),
      "Configured tool link: %s",
      TOOL_LINK);

    RCLCPP_INFO(
      get_logger(),
      "MoveIt end-effector link: %s",
      detected_tool_link.c_str());

    initialize_height();
  }


private:
  /*
   * Detector message:
   *
   * data[0] = visible
   * data[1] = normalized image error X
   * data[2] = normalized image error Y
   * data[3] = confidence
   */
  void hotspot_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Invalid hotspot message: expected four values");

      return;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);

    target_visible_ = msg->data[0] >= 0.5F;
    error_x_ = static_cast<double>(msg->data[1]);
    error_y_ = static_cast<double>(msg->data[2]);
    confidence_ = static_cast<double>(msg->data[3]);

    last_target_time_ = now();
    have_target_message_ = true;
  }


  void initialize_height()
  {
    try {
      const geometry_msgs::msg::PoseStamped current_pose =
        move_group_->getCurrentPose(TOOL_LINK);

      initial_height_ = current_pose.pose.position.z;
      height_initialized_ = true;

      RCLCPP_WARN(
        get_logger(),
        "Initial tool height saved: z=%+.4f m",
        initial_height_);

      RCLCPP_INFO(
        get_logger(),
        "Initial tool position: x=%+.4f y=%+.4f z=%+.4f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z);
    } catch (const std::exception & exception) {
      height_initialized_ = false;

      RCLCPP_ERROR(
        get_logger(),
        "Could not initialize tool height: %s",
        exception.what());
    }
  }


  bool read_target(
    bool & visible,
    double & error_x,
    double & error_y,
    double & confidence,
    rclcpp::Time & target_time)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);

    if (!have_target_message_) {
      return false;
    }

    visible = target_visible_;
    error_x = error_x_;
    error_y = error_y_;
    confidence = confidence_;
    target_time = last_target_time_;

    return true;
  }


  void planning_timer_callback()
  {
    if (!move_group_) {
      return;
    }

    if (planning_or_executing_) {
      return;
    }

    if (!height_initialized_) {
      initialize_height();

      if (!height_initialized_) {
        return;
      }
    }

    bool visible = false;
    double error_x = 0.0;
    double error_y = 0.0;
    double confidence = 0.0;
    rclcpp::Time target_time(0, 0, get_clock()->get_clock_type());

    if (!read_target(
        visible,
        error_x,
        error_y,
        confidence,
        target_time))
    {
      return;
    }

    const double target_age =
      (now() - target_time).seconds();

    if (
      !visible ||
      target_age > TARGET_TIMEOUT_SECONDS ||
      confidence < MIN_CONFIDENCE)
    {
      return;
    }

    const bool x_centered =
      std::abs(error_x) <= IMAGE_DEADBAND_X;

    const bool y_centered =
      std::abs(error_y) <= IMAGE_DEADBAND_Y;

    if (x_centered && y_centered) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Hotspot centered: error=(%+.3f, %+.3f)",
        error_x,
        error_y);

      return;
    }

    planning_or_executing_ = true;

    plan_and_execute_correction(
      error_x,
      error_y);

    planning_or_executing_ = false;
  }


  void plan_and_execute_correction(
    double error_x,
    double error_y)
  {
    try {
      move_group_->setStartStateToCurrentState();

      const geometry_msgs::msg::PoseStamped current_pose =
        move_group_->getCurrentPose(TOOL_LINK);

      /*
       * Mapping matches the existing hotspot_servo.py:
       *
       * err_x positive:
       *   hotspot right
       *   move tool toward negative base Y
       *
       * err_y positive:
       *   hotspot lower
       *   move tool toward negative base X
       */
      double correction_x =
        -error_y * CARTESIAN_STEP_SCALE;

      double correction_y =
        -error_x * CARTESIAN_STEP_SCALE;

      correction_x = std::clamp(
        correction_x,
        -MAX_CORRECTION_PER_PLAN,
        MAX_CORRECTION_PER_PLAN);

      correction_y = std::clamp(
        correction_y,
        -MAX_CORRECTION_PER_PLAN,
        MAX_CORRECTION_PER_PLAN);

      /*
       * Do not move an axis that is already centered.
       */
      if (std::abs(error_x) <= IMAGE_DEADBAND_X) {
        correction_y = 0.0;
      }

      if (std::abs(error_y) <= IMAGE_DEADBAND_Y) {
        correction_x = 0.0;
      }

      const double target_x =
        current_pose.pose.position.x + correction_x;

      const double target_y =
        current_pose.pose.position.y + correction_y;

      /*
       * Always restore the height measured when this node started.
       */
      const double target_z = initial_height_;

      RCLCPP_INFO(
        get_logger(),
        "Current xyz=(%+.4f, %+.4f, %+.4f), "
        "error=(%+.3f, %+.3f), "
        "target xyz=(%+.4f, %+.4f, %+.4f)",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z,
        error_x,
        error_y,
        target_x,
        target_y,
        target_z);

      /*
       * Position-only goal:
       *
       * No orientation target is set. MoveIt may choose any reachable
       * end-effector orientation.
       */
      const bool target_accepted =
        move_group_->setPositionTarget(
          target_x,
          target_y,
          target_z,
          TOOL_LINK);

      if (!target_accepted) {
        RCLCPP_WARN(
          get_logger(),
          "MoveIt rejected the position target");

        move_group_->clearPoseTargets();
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;

      const moveit::core::MoveItErrorCode planning_result =
        move_group_->plan(plan);

      if (
        planning_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(
          get_logger(),
          "Could not find a path for the correction");

        move_group_->clearPoseTargets();
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "Plan found; executing trajectory");

      const moveit::core::MoveItErrorCode execution_result =
        move_group_->execute(plan);

      if (
        execution_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Trajectory execution failed");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Trajectory execution completed");
      }

      move_group_->clearPoseTargets();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Planner exception: %s",
        exception.what());

      if (move_group_) {
        move_group_->clearPoseTargets();
      }
    }
  }


  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    hotspot_subscription_;

  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  std::shared_ptr<
    moveit::planning_interface::MoveGroupInterface>
    move_group_;

  std::mutex target_mutex_;

  bool target_visible_{false};
  bool have_target_message_{false};
  bool planning_or_executing_{false};
  bool height_initialized_{false};

  double error_x_{0.0};
  double error_y_{0.0};
  double confidence_{0.0};
  double initial_height_{0.0};

  rclcpp::Time last_target_time_{
    0,
    0,
    RCL_ROS_TIME
  };

  /*
   * Tracking parameters.
   */
  static constexpr double MIN_CONFIDENCE = 2.0;
  static constexpr double TARGET_TIMEOUT_SECONDS = 0.5;

  static constexpr double IMAGE_DEADBAND_X = 0.30;
  static constexpr double IMAGE_DEADBAND_Y = 0.30;

  /*
   * At full normalized error, request a 10 mm correction.
   */
  static constexpr double CARTESIAN_STEP_SCALE = 0.010;

  /*
   * Never request more than 10 mm per plan.
   */
  static constexpr double MAX_CORRECTION_PER_PLAN = 0.010;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HotspotPathPlanner>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  /*
   * The executor must already be running so MoveGroupInterface
   * can receive /joint_states while requesting the current pose.
   */
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  node->initialize_move_group();

  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
