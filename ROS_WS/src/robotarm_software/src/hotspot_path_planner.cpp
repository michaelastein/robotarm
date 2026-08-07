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


/**
 * @file hotspot_path_planner.cpp
 *
 * @brief MoveIt-based hotspot centering node.
 *
 * The node subscribes to normalized image errors from the hotspot detector and
 * periodically requests small Cartesian corrections from MoveIt.
 *
 * Hotspot message format:
 *   data[0]:
 *     Visibility flag. Values greater than or equal to 0.5 indicate that the
 *     hotspot is visible.
 *
 *   data[1]:
 *     Normalized horizontal image error.
 *     Negative means the hotspot is left of center.
 *     Positive means the hotspot is right of center.
 *
 *   data[2]:
 *     Normalized vertical image error.
 *     Negative means the hotspot is above center.
 *     Positive means the hotspot is below center.
 *
 *   data[3]:
 *     Detector confidence.
 *
 * Image-to-robot mapping:
 *   Positive image X error moves the tool toward negative base-frame Y.
 *   Positive image Y error moves the tool toward negative base-frame X.
 *
 * Planning behavior:
 *   - New hotspot data may arrive at camera frequency.
 *   - Planning is checked only at PLANNING_PERIOD.
 *   - Only fresh, visible, sufficiently confident targets are used.
 *   - Independent image deadbands prevent unnecessary X or Y correction.
 *   - The complete XY correction vector is limited per plan.
 *   - The tool height measured during initialization is held constant.
 *   - Position-only goals are used; no orientation target is specified.
 */


namespace
{

constexpr char kPlanningGroup[] = "arm";
constexpr char kToolLink[] = "tool_tip_link";
constexpr char kHotspotTopic[] = "/hotspot/target";

constexpr auto kPlanningPeriod = 200ms;
constexpr auto kMoveGroupStartupDelay = 200ms;

constexpr double kPlanningTimeSeconds = 3.0;
constexpr int kPlanningAttempts = 5;
constexpr double kGoalPositionToleranceMeters = 0.008;

constexpr double kVelocityScalingFactor = 0.38;
constexpr double kAccelerationScalingFactor = 0.18;

constexpr double kMinimumConfidence = 2.0;
constexpr double kTargetTimeoutSeconds = 0.5;

constexpr double kImageDeadbandX = 0.1;
constexpr double kImageDeadbandY = 0.1;

constexpr double kMaximumCorrectionPerPlanMeters = 0.090;

}  // namespace


class HotspotPathPlanner : public rclcpp::Node
{
public:
  /**
   * @brief Construct the ROS node, hotspot subscription, and planning timer.
   *
   * The MoveGroupInterface is initialized separately after the executor has
   * started so that current robot state messages can be processed.
   */
  HotspotPathPlanner()
  : Node(
      "hotspot_path_planner",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true))
  {
    hotspot_subscription_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      kHotspotTopic,
      10,
      std::bind(
        &HotspotPathPlanner::hotspot_callback,
        this,
        std::placeholders::_1));

    planning_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    planning_timer_ = create_wall_timer(
      kPlanningPeriod,
      std::bind(
        &HotspotPathPlanner::planning_timer_callback,
        this),
      planning_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "Hotspot path planner started");
  }

  /**
   * @brief Create and configure the MoveIt MoveGroupInterface.
   *
   * The method configures planning time, planning attempts, position tolerance,
   * velocity scaling, and acceleration scaling. It then records the current
   * tool height for later position-only corrections.
   *
   * @throws std::exception
   *   MoveIt may throw while accessing the robot state or current pose.
   */
  void initialize_move_group()
  {
    move_group_ =
      std::make_shared<
      moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      kPlanningGroup);

    move_group_->setPlanningTime(kPlanningTimeSeconds);
    move_group_->setNumPlanningAttempts(kPlanningAttempts);
    move_group_->setGoalPositionTolerance(
      kGoalPositionToleranceMeters);
    move_group_->setMaxVelocityScalingFactor(
      kVelocityScalingFactor);
    move_group_->setMaxAccelerationScalingFactor(
      kAccelerationScalingFactor);

    initialize_height();
  }


private:
  /**
   * @brief Store the latest hotspot detector message.
   *
   * @param msg
   *   Float32MultiArray containing visibility, normalized X error, normalized
   *   Y error, and confidence.
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

  /**
   * @brief Save the current tool height as the fixed correction height.
   *
   * The current pose is requested for kToolLink. Its Z coordinate is stored and
   * reused for every later position target.
   */
  void initialize_height()
  {
    try {
      const geometry_msgs::msg::PoseStamped current_pose =
        move_group_->getCurrentPose(kToolLink);

      initial_height_ = current_pose.pose.position.z;
      height_initialized_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Initial tool height saved: z=%+.4f m",
        initial_height_);
    } catch (const std::exception & exception) {
      height_initialized_ = false;

      RCLCPP_ERROR(
        get_logger(),
        "Could not initialize tool height: %s",
        exception.what());
    }
  }

  /**
   * @brief Copy the latest detector state under the target mutex.
   *
   * @param visible
   *   Receives the current visibility flag.
   *
   * @param error_x
   *   Receives the normalized horizontal image error.
   *
   * @param error_y
   *   Receives the normalized vertical image error.
   *
   * @param confidence
   *   Receives the latest detector confidence.
   *
   * @param target_time
   *   Receives the ROS timestamp at which the latest detector message arrived.
   *
   * @return
   *   True when at least one detector message has been received; otherwise
   *   false.
   */
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

  /**
   * @brief Check the latest hotspot state and start one correction plan.
   *
   * Planning is skipped while MoveIt is uninitialized, another plan is active,
   * the target is missing or stale, confidence is too low, or both image axes
   * are inside their deadbands.
   */
  void planning_timer_callback()
  {
    if (!move_group_ || planning_or_executing_) {
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
    rclcpp::Time target_time(
      0,
      0,
      get_clock()->get_clock_type());

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
      target_age > kTargetTimeoutSeconds ||
      confidence < kMinimumConfidence)
    {
      return;
    }

    const bool x_centered =
      std::abs(error_x) <= kImageDeadbandX;

    const bool y_centered =
      std::abs(error_y) <= kImageDeadbandY;

    if (x_centered && y_centered) {
      return;
    }

    planning_or_executing_ = true;
    plan_and_execute_correction(error_x, error_y);
    planning_or_executing_ = false;
  }

  /**
   * @brief Plan and execute one adaptive Cartesian correction.
   *
   * @param error_x
   *   Normalized horizontal image error. Positive values move the tool toward
   *   negative base-frame Y.
   *
   * @param error_y
   *   Normalized vertical image error. Positive values move the tool toward
   *   negative base-frame X.
   *
   * The correction scale increases with image error. The final XY vector is
   * limited to kMaximumCorrectionPerPlanMeters. The target Z coordinate is
   * always set to the height saved during initialization.
   */
  void plan_and_execute_correction(
    double error_x,
    double error_y)
  {
    try {
      move_group_->setStartStateToCurrentState();

      const geometry_msgs::msg::PoseStamped current_pose =
        move_group_->getCurrentPose(kToolLink);

      const double largest_image_error =
        std::max(
        std::abs(error_x),
        std::abs(error_y));

      double adaptive_scale = 0.020;

      if (largest_image_error >= 0.70) {
        adaptive_scale = 0.120;
      } else if (largest_image_error >= 0.45) {
        adaptive_scale = 0.090;
      } else if (largest_image_error >= 0.25) {
        adaptive_scale = 0.060;
      } else if (largest_image_error >= 0.15) {
        adaptive_scale = 0.028;
      }

      double correction_x =
        -error_y * adaptive_scale;

      double correction_y =
        -error_x * adaptive_scale;

      if (std::abs(error_x) <= kImageDeadbandX) {
        correction_y = 0.0;
      }

      if (std::abs(error_y) <= kImageDeadbandY) {
        correction_x = 0.0;
      }

      const double correction_length =
        std::hypot(correction_x, correction_y);

      if (
        correction_length > kMaximumCorrectionPerPlanMeters &&
        correction_length > 1e-9)
      {
        const double limit_factor =
          kMaximumCorrectionPerPlanMeters /
          correction_length;

        correction_x *= limit_factor;
        correction_y *= limit_factor;
      }

      const double target_x =
        current_pose.pose.position.x + correction_x;

      const double target_y =
        current_pose.pose.position.y + correction_y;

      const double target_z = initial_height_;

      const bool target_accepted =
        move_group_->setPositionTarget(
        target_x,
        target_y,
        target_z,
        kToolLink);

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

      const moveit::core::MoveItErrorCode execution_result =
        move_group_->execute(plan);

      if (
        execution_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Trajectory execution failed");
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

  rclcpp::CallbackGroup::SharedPtr
    planning_callback_group_;

  rclcpp::TimerBase::SharedPtr
    planning_timer_;

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
};


/**
 * @brief Initialize ROS, start the executor, initialize MoveIt, and shut down.
 *
 * @param argc
 *   Number of process command-line arguments.
 *
 * @param argv
 *   Process command-line argument array.
 *
 * @return
 *   Zero after a normal shutdown.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<HotspotPathPlanner>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  std::this_thread::sleep_for(
    kMoveGroupStartupDelay);

  node->initialize_move_group();

  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
