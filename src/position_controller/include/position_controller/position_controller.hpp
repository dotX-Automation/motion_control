/**
 * Position Controller node definition.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 *
 * July 25, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POSITION_CONTROLLER_HPP
#define POSITION_CONTROLLER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <dua_node/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <dua_interfaces/action/arm.hpp>
#include <dua_interfaces/action/disarm.hpp>
#include <dua_interfaces/action/reach.hpp>
#include <dua_interfaces/action/turn.hpp>

#include <dua_interfaces/msg/command_result_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pose_kit/pose.hpp>
#include <pose_kit/dynamic_pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

typedef std::chrono::seconds          sec;
typedef std::chrono::milliseconds     ms;
typedef std::chrono::microseconds     us;
typedef std::chrono::nanoseconds      ns;
typedef std::chrono::duration<double> fsec;

using namespace dua_interfaces::action;
using namespace dua_interfaces::msg;
using namespace Eigen;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std::chrono_literals;
using namespace std_msgs::msg;
using namespace std_srvs::srv;

using ArmGoalHandle = rclcpp_action::ServerGoalHandle<Arm>;
using ArmGoalSharedPtr = std::shared_ptr<const Arm::Goal>;
using ArmGoalHandleSharedPtr = std::shared_ptr<ArmGoalHandle>;

using DisarmGoalHandle = rclcpp_action::ServerGoalHandle<Disarm>;
using DisarmGoalSharedPtr = std::shared_ptr<const Disarm::Goal>;
using DisarmGoalHandleSharedPtr = std::shared_ptr<DisarmGoalHandle>;

using ReachGoalHandle = rclcpp_action::ServerGoalHandle<Reach>;
using ReachGoalSharedPtr = std::shared_ptr<const Reach::Goal>;
using ReachGoalHandleSharedPtr = std::shared_ptr<ReachGoalHandle>;

using TurnGoalHandle = rclcpp_action::ServerGoalHandle<Turn>;
using TurnGoalSharedPtr = std::shared_ptr<const Turn::Goal>;
using TurnGoalHandleSharedPtr = std::shared_ptr<TurnGoalHandle>;

#define NOOP ((void)0)
#define UNUSED(arg) (void)(arg)

namespace position_controller {

/**
 * Low-level drive operations module, abstracts a PX4-based FMU.
 */
class PositionControllerNode : public dua_node::NodeBase
{
public:
  PositionControllerNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~PositionControllerNode();

private:
  /* Node parameters. */
  double ctrl_angular_err_gain_ = 0.0;
  double ctrl_angular_precision_ = 0.0;
  double ctrl_angular_vel_min_ = 0.0;
  double ctrl_angular_vel_max_ = 0.0;
  double ctrl_linearh_err_gain_ = 0.0;
  double ctrl_linearh_precision_ = 0.0;
  double ctrl_linearh_vel_min_ = 0.0;
  double ctrl_linearh_vel_max_ = 0.0;
  double ctrl_linearv_err_gain_ = 0.0;
  double ctrl_linearv_precision_ = 0.0;
  double ctrl_linearv_vel_min_ = 0.0;
  double ctrl_linearv_vel_max_ = 0.0;
  double ctrl_rt_counter_coeff_ = 0.0;
  int64_t ctrl_sampling_time_ms_ = 0;
  std::string tf_base_link_ = "";
  std::string tf_global_link_ = "";
  std::string tf_odom_link_ = "";
  bool tf_use_odom_link_as_main_ = false;
  bool two_d_mode_ = false;

  /* Node initialization routines. */
  void init_cgroups();
  void init_parameters();
  void init_tf_listeners();
  void init_publishers();
  void init_actions();

  /* Synchronization primitives. */
  std::mutex actions_lock_;

  /* Utility routines. */
  void stop_rover();
  double wrap_angle(double angle);
  long count_wraps(double angle);

  /* TF Variables */
  std::string map_frame_;
  std::string odom_frame_;
  std::string main_frame_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* TF routines. */
  bool tf_get_map_to_odom(rclcpp::Time time, rclcpp::Duration timeout, 
    TransformStamped &transform);
  bool tf_get_odom_to_base(rclcpp::Time time, rclcpp::Duration timeout,
    TransformStamped &transform);
  bool tf_pose_to_map(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &pos_lin_in, const Vector3d &pos_ang_in,
    Vector3d &pos_lin_out, Vector3d &pos_ang_out);
  bool tf_pose_to_map(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &pos_lin_in, const Quaterniond &quat_in,
    Vector3d &pos_lin_out, Vector3d &pos_ang_out);
  bool tf_pose_to_odom(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &pos_lin_in, const Vector3d &pos_ang_in,
    Vector3d &pos_lin_out, Vector3d &pos_ang_out);
  bool tf_pose_to_odom(std::string frame,
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &pos_lin_in, const Quaterniond &quat_in,
    Vector3d &pos_lin_out, Vector3d &pos_ang_out);
  bool tf_pose_to_main(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &pos_lin_in, const Vector3d &pos_ang_in,
    Vector3d &pos_lin_out, Vector3d &pos_ang_out);
  bool tf_pose_to_main(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &pos_lin_in, const Quaterniond &quat_in,
    Vector3d &pos_lin_out, Vector3d &pos_ang_out);
  bool tf_twist_to_map(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
    Vector3d &vel_lin_out, Vector3d &vel_ang_out);
  bool tf_twist_to_odom(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
    Vector3d &vel_lin_out, Vector3d &vel_ang_out);
  bool tf_twist_to_main(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
    Vector3d &vel_lin_out, Vector3d &vel_ang_out);
  bool tf_twist_to_body(std::string frame, 
    rclcpp::Time time, rclcpp::Duration timeout,
    const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
    Vector3d &vel_lin_out, Vector3d &vel_ang_out);

  /* Publishers */
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<Empty>::SharedPtr watchdog_pub_;

  /* Publishers Topics */
  static const std::string cmd_vel_pub_topic_;

  /* Publishers routines */
  void publish_cmd_vel(const Vector3d & vel_lin, const Vector3d & vel_ang);
  inline void publish_cmd_vel(double v_x, double v_y, double w_z);

  /* Action Servers. */
  rclcpp_action::Server<Reach>::SharedPtr reach_server_;
  rclcpp_action::Server<Turn>::SharedPtr turn_server_;

  /* Actions Servers Callback Groups */
  rclcpp::CallbackGroup::SharedPtr actions_cgroup_;

  /* Actions Servers Goal Request Handlers */
  rclcpp_action::GoalResponse handle_reach_goal(const rclcpp_action::GoalUUID & uuid, ReachGoalSharedPtr goal);
  rclcpp_action::GoalResponse handle_turn_goal(const rclcpp_action::GoalUUID & uuid, TurnGoalSharedPtr goal);

  /* Actions Servers Cancellation Request Handlers */
  rclcpp_action::CancelResponse handle_reach_cancel(const ReachGoalHandleSharedPtr goal_handle);
  rclcpp_action::CancelResponse handle_turn_cancel(const TurnGoalHandleSharedPtr goal_handle);

  /* Actions Servers Acceptance handlers. */
  void handle_reach_accepted(const ReachGoalHandleSharedPtr goal_handle);
  void handle_turn_accepted(const TurnGoalHandleSharedPtr goal_handle);

  /* Actions Servers Routines. */
  void reach(const ReachGoalHandleSharedPtr goal_handle);
  void turn(const TurnGoalHandleSharedPtr goal_handle);
};

} // namespace position_controller

#endif // POSITION_CONTROLLER_HPP
