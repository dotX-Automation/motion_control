/**
 * Position Controller drive routines.
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

#include <position_controller/position_controller.hpp>

namespace position_controller
{

/**
 * @brief Moves the rover to a target position.
 *
 * @param goal_handle Action goal handle pointer.
 */
void PositionControllerNode::reach(const ReachGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Reach::Result>();
  auto feedback = std::make_shared<Reach::Feedback>();

  // Check if some other operation is in progress
  if (!actions_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(tf_base_link_);
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  pose_kit::DynamicPose target(goal_handle->get_goal()->target_pose);

  Vector3d ref_pos_lin;
  Vector3d ref_pos_ang;
  Quaterniond quat;
  bool tf_valid;
  TransformStamped transform;

  tf_valid = tf_pose_to_main(
    target.get_frame_id(),
    rclcpp::Time(),
    rclcpp::Duration(0,0),
    Vector3d(
      target.get_position().x(),
      target.get_position().y(),
      target.get_position().z()),
    Vector3d(
      target.get_rpy().alpha(),
      target.get_rpy().beta(),
      target.get_rpy().gamma()),
    ref_pos_lin,
    ref_pos_ang);

  if(!tf_valid) {
    RCLCPP_ERROR(this->get_logger(), "Goal invalid frame, aborting");
    stop_rover();
    actions_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(tf_base_link_);
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Goal invalid frame");
    goal_handle->abort(result);
    return;
  }

  double lin_precision = abs(goal_handle->get_goal()->reach_radius);
  double alt_precision = ctrl_linearv_precision_;
  double ang_precision = ctrl_angular_precision_;

  bool stabilize = goal_handle->get_goal()->stop_at_target;

  RCLCPP_INFO(
    this->get_logger(),
    "Reaching position (%f, %f, %f), with heading: %f°",
    ref_pos_lin.x(),
    ref_pos_lin.y(),
    ref_pos_lin.z(),
    ref_pos_ang.z() * 180.0 / M_PI);

  while(true) {
    if(goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Reach canceled");
      stop_rover();
      actions_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(tf_base_link_);
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Operation canceled");
      goal_handle->canceled(result);
      break;
    }
    
    Vector3d uxv_pos_lin;
    Vector3d uxv_pos_ang;

    try {
      transform = tf_buffer_->lookupTransform(
        main_frame_,
        base_frame_,
        rclcpp::Time(),
        rclcpp::Duration(0,0));
      tf_valid = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(), "TF main->base exception: %s", e.what());
      tf_valid = false;
    }

    quat = Quaterniond(
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z);

    uxv_pos_lin = Vector3d(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);

    uxv_pos_ang = EulerAnglesXYZd(quat).angles();
  
    if(!tf_valid) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tf transform, aborting");
      stop_rover();
      actions_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(tf_base_link_);
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Invalid tf transform");
      goal_handle->abort(result);
      break;
    }

    Vector4d diff4d(
        ref_pos_lin.x() - uxv_pos_lin.x(),
        ref_pos_lin.y() - uxv_pos_lin.y(),
        ref_pos_lin.z() - uxv_pos_lin.z(),
        wrap_angle(ref_pos_ang.z() - uxv_pos_ang.z()));

    Vector3d diff3d(diff4d.x(), diff4d.y(), diff4d.z());
    Vector2d diff2d(diff4d.x(), diff4d.y());

    bool arrived = diff2d.norm() <= lin_precision &&
                   (two_d_mode_ || (abs(diff4d.z()) <= alt_precision)) &&
                   abs(diff4d.w()) <= ang_precision;

    if (arrived) {
      RCLCPP_WARN(this->get_logger(), "Reach completed");
      if(stabilize) {
        stop_rover();
      }
      actions_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(tf_base_link_);
      result->result.set__result(CommandResultStamped::SUCCESS);
      result->result.set__error_msg("");
      goal_handle->succeed(result);
      break;
    } else {
      double vel_h = ctrl_linearh_err_gain_ * diff2d.norm();
      double vel_v = ctrl_linearv_err_gain_ * diff4d.z();
      double omega = ctrl_angular_err_gain_ * diff4d.w();

      if(vel_h > 0) {
        vel_h = std::min(std::max(vel_h, +ctrl_linearh_vel_min_), +ctrl_linearh_vel_max_);
      }

      if(vel_v > 0) {
        vel_v = std::min(std::max(vel_h, +ctrl_linearv_vel_min_), +ctrl_linearv_vel_max_);
      }

      if(omega > 0) {
        omega = std::min(std::max(omega, +ctrl_angular_vel_min_), +ctrl_angular_vel_max_);
      } else if(omega < 0) {
        omega = std::min(std::max(omega, -ctrl_angular_vel_max_), -ctrl_angular_vel_min_);
      }

      double angle = std::atan2(diff2d.y(), diff2d.x()) - omega * ctrl_rt_counter_coeff_;

      Vector3d cmd_vel_lin;
      Vector3d cmd_vel_ang;

      tf_twist_to_body(
        main_frame_,
        rclcpp::Time(),
        rclcpp::Duration(0,0),
        Vector3d(
          vel_h * std::cos(angle),
          vel_h * std::sin(angle),
          two_d_mode_ ? 0.0 : vel_v),
        Vector3d(
          0.0,
          0.0,
          omega),
        cmd_vel_lin,
        cmd_vel_ang);

      publish_cmd_vel(cmd_vel_lin, cmd_vel_ang);
    }

    PoseStamped pose;
    Quaterniond uxv_attidute = AngleAxisd(uxv_pos_ang.x(), Vector3d::UnitX())
                             * AngleAxisd(uxv_pos_ang.y(), Vector3d::UnitY())
                             * AngleAxisd(uxv_pos_ang.z(), Vector3d::UnitZ());

    pose.header.set__frame_id(main_frame_);
    pose.header.set__stamp(this->get_clock()->now());
    pose.pose.position.set__x(uxv_pos_lin.x());
    pose.pose.position.set__y(uxv_pos_lin.y());
    pose.pose.position.set__z(uxv_pos_lin.z());
    pose.pose.orientation.set__w(uxv_attidute.w());
    pose.pose.orientation.set__x(uxv_attidute.x());
    pose.pose.orientation.set__y(uxv_attidute.y());
    pose.pose.orientation.set__z(uxv_attidute.z());

    feedback->set__current_pose(pose);
    feedback->set__distance_from_target(two_d_mode_ ? diff2d.norm() : diff3d.norm());
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(ms(ctrl_sampling_time_ms_));
  }
}

/**
 * @brief Performs a turn up to the given heading.
 *
 * @param goal_handle Action goal handle pointer.
 */
void PositionControllerNode::turn(const TurnGoalHandleSharedPtr goal_handle)
{
  auto result = std::make_shared<Turn::Result>();
  auto feedback = std::make_shared<Turn::Feedback>();

  // Check if some other operation is in progress
  if (!actions_lock_.try_lock()) {
    RCLCPP_ERROR(this->get_logger(), "Server is busy, aborting");
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(tf_base_link_);
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Server is busy");
    goal_handle->abort(result);
    return;
  }

  Vector3d ref_pos_lin;
  Vector3d ref_pos_ang;
  Vector3d vec;
  Quaterniond quat;
  bool tf_valid;
  TransformStamped transform;

  tf_valid = tf_pose_to_main(
    goal_handle->get_goal()->header.frame_id,
    rclcpp::Time(),
    rclcpp::Duration(0,0),
    Vector3d::Zero(),
    Vector3d(
      0.0,
      0.0,
      goal_handle->get_goal()->heading),
    vec,
    ref_pos_ang);

  if(!tf_valid) {
    RCLCPP_ERROR(this->get_logger(), "Goal invalid frame, aborting");
    stop_rover();
    actions_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(tf_base_link_);
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Goal invalid frame");
    goal_handle->abort(result);
    return;
  }

  try {
    transform = tf_buffer_->lookupTransform(
      main_frame_,
      base_frame_,
      rclcpp::Time(),
      rclcpp::Duration(0,0));
    tf_valid = true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "TF main->base exception: %s", e.what());
    tf_valid = false;
  }

  ref_pos_lin = Vector3d(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);

  if(!tf_valid) {
    RCLCPP_ERROR(this->get_logger(), "Invalid tf transform, aborting");
    stop_rover();
    actions_lock_.unlock();
    result->result.header.set__stamp(this->get_clock()->now());
    result->result.header.set__frame_id(tf_base_link_);
    result->result.set__result(CommandResultStamped::FAILED);
    result->result.set__error_msg("Invalid tf transform");
    goal_handle->abort(result);
  }

  double lin_precision = ctrl_linearh_precision_;
  double alt_precision = ctrl_linearv_precision_;
  double ang_precision = ctrl_angular_precision_;

  bool stabilize = true;

  RCLCPP_INFO(
    this->get_logger(),
    "Turning to heading %f°, with position (%f, %f, %f)",
    ref_pos_ang.z() * 180.0 / M_PI,
    ref_pos_lin.x(),
    ref_pos_lin.y(),
    ref_pos_lin.z());

  while(true) {
    if(goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Turn canceled");
      stop_rover();
      actions_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(tf_base_link_);
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Operation canceled");
      goal_handle->canceled(result);
      break;
    }
    
    Vector3d uxv_pos_lin;
    Vector3d uxv_pos_ang;

    try {
      transform = tf_buffer_->lookupTransform(
        main_frame_,
        base_frame_,
        rclcpp::Time(),
        rclcpp::Duration(0,0));
      tf_valid = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(), "TF main->base exception: %s", e.what());
      tf_valid = false;
    }

    quat = Quaterniond(
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z);

    uxv_pos_lin = Vector3d(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);

    uxv_pos_ang = EulerAnglesXYZd(quat).angles();

    if(!tf_valid) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tf transform, aborting");
      stop_rover();
      actions_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(tf_base_link_);
      result->result.set__result(CommandResultStamped::FAILED);
      result->result.set__error_msg("Invalid tf transform");
      goal_handle->abort(result);
      break;
    }

    Vector4d diff4d(
        ref_pos_lin.x() - uxv_pos_lin.x(),
        ref_pos_lin.y() - uxv_pos_lin.y(),
        ref_pos_lin.z() - uxv_pos_lin.z(),
        wrap_angle(ref_pos_ang.z() - uxv_pos_ang.z()));

    Vector3d diff3d(diff4d.x(), diff4d.y(), diff4d.z());
    Vector2d diff2d(diff4d.x(), diff4d.y());

    bool arrived = diff2d.norm() <= lin_precision &&
                   (two_d_mode_ || (abs(diff4d.z()) <= alt_precision)) &&
                   abs(diff4d.w()) <= ang_precision;

    if (arrived) {
      RCLCPP_WARN(this->get_logger(), "Turn completed");
      if(stabilize) {
        stop_rover();
      }
      actions_lock_.unlock();
      result->result.header.set__stamp(this->get_clock()->now());
      result->result.header.set__frame_id(tf_base_link_);
      result->result.set__result(CommandResultStamped::SUCCESS);
      result->result.set__error_msg("");
      goal_handle->succeed(result);
      break;
    } else {
      double vel_h = ctrl_linearh_err_gain_ * diff2d.norm();
      double vel_v = ctrl_linearv_err_gain_ * diff4d.z();
      double omega = ctrl_angular_err_gain_ * diff4d.w();

      if(vel_h > 0) {
        vel_h = std::min(std::max(vel_h, +ctrl_linearh_vel_min_), +ctrl_linearh_vel_max_);
      }

      if(vel_v > 0) {
        vel_v = std::min(std::max(vel_h, +ctrl_linearv_vel_min_), +ctrl_linearv_vel_max_);
      }

      if(omega > 0) {
        omega = std::min(std::max(omega, +ctrl_angular_vel_min_), +ctrl_angular_vel_max_);
      } else if(omega < 0) {
        omega = std::min(std::max(omega, -ctrl_angular_vel_max_), -ctrl_angular_vel_min_);
      }

      double angle = std::atan2(diff2d.y(), diff2d.x()) - omega * ctrl_rt_counter_coeff_;

      Vector3d cmd_vel_lin;
      Vector3d cmd_vel_ang;

      tf_twist_to_body(
        main_frame_,
        rclcpp::Time(),
        rclcpp::Duration(0,0),
        Vector3d(
          vel_h * std::cos(angle),
          vel_h * std::sin(angle),
          two_d_mode_ ? 0.0 : vel_v),
        Vector3d(
          0.0,
          0.0,
          omega),
        cmd_vel_lin,
        cmd_vel_ang);

      publish_cmd_vel(cmd_vel_lin, cmd_vel_ang);
    }

    double angle = wrap_angle(uxv_pos_ang.z());

    feedback->set__current_yaw(angle);
    feedback->set__current_yaw_deg(angle * 180.0 / M_PI);
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(ms(ctrl_sampling_time_ms_));
  }
}

} // namespace position_controller
