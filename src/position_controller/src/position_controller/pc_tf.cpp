/**
 * Position Controller service callbacks.
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
 * @brief Retrieve transform from map frame to odom frame.
 *
 * @param time transform time.
 * @param transform output transform.
 */
bool PositionControllerNode::tf_get_map_to_odom(rclcpp::Time time, rclcpp::Duration timeout,
  TransformStamped &transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame_,
      odom_frame_,
      time,
      timeout);
  } catch (const tf2::TransformException & e) {
    if(time.nanoseconds() > 0) {
      RCLCPP_INFO(this->get_logger(), "TF global->odom retry without time");
      return tf_get_map_to_odom(rclcpp::Time(), rclcpp::Duration(0,0), transform);
    } else {
      RCLCPP_WARN(this->get_logger(), "TF global->odom exception: %s", e.what());
      return false;
    }
  }

  return true;
}

/**
 * @brief Retrieve transform from odom frame to base frame.
 *
 * @param time transform time.
 * @param transform output transform.
 */
bool PositionControllerNode::tf_get_odom_to_base(rclcpp::Time time, rclcpp::Duration timeout,
  TransformStamped &transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      odom_frame_,
      base_frame_,
      time,
      timeout);
  } catch (const tf2::TransformException & e) {
    if(time.nanoseconds() > 0) {
      RCLCPP_INFO(this->get_logger(), "TF odom->base retry without time");
      return tf_get_odom_to_base(rclcpp::Time(), rclcpp::Duration(0,0), transform);
    } else {
      RCLCPP_WARN(this->get_logger(), "TF odom->base exception: %s", e.what());
      return false;
    }
  }

  return true;
}

/**
 * @brief Transform pose from input frame to map frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input position.
 * @param pos_ang_in input attitude.
 * @param pos_lin_out output position.
 * @param pos_ang_out output attitude.
 */
bool PositionControllerNode::tf_pose_to_map(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &pos_lin_in, const Vector3d &pos_ang_in,
  Vector3d &pos_lin_out, Vector3d &pos_ang_out)
{
  bool do_tf_odom_map = frame == odom_frame_;
  bool valid_frame = frame == map_frame_ || do_tf_odom_map;

  if(!valid_frame) {
    RCLCPP_WARN(this->get_logger(), "Position setpoint invalid frame");
    return false;
  }

  pos_lin_out = pos_lin_in;
  pos_ang_out = pos_ang_in;

  TransformStamped transform;
  Eigen::Isometry3d isometry;

  if(do_tf_odom_map) {
    if(!tf_get_map_to_odom(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform map->odom");
      return false;
    }

    isometry.translation() = pos_lin_out;
    isometry.linear() = EulerAnglesXYZd(pos_ang_out).matrix();
    isometry = tf2::transformToEigen(transform) * isometry;

    pos_lin_out = isometry.translation();
    pos_ang_out = EulerAnglesXYZd(isometry.rotation()).angles();
  }

  return true;
}

/**
 * @brief Transform pose from input frame to map frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input position.
 * @param pos_ang_in input attitude.
 * @param pos_lin_out output position.
 * @param pos_ang_out output attitude.
 */
bool PositionControllerNode::tf_pose_to_map(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &pos_lin_in, const Quaterniond &quat_in,
  Vector3d &pos_lin_out, Vector3d &pos_ang_out)
{
  Vector3d pos_ang_in = EulerAnglesXYZd(quat_in).angles();
  return tf_pose_to_map(frame, time, timeout, pos_lin_in, pos_ang_in, pos_lin_out, pos_ang_out);
}

/**
 * @brief Transform pose from input frame to odom frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input position.
 * @param pos_ang_in input attitude.
 * @param pos_lin_out output position.
 * @param pos_ang_out output attitude.
 */
bool PositionControllerNode::tf_pose_to_odom(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &pos_lin_in, const Vector3d &pos_ang_in,
  Vector3d &pos_lin_out, Vector3d &pos_ang_out)
{
  bool do_tf_map_odom = frame == map_frame_;
  bool valid_frame = frame == odom_frame_ || do_tf_map_odom;

  if(!valid_frame) {
    RCLCPP_WARN(this->get_logger(), "Position setpoint invalid frame");
    return false;
  }

  pos_lin_out = pos_lin_in;
  pos_ang_out = pos_ang_in;

  TransformStamped transform;
  Eigen::Isometry3d isometry;

  if(do_tf_map_odom) {
    if(!tf_get_map_to_odom(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform map->odom");
      return false;
    }

    isometry.translation() = pos_lin_out;
    isometry.linear() = EulerAnglesXYZd(pos_ang_out).matrix();
    isometry = tf2::transformToEigen(transform).inverse() * isometry;

    pos_lin_out = isometry.translation();
    pos_ang_out = EulerAnglesXYZd(isometry.rotation()).angles();
  }

  return true;
}

/**
 * @brief Transform pose from input frame to odom frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input position.
 * @param pos_ang_in input attitude.
 * @param pos_lin_out output position.
 * @param pos_ang_out output attitude.
 */
bool PositionControllerNode::tf_pose_to_odom(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &pos_lin_in, const Quaterniond &quat_in,
  Vector3d &pos_lin_out, Vector3d &pos_ang_out)
{
  Vector3d pos_ang_in = EulerAnglesXYZd(quat_in).angles();
  return tf_pose_to_odom(frame, time, timeout, pos_lin_in, pos_ang_in, pos_lin_out, pos_ang_out);
}

/**
 * @brief Transform pose from input frame to main frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input position.
 * @param pos_ang_in input attitude.
 * @param pos_lin_out output position.
 * @param pos_ang_out output attitude.
 */
bool PositionControllerNode::tf_pose_to_main(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &pos_lin_in, const Vector3d &pos_ang_in,
  Vector3d &pos_lin_out, Vector3d &pos_ang_out)
{
  if(tf_use_odom_link_as_main_) {
    return tf_pose_to_odom(frame, time, timeout, pos_lin_in, pos_ang_in, pos_lin_out, pos_ang_out);
  } else {
    return tf_pose_to_map(frame, time, timeout, pos_lin_in, pos_ang_in, pos_lin_out, pos_ang_out);
  }
}

/**
 * @brief Transform pose from input frame to main frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input position.
 * @param pos_ang_in input attitude.
 * @param pos_lin_out output position.
 * @param pos_ang_out output attitude.
 */
bool PositionControllerNode::tf_pose_to_main(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &pos_lin_in, const Quaterniond &quat_in,
  Vector3d &pos_lin_out, Vector3d &pos_ang_out)
{
  if(tf_use_odom_link_as_main_) {
    return tf_pose_to_odom(frame, time, timeout, pos_lin_in, quat_in, pos_lin_out, pos_ang_out);
  } else {
    return tf_pose_to_map(frame, time, timeout, pos_lin_in, quat_in, pos_lin_out, pos_ang_out);
  }
}

/**
 * @brief Transform twist from input frame to map frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input linear velocity.
 * @param pos_ang_in input angular velocity.
 * @param pos_lin_out output linear velocity.
 * @param pos_ang_out output angular velocity.
 */
bool PositionControllerNode::tf_twist_to_map(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
  Vector3d &vel_lin_out, Vector3d &vel_ang_out)
{
  bool do_tf_base_odom = frame == base_frame_;
  bool do_tf_odom_map = frame == odom_frame_ || do_tf_base_odom;
  bool valid_frame = frame == map_frame_ || do_tf_odom_map;

  if(!valid_frame) {
    RCLCPP_WARN(this->get_logger(), "Twist invalid frame");
    return false;
  }

  vel_lin_out = vel_lin_in;
  vel_ang_out = vel_ang_in;

  TransformStamped transform;
  Eigen::Matrix3d rotation;

  if(do_tf_base_odom) {
    if(!tf_get_odom_to_base(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform odom->base");
      return false;
    }

    rotation = tf2::transformToEigen(transform).rotation();
    vel_lin_out = rotation * vel_lin_out;
  }

  if(do_tf_odom_map) {
    if(!tf_get_map_to_odom(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform map->odom");
      return false;
    }

    rotation = tf2::transformToEigen(transform).rotation();
    vel_lin_out = rotation * vel_lin_out;
  }

  return true;
}

/**
 * @brief Transform twist from input frame to odom frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input linear velocity.
 * @param pos_ang_in input angular velocity.
 * @param pos_lin_out output linear velocity.
 * @param pos_ang_out output angular velocity.
 */
bool PositionControllerNode::tf_twist_to_odom(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
  Vector3d &vel_lin_out, Vector3d &vel_ang_out)
{
  bool do_tf_map_odom = frame == map_frame_;
  bool do_tf_base_odom = frame == base_frame_;
  bool valid_frame = frame == odom_frame_ || do_tf_map_odom || do_tf_base_odom;

  if(!valid_frame) {
    RCLCPP_WARN(this->get_logger(), "Twist invalid frame");
    return false;
  }

  vel_lin_out = vel_lin_in;
  vel_ang_out = vel_ang_in;

  TransformStamped transform;
  Eigen::Matrix3d rotation;

  if(do_tf_map_odom) {
    if(!tf_get_map_to_odom(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform map->odom");
      return false;
    }

    rotation = tf2::transformToEigen(transform).rotation().transpose();
    vel_lin_out = rotation * vel_lin_out;
  }

  if(do_tf_base_odom) {
    if(!tf_get_odom_to_base(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform odom->base");
      return false;
    }

    rotation = tf2::transformToEigen(transform).rotation();
    vel_lin_out = rotation * vel_lin_out;
  }

  return true;
}

/**
 * @brief Transform twist from input frame to main frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input linear velocity.
 * @param pos_ang_in input angular velocity.
 * @param pos_lin_out output linear velocity.
 * @param pos_ang_out output angular velocity.
 */
bool PositionControllerNode::tf_twist_to_main(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
  Vector3d &vel_lin_out, Vector3d &vel_ang_out)
{
  if(tf_use_odom_link_as_main_) {
    return tf_twist_to_odom(frame, time, timeout, vel_lin_in, vel_ang_in, vel_lin_out, vel_ang_out);
  } else {
    return tf_twist_to_map(frame, time, timeout, vel_lin_in, vel_ang_in, vel_lin_out, vel_ang_out);
  }
}

/**
 * @brief Transform twist from input frame to body frame.
 *
 * @param frame Starting frame.
 * @param pos_lin_in input linear velocity.
 * @param pos_ang_in input angular velocity.
 * @param pos_lin_out output linear velocity.
 * @param pos_ang_out output angular velocity.
 */
bool PositionControllerNode::tf_twist_to_body(std::string frame,
  rclcpp::Time time, rclcpp::Duration timeout,
  const Vector3d &vel_lin_in, const Vector3d &vel_ang_in,
  Vector3d &vel_lin_out, Vector3d &vel_ang_out)
{
  bool do_tf_map_odom = frame == map_frame_;
  bool do_tf_odom_base = frame == odom_frame_ || do_tf_map_odom;
  bool valid_frame = frame == base_frame_ || do_tf_odom_base;

  if(!valid_frame) {
    RCLCPP_WARN(this->get_logger(), "Twist invalid frame");
    return false;
  }

  vel_lin_out = vel_lin_in;
  vel_ang_out = vel_ang_in;

  TransformStamped transform;
  Eigen::Matrix3d rotation;

  if(do_tf_map_odom) {
    if(!tf_get_map_to_odom(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform map->odom");
      return false;
    }

    rotation = tf2::transformToEigen(transform).rotation().transpose();
    vel_lin_out = rotation * vel_lin_out;
  }

  if(do_tf_odom_base) {
    if(!tf_get_odom_to_base(time, timeout, transform)) {
      RCLCPP_WARN(this->get_logger(), "Cannot retrieve transform odom->base");
      return false;
    }

    rotation = tf2::transformToEigen(transform).rotation().transpose();
    vel_lin_out = rotation * vel_lin_out;
  }

  return true;
}

} // namespace position_controller