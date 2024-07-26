/**
 * Position Controller action callbacks.
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

#define UNUSED(arg) (void)(arg)

#include <position_controller/position_controller.hpp>

namespace position_controller
{

/**
 * @brief Handles a new Reach goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse PositionControllerNode::handle_reach_goal(
  const rclcpp_action::GoalUUID & uuid,
  ReachGoalSharedPtr goal)
{
  UNUSED(uuid);

  RCLCPP_INFO(this->get_logger(), "Received reach request");

  if(goal->target_pose.header.frame_id != map_frame_ &&
     goal->target_pose.header.frame_id != odom_frame_)
  {
    RCLCPP_ERROR(this->get_logger(), "Reach request rejected, invalid frame ID");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Turn goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse PositionControllerNode::handle_turn_goal(
  const rclcpp_action::GoalUUID & uuid,
  TurnGoalSharedPtr goal)
{
  UNUSED(uuid);

  RCLCPP_INFO(this->get_logger(), "Received turn request");

  if(goal->header.frame_id != map_frame_ &&
     goal->header.frame_id != odom_frame_)
  {
    RCLCPP_ERROR(this->get_logger(), "Turn request rejected, invalid frame ID");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handles a new Reach cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse PositionControllerNode::handle_reach_cancel(
  const ReachGoalHandleSharedPtr goal_handle)
{
  UNUSED(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Received reach cancellation request");

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Handles a new Turn cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse PositionControllerNode::handle_turn_cancel(
  const TurnGoalHandleSharedPtr goal_handle)
{
  UNUSED(goal_handle);

  RCLCPP_INFO(this->get_logger(), "Received turn cancellation request");

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Starts execution of a Reach.
 *
 * @param goal_handle Handle to the goal object.
 */
void PositionControllerNode::handle_reach_accepted(const ReachGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &PositionControllerNode::reach,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

/**
 * @brief Starts execution of a Turn.
 *
 * @param goal_handle Handle to the goal object.
 */
void PositionControllerNode::handle_turn_accepted(const TurnGoalHandleSharedPtr goal_handle)
{
  std::thread{
    std::bind(
      &PositionControllerNode::turn,
      this,
      std::placeholders::_1),
    goal_handle}.detach();
}

} // namespace position_controller
