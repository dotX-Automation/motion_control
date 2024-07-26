/**
 * Position Controller node initialization routines.
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
 * @brief Drive Control node constructor.
 *
 * @param node_opts Options for the base node.
 */
PositionControllerNode::PositionControllerNode(const rclcpp::NodeOptions & node_options)
: NodeBase("position_controller", node_options, true)
{
  init_cgroups();
  init_parameters();
  init_tf_listeners();
  init_publishers();
  init_actions();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Drive Control node destructor.
 */
PositionControllerNode::~PositionControllerNode()
{

}

/**
 * @brief Routine to initialize callback groups.
 */
void PositionControllerNode::init_cgroups()
{
  // Actions
  actions_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize TF listeners and their timer.
 */
void PositionControllerNode::init_tf_listeners()
{
  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize local data
  map_frame_ = tf_global_link_;
  odom_frame_ = tf_odom_link_;
  main_frame_ = tf_use_odom_link_as_main_ ? odom_frame_ : map_frame_;
  base_frame_ = tf_base_link_;
}

/**
 * @brief Routine to initialize topic publishers.
 */
void PositionControllerNode::init_publishers()
{
  // Command twist
  cmd_vel_pub_ = this->create_publisher<Twist>(
    cmd_vel_pub_topic_,
    dua_qos::Reliable::get_datum_qos());
}

/**
 * @brief Routine to initialize actions.
 */
void PositionControllerNode::init_actions()
{
  // reach
  reach_server_ = rclcpp_action::create_server<Reach>(
    this,
    "~/reach",
    std::bind(
      &PositionControllerNode::handle_reach_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &PositionControllerNode::handle_reach_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &PositionControllerNode::handle_reach_accepted,
      this,
      std::placeholders::_1),
    dua_qos::get_action_server_options(),
    actions_cgroup_);

  // turn
  turn_server_ = rclcpp_action::create_server<Turn>(
    this,
    "~/turn",
    std::bind(
      &PositionControllerNode::handle_turn_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &PositionControllerNode::handle_turn_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &PositionControllerNode::handle_turn_accepted,
      this,
      std::placeholders::_1),
    dua_qos::get_action_server_options(),
    actions_cgroup_);
}

} // namespace position_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(position_controller::PositionControllerNode)
