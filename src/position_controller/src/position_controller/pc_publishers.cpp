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
 * @brief Disarms the rover.
 *
 * @param linear linear velocities along body frame axes.
 * @param angular angular velocity around body frame axes.
 */
void PositionControllerNode::publish_cmd_vel(const Vector3d & linear, const Vector3d & angular) {
  Twist msg;
  msg.linear.x = linear.x();
  msg.linear.y = linear.y();
  msg.linear.z = linear.z();
  msg.angular.x = angular.x();
  msg.angular.y = angular.y();
  msg.angular.z = angular.z();
  cmd_vel_pub_->publish(msg);
}

/**
 * @brief Disarms the rover.
 *
 * @param v_x linear velocity along body frame x axis.
 * @param v_y linear velocity along body frame y axis.
 * @param w_z angular velocity around body frame z axis.
 */
inline void PositionControllerNode::publish_cmd_vel(double v_x, double v_y, double w_z) {
  publish_cmd_vel(Vector3d(v_x, v_y, 0.0), Vector3d(0.0, 0.0, w_z));
}

} // namespace position_controller