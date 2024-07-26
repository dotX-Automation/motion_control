/**
 * Position Controller node auxiliary routines.
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
 * @brief Stops the rover at the current position.
 */
void PositionControllerNode::stop_rover()
{
  publish_cmd_vel(Vector3d::Zero(), Vector3d::Zero());
}

/**
 * @brief Wrap angle between -pi and pi.
 * 
 * @param angle Angle in radians.
 */
double PositionControllerNode::wrap_angle(double angle) {
  if(angle >= 0) {
      return std::fmod(angle + M_PI, 2*M_PI) - M_PI;
  } else {
      return M_PI - std::fmod(M_PI - angle, 2*M_PI);
  }
}

/**
 * @brief Count angle wraps occurences between -pi and pi.
 * 
 * @param angle Angle in radians.
 */
long PositionControllerNode::count_wraps(double angle) {
  if(angle > M_PI) {
    return +std::ceil((angle - M_PI) / (2*M_PI));
  } else if(angle < -M_PI) {
    return -std::ceil((M_PI - angle) / (2*M_PI));
  } else {
    return 0;
  }
}

} // namespace position_controller
