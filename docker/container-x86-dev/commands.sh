#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# June 13, 2024

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# shellcheck disable=SC1090

# Source DUA commands
source ~/.dua_submod.sh
source ~/.dua_subtree.sh

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI].
function degrad {
  local angle_in_degrees="$1"
  angle_in_radians=$(python3 -c "import sys, math; angle=float(sys.argv[1]); print(math.radians((angle + 180) % 360 - 180))" "$angle_in_degrees")
  echo "$angle_in_radians"
}

# Routine to update dua-utils.
function utils-update {
  CURR_SHELL=$(ps -p $$ | awk 'NR==2 {print $4}')

  pushd || return
  cd /opt/ros/dua-utils || return
  git pull
  git submodule update --init --recursive
  rm -rf install
  colcon build --merge-install
  rm -rf build log
  source "install/local_setup.$CURR_SHELL"
  popd || return
}

# Moves the robot to a target position
function reach {
  # Check input arguments
  if [[ $# -ne 4 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    reach X Y YAW RADIUS"
    echo >&2 "XY must be w.r.t. a NWU reference frame, YAW must be in [-180° +180°], RADIUS is absolute."
    return 1
  fi

  local yaw_rad
  yaw_rad="$(degrad "$3")"

  ros2 action send_goal -f \
    "$NAMESPACE"/position_controller/reach \
    dua_interfaces/action/Reach \
      "{ \
        target_pose: { \
          header: {frame_id: map}, \
          pose: { \
            position: {x: $1, y: $2}, \
            orientation: { \
              w: $(python3 -c "import math; print(math.cos($yaw_rad/2.0))"), \
              x: 0.0, \
              y: 0.0, \
              z: $(python3 -c "import math; print(math.sin($yaw_rad/2.0))") \
            } \
          } \
       },
       reach_radius: $4, \
       stop_at_target: true \
      }"
}

# Performs a turn to the desired heading
function turn {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    turn YAW"
    echo >&2 "YAW must be in [-180° +180°]."
    return 1
  fi

  ros2 action send_goal -f "$NAMESPACE"/position_controller/turn dua_interfaces/action/Turn "{header: {frame_id: map}, heading: $(degrad "$1")}"
}
