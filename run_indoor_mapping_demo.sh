#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/noetic/setup.bash
set -u

for setup_file in \
  "$HOME/cartographer_ws/install_isolated/setup.bash" \
  "$HOME/cartographer_ws/devel_isolated/setup.bash" \
  "$HOME/cartographer_ws_v3/install_isolated/setup.bash" \
  "$HOME/cartographer_ws_v3/devel_isolated/setup.bash" \
  "$HOME/cartographer_ws_v2/install_isolated/setup.bash" \
  "$HOME/cartographer_ws_v2/devel_isolated/setup.bash" \
  "$HOME/cartographer_ws_explorelite_stage/install_isolated/setup.bash" \
  "$HOME/cartographer_ws_explorelite_stage/devel_isolated/setup.bash" \
  "$SCRIPT_DIR/../cartographer_indoor_active_slam/ros_workspace/install_isolated/setup.bash" \
  "$SCRIPT_DIR/../cartographer_indoor_active_slam/ros_workspace/devel_isolated/setup.bash"; do
  if [[ -f "$setup_file" ]]; then
    source "$setup_file"
  fi
done

LOCAL_OCTOMAP_PREFIX="$HOME/.local/ros/octomap_noetic/opt/ros/noetic"
if [[ -d "$LOCAL_OCTOMAP_PREFIX" ]]; then
  export PATH="$LOCAL_OCTOMAP_PREFIX/bin:${PATH:-}"
  export LD_LIBRARY_PATH="$LOCAL_OCTOMAP_PREFIX/lib:${LD_LIBRARY_PATH:-}"
  export CMAKE_PREFIX_PATH="$LOCAL_OCTOMAP_PREFIX:${CMAKE_PREFIX_PATH:-}"
  export ROS_PACKAGE_PATH="$LOCAL_OCTOMAP_PREFIX/share:${ROS_PACKAGE_PATH:-}"
  local_octomap_node="$LOCAL_OCTOMAP_PREFIX/lib/octomap_server/octomap_server_node"
  package_octomap_node="$LOCAL_OCTOMAP_PREFIX/share/octomap_server/octomap_server_node"
  if [[ -x "$local_octomap_node" && ! -e "$package_octomap_node" ]]; then
    ln -s ../../lib/octomap_server/octomap_server_node "$package_octomap_node"
  fi
fi

export ROS_PACKAGE_PATH="$SCRIPT_DIR/indoor_cartographer_demo:$HOME:${ROS_PACKAGE_PATH:-}"

if ! rospack find cartographer_ros >/dev/null 2>&1; then
  echo "cartographer_ros was not found. Source a workspace containing cartographer_ros first." >&2
  exit 1
fi

if [[ " $* " == *" slam_mode:=hybrid "* ]] && ! rospack find octomap_server >/dev/null 2>&1; then
  echo "octomap_server was not found. Install it or unpack it under $LOCAL_OCTOMAP_PREFIX." >&2
  exit 1
fi

roslaunch "$SCRIPT_DIR/indoor_cartographer_demo/launch/indoor_cartographer.launch" "$@"
