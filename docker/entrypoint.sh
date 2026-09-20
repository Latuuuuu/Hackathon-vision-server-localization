#!/usr/bin/env bash
# Container entrypoint: build the workspace if needed, source it, then run the given command.
#   AUTO_BUILD=auto (default)  build only when install/setup.bash is missing
#   AUTO_BUILD=always          build on every start
#   AUTO_BUILD=never           never build
set -e

WS="${ROS_WORKSPACE:-/home/vision/vision_ws}"
source /opt/ros/humble/setup.bash
cd "$WS"

need_build=0
case "${AUTO_BUILD:-auto}" in
    always) need_build=1 ;;
    never)  need_build=0 ;;
    *)      [ -f "$WS/install/setup.bash" ] || need_build=1 ;;
esac

if [ "$need_build" = "1" ]; then
    echo "[entrypoint] building the workspace (this takes a few minutes on the first start)"
    # One worker at a time: the full parallel build needs more RAM than some machines have
    MAKEFLAGS=-j1 colcon build --parallel-workers 1
fi

if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
else
    echo "[entrypoint] WARNING: $WS/install/setup.bash not found (AUTO_BUILD=never?)" >&2
fi

exec "$@"
