#!/bin/bash
set -e

# The binary links against the rcl/rmw shared libraries, so the ROS environment
# has to be on LD_LIBRARY_PATH before it can even start.
source "/opt/ros/$ROS_DISTRO/setup.bash"

exec "$@"
