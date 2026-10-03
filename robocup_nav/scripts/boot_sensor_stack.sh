#!/bin/bash
# System-service entry only. Never launches a flight controller or an EV writer.
set -e
source /opt/ros/humble/setup.bash
source /home/cfly/ros2_ws/install/local_setup.bash
# Do not anchor camera time before the host reports synchronization.
test "$(timedatectl show -p NTPSynchronized --value)" = yes
exec python3 /home/cfly/ros2_ws/robocup_nav/scripts/perception_reboot_session.py --authorized-sensor-recovery
