#!/bin/bash
# Ground-stationary boot only. No arm/mode/trajectory/servo interface.
set -e
source /opt/ros/humble/setup.bash
source /home/cfly/ros2_ws/install/local_setup.bash
test "$(timedatectl show -p NTPSynchronized --value)" = yes
exec python3 /home/cfly/ros2_ws/robocup_nav/scripts/resident_ev_service.py --execute-real-ev
