"""Host PX4 transport; perception SHM lives in its own container process.

Only changes the calling process environment. No ROS, Agent or device startup.
Matches the existing successful disarmed EV/telemetry host bridge profile.
"""
import os


def configure():
    profile='/home/cfly/ros2_ws/config/fastdds_bridge.xml'
    os.environ.update(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',
                      FASTRTPS_DEFAULT_PROFILES_FILE=profile,
                      FASTDDS_DEFAULT_PROFILES_FILE=profile)
