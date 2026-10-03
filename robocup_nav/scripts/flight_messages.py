"""Serialize supervisor output to PX4 messages without publishing or devices.

Caller must enforce exclusive ownership and approved runtime routing. The
candidate runtime only publishes these to /robocup/flight_shadow/*.
"""
import math


def build_messages(output, timestamp_us):
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
    if type(timestamp_us) is not int or timestamp_us <= 0:
        raise ValueError('positive integer ROS timestamp required')
    result = {}
    if output.stream:
        if (output.position is None or len(output.position) != 3
                or output.yaw is None
                or not all(math.isfinite(v) for v in (*output.position, output.yaw))):
            raise ValueError('invalid local setpoint')
        hb = OffboardControlMode()
        hb.timestamp, hb.position = timestamp_us, True
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp_us
        sp.position = [float(v) for v in output.position]
        sp.yaw = float(output.yaw)
        sp.velocity = [math.nan]*3
        sp.acceleration = [math.nan]*3
        sp.jerk = [math.nan]*3
        sp.yawspeed = math.nan
        result.update(offboard_control_mode=hb, trajectory_setpoint=sp)
    if output.command:
        codes = {'OFFBOARD': VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                 'ARM': VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                 'LAND': VehicleCommand.VEHICLE_CMD_NAV_LAND}
        if output.command not in codes:
            raise ValueError('unsupported command')
        cmd = VehicleCommand()
        cmd.timestamp = timestamp_us
        cmd.command = codes[output.command]
        cmd.target_system = cmd.source_system = 1
        cmd.target_component = cmd.source_component = 1
        cmd.from_external = True
        if output.command == 'OFFBOARD':
            cmd.param1, cmd.param2 = 1.0, 6.0
        elif output.command == 'ARM':
            cmd.param1 = 1.0
        else:
            # LAND here, not latitude/longitude (0,0). No forced disarm.
            cmd.param4 = cmd.param5 = cmd.param6 = cmd.param7 = math.nan
        result['vehicle_command'] = cmd
    return result
