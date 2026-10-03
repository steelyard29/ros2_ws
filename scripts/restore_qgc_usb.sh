#!/usr/bin/env bash
# Restore QGC USB access: SYS_USB_AUTO=2, save, reboot
set -euo pipefail
PORT="${1:-/dev/ttyACM0}"

echo "Waiting for $PORT ..."
for i in $(seq 1 60); do
  [[ -e "$PORT" ]] && break
  sleep 0.5
done
[[ -e "$PORT" ]] || { echo "No $PORT"; exit 1; }
sleep 2

python3 - <<PY
import sys, time
from pymavlink import mavutil

port = "$PORT"
print(f"Trying MAVLink on {port}...")
m = mavutil.mavlink_connection(port, baud=57600)
hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=8)
if hb:
    print("MAVLink OK", hb)
    def setp(name, val):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=4)
        print(name, msg.param_value if msg else "FAIL")
        return bool(msg)
    setp("SYS_USB_AUTO", 2)
    # optional: leave DDS on TELEM1
    setp("UXRCE_DDS_CFG", 101)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0,0,0,0,0,0)
    print("save ack", m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5))
    time.sleep(1)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0,0,0,0,0,0)
    print("reboot sent")
    m.close()
    sys.exit(0)

print("No MAVLink — trying NSH console...")
import serial
ser = serial.Serial(port, 57600, timeout=0.5, write_timeout=2, dsrdtr=False, rtscts=False)
time.sleep(0.5)
ser.reset_input_buffer()
for _ in range(15):
    ser.write(b"\r\n")
    time.sleep(0.1)
buf = ser.read(2000)
print("console:", buf[:200])
if b"nsh>" not in buf and b"pxh>" not in buf:
    ser.baudrate = 115200
    ser.reset_input_buffer()
    for _ in range(10):
        ser.write(b"\r\n"); time.sleep(0.1)
    buf = ser.read(2000)
    print("115200:", buf[:200])
if b"nsh>" not in buf and b"pxh>" not in buf:
    print("FAIL: neither MAVLink nor NSH")
    ser.close(); sys.exit(2)

def nsh(cmd, wait=1.0):
    ser.write((cmd + "\r\n").encode())
    time.sleep(wait)
    out = ser.read(3000)
    print(">>>", cmd)
    print(out.decode("utf-8", "replace")[-500:])
    return out

nsh("param set SYS_USB_AUTO 2")
nsh("param show SYS_USB_AUTO")
nsh("param save", 2.0)
nsh("reboot", 0.5)
ser.close()
print("Done via NSH")
PY
