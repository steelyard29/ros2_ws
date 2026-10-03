"""USB MAVLink relay to QGC on the current trusted Wi-Fi subnet.

Never creates heartbeats, parameter writes, arming or flight commands. Forwards
real QGC messages only after a GCS heartbeat. This is NOT authentication: use
a trusted private Wi-Fi. Does not touch Telem1/DDS. No disk telemetry logging.
"""
import ipaddress
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import time
import serial
from pymavlink.dialects.v20 import common

DEVICE='/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00'
INTERFACE='wlP1p1s0'
STATE=Path('/home/cfly/.local/state/robocup-qgc/status.json')


def wifi():
    rows=json.loads(subprocess.check_output(['ip','-j','-4','address','show','dev',INTERFACE],text=True,timeout=3))
    for row in rows:
        if 'UP' not in row.get('flags',[]):continue
        for a in row.get('addr_info',[]):
            if a.get('scope')=='global':
                address=ipaddress.IPv4Interface(f"{a['local']}/{a['prefixlen']}")
                return str(address.ip),address.network
    raise RuntimeError('Wi-Fi IPv4 not ready')


class GroundStation:
    def __init__(self,network,own_ip):
        self.network=network;self.own_ip=own_ip;self.peer=None;self.at=-1.
        self.parser=common.MAVLink(None);self.parser.robust_parsing=True

    def active(self,now):
        if self.peer and now-self.at>10:self.peer=None
        return self.peer

    def receive(self,data,peer,now):
        if peer[0]==self.own_ip or ipaddress.ip_address(peer[0]) not in self.network:return []
        active=self.active(now)
        if active and active!=peer:return []
        # UDP QGC packets carry complete MAVLink frames. Do not carry partial
        # parser state between unauthenticated peers.
        parser=self.parser if active else common.MAVLink(None)
        parser.robust_parsing=True
        try:messages=parser.parse_buffer(data) or []
        except Exception:return []
        output=[]
        for m in messages:
            if m.get_type()=='BAD_DATA':continue
            if m.get_type()=='HEARTBEAT' and m.type==common.MAV_TYPE_GCS:
                self.peer=peer;self.at=now;self.parser=parser
            if self.peer==peer:output.append(bytes(m.get_msgbuf()))
        return output


def main():
    import fcntl
    STATE.parent.mkdir(parents=True,exist_ok=True)
    lock=open(STATE.parent/'bridge.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    while running:
        try:
            local,network=wifi()
            with serial.Serial(DEVICE,115200,timeout=0,write_timeout=1,exclusive=True) as usb, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp:
                udp.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
                udp.bind((local,14555));udp.setblocking(False)
                station=GroundStation(network,local)
                checked=reported=0.;rx=tx=0
                print(f'USB connected; discover QGC at {network.broadcast_address}:14550; local {local}:14555',flush=True)
                while running:
                    now=time.monotonic()
                    if now-checked>3:
                        if wifi()!=(local,network):raise RuntimeError('Wi-Fi address changed; reconnect')
                        if not os.path.exists(DEVICE):raise RuntimeError('PX4 USB disconnected')
                        checked=now
                    ready,_,_=select.select([usb.fileno(),udp],[],[],.2)
                    if usb.fileno() in ready:
                        data=usb.read(1400)
                        if not data:raise RuntimeError('USB EOF')
                        peer=station.active(now)
                        udp.sendto(data,peer or (str(network.broadcast_address),14550));rx+=len(data)
                    if udp in ready:
                        data,peer=udp.recvfrom(65535)
                        for frame in station.receive(data,peer,now):
                            usb.write(frame);tx+=len(frame)
                    if now-reported>2:
                        status=dict(at=time.time(),usb_connected=True,local_ip=local,
                            discovery_port=14550,return_port=14555,qgc_peer=station.active(now),
                            px4_to_wifi_bytes=rx,qgc_to_px4_bytes=tx,
                            generates_vehicle_commands=False)
                        temporary=STATE.with_suffix('.tmp')
                        temporary.write_text(json.dumps(status,indent=2));temporary.replace(STATE)
                        reported=now
        except (OSError,RuntimeError,serial.SerialException,subprocess.SubprocessError) as exc:
            print(f'Waiting/reconnecting: {exc}',flush=True)
            STATE.write_text(json.dumps(dict(at=time.time(),usb_connected=False,error=str(exc))))
            for _ in range(15):
                if not running:break
                time.sleep(.2)
    STATE.write_text(json.dumps(dict(at=time.time(),stopped=True)))


if __name__=='__main__':main()
