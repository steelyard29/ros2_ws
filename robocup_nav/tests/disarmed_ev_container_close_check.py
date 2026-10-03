"""Host -> Docker isolated reader stop handshake; no sensors or PX4 input."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from disarmed_ev_lifecycle import atomic_json,stop_reader

def main():
    out=ROOT/'evidence'/time.strftime('disarmed_ev_container_close_%Y%m%d_%H%M%S');out.mkdir()
    session=uuid.uuid4().hex;control=out/'control.json';receipt=out/'closed.json'
    atomic_json(control,dict(session=session,stop=False))
    c=lambda p:str(p).replace('/home/cfly/ros2_ws/','/workspaces/ros2_ws/')
    started=time.monotonic();report=dict(passed=False,hardware=False,domain=180)
    log=(out/'stderr.log').open('w')
    cmd=['docker','exec','-e','ROS_LOCALHOST_ONLY=1','isaac_ros_dev','bash','-c',
         'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
         'exec python3 /workspaces/ros2_ws/robocup_nav/scripts/disarmed_ev_sensor_reader.py '
         f'--isolated --session {session} --deadline {started+8} '
         f'--control-file {c(control)} --receipt-file {c(receipt)}']
    child=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=log)
    os.set_blocking(child.stdout.fileno(),False)
    try:
        time.sleep(1.)
        result=stop_reader(child,control,receipt,session)
        assert result['confirmed'],result
        assert result['receipt']['reason'] in ('stop_requested','stop_requested_before_start'),result
        report.update(passed=True,cleanup=result)
    except Exception as exc:report['error']=repr(exc)
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=2)
        log.close();report['elapsed_s']=time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
