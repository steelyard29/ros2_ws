"""Single authorized sensor recovery after reboot, including Docker in 90s.

Only an initially exited, inspected idle container is owned. Success retains
perception; any failure stops this exact container. No PX4 or route goal API.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

from task_sensor_session import validate_container
from disarmed_ev_lifecycle import atomic_json

ROOT=Path(__file__).resolve().parents[1]
CONTAINER='isaac_ros_dev'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--authorized-sensor-recovery',action='store_true')
    args=parser.parse_args()
    if not args.authorized_sensor_recovery:
        print('Describe only: once <=90s, restore D435i/VIO/downward/lidar/TF/map/navigation candidates; no PX4 output.')
        return 0
    before=json.loads(subprocess.check_output(['docker','inspect',CONTAINER],text=True,timeout=4))[0]
    validate_container(before)
    cid=before['Id']
    out=ROOT/'evidence'/time.strftime('perception_reboot_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();end=started+90
    report=dict(passed=False,flight_ready=False,authorization_consumed=True,hardware_retry=False,
        limit_s=90,started_monotonic=started,deadline_monotonic=end,container_id=cid,
        px4_output=False,dds_agent_started=False,initial_container_state='exited')
    atomic_json(out/'session.json',report)
    process=None
    with (out/'session.log').open('w') as log:
        try:
            subprocess.run(['docker','start',cid],check=True,timeout=10,stdout=log,stderr=log)
            command=('set -e; source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
                'exec python3 /workspaces/ros2_ws/robocup_nav/scripts/perception_transport_trial.py '
                '--authorized-sensor-rollout --expect-stopped --restore-support-sensors '
                '--session-deadline '+str(end))
            process=subprocess.Popen(['docker','exec','-e','ROS_DOMAIN_ID=176','-e','ROS_LOCALHOST_ONLY=1',
                cid,'bash','-c',command],stdout=log,stderr=log)
            process.wait(timeout=max(.01,end-8-time.monotonic()))
            report['trial_exit_code']=process.returncode
            state=json.loads(subprocess.check_output(['docker','inspect','-f','{{json .State}}',cid],text=True,timeout=1))
            report['container_running']=state['Running']
            report['container_stopped']=not state['Running'] and state['Pid']==0
            report['passed']=process.returncode==0 and state['Running'] and time.monotonic()<end-8
        except Exception as exc:report['error']=repr(exc)
        finally:
            if not report['passed']:
                try:subprocess.run(['docker','stop','--time','1',cid],check=True,timeout=3,stdout=log,stderr=log)
                except Exception as exc:
                    report['stop_error']=repr(exc)
                    try:subprocess.run(['docker','kill',cid],check=True,timeout=2,stdout=log,stderr=log)
                    except Exception as exc:report['kill_error']=repr(exc)
            if process and process.poll() is None:
                process.kill()
                try:process.wait(timeout=.5)
                except subprocess.TimeoutExpired:report['docker_exec_reap_timeout']=True
            if not report['passed']:
                try:
                    state=json.loads(subprocess.check_output(['docker','inspect','-f','{{json .State}}',cid],text=True,timeout=1))
                    report['container_running']=state['Running']
                    report['container_stopped']=not state['Running'] and state['Pid']==0
                except Exception as exc:report['state_error']=repr(exc)
            report['elapsed_s']=time.monotonic()-started
            report['deadline_met']=time.monotonic()<=end
            report['passed']=report['passed'] and report['deadline_met']
            atomic_json(out/'session.json',report)
    print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
