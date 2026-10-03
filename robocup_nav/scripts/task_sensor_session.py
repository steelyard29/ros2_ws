#!/usr/bin/env python3
"""One explicitly authorized <=90s sensor session; no flight or servo route.

Only owns an initially STOPPED, inspected container. Container shutdown is the
outer fallback, so killing docker-exec alone cannot leave cameras running.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
CONTAINER='isaac_ros_dev'


def validate_container(c):
    if c['State']['Running'] or c['State']['Status']!='exited':
        raise ValueError('requires initially exited container; will not stop other work')
    if c['Config']['Entrypoint']!=['/usr/local/bin/scripts/deploy-entrypoint.sh'] or c['Config']['Cmd']!=['sleep','infinity']:
        raise ValueError('unexpected container startup command')
    if not any(m.get('Source')==str(ROOT.parent) and m.get('Destination')=='/workspaces/ros2_ws'
               for m in c['Mounts']):raise ValueError('workspace mount mismatch')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--authorized-sensor-only',action='store_true')
    args=p.parse_args()
    if not args.authorized_sensor_only:p.error('explicit one-session sensor authorization required')
    before=json.loads(subprocess.check_output(['docker','inspect',CONTAINER],text=True,timeout=4))[0]
    validate_container(before)
    out=ROOT/'evidence'/time.strftime('task_sensor_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();end=started+90;active_end=started+70
    report=dict(authorization_consumed=True,limit_s=90,started_monotonic=started,
        active_end_monotonic=active_end,hardware_retry=False,flight_authorized=False,
        passed=False,initial_container_state='exited')
    (out/'session.json').write_text(json.dumps(report,indent=2)+'\n')
    process=None
    log=(out/'session.log').open('w')
    try:
        subprocess.run(['docker','start',CONTAINER],check=True,timeout=10,stdout=log,stderr=log)
        target='/workspaces/ros2_ws/robocup_nav/evidence/'+out.name
        command=('set -e; source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
                 'exec python3 /workspaces/ros2_ws/robocup_nav/tests/task_sensor_probe.py '
                 '--active-end '+str(active_end)+' --out '+target)
        process=subprocess.Popen(['docker','exec','-e','ROS_DOMAIN_ID=176','-e','ROS_LOCALHOST_ONLY=1',
                                  CONTAINER,'bash','-c',command],stdout=log,stderr=log)
        try:process.wait(timeout=max(.1,started+81-time.monotonic()))
        except subprocess.TimeoutExpired:report['probe_timeout']=True
        report['probe_exit_code']=process.poll()
    except Exception as exc:report['error']=repr(exc)
    finally:
        # Exact owned container only. QGC/Agent host services are not touched.
        try:
            subprocess.run(['docker','stop','--time','1',CONTAINER],check=True,timeout=3,stdout=log,stderr=log)
        except Exception as exc:
            report['stop_error']=repr(exc)
            try:subprocess.run(['docker','kill',CONTAINER],timeout=2,stdout=log,stderr=log)
            except Exception as kill_error:report['kill_error']=repr(kill_error)
        if process and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=1)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=1)
        try:
            state=json.loads(subprocess.check_output(['docker','inspect','-f','{{json .State}}',CONTAINER],text=True,timeout=2))
            report['container_stopped']=not state['Running'] and state['Pid']==0
        except Exception as exc:report['state_error']=repr(exc)
        report['elapsed_s']=time.monotonic()-started
        report['deadline_met']=time.monotonic()<=end
        probe_path=out/'report.json'
        if probe_path.exists():report['probe_report']=str(probe_path)
        report['passed']=bool(report.get('container_stopped') and report['deadline_met']
            and report.get('probe_exit_code')==0 and probe_path.exists())
        (out/'session.json').write_text(json.dumps(report,indent=2)+'\n')
        log.close();print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
