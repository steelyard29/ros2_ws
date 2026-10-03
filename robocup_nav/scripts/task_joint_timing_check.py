"""Same-window task-input + independent PX4 subscriber. No EV or controls.

Default describes. Explicit execution uses existing sensors only, <=30 seconds,
one attempt; original task observation gates and self-deadline remain intact.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]


def observe(path,deadline):
    from px4_runtime_transport import configure
    configure()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import EstimatorStatusFlags
    from disarmed_ev_lifecycle import atomic_json
    rclpy.init(domain_id=0)
    node=rclpy.create_node('joint_flags_independent_readonly',enable_rosout=False,
                           start_parameter_services=False)
    rows=[]
    def callback(m):
        now_ns=node.get_clock().now().nanoseconds
        rows.append(dict(stamp_us=int(m.timestamp),at=time.monotonic(),
                         age_s=(now_ns-int(m.timestamp)*1000)/1e9))
    node.create_subscription(EstimatorStatusFlags,'/fmu/out/estimator_status_flags',
                             callback,qos_profile_sensor_data)
    try:
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        node.destroy_node();rclpy.try_shutdown()
        atomic_json(path,dict(rows=rows,closed=True,publishers_created=0))


def compare(task_rows,independent):
    indexed={r['stamp_us']:r for r in independent}
    return [dict(stamp_us=r['stamp_us'],task_age_s=r['age_s'],
                 independent_age_s=indexed[r['stamp_us']]['age_s'],
                 task_minus_independent_receipt_s=r['at']-indexed[r['stamp_us']]['at'])
            for r in task_rows if r['stamp_us'] in indexed]


def shared_gaps(task_rows,independent,limit_s=2.):
    """Common observed endpoints do not prove an intermediate packet existed."""
    def gaps(rows):
        return {(a['stamp_us'],b['stamp_us']):b['at']-a['at']
                for a,b in zip(rows,rows[1:]) if b['at']-a['at']>limit_s}
    first,second=gaps(task_rows),gaps(independent)
    return [dict(from_stamp_us=a,to_stamp_us=b,source_gap_s=(b-a)/1e6,
                 task_gap_s=first[(a,b)],independent_gap_s=second[(a,b)])
            for a,b in sorted(first.keys()&second.keys())]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-readonly',action='store_true')
    p.add_argument('--observer',type=Path);p.add_argument('--deadline',type=float)
    a=p.parse_args()
    if a.observer:
        if a.deadline is None or not 0<a.deadline-time.monotonic()<=27:
            p.error('observer deadline must be within 27 seconds')
        observe(a.observer,a.deadline);return 0
    if not a.execute_readonly:
        print('Describe only: one <=30s simultaneous task/independent flags check; no EV/control.');return 0
    from disarmed_ev_lifecycle import atomic_json
    out=ROOT/'evidence'/('task_joint_timing_'+uuid.uuid4().hex);out.mkdir()
    start=time.monotonic();children=[]
    report=dict(read_only=True,ev=False,controls=False,started_at=start)
    try:
        with (out/'task.log').open('w') as log,(out/'observer.log').open('w') as obslog:
            observer=subprocess.Popen([sys.executable,__file__,'--observer',str(out/'independent.json'),
                '--deadline',str(start+26)],stdout=obslog,stderr=obslog)
            children.append(observer)
            task=subprocess.Popen([sys.executable,str(ROOT/'scripts/task_readonly_session.py'),
                '--observe-existing-inputs','--container-inputs'],stdout=log,stderr=log)
            children.append(task)
            while any(c.poll() is None for c in children) and time.monotonic()<start+29:
                time.sleep(.02)
            report['exit_codes']=[c.poll() for c in children]
        paths=[Path(line) for line in (out/'task.log').read_text().splitlines()
               if line.startswith(str(ROOT/'evidence/task_readonly_'))]
        if len(paths)!=1:raise RuntimeError('task evidence path missing or ambiguous')
        task_report=json.loads((paths[0]/'report.json').read_text())
        observer_report=json.loads((out/'independent.json').read_text())
        report.update(task_evidence=str(paths[0]),task_error=task_report.get('error'),
            source_rejection=task_report.get('runtime',{}).get('source_rejection'),
            comparison=compare(task_report.get('runtime',{}).get('flags_timing',[]),observer_report['rows']),
            common_gaps=shared_gaps(task_report.get('runtime',{}).get('flags_timing',[]),observer_report['rows']))
    except Exception as exc:report['error']=repr(exc)
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=.3)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=.3)
        report.update(elapsed_s=time.monotonic()-start,closed=all(c.poll() is not None for c in children))
        atomic_json(out/'report.json',report)
    print(str(out))
    return 0 if not report.get('error') and report['closed'] else 2


if __name__=='__main__':raise SystemExit(main())
