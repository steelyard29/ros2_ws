"""Bounded existing-VIO reader observation only. Default: describe and exit.

No ROS publishers, PX4 messages, Agent startup, camera startup or control APIs.
Uses the same container reader/pipe as commissioning, without its EV consumer.
"""
import argparse
import json
import os
from pathlib import Path
import select
import subprocess
import time
import uuid
from disarmed_ev_lifecycle import atomic_json, stop_reader
from bench_session_deadline import DeadlineAlarm
from vio_input_timing import stereo_gap_evidence
from reader_delay_analysis import delay_evidence, required_streams_present

ROOT = Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observe-existing-vio',action='store_true')
    p.add_argument('--include-stereo-inputs',action='store_true',
                   help='observe IR headers alongside VIO; no pixel recording, but adds subscriber load')
    args=p.parse_args()
    if not args.observe_existing_vio:
        print('Describe only. Explicit flag observes existing domain176 VIO for <=25s; no PX4 output.')
        return 0
    started=time.monotonic();sid=uuid.uuid4().hex
    out=ROOT/'evidence'/('ev_readonly_timing_'+sid);out.mkdir()
    control=out/'control.json';receipt=out/'closed.json'
    atomic_json(control,dict(session=sid,stop=False))
    container=lambda path:str(path).replace('/home/cfly/ros2_ws/','/workspaces/ros2_ws/')
    report=dict(subscription_only=True,px4_output=False,session=sid,complete=False)
    report['stereo_input_timing_requested']=args.include_stereo_inputs
    packets=[];child=None;alarm=DeadlineAlarm(started+28);alarm.arm()
    with (out/'stderr.log').open('w') as log:
        try:
            child=subprocess.Popen(['docker','exec','-e','ROS_LOCALHOST_ONLY=1','isaac_ros_dev','bash','-c',
                'source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh; '
                'exec python3 /workspaces/ros2_ws/robocup_nav/scripts/disarmed_ev_sensor_reader.py '
                f'--session {sid} --deadline {started+25} '
                f'--control-file {container(control)} --receipt-file {container(receipt)} '
                + ('--trace-image-inputs' if args.include_stereo_inputs else '')],
                stdout=subprocess.PIPE,stderr=log)
            os.set_blocking(child.stdout.fileno(),False);buffer=b''
            while time.monotonic()<started+24:
                if not select.select([child.stdout],[],[],.02)[0]:continue
                chunk=os.read(child.stdout.fileno(),65536)
                if not chunk:break
                buffer+=chunk
                if len(buffer)>131072:raise RuntimeError('observer pipe backlog')
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);d=json.loads(line)
                    d['host_receipt_monotonic_s']=time.monotonic()
                    if 'stamp_ns' in d:d['host_source_age_s']=(time.time_ns()-d['stamp_ns'])/1e9
                    if d['session']!=sid or d['seq']!=len(packets)+1:
                        raise RuntimeError('reader session/sequence mismatch')
                    packets.append(d)
                if any(x['kind']=='fault' for x in packets[-4:]):break
            report['complete']=time.monotonic()>=started+24 and not any(x['kind']=='fault' for x in packets)
        except Exception as exc:report['error']=repr(exc)
        finally:
            if child:
                try:report['cleanup']=stop_reader(child,control,receipt,sid)
                except Exception as exc:report['cleanup_error']=repr(exc)
                if child.poll() is None:
                    child.terminate()
                    try:child.wait(timeout=.5)
                    except subprocess.TimeoutExpired:child.kill();child.wait(timeout=.5)
            alarm.cancel()
    report['streams']={}
    for kind in ('pose','tracking','infra1','infra2'):
        rows=[d for d in packets if d['kind']==kind]
        if not rows:continue
        gaps=[(b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(rows,rows[1:])]
        report['streams'][kind]=dict(count=len(rows),source_span_s=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9,
            max_source_gap_s=max(gaps,default=0),source_gaps_over_300ms=sum(g>.3 for g in gaps),
            nonincreasing_source_stamps=sum(g<=0 for g in gaps),
            max_host_source_age_s=max(d['host_source_age_s'] for d in rows),
            max_reader_to_host_s=max(d['host_receipt_monotonic_s']-d['reader_receipt_monotonic_s'] for d in rows),
            max_reader_receipt_gap_s=max((b['reader_receipt_monotonic_s']-a['reader_receipt_monotonic_s'] for a,b in zip(rows,rows[1:])),default=0))
        if kind=='tracking':report['streams'][kind]['states']=sorted(set(d['state'] for d in rows))
    if args.include_stereo_inputs:
        report['stereo_gap_evidence']=stereo_gap_evidence(packets)
    if not required_streams_present(report['streams'],args.include_stereo_inputs):
        report['complete']=False
        report['error']='missing one or more requested input streams'
    timing=(report.get('cleanup',{}).get('receipt') or {}).get('reader_timing')
    report['reader_delay_evidence']=delay_evidence(packets,timing)
    report['elapsed_s']=time.monotonic()-started
    atomic_json(out/'packets.json',packets);atomic_json(out/'report.json',report)
    print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['complete'] and report.get('cleanup',{}).get('confirmed') else 1


if __name__=='__main__':raise SystemExit(main())
