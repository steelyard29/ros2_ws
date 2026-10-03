"""One bounded real EV + independent flags observation; no flight commands.

Default describe only. Service allowed 50 seconds, observer 54, cleanup capped
below 60 seconds. A failed service is not restarted. Reader cleanup remains
owned by the EV service, not by this observer.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-authorized-ev',action='store_true')
    p.add_argument('--observer',type=Path);p.add_argument('--deadline',type=float)
    a=p.parse_args()
    if a.observer:
        if a.deadline is None or not 0<a.deadline-time.monotonic()<=55:
            p.error('observer requires deadline within 55 seconds')
        from task_joint_timing_check import observe
        observe(a.observer,a.deadline);return 0
    if not a.execute_authorized_ev:
        print('Describe only: <=60s EV and independent flags, no control, no retry.');return 0
    from disarmed_ev_lifecycle import atomic_json
    from task_joint_timing_check import compare,shared_gaps
    out=ROOT/'evidence'/('resident_joint_'+uuid.uuid4().hex);out.mkdir()
    start=time.monotonic();children=[]
    report=dict(started_at=start,controls=False,automatic_retry=False)
    try:
        with (out/'service.log').open('w') as log,(out/'observer.log').open('w') as obslog:
            observer=subprocess.Popen([sys.executable,__file__,'--observer',str(out/'independent.json'),
                '--deadline',str(start+54)],stdout=obslog,stderr=obslog)
            children.append(observer)
            service=subprocess.Popen(['timeout','--foreground','--signal=TERM','--kill-after=3s','50s',
                sys.executable,str(ROOT/'scripts/resident_ev_service.py'),'--execute-real-ev'],
                stdout=log,stderr=log)
            children.append(service)
            while any(c.poll() is None for c in children) and time.monotonic()<start+57:
                time.sleep(.02)
            report['exit_codes']=[c.poll() for c in children]
        summary=json.loads((out/'service.log').read_text())
        path=Path(summary['evidence']);path.relative_to(ROOT/'evidence')
        service_report=json.loads((path/'report.json').read_text())
        independent=json.loads((out/'independent.json').read_text())['rows']
        rows=[dict(stamp_us=r['stamp_us'],at=r['at'],age_s=r['source_age_s'])
              for r in service_report['observation']['fusion_samples']]
        report.update(service_evidence=str(path),service_exit=service_report['exit_code'],
            service_reason=service_report['reason'],reader_closed=service_report['closed'],
            comparison=compare(rows,independent),common_gaps=shared_gaps(rows,independent),
            ev=service_report['ev'],fusion=service_report['observation']['fusion'])
    except Exception as exc:report['error']=repr(exc)
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=.3)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=.3)
        report.update(elapsed_s=time.monotonic()-start,child_processes_closed=all(c.poll() is not None for c in children))
        atomic_json(out/'report.json',report)
    print(str(out))
    return 0 if not report.get('error') and report.get('service_exit')==0 and report.get('reader_closed') else 2


if __name__=='__main__':raise SystemExit(main())
