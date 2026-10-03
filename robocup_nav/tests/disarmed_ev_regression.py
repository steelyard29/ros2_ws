"""Persist no-hardware software regression evidence for the dedicated entry."""
from pathlib import Path
import hashlib
import json
import sys
import time
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
MODULES=['test_disarmed_ev_preoutput','test_disarmed_ev_fusion_evidence','test_vio_input_timing','test_preflight_source_guard','test_disarmed_ev_transport','test_disarmed_ev_session','test_disarmed_ev_timing','test_disarmed_ev_lifecycle','test_ev_odometry','test_ev_adapter_faults',
         'test_ev_session_guard','test_bench_session_deadline','test_flight_runtime']
if __name__=='__main__':
    out=ROOT/'evidence'/time.strftime('disarmed_ev_regression_%Y%m%d_%H%M%S');out.mkdir()
    started=time.monotonic()
    with (out/'unittest.log').open('w') as log:
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromNames(MODULES))
    files=[*ROOT.glob('scripts/disarmed_ev_*.py'),*ROOT.glob('tests/*disarmed_ev*.py'),ROOT/'scripts/flight_bench_check.py',
        ROOT/'scripts/preflight_source_guard.py',ROOT/'scripts/preflight_observer.py',
        ROOT/'tests/test_preflight_source_guard.py',
        ROOT/'scripts/vio_input_timing.py',ROOT/'tests/test_vio_input_timing.py',
        ROOT/'scripts/ev_bridge.py',ROOT/'config/platform.yaml',Path('/home/cfly/param.params.txt')]
    report=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),
        errors=len(result.errors),skipped=len(result.skipped),hardware=False,elapsed_s=time.monotonic()-started,
        modules=MODULES,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(evidence=str(out),**report),indent=2))
    raise SystemExit(0 if report['passed'] else 1)
