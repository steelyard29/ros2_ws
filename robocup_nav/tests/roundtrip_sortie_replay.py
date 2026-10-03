"""Run the exact two-leg task in an ideal plant, with NO ROS publishers/devices."""
import json
import hashlib
from pathlib import Path
import time
from test_roundtrip_sortie import RoundTripRig


def main():
    rig=RoundTripRig()
    result=rig.run()
    report=dict(state=result['state'],reason=result.get('reason',''),
        simulated_elapsed_s=rig.t,maximum_forward_m=rig.max_forward,
        final_position_odom_m=list(map(float,rig.p)),legs=sorted(rig.legs),
        requested_relative_rise_m=.86,requested_px4_center_above_ground_m=1.,
        measured_ground_px4_center_above_ground_m=.14,
        maximum_simulated_px4_height_m=rig.max_px4_height,requested_forward_m=2.5,
        terminal_disarmed=not rig.armed,transitions=rig.mission.transitions,
        assumptions=['ideal plant','perfect localization','free synthetic map',
                     'synthetic H and landing geometry','direct goal-seeking velocity fixture'],
        actual_astar_apf_used_in_this_replay=False,real_px4_firmware_simulation=False,
        real_hardware_used=False,real_flight_adapter_available=False,flight_authorized=False,
        passed=result['state']=='DONE' and rig.legs=={'OUTBOUND','RETURN'}
            and 2.4<rig.max_forward<2.6 and .99<rig.max_px4_height<=1.00001
            and sum(v*v for v in rig.p[:2])<.05**2 and rig.p[2]<.003 and not rig.armed)
    root=Path(__file__).resolve().parents[1]
    report['source_sha256']={name:hashlib.sha256((root/name).read_bytes()).hexdigest()
        for name in ('scripts/roundtrip_flight_test.py','scripts/navigation_landing_shadow.py',
                     'scripts/sortie_shadow.py','scripts/landing_sequence_shadow.py',
                     'tests/test_roundtrip_sortie.py','tests/roundtrip_sortie_replay.py')}
    out=root/'evidence'/time.strftime('roundtrip_replay_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
