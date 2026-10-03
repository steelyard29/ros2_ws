#!/usr/bin/env python3
"""Persist targeted unit-regression results and source hashes; no ROS nodes."""
from pathlib import Path
import sys
import time
import json
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_release import sha

MODULES='''test_task_site_bounds test_landing_projection test_task_transport_failure test_task_navigation_pipe test_task_pose_pairing test_task_perception_pipe test_task_readonly test_perception_reboot_session test_task_domain_io test_perception_observer_offline test_perception_trial_offline test_perception_acceptance test_perception_transport test_competition_rules test_navigation_goal_link test_task_handoff test_task_scene_input test_task_map_profile
test_flight_supervisor test_flight_live_core test_flight_release test_flight_runtime
test_roundtrip_flight_test test_roundtrip_sortie test_navigation_landing_shadow
test_navigation_shadow_pipeline test_landing_sequence_shadow test_sortie_shadow
test_sortie_ready_shadow test_sortie_legacy_boundary'''.split()


MODULES.append('test_apf_domain_policy')
MODULES.append('test_task_mixed_plant')
MODULES.append('test_task_dynamic_sources')
MODULES.append('test_px4_runtime_transport')

if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    out=root/'evidence'/time.strftime('task_regression_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    with (out/'unittest.log').open('w') as log:
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromNames(MODULES))
    files=[*root.glob('scripts/task_*.py'),*root.glob('scripts/navigation_goal_*.py'),
        root/'scripts/flight_runtime.py',root/'scripts/live_flight_runtime.py',
        root/'scripts/flight_supervisor.py',root/'scripts/live_flight_core.py',
        root/'legacy_apf_shadow/node.cpp',root/'launch/task_navigation.launch.py',
        root/'launch/ground_task_navigation.launch.py',root/'launch/nvblox.launch.py',
        root/'config/roundtrip_task.yaml',root/'config/competition_rules.yaml',
        root/'tests/test_competition_rules.py',root/'launch/perception.launch.py',
        root/'config/perception_dds_shm16m.xml',root/'tests/test_perception_transport.py',
        root/'scripts/perception_acceptance.py',root/'scripts/perception_transport_trial.py',
        root/'scripts/perception_reboot_session.py',root/'tests/test_perception_reboot_session.py',
        root/'tests/perception_recovery_observer.py',root/'tests/test_perception_acceptance.py',
        root/'tests/test_perception_observer_offline.py',root/'tests/test_perception_trial_offline.py',
        root/'scripts/nvblox_guard.py',root/'tests/test_task_domain_io.py',root/'tests/task_dual_domain_check.py']
    files.append(root/'tests/test_task_readonly.py')
    files.append(root/'tests/test_task_perception_pipe.py')
    files.append(root/'tests/task_pipe_check.py')
    files.append(root/'tests/task_runtime_pipe_check.py')
    files.append(root/'tests/test_task_pose_pairing.py')
    files.append(root/'tests/test_task_navigation_pipe.py')
    files.append(root/'tests/test_task_transport_failure.py')
    files.extend([root/'scripts/landing_projection.py',root/'tests/test_landing_projection.py',root/'tests/test_task_scene_input.py'])
    files.extend([root/'tests/test_task_site_bounds.py',root/'scripts/roundtrip_flight_test.py',
                  root/'scripts/navigation_landing_shadow.py',root/'scripts/landing_sequence_shadow.py'])
    files.append(root/'tests/task_configured_sortie_replay.py')
    files.extend([root/'scripts/px4_runtime_transport.py',root/'tests/test_px4_runtime_transport.py'])
    files.append(root/'tests/task_full_pipe_check.py')
    files.extend([root/'tests/task_dynamic_sources.py',root/'tests/test_task_dynamic_sources.py'])
    files.extend([root/'tests/task_mixed_plant.py',root/'tests/test_task_mixed_plant.py',root/'tests/test_task_handoff.py'])
    files.extend([root/'tests/task_actual_navigation_replay.py',root/'tests/test_apf_domain_policy.py',
                  root/'legacy_apf_shadow/domain_policy.hpp'])
    files.extend([root/'tests/image_transport_probe.py',root/'tests/fixtures/fastdds_udp_only.xml'])
    report=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),
        errors=len(result.errors),skipped=len(result.skipped),elapsed_s=time.monotonic()-started,
        modules=MODULES,hardware=False,source_sha256={str(p.relative_to(root)):sha(p) for p in files})
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(evidence=str(out),**report),indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
