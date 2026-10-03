#!/usr/bin/env python3
"""Read-only PX4 v1.16.2 switch DDS preparation. Never build, flash or connect.

Read the pinned tag from an existing repository, not its possibly newer working
tree. Print a minimal candidate patch or a compatibility report. Applying it to
the exact board/firmware is a separately approved maintenance operation.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import subprocess

VERSION = 'v1.16.2'
DDS_PATH = 'src/modules/uxrce_dds_client/dds_topics.yaml'
TOPIC = '/fmu/out/manual_control_switches'


def canonical_message(text):
    return [' '.join(line.split('#', 1)[0].split()) for line in text.splitlines()
            if line.split('#', 1)[0].strip()]


def add_switch_publication(text):
    if TOPIC in text:
        raise ValueError('switch topic already present; review instead of duplicate insertion')
    anchor = ('  - topic: /fmu/out/manual_control_setpoint\n'
              '    type: px4_msgs::msg::ManualControlSetpoint\n')
    if text.count(anchor) != 1:
        raise ValueError('unexpected DDS layout')
    return text.replace(anchor, anchor+'\n  - topic: '+TOPIC+
                        '\n    type: px4_msgs::msg::ManualControlSwitches\n', 1)


def inspect(repo, messages):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(repo), *args], text=True,
                                       timeout=10)
    source = git('show', VERSION+':'+DDS_PATH)
    msg = git('show', VERSION+':msg/ManualControlSwitches.msg')
    local = (messages/'ManualControlSwitches.msg').read_text()
    rc = git('show', VERSION+':src/modules/rc_update/rc_update.cpp')
    patched = add_switch_publication(source)
    compatible = canonical_message(msg) == canonical_message(local)
    report = {
        'firmware_contract': VERSION,
        'reference_commit': git('rev-parse', VERSION+'^{commit}').strip(),
        'working_tree_revision': git('describe', '--tags', '--always').strip(),
        'message_compatible': compatible,
        'message_sha256': hashlib.sha256(msg.encode()).hexdigest(),
        'dds_source_sha256': hashlib.sha256(source.encode()).hexdigest(),
        'switch_topic': TOPIC,
        'rc_source_documents_one_hz': 'publish immediately on change or at ~1 Hz' in rc,
        'source_files_modified': False, 'hardware_accessed': False,
        'built': False, 'flashed': False, 'flight_approved': False,
        'remaining': ['identify actual board and installed build hash',
                      'backup current firmware and parameters; approve maintenance route',
                      'build exact board from reviewed v1.16.2 base with existing custom changes preserved',
                      'check installed ROS message definition and all existing DDS contracts',
                      'separately approve flashing; revalidate every affected bench gate',
                      'observe actual DDS switches, sample times, RC loss and three physical positions'],
    }
    patch = ''.join(difflib.unified_diff(source.splitlines(True), patched.splitlines(True),
                                       fromfile='a/'+DDS_PATH, tofile='b/'+DDS_PATH))
    return report, patch


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=root/'src/third_party/PX4-Autopilot')
    parser.add_argument('--messages', type=Path, default=root/'src/third_party/px4_msgs/msg')
    parser.add_argument('--format', choices=('report', 'patch'), default='report')
    args = parser.parse_args()
    report, patch = inspect(args.repo, args.messages)
    if not report['message_compatible']:
        print(json.dumps(report, indent=2))
        return 1
    print(patch if args.format == 'patch' else json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
