"""Navigation-specific release: a vertical-only FlightPermit is insufficient.

Only validates operator-reviewed existing artifacts; never creates acceptance
evidence or changes flight enable/parameters. Uses the original one-use marker.
"""
from dataclasses import dataclass,fields
import hashlib
import json
import math
from pathlib import Path
import yaml
from flight_release import FlightPermit,ROOT,sha,validate as validate_vertical
from landing_projection import rotation_matrix,nominal_landing_clearance
from task_site_bounds import resolve_bounds

TASK_EVIDENCE=('navigation_input_output','map_coverage_and_braking',
               'h_alignment_and_landing','task_takeover_and_loss')


def adapter_sha():
    paths=[*sorted((ROOT/'legacy_apf_shadow').glob('*.cpp')),
           *sorted((ROOT/'legacy_apf_shadow').glob('*.hpp')),
           ROOT/'legacy_apf_shadow/CMakeLists.txt',
           ROOT.parent/'src/uav_task/src/velocity_apf_controller.cpp',
           ROOT.parent/'src/uav_task/include/velocity_apf_controller.hpp']
    h=hashlib.sha256()
    for p in paths:h.update(str(p).encode()+b'\0'+p.read_bytes()+b'\0')
    return h.hexdigest()


def validate_task_config(c):
    if c.get('schema')!=1 or c.get('profile')!='roundtrip_1m_2p5m':raise ValueError('task profile mismatch')
    if c.get('landing_method')!='h_align_then_px4_land':raise ValueError('unsupported landing method')
    for key in ('landing_method_operator_confirmed','alignment_reviewed','full_height_slice_reviewed',
                'takeoff_column_reviewed','landing_column_reviewed'):
        if c.get(key) is not True:raise ValueError('unreviewed task prerequisite: '+key)
    resolve_bounds(c,(0.,0.),0.)
    error=c.get('landing_error_budget_m')
    clearance=nominal_landing_clearance(c.get('landing_geometry'))
    if (type(error) not in (int,float) or not math.isfinite(error)
            or not 0<=error<min(.05,clearance-.05)):
        raise ValueError('landing total uncertainty budget required within nominal 10cm clearance')
    v=c['stopping']
    if set(v)!={'body_radius','uncertainty','latency','braking'}:raise ValueError('stopping model fields')
    if (not all(type(x) in (int,float) and math.isfinite(x) for x in v.values())
            or v['body_radius']<math.hypot(.6,.6)/2 or min(v['uncertainty'],v['latency'])<0
            or v['braking']<=0):raise ValueError('invalid reviewed stopping model')
    camera=c['camera']
    offset=camera.get('offset_flu_m')
    if (not isinstance(offset,list) or len(offset)!=3
            or not all(type(x) in (int,float) and math.isfinite(x) for x in offset)):
        raise ValueError('explicit finite optical-center offset required')
    if camera['axes_reviewed'] is not True or camera['calibration_use_authorized'] is not True:
        raise ValueError('camera calibration use/axes not reviewed')
    rotation_matrix(camera['body_from_optical'])
    return c


@dataclass(frozen=True)
class TaskFlightPermit(FlightPermit):
    task_file: Path
    task_hash: str
    adapter_hash: str
    task_evidence: tuple

    @property
    def task_config(self):
        if sha(self.task_file)!=self.task_hash:raise ValueError('task config changed')
        return validate_task_config(yaml.safe_load(self.task_file.read_text()))

    def unchanged(self):
        super().unchanged()
        if (sha(self.task_file)!=self.task_hash or adapter_sha()!=self.adapter_hash
                or any(sha(p)!=h for p,h in self.task_evidence)):
            raise ValueError('task adapter/config/evidence changed')


def validate(release_path,params_path,platform_path):
    base,params=validate_vertical(release_path,params_path,platform_path)
    if not math.isclose(base.height,.86,abs_tol=1e-9):raise ValueError('task needs 0.86m relative rise')
    d=json.loads(Path(release_path).read_text()).get('navigation_task',{})
    if d.get('profile')!='roundtrip_1m_2p5m':raise ValueError('vertical permit cannot authorize horizontal task')
    path=Path(d.get('config_path','')).resolve()
    if path!=(ROOT/'config/roundtrip_task.yaml').resolve() or sha(path)!=d.get('config_sha256'):
        raise ValueError('task config binding mismatch')
    validate_task_config(yaml.safe_load(path.read_text()))
    if adapter_sha()!=d.get('adapter_sha256'):raise ValueError('APF source binding mismatch')
    entries=[]
    for key in (*TASK_EVIDENCE,'apf_binary'):
        item=d.get('evidence',{}).get(key,{})
        p=Path(item.get('path',''))
        if (item.get('reviewed') is not True or not p.is_absolute() or not p.is_file()
                or not p.stat().st_size or sha(p)!=item.get('sha256')):
            raise ValueError('missing reviewed task evidence: '+key)
        entries.append((p.resolve(),item['sha256']))
    permit=TaskFlightPermit(**{f.name:getattr(base,f.name) for f in fields(FlightPermit)},
        task_file=path,task_hash=d['config_sha256'],adapter_hash=d['adapter_sha256'],
        task_evidence=tuple(entries))
    permit.before_arm()
    return permit,params
