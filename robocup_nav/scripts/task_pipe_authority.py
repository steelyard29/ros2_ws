"""Local launch contract, not authentication against a writable host workspace.

Only a validated, consumed dedicated task permit can grant real navigation
topics. Reader rechecks the shared one-use marker and session/deadline binding.
No arbitrary topics, PX4 publishers, new flight permission or automatic renewal.
"""
import json
import math
from pathlib import Path
import time
import uuid

READONLY_SECONDS=25.
SORTIE_SECONDS=200.  # startup + <=175 s executive + shutdown margin
EXECUTIVE_SECONDS=175.
ISOLATED_SORTIE_SECONDS=150.


def reader_budget(*,isolated=False,navigation_writes=False,live=False,isolated_sortie=False):
    if isolated_sortie:
        if not isolated or not navigation_writes or live:
            raise ValueError('long synthetic reader requires isolated navigation only')
        return ISOLATED_SORTIE_SECONDS
    return SORTIE_SECONDS if live else READONLY_SECONDS


def grant_for(permit, session, deadline):
    from task_flight_release import TaskFlightPermit
    from flight_release import ROOT
    if type(permit) is not TaskFlightPermit:
        raise ValueError('real task pipe requires dedicated TaskFlightPermit')
    permit.before_arm()
    grant=dict(session=session,flight_session=permit.session_id,
               release_sha256=permit.release_hash,deadline=deadline,expires=permit.expires)
    verify_grant(grant,session,deadline,ROOT)
    return grant


def verify_grant(grant, session, deadline, root, *, monotonic=None, wall=None):
    now=time.monotonic() if monotonic is None else monotonic
    wall=time.time() if wall is None else wall
    if not isinstance(grant,dict):raise ValueError('missing task pipe authority')
    flight=grant.get('flight_session')
    if not isinstance(flight,str) or str(uuid.UUID(flight))!=flight:
        raise ValueError('noncanonical flight session')
    remaining=deadline-now
    if (not math.isfinite(remaining) or not 0<remaining<=SORTIE_SECONDS
            or grant.get('session')!=session or grant.get('deadline')!=deadline
            or type(grant.get('expires')) not in (int,float)
            or not math.isfinite(grant['expires']) or wall+remaining>grant['expires']):
        raise ValueError('task pipe session/deadline/expiry mismatch')
    digest=grant.get('release_sha256')
    if (not isinstance(digest,str) or len(digest)!=64
            or any(x not in '0123456789abcdef' for x in digest)):
        raise ValueError('invalid task release hash')
    marker=Path(root)/'evidence/live_release_consumed'/(flight+'.json')
    record=json.loads(marker.read_text())
    if record.get('session_id')!=flight or record.get('release_sha256')!=digest:
        raise ValueError('task release not consumed or marker mismatch')
    return grant
