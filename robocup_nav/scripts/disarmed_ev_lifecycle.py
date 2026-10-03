"""No ROS/device access: discovery classification and reader close handshake."""
import json
import math
import os
from pathlib import Path
import select
import time


def discovery_state(present, ev_topic, sources, changed, elapsed, previously_ready):
    """Keep the existing 2 s discovery limit; distinguish absence from conflict."""
    missing=[topic for topic,count in sources.items() if count==0]
    duplicates=[topic for topic,count in sources.items() if count>1]
    extras=[topic for topic,count in present.items() if count>0 and (topic!=ev_topic or count>1)]
    if present.get(ev_topic,0)==0:missing.append(ev_topic)
    if duplicates or extras:
        state,reason='conflict','duplicate input or competing flight publisher'
    elif changed:
        state,reason='generation_changed','PX4 source generation changed'
    elif missing:
        if previously_ready:state,reason='source_lost','previously discovered source missing'
        elif elapsed>=2:state,reason='discovery_timeout','required source not discovered within 2 seconds'
        else:state,reason='discovering','waiting for required sources; no output'
    else:state,reason='ready','all required sources uniquely discovered'
    return dict(state=state,reason=reason,missing=missing,duplicates=duplicates,
                competing=extras,changed=changed,source_counts=dict(sources),
                flight_input_counts=dict(present),elapsed_s=elapsed)


def atomic_json(path, data):
    path=Path(path);tmp=path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(data)+'\n');os.replace(tmp,path)


def request_stop(path, session):
    when=time.monotonic()
    atomic_json(path,dict(session=session,stop=True,requested_at=when))
    return when


def stop_requested(path, session):
    data=json.loads(Path(path).read_text())
    if data.get('session')!=session or type(data.get('stop')) is not bool:
        raise ValueError('reader control session/schema mismatch')
    return data['stop']


def check_receipt(path, session, returncode, now):
    try:
        d=json.loads(Path(path).read_text())
        times=[d['started_at'],d['closed_at'],now]
        ok=(d['session']==session and d['ros_closed'] is True
            and type(d['exit_code']) is int and returncode==d['exit_code']
            and all(math.isfinite(x) for x in times) and 0<times[0]<=times[1]<=times[2])
        return dict(confirmed=bool(ok),receipt=d,client_exit_code=returncode)
    except (OSError,ValueError,TypeError,KeyError):
        return dict(confirmed=False,receipt=None,client_exit_code=returncode)


def stop_reader(reader, control, receipt, session, timeout=2.):
    """Caller must destroy EV publisher first. Drain pipe while reader closes."""
    requested=request_stop(control,session);end=requested+timeout
    while reader.poll() is None and time.monotonic()<end:
        if reader.stdout and select.select([reader.stdout],[],[],.02)[0]:
            os.read(reader.stdout.fileno(),65536)
        else:time.sleep(.01)
    result=check_receipt(receipt,session,reader.poll(),time.monotonic())
    result.update(stop_requested_at=requested,checked_at=time.monotonic())
    return result
