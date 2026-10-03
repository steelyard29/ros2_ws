"""Correlate measured reader delays, never infer an unmeasured root cause."""


def required_streams_present(streams, stereo=False):
    keys=('pose','tracking','infra1','infra2') if stereo else ('pose','tracking')
    return all(streams.get(k,{}).get('count',0)>0 for k in keys)


def delay_evidence(packets, timing):
    poses=[p for p in packets if p.get('kind')=='pose']
    events=[]
    for previous,current in zip(poses,poses[1:]):
        before=previous['reader_receipt_monotonic_s']
        after=current['reader_receipt_monotonic_s']
        source_gap=(current['stamp_ns']-previous['stamp_ns'])/1e9
        clock=current.get('reader_clock',{})
        age=((clock['ros_ns']-current['stamp_ns'])/1e9 if 'ros_ns' in clock else None)
        if after-before<=.3 and (age is None or age<=.25):continue
        overlaps=[e for e in (timing or {}).get('slow_calls',[])
                  if e['start']<after and e['end']>before]
        events.append(dict(seq=current.get('seq'),reader_gap_s=after-before,
            source_gap_s=source_gap,reader_source_age_s=age,
            overlapping_measured_calls=overlaps))
    return dict(scope='Timing correlation only; overlap is not proof of causation.',
        timing_available=timing is not None,
        bounded_slow_history=True,pose_samples=len(poses),events=events,
        warning='Absent overlap does not exclude earlier truncated events, scheduling, DDS or upstream delay.')
