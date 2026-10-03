"""Match existing source message timestamps, not arrival times or fabricated time.

PX4 stamp uses the existing VehicleLocalPosition.timestamp contract (DDS-adjusted
microseconds converted to ns), not a newly assumed timestamp_sample clock.
Only task initialization uses this bounded history; no interpolation/extrapolation.
"""


def closest_local(vio_ns,history,now_ns,now,resets):
    candidates=[]
    for stamp,sample in history:
        if (type(stamp) is int and stamp>0 and 0<=now_ns-stamp<=250_000_000
                and 0<=now-sample.local_at<=.25
                and sample.xy_valid and sample.z_valid and not sample.armed and sample.landed
                and tuple(sample.reset_counters)==tuple(resets)
                and abs(stamp-vio_ns)<=50_000_000):
            candidates.append((stamp,sample))
    return min(candidates,key=lambda row:abs(row[0]-vio_ns),default=None)
