"""Clock observations only; never correct timestamps or grant output permission."""
import time


def snapshot(ros_now, *, wall=time.time_ns, monotonic=time.monotonic_ns):
    before=monotonic()
    ros=ros_now()
    system=wall()
    after=monotonic()
    return dict(ros_ns=ros,system_ns=system,monotonic_before_ns=before,
                monotonic_after_ns=after,read_span_ns=after-before)
