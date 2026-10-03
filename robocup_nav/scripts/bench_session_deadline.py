"""Optional monotonic budget for ONE authorized bench session; no hardware."""
import math
import signal
import time


class SessionBudget:
    CLEANUP = 17.0
    FINAL_SAMPLE = 3.0

    def __init__(self, deadline, clock=time.monotonic):
        self.clock = clock
        self.started = clock()
        if deadline is not None and (not math.isfinite(deadline)
                or not 25 <= deadline-self.started <= 60):
            raise ValueError('session deadline must be 25..60 seconds ahead')
        self.deadline = deadline

    @property
    def active_end(self):
        return math.inf if self.deadline is None else self.deadline-self.CLEANUP

    def stage_end(self, duration, *, final=False):
        reserve = 0 if final else self.FINAL_SAMPLE
        return min(self.clock()+duration, self.active_end-reserve)

    def require_active(self):
        if self.clock() >= self.active_end-self.FINAL_SAMPLE:
            raise TimeoutError('session budget exhausted before activation')

    def cleanup_timeout(self, maximum):
        return maximum if self.deadline is None else max(0, min(maximum, self.deadline-self.clock()-1))


class DeadlineAlarm:
    """Interrupt blocking work before the reserved cleanup period, Unix main thread."""
    def __init__(self, deadline):
        self.deadline = deadline
        self.previous = None

    def arm(self):
        if not math.isfinite(self.deadline):
            return
        if signal.getitimer(signal.ITIMER_REAL)[0] != 0:
            raise RuntimeError('refusing to replace another timer')
        remaining = self.deadline-time.monotonic()
        if remaining <= 0:
            raise TimeoutError('deadline expired before activation')
        self.previous = signal.signal(signal.SIGALRM, self.expired)
        signal.setitimer(signal.ITIMER_REAL, remaining)

    @staticmethod
    def expired(signum, frame):
        raise TimeoutError('authorized session active window expired')

    def cancel(self):
        if self.previous is not None:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, self.previous)
            self.previous = None
