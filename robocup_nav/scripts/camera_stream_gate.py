"""ROS-free evidence for consecutive, fresh sensor messages at startup."""
import math


class StreamWindow:
    def __init__(self, min_frames, settle_s, max_gap_s):
        if (type(min_frames) is not int or min_frames < 2
                or not all(math.isfinite(x) and x > 0 for x in (settle_s, max_gap_s))):
            raise ValueError('invalid stream window limits')
        self.min_frames, self.settle_s, self.max_gap_s = min_frames, settle_s, max_gap_s
        self.count = self.total = self.rejected = 0
        self.first = self.last = self.first_stamp = self.last_stamp = None

    def reset(self):
        self.count = 0
        self.first = self.last = self.first_stamp = self.last_stamp = None

    def observe(self, stamp_ns, now, age_s, values_finite=True):
        if (type(stamp_ns) is not int or stamp_ns <= 0 or not values_finite
                or not math.isfinite(now) or not math.isfinite(age_s) or not -.05 <= age_s <= .5):
            self.rejected += 1
            self.reset()
            return False
        if self.last is not None:
            delta = (stamp_ns-self.last_stamp)/1e9
            if delta <= 0 or now < self.last:
                self.rejected += 1
                self.reset()
                return False
            if delta > self.max_gap_s or now-self.last > self.max_gap_s:
                self.reset()
        if self.first is None:
            self.first, self.first_stamp = now, stamp_ns
        self.last, self.last_stamp = now, stamp_ns
        self.count += 1
        self.total += 1
        return True

    def ready(self, now):
        return (self.last is not None and math.isfinite(now)
                and 0 <= now-self.last <= self.max_gap_s and self.count >= self.min_frames
                and self.last-self.first >= self.settle_s
                and (self.last_stamp-self.first_stamp)/1e9 >= self.settle_s)


def parameter_result_ok(result, expected_count):
    return (result is not None and len(result.results) == expected_count
            and expected_count > 0 and all(item.successful for item in result.results))
