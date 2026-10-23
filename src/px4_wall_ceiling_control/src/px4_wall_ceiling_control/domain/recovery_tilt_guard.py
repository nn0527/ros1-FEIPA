"""Measured-tilt priority for recovery, independent of velocity feedback epochs."""
import math


class RecoveryTiltGuard:
    def __init__(self, soft, stop, hard, hold_s, max_gap_s, resume_rate):
        values = (soft, stop, hard, hold_s, max_gap_s, resume_rate)
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or not soft < stop < hard < math.pi/2 or hold_s > 2.):
            raise ValueError('invalid recovery tilt-priority settings')
        self.soft, self.stop, self.hold_s = soft, stop, hold_s
        self.max_gap_s, self.resume_rate = max_gap_s, resume_rate
        self.paused = False
        self.reset()

    def reset(self, preserve_pause=True):
        self.paused = self.paused if preserve_pause else False
        self.weight = 0. if self.paused else 1.
        self.last_source = self.last_at = None
        self.since_source = self.since_at = None
        self.samples = 0

    def suspend(self):
        self.paused = True
        self.reset()

    def update(self, now, source, tilt, rate):
        if (not all(math.isfinite(v) for v in (now, source, tilt, rate))
                or source <= 0 or tilt < 0 or rate < 0):
            self.suspend()
            return self.weight
        backwards = ((self.last_at is not None and now < self.last_at)
                     or (self.last_source is not None and source < self.last_source))
        gap = ((self.last_at is not None and now-self.last_at > self.max_gap_s)
               or (self.last_source is not None and source-self.last_source > self.max_gap_s))
        new_sample = self.last_source is None or source > self.last_source
        if tilt >= self.stop-1.e-9 or backwards:
            self.suspend()
        if self.paused:
            if tilt > self.soft+1.e-9 or rate > self.resume_rate or backwards or gap:
                self.since_source = self.since_at = None
                self.samples = 0
            if tilt <= self.soft+1.e-9 and rate <= self.resume_rate and not backwards and new_sample:
                if self.since_source is None:
                    self.since_source, self.since_at = source, now
                self.samples += 1
                # Both real elapsed time and distinct source samples must qualify.
                if (self.samples >= 2 and source-self.since_source >= self.hold_s-1.e-9
                        and now-self.since_at >= self.hold_s-1.e-9):
                    self.paused = False
            self.weight = 0. if self.paused else 1.
        else:
            self.weight = (1. if tilt <= self.soft+1.e-9 else
                           max(0., min(1., (self.stop-tilt)/(self.stop-self.soft))))
        self.last_at, self.last_source = now, source
        return self.weight
