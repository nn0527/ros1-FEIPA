"""Height relative to the last fresh ground sample before arming, in ENU."""
import math


class TakeoffHeightReference:
    def __init__(self, max_sample_age_s=0.5):
        self.max_sample_age_s = max_sample_age_s
        self.armed = None
        self.ground_sample = None
        self.ground_z = None

    def observe_state(self, connected, armed, now):
        if not connected:
            return  # A link outage is not a landing/disarm event.
        if armed and self.armed is not True:
            sample = self.ground_sample
            self.ground_z = (sample[0] if sample and
                             0 <= now - sample[1] <= self.max_sample_age_s else None)
        elif not armed and self.armed is not False:
            self.ground_sample = None
            self.ground_z = None
        self.armed = bool(armed)

    def observe_odom(self, z, now, confirmed_disarmed):
        if confirmed_disarmed and self.armed is False and math.isfinite(z):
            self.ground_sample = (z, now)

    def height(self, z):
        reference = self.ground_z if self.armed else (
            self.ground_sample[0] if self.ground_sample else None)
        return z - reference if reference is not None and math.isfinite(z) else None
