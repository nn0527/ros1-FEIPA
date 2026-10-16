"""Sample-driven, time-bounded suspension of departure height correction.

This class never fabricates height or velocity. Safety/authority freshness is
checked by the ROS adapter; this component only assesses consistent samples.
Budgets include verification and correction blending; reset is for a new maneuver.
"""
import math


class HeightContinuation:
    def __init__(self, jump_m=.25, entry_tilt_deg=30., episode_s=1.2,
                 total_s=1.5, confirm_s=.30, min_samples=4, max_gap_s=.20,
                 step_residual_m=.05, residual_rate_mps=.20,
                 restore_tilt_deg=10., restore_speed_mps=.20, blend_s=.30):
        values = (jump_m, entry_tilt_deg, episode_s, total_s, confirm_s,
                  max_gap_s, step_residual_m, residual_rate_mps,
                  restore_tilt_deg, restore_speed_mps, blend_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('height continuation limits must be finite and positive')
        if (min_samples < 4 or int(min_samples) != min_samples or
                restore_tilt_deg > entry_tilt_deg or entry_tilt_deg >= 90 or
                episode_s > total_s or step_residual_m > jump_m):
            raise ValueError('inconsistent height continuation limits')
        self.jump_m, self.entry_tilt_deg = jump_m, entry_tilt_deg
        self.episode_s, self.total_s = episode_s, total_s
        self.confirm_s, self.min_samples, self.max_gap_s = confirm_s, int(min_samples), max_gap_s
        self.step_residual_m, self.residual_rate_mps = step_residual_m, residual_rate_mps
        self.restore_tilt_deg, self.restore_speed_mps = restore_tilt_deg, restore_speed_mps
        self.blend_s = blend_s
        self.reset()

    def reset(self):
        self.state = 'NORMAL'
        self.previous = None
        self.last_now = self.episode_at = self.blend_at = None
        self.total_used = 0.
        self.residual = self.peak_residual = 0.
        self.window = []
        self.window_residual = 0.
        self.reanchor_height = None
        self.weight = 1.
        self.failure = ''

    @property
    def ready(self):
        return self.state == 'NORMAL' and not self.failure

    def fail(self, reason):
        self.state, self.failure, self.weight = 'FAILED', reason, 0.
        self.window = []

    def episode_age(self, now):
        return 0. if self.episode_at is None else max(0., now-self.episode_at)

    def _seed_window(self, sample, qualified):
        self.window = [sample] if qualified else []
        self.window_residual = 0.

    def observe(self, now, source, height, velocity, tilt_deg):
        self.reanchor_height = None
        if self.state == 'FAILED':
            return
        if not all(math.isfinite(v) for v in (now, source, height, velocity, tilt_deg)) or source <= 0:
            self.fail('高度续接反馈数值无效')
            return
        if self.last_now is not None and now < self.last_now:
            self.fail('高度续接时钟倒退')
            return
        active = self.state in ('SUPPORT', 'BLENDING')
        if active:
            self.total_used += max(0., now-self.last_now)
            if self.episode_age(now) > self.episode_s+1e-9 or self.total_used > self.total_s+1e-9:
                self.fail('高度续接时间预算耗尽')
                return
            if tilt_deg > self.entry_tilt_deg+1e-9:
                self.fail('高度续接实测倾角越界')
                return
        self.last_now = now
        # Old/duplicate samples never move the high-water mark or recovery window.
        if self.previous is not None and source <= self.previous[0]:
            return
        sample = (source, height, velocity, now)
        old, self.previous = self.previous, sample
        gap = None if old is None else source-old[0]
        self.residual = 0. if old is None else abs(height-old[1]-.5*(velocity+old[2])*gap)
        self.peak_residual = max(self.peak_residual, self.residual)
        if self.state == 'NORMAL':
            if old is None or self.residual <= self.jump_m+1e-12:
                return
            if tilt_deg > self.entry_tilt_deg+1e-9:
                self.fail('脱离高度估计发生不连续跳变；实测倾角超过续接范围')
                return
            if self.total_used >= self.total_s-1e-9:
                self.fail('高度续接累计时间预算耗尽')
                return
            self.state, self.weight, self.episode_at = 'SUPPORT', 0., now
            self.blend_at = None
            self._seed_window(sample, False)
        qualified = tilt_deg <= self.restore_tilt_deg+1e-9 and abs(velocity) <= self.restore_speed_mps+1e-9
        consistent = (old is not None and gap <= self.max_gap_s+1e-9
                      and self.residual <= self.step_residual_m+1e-12)
        if self.state == 'BLENDING':
            if not qualified or not consistent:
                self.state, self.weight, self.blend_at = 'SUPPORT', 0., None
                self._seed_window(sample, qualified)
                return
            self.weight = min(1., max(0., (now-self.blend_at)/self.blend_s))
            if self.weight >= 1.-1e-9:
                self.state, self.weight, self.episode_at = 'NORMAL', 1., None
            return
        if self.state != 'SUPPORT':
            return
        if not qualified or not consistent or not self.window:
            self._seed_window(sample, qualified)
            return
        duration = source-self.window[0][0]
        accumulated = self.window_residual+self.residual
        if accumulated > self.residual_rate_mps*duration+1e-12:
            self._seed_window(sample, qualified)
            return
        self.window.append(sample)
        self.window_residual = accumulated
        if (duration+1e-9 >= self.confirm_s and now-self.window[0][3]+1e-9 >= self.confirm_s
                and len(self.window) >= self.min_samples):
            self.state, self.blend_at, self.weight = 'BLENDING', now, 0.
            self.reanchor_height = height
