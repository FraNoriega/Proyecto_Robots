"""ROS-independent LiDAR proximity guard for simulated vehicle, units: metres.

It is an emergency STOP, not collision avoidance or replanning.
"""
import math
import time


class LidarGuard:
    def __init__(self, stop_front=0.55, slow_front=1.00, stop_any=0.22,
                 scan_timeout=1.5, forward_half_angle=math.radians(30)):
        self.stop_front = float(stop_front)
        self.slow_front = float(slow_front)
        self.stop_any = float(stop_any)
        self.scan_timeout = float(scan_timeout)
        self.forward_half_angle = float(forward_half_angle)
        self.last_scan_time = None
        self.front_min = math.inf
        self.any_min = math.inf

    def update(self, ranges, angle_min, angle_increment, range_min, range_max, now=None):
        now = time.monotonic() if now is None else now
        front = math.inf
        any_distance = math.inf
        for i, distance in enumerate(ranges):
            if not math.isfinite(distance) or not (range_min <= distance <= range_max):
                continue
            any_distance = min(any_distance, distance)
            angle = math.atan2(math.sin(angle_min+i*angle_increment),
                               math.cos(angle_min+i*angle_increment))
            if abs(angle) <= self.forward_half_angle:
                front = min(front, distance)
        self.front_min = front
        self.any_min = any_distance
        self.last_scan_time = now

    def factor(self, now=None):
        """Return speed scale 0..1 and a human-readable stop/slow reason."""
        now = time.monotonic() if now is None else now
        if self.last_scan_time is None or now-self.last_scan_time > self.scan_timeout:
            return 0.0, 'No hay /scan reciente (LiDAR/bridge).'
        if self.any_min <= self.stop_any:
            return 0.0, f'Obstáculo demasiado cercano: {self.any_min:.2f} m.'
        if self.front_min <= self.stop_front:
            return 0.0, f'Obstáculo frontal: {self.front_min:.2f} m.'
        if self.front_min < self.slow_front:
            scale = max(0.15, min(1.0, (self.front_min-self.stop_front) /
                                   (self.slow_front-self.stop_front)))
            return scale, f'Aproximación a obstáculo: {self.front_min:.2f} m.'
        return 1.0, ''
