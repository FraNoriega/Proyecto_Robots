"""Pure pursuit controller independent of ROS for unit testing."""
from math import sin, cos, atan2, hypot


class PurePursuit:
    def __init__(self, samples, speed=0.12, lookahead=0.30, radius_min=0.75):
        self.samples = samples
        self.speed = speed
        self.lookahead = lookahead
        self.radius_min = radius_min
        self.progress = 0
        self.done = False
        self.lateral_error = 0.0
        self.target_index = 0

    def update(self, x, y, yaw):
        if self.done:
            return 0.0, 0.0
        n = len(self.samples)
        stop = min(n, self.progress + 45)
        closest = min(range(self.progress, stop),
                      key=lambda i: (self.samples[i].x - x)**2 +
                                    (self.samples[i].y - y)**2)
        self.progress = closest
        self.lateral_error = hypot(self.samples[closest].x-x, self.samples[closest].y-y)
        finish = self.samples[-1]
        if closest >= n-12 and hypot(x-finish.x, y-finish.y) < 0.12:
            self.done = True
            return 0.0, 0.0
        target_idx = closest
        # Avoid jumping to another geometrically close stroke: accumulate
        # *path distance*, rather than simply first Euclidean-distance hit.
        span = 0.0
        while target_idx < n-1 and span < self.lookahead:
            a, b = self.samples[target_idx], self.samples[target_idx+1]
            span += hypot(b.x-a.x, b.y-a.y)
            target_idx += 1
        self.target_index = target_idx
        target = self.samples[target_idx]
        dx = target.x - x
        dy = target.y - y
        distance = max(0.001, hypot(dx, dy))
        alpha = atan2(dy, dx) - yaw
        curvature = 2.0 * sin(alpha) / distance
        curvature = min(1/self.radius_min, max(-1/self.radius_min, curvature))
        # Lower speed on tight bends or moderate tracking error.
        turn_factor = 1.0 / (1.0 + 0.35 * abs(curvature))
        error_factor = max(0.35, 1.0 - self.lateral_error / 0.80)
        v = max(0.055, self.speed * turn_factor * error_factor)
        if target_idx > n-20:
            v = min(v, max(0.09, self.speed * hypot(x-finish.x, y-finish.y)/0.40))
        return v, v * curvature
