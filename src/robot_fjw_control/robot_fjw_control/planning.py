"""Geometric F-J-W strokes and curvature-constrained Dubins transfers.

No ROS imports: geometry is easily unit-testable without ROS/Gazebo.
All units are SI (m and radians). Curvature bound is 1 / TURN_RADIUS.
"""
from dataclasses import dataclass
from math import atan2, sin, cos, sqrt, acos, pi, hypot, isfinite

STEP = 0.035
TURN_RADIUS = 0.75  # > the measured robot minimum ~0.506 m
TAU = 2 * pi


def mod2pi(x):
    return x % TAU


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Sample:
    x: float
    y: float
    yaw: float
    draw: bool
    letter: str


def advance(p, length, curvature):
    """Exact bicycle-model integration of a straight or circular segment."""
    if abs(curvature) < 1.e-12:
        return Pose(p.x + length * cos(p.yaw), p.y + length * sin(p.yaw), p.yaw)
    end_yaw = p.yaw + length * curvature
    return Pose(p.x + (sin(end_yaw) - sin(p.yaw)) / curvature,
                p.y + (cos(p.yaw) - cos(end_yaw)) / curvature,
                end_yaw)


def _dubins_candidates(alpha, beta, d):
    sa, sb = sin(alpha), sin(beta)
    ca, cb = cos(alpha), cos(beta)
    cab = cos(alpha - beta)
    ans = []

    p2 = 2 + d*d - 2*cab + 2*d*(sa - sb)
    if p2 >= -1e-10:
        u = atan2(cb - ca, d + sa - sb)
        ans.append(('LSL', [mod2pi(-alpha + u), sqrt(max(0, p2)), mod2pi(beta - u)]))

    p2 = 2 + d*d - 2*cab + 2*d*(sb - sa)
    if p2 >= -1e-10:
        u = atan2(ca - cb, d - sa + sb)
        ans.append(('RSR', [mod2pi(alpha - u), sqrt(max(0, p2)), mod2pi(-beta + u)]))

    p2 = -2 + d*d + 2*cab + 2*d*(sa + sb)
    if p2 >= -1e-10:
        u = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, sqrt(max(0, p2)))
        ans.append(('LSR', [mod2pi(-alpha + u), sqrt(max(0, p2)), mod2pi(-beta + u)]))

    p2 = d*d - 2 + 2*cab - 2*d*(sa + sb)
    if p2 >= -1e-10:
        u = atan2(ca + cb, d - sa - sb) - atan2(2.0, sqrt(max(0, p2)))
        ans.append(('RSL', [mod2pi(alpha - u), sqrt(max(0, p2)), mod2pi(beta - u)]))

    u = (6 - d*d + 2*cab + 2*d*(sa - sb)) / 8.0
    if abs(u) <= 1.0:
        p = mod2pi(TAU - acos(u))
        t = mod2pi(alpha - atan2(ca - cb, d - sa + sb) + p/2)
        ans.append(('RLR', [t, p, mod2pi(alpha - beta - t + p)]))

    u = (6 - d*d + 2*cab + 2*d*(-sa + sb)) / 8.0
    if abs(u) <= 1.0:
        p = mod2pi(TAU - acos(u))
        t = mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p/2)
        ans.append(('LRL', [t, p, mod2pi(beta - alpha - t + p)]))
    return ans


def dubins(start, end, radius=TURN_RADIUS):
    """Return shortest forward-only (L/S/R, segment length in m) plan."""
    dx = end.x - start.x
    dy = end.y - start.y
    d = hypot(dx, dy) / radius
    direction = atan2(dy, dx)
    alpha, beta = mod2pi(start.yaw - direction), mod2pi(end.yaw - direction)
    candidates = _dubins_candidates(alpha, beta, d)
    if not candidates:
        raise ValueError('No Dubins connection found')
    word, values = min(candidates, key=lambda c: sum(c[1]))
    controls = list(zip(word, [v * radius for v in values]))
    # Assert the generated motion really reaches the target, even with angle wrapping.
    p = start
    for mode, length in controls:
        p = advance(p, length, {'L': 1/radius, 'S': 0, 'R': -1/radius}[mode])
    position_error = hypot(p.x-end.x, p.y-end.y)
    heading_error = abs(atan2(sin(p.yaw-end.yaw), cos(p.yaw-end.yaw)))
    if position_error > 2e-6 or heading_error > 2e-6:
        raise AssertionError(f'Dubins endpoint mismatch: {word} pos={position_error} yaw={heading_error}')
    return controls


class Route:
    def __init__(self):
        self.pose = Pose(-4.45, -1.25, pi/2)
        self.samples = [Sample(self.pose.x, self.pose.y, self.pose.yaw, False, '')]
        self.segment_index = 0

    def motion(self, length, curvature, draw, letter):
        if length < 0:
            raise ValueError('motions use nonnegative distance and forward gear')
        steps = max(1, round(length / STEP))
        ds = length / steps
        for _ in range(steps):
            self.pose = advance(self.pose, ds, curvature)
            self.samples.append(Sample(self.pose.x, self.pose.y, self.pose.yaw, draw, letter))

    def connect(self, x, y, yaw):
        end = Pose(x, y, yaw)
        if hypot(self.pose.x-end.x, self.pose.y-end.y) < 1e-5 and \
             abs(atan2(sin(self.pose.yaw-yaw), cos(self.pose.yaw-yaw))) < 1e-5:
            return
        for kind, distance in dubins(self.pose, end):
            self.motion(distance, {'L': 1/TURN_RADIUS, 'S': 0.0, 'R': -1/TURN_RADIUS}[kind], False, '')

    def stroke(self, x1, y1, x2, y2, letter):
        angle = atan2(y2-y1, x2-x1)
        self.connect(x1, y1, angle)
        self.motion(hypot(x2-x1, y2-y1), 0.0, True, letter)


def make_fjw():
    """F, J and W as drawable strokes and physically reachable unmarked transfers."""
    r = Route()
    # F: vertical spine, upper arm, middle arm.
    r.stroke(-4.45, -1.25, -4.45, 1.25, 'F')
    r.stroke(-4.45, 1.25, -2.75, 1.25, 'F')
    r.stroke(-4.45, 0.10, -3.20, 0.10, 'F')

    # J: downward vertical followed by a tangent semicircular-style bottom hook.
    r.stroke(0.45, 1.25, 0.45, -0.45, 'J')
    r.motion(TURN_RADIUS*pi/2, -1/TURN_RADIUS, True, 'J')
    r.motion(0.30, 0.0, True, 'J')

    # W: four sloping, finite-radius reachable strokes. The corners are broken
    # into independently navigable strokes with no drawing on transfers.
    r.stroke(2.0, 1.25, 2.52, -1.25, 'W')
    r.stroke(2.52, -1.25, 3.05, 0.92, 'W')
    r.stroke(3.05, 0.92, 3.58, -1.25, 'W')
    r.stroke(3.58, -1.25, 4.12, 1.25, 'W')
    return r.samples


def transform_samples(samples, origin):
    """Rotate/translate drawing to match initial odometry pose."""
    first = samples[0]
    angle = origin.yaw - first.yaw
    c, s = cos(angle), sin(angle)
    out = []
    for v in samples:
        dx, dy = v.x-first.x, v.y-first.y
        out.append(Sample(origin.x + c*dx - s*dy,
                          origin.y + s*dx + c*dy,
                          v.yaw + angle, v.draw, v.letter))
    return out
