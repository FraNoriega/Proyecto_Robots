import math
from robot_fjw_control.safety import LidarGuard


def test_front_and_side_safety():
    g = LidarGuard()
    assert g.factor(now=0)[0] == 0
    # One ray every 90 deg starting at -180: forward = index 2.
    g.update([math.inf, 2, 1.5, 2], -math.pi, math.pi/2, .1, 10, now=1)
    assert g.factor(now=1.1)[0] == 1
    g.update([math.inf, 2, 0.40, 2], -math.pi, math.pi/2, .1, 10, now=2)
    assert g.factor(now=2.1)[0] == 0
    g.update([math.inf, 0.18, 1.5, 2], -math.pi, math.pi/2, .1, 10, now=3)
    assert g.factor(now=3.1)[0] == 0
    g.update([math.inf, 2, 0.8, 2], -math.pi, math.pi/2, .1, 10, now=4)
    assert 0 < g.factor(now=4.1)[0] < 1
    assert g.factor(now=6)[0] == 0
