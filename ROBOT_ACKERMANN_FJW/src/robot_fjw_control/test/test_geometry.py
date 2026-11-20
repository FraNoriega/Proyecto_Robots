import math
import random
from robot_fjw_control.planning import Pose, dubins, make_fjw, transform_samples, advance
from robot_fjw_control.tracking import PurePursuit


def test_dubins_endpoint_randomized():
    rng = random.Random(5)
    for _ in range(200):
        a = Pose(rng.uniform(-5,5), rng.uniform(-5,5), rng.uniform(-3,3))
        b = Pose(rng.uniform(-5,5), rng.uniform(-5,5), rng.uniform(-3,3))
        dubins(a, b)


def test_complete_route():
    path = transform_samples(make_fjw(), Pose(0,0,0))
    follower = PurePursuit(path)
    robot = Pose(0,0,0)
    for _ in range(20000):
        v, omega = follower.update(robot.x, robot.y, robot.yaw)
        if follower.done:
            break
        robot = advance(robot, v*0.05, omega/v if v else 0)
    assert follower.done
    assert math.hypot(robot.x-path[-1].x, robot.y-path[-1].y) < 0.12
