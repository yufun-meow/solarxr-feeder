"""Checks on the transform solver, against data with a known answer.

Synthetic data is the only way to check this: real captures have no ground
truth, so a fit that is merely self-consistent can look perfect and still be
wrong.
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from solarxr_feeder import solve
from solarxr_feeder.transform import Transform


def _rigid_points(turn_degrees, translation, count=2000, noise=0.006, seed=7):
    turn = math.radians(turn_degrees)
    c, s = math.cos(turn), math.sin(turn)
    rotation = [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    rng = random.Random(seed)
    src, dst = [], []
    for _ in range(count):
        p = [rng.uniform(-2, 2), rng.uniform(-1, 1), rng.uniform(-2, 2)]
        d = [sum(rotation[i][j] * p[j] for j in range(3)) + translation[i]
             for i in range(3)]
        d = [d[i] + rng.gauss(0, noise) for i in range(3)]
        src.append(p)
        dst.append(d)
    return src, dst


def test_yaw_recovered_exactly():
    src, dst = _rigid_points(73.0, [-2.05, 1.53, -1.14])
    quat, translation, rms = solve.robust_solve_yaw(src, dst)
    yaw = math.degrees(2.0 * math.atan2(quat[1], quat[3]))
    assert abs(yaw - 73.0) < 0.5, f"yaw {yaw:.2f}° should be 73°"
    for axis in range(3):
        assert abs(translation[axis] - [-2.05, 1.53, -1.14][axis]) < 0.005
    assert rms < 0.02


def test_yaw_fit_never_tilts():
    """The tilt is the whole point of the constraint, so it must be exactly 0."""
    src, dst = _rigid_points(120.0, [1.0, 2.0, -3.0])
    quat, _t, _r = solve.robust_solve_yaw(src, dst)
    assert quat[0] == 0.0 and quat[2] == 0.0, "yaw-only fit produced a tilt"


def test_outliers_are_rejected():
    """A fifth of the pairs are wrong; the answer should barely move."""
    src, dst = _rigid_points(73.0, [-2.05, 1.53, -1.14])
    rng = random.Random(3)
    for i in rng.sample(range(len(src)), len(src) // 5):
        dst[i] = [dst[i][0] + rng.uniform(-0.5, 0.5) for _ in range(3)]

    clean_src, clean_dst = _rigid_points(73.0, [-2.05, 1.53, -1.14])
    quat, translation, _rms = solve.robust_solve_yaw(src, dst)
    yaw = math.degrees(2.0 * math.atan2(quat[1], quat[3]))
    assert abs(yaw - 73.0) < 1.0, f"yaw {yaw:.2f}° should still be ~73°"
    for axis in range(3):
        error = abs(translation[axis] - [-2.05, 1.53, -1.14][axis])
        assert error < 0.02, f"axis {axis} off by {error * 1000:.0f} mm"


def test_yaw_spread_measures_turning_only():
    def heading(degrees):
        half = math.radians(degrees) / 2.0
        return (0.0, math.sin(half), 0.0, math.cos(half))

    assert abs(solve.yaw_spread([heading(0), heading(90), heading(180)]) - 180.0) < 1e-6
    assert abs(solve.yaw_spread([heading(0), heading(30)]) - 30.0) < 1e-6
    # Pitching does not change the heading, so it does not count as coverage.
    assert solve.yaw_spread([(0.3, 0.0, 0.0, 0.95), (0.0, 0.0, 0.3, 0.95)]) < 1e-6


def test_transform_round_trip():
    transform = Transform.heading_only(math.radians(45.0), [1.0, 2.0, 3.0])
    point = (0.0, 0.0, 1.0)
    mapped = transform.apply_position(point)
    # A 45° turn about +Y takes +Z to +X (scaled by sin/cos).
    assert abs(mapped[0] - (1.0 + math.sin(math.radians(45.0)))) < 1e-9
    assert abs(mapped[1] - 2.0) < 1e-9


if __name__ == "__main__":
    passed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"  ok  {name}")
            passed += 1
    print(f"\n{passed} checks passed")
