"""Fitting one tracking space onto another.

Two sets of corresponding points, each pair being the same physical place
seen by two different systems. The job is the rotation and translation that
takes one set onto the other.

Two solvers are here and the difference matters. `solve_rigid` fits all six
degrees of freedom. `solve_yaw` fits only a turn about the vertical axis,
which is what this problem actually allows - see its docstring. In practice
the constrained fit is the one to use: the free fit will invent a tilt out of
noise, and when the devices are carried around without being turned it has
nothing to pin that tilt down, so each run lands somewhere different.

Both have a RANSAC wrapper, because real capture data contains pairs that are
simply wrong - a device slipped, or lost tracking for a moment and kept
reporting its last position while the other one moved on.
"""

import math
import random

from .transform import quat_to_matrix


def _jacobi_largest(matrix, sweeps=60):
    """Largest eigenvector of a symmetric 4x4, by cyclic Jacobi rotations."""
    a = [row[:] for row in matrix]
    v = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]

    for _ in range(sweeps):
        off = 0.0
        for i in range(4):
            for j in range(i + 1, 4):
                off += a[i][j] * a[i][j]
        if off < 1e-18:
            break

        for p in range(3):
            for q in range(p + 1, 4):
                if abs(a[p][q]) < 1e-18:
                    continue
                theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
                t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c
                for k in range(4):
                    akp = a[k][p]
                    akq = a[k][q]
                    a[k][p] = c * akp - s * akq
                    a[k][q] = s * akp + c * akq
                for k in range(4):
                    apk = a[p][k]
                    aqk = a[q][k]
                    a[p][k] = c * apk - s * aqk
                    a[q][k] = s * apk + c * aqk
                for k in range(4):
                    vkp = v[k][p]
                    vkq = v[k][q]
                    v[k][p] = c * vkp - s * vkq
                    v[k][q] = s * vkp + c * vkq

    best = max(range(4), key=lambda i: a[i][i])
    vec = [v[i][best] for i in range(4)]
    norm = math.sqrt(sum(x * x for x in vec))
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0], 0.0
    return [x / norm for x in vec], a[best][best]


def solve_rigid(src, dst):
    """Rotation and translation taking src points onto dst points.

    Horn's method: the quaternion is the largest eigenvector of a 4x4 built
    from the cross-covariance of the two centred point sets.

    Returns (quat, translation, rms_error), quat as x, y, z, w.
    """
    n = len(src)
    if n < 3:
        raise ValueError("need at least three samples")

    cs = [sum(p[i] for p in src) / n for i in range(3)]
    cd = [sum(p[i] for p in dst) / n for i in range(3)]

    s = [[0.0] * 3 for _ in range(3)]
    for a, b in zip(src, dst):
        va = [a[i] - cs[i] for i in range(3)]
        vb = [b[i] - cd[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                s[i][j] += va[i] * vb[j]

    sxx, sxy, sxz = s[0]
    syx, syy, syz = s[1]
    szx, szy, szz = s[2]

    n_matrix = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ]

    q, _ = _jacobi_largest(n_matrix)
    # Horn orders the eigenvector (w, x, y, z); everything else uses (x, y, z, w).
    quat = [q[1], q[2], q[3], q[0]]

    r = quat_to_matrix(quat)
    t = [cd[i] - sum(r[i][j] * cs[j] for j in range(3)) for i in range(3)]

    err = 0.0
    for a, b in zip(src, dst):
        pa = [sum(r[i][j] * a[j] for j in range(3)) + t[i] for i in range(3)]
        err += sum((pa[i] - b[i]) ** 2 for i in range(3))
    return quat, t, math.sqrt(err / n)


def solve_yaw(src, dst):
    """A turn about the vertical axis plus an offset, fitted in closed form.

    Both sides track their orientation with accelerometers, so both know which
    way is down. The same gravity vector cannot point two different ways,
    which means the only rotation between the two spaces is a turn about the
    vertical axis.

    Fitting all three rotation axes lets sensor noise invent a tilt that
    cannot physically exist, and - worse - when the devices are carried around
    without being turned, the extra freedom has nothing to pin it down and the
    answer wanders between runs.

    Returns (quat, translation, rms_error) in the same shape as solve_rigid.
    """
    n = len(src)
    if n < 2:
        raise ValueError("need at least two samples")

    ax = sum(p[0] for p in src) / n
    ay = sum(p[1] for p in src) / n
    az = sum(p[2] for p in src) / n
    bx = sum(p[0] for p in dst) / n
    by = sum(p[1] for p in dst) / n
    bz = sum(p[2] for p in dst) / n

    # In the ground plane the turn is  x' = c*x + s*z,  z' = -s*x + c*z.
    # Maximising the agreement between the turned source and the target gives
    # it directly.
    s1 = s2 = 0.0
    for a, b in zip(src, dst):
        au, av = a[0] - ax, a[2] - az
        bu, bv = b[0] - bx, b[2] - bz
        s1 += bu * au + bv * av
        s2 += bu * av - bv * au

    if abs(s1) < 1e-12 and abs(s2) < 1e-12:
        turn = 0.0
    else:
        turn = math.atan2(s2, s1)
    c, s = math.cos(turn), math.sin(turn)

    trans = [bx - (c * ax + s * az), by - ay, bz - (-s * ax + c * az)]
    quat = [0.0, math.sin(turn / 2.0), 0.0, math.cos(turn / 2.0)]

    err = 0.0
    for a, b in zip(src, dst):
        pa = (c * a[0] + s * a[2] + trans[0],
              a[1] + trans[1],
              -s * a[0] + c * a[2] + trans[2])
        err += sum((pa[i] - b[i]) ** 2 for i in range(3))
    return quat, trans, math.sqrt(err / n)


def residuals(quat, translation, src, dst):
    r = quat_to_matrix(quat)
    out = []
    for a, b in zip(src, dst):
        pa = [sum(r[i][j] * a[j] for j in range(3)) + translation[i]
              for i in range(3)]
        out.append(math.sqrt(sum((pa[i] - b[i]) ** 2 for i in range(3))))
    return out


def _robust(src, dst, solver, min_points, iterations, threshold, seed):
    n = len(src)
    if n < min_points:
        raise ValueError(f"need at least {min_points} samples")

    if threshold is None:
        # 2 cm: below the noise of holding two devices together by hand, above
        # the millimetres both systems achieve on their own.
        threshold = 0.02

    rng = random.Random(seed)
    pool = list(range(n))
    # 1200 points is plenty to find a consensus and keeps pure-Python RANSAC
    # quick.
    if n > 1200:
        pool = rng.sample(pool, 1200)

    best_inliers = None
    best_count = -1
    best_q = best_t = None

    for _ in range(iterations):
        pick = rng.sample(pool, min_points)
        try:
            q, t, _r = solver([src[i] for i in pick], [dst[i] for i in pick])
        except (ValueError, ZeroDivisionError):
            continue
        res = residuals(q, t, [src[i] for i in pool], [dst[i] for i in pool])
        inliers = [pool[i] for i, r in enumerate(res) if r < threshold]
        if len(inliers) > best_count:
            best_count = len(inliers)
            best_inliers = inliers
            best_q, best_t = q, t

    if best_inliers is None or len(best_inliers) < min_points:
        # Nothing agreed; fall back to the plain fit.
        return solver(src, dst)

    inl = best_inliers
    q, t, r = solver([src[i] for i in inl], [dst[i] for i in inl])
    for _ in range(3):
        res = residuals(q, t, src, dst)
        inl = [i for i, v in enumerate(res) if v < threshold]
        if len(inl) < min_points:
            break
        q2, t2, r2 = solver([src[i] for i in inl], [dst[i] for i in inl])
        if r2 >= r:
            break
        q, t, r = q2, t2, r2

    return q, t, r


def robust_solve(src, dst, iterations=120, threshold=None, seed=1):
    """Full six-degree-of-freedom fit that tolerates bad pairs."""
    return _robust(src, dst, solve_rigid, 3, iterations, threshold, seed)


def robust_solve_yaw(src, dst, iterations=120, threshold=None, seed=1):
    """Heading-only fit that tolerates bad pairs."""
    return _robust(src, dst, solve_yaw, 2, iterations, threshold, seed)


def quat_angle_between(a, b):
    """Angle in degrees between two orientations."""
    dot = abs(sum(a[i] * b[i] for i in range(4)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def yaw_spread(quats):
    """How far a set of orientations was turned about the vertical, in degrees.

    Only the turn matters for a heading-only fit: the fixed offset between two
    devices turns with them, so turning them is what lets the offset be told
    apart from the translation. Pitching them up and down does not help.

    A set of headings has no natural start, so the spread is what remains
    after the widest empty arc - the smallest wedge containing them all.
    """
    if len(quats) < 2:
        return 0.0
    angles = sorted(math.degrees(_yaw_of(q)) % 360.0 for q in quats)
    gaps = [angles[i + 1] - angles[i] for i in range(len(angles) - 1)]
    gaps.append(angles[0] + 360.0 - angles[-1])
    return 360.0 - max(gaps)


def _yaw_of(quat):
    x, y, z, w = quat
    return math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))
