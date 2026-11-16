"""Frames, quaternions and the mapping between two tracking spaces.

A tracker source reports poses in whatever space its own hardware uses. The
runtime places SolarXR trackers in the headset's tracking origin, which is a
different space. Without a mapping the trackers are correct relative to each
other and wrong relative to everything else - they float somewhere near the
players' feet, or at the origin, or off in a corner.

`Transform` is that mapping. `identity()` is right only when the source and
the headset already share a space, which is unusual.

See tools/calibrate.py for how the mapping is measured.
"""

import json
import math
import os


def quat_to_matrix(q):
    """3x3 rotation matrix for a (x, y, z, w) quaternion."""
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_from_yaw(yaw_radians):
    """Heading-only rotation about +Y."""
    half = yaw_radians / 2.0
    return [0.0, math.sin(half), 0.0, math.cos(half)]


def yaw_of(quat):
    """Heading of an orientation, in radians, ignoring tilt."""
    x, y, z, w = quat
    return math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))


class Transform:
    """Rigid transform from a source's space into the headset's."""

    def __init__(self, quat, position):
        self.quat = list(quat)
        self.position = list(position)
        self.matrix = quat_to_matrix(self.quat)

    def __repr__(self):
        return (f"Transform(quat={self.quat}, position={self.position})")

    @classmethod
    def identity(cls):
        return cls([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    @classmethod
    def heading_only(cls, yaw_radians, position):
        """A yaw plus an offset, with the tilt fixed at zero.

        Both sides of this problem track their own orientation with
        accelerometers, so both agree on which way is down. The same gravity
        vector cannot point two different ways, which means the rotation
        between the two spaces is a turn about the vertical axis and nothing
        else. A full six-degree-of-freedom fit will happily invent a tilt out
        of sensor noise, and when the pair happens to be carried around
        without being turned, the extra freedom has nothing to pin it down and
        the answer wanders from run to run.
        """
        return cls(quat_from_yaw(yaw_radians), position)

    @classmethod
    def load(cls, path):
        """Read a transform written by the calibration tool, or None."""
        if not path or not os.path.exists(path):
            return None
        with open(path) as f:
            data = json.load(f)
        rotation = data["rotation"]
        position = data["translation"]
        return cls(
            [rotation["x"], rotation["y"], rotation["z"], rotation["w"]],
            [position["x"], position["y"], position["z"]],
        )

    @property
    def yaw_degrees(self):
        return math.degrees(yaw_of(self.quat))

    def apply_position(self, point):
        m = self.matrix
        return (
            m[0][0] * point[0] + m[0][1] * point[1] + m[0][2] * point[2] + self.position[0],
            m[1][0] * point[0] + m[1][1] * point[1] + m[1][2] * point[2] + self.position[1],
            m[2][0] * point[0] + m[2][1] * point[1] + m[2][2] * point[2] + self.position[2],
        )

    def apply_orientation(self, orientation):
        """Map an orientation into the headset's space.

        An orientation takes the device's own axes into its room's axes.
        Moving to a different room means composing the frame rotation in
        front of it, not conjugating it - so this is a left-multiply.
        """
        return tuple(quat_mul(self.quat, list(orientation)))
