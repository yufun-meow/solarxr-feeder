"""Poses from a CV1 tracker rig, over UDP.

The Windows side reads Oculus pose data and broadcasts it; this reads it
back. See docs/reference-reader.c for the sender and docs/PACKET.md for the
wire format.

The format carries two separate notions per device, and the difference
matters:

    valid    this device has been seen at some point, and `pose` is a real
             place it once was
    visible  the device is being seen right now

A device that goes out of view keeps being sent with `visible` clear and its
last pose, so it holds still instead of snapping to the origin. This source
keeps the same behaviour: the pose is reported every frame, and only the
visibility changes.
"""

import os
import socket
import struct
import time

from solarxr_feeder.service import TrackerSource

MAGIC = 0x54424631          # "1FBT"
VERSION = 1

# magic, version, flags, reserved (a short, not a byte), time, three poses,
# then the two bitmasks. 104 bytes.
PACKET = struct.Struct("<IBBHd" + "7f" * 3 + "HH")

# The sender always writes the devices in this order.
ROLES = ("left_foot", "right_foot", "waist")

# Role -> bit index in `valid` / `visible`.
BIT = {role: index for index, role in enumerate(ROLES)}

# The tracked object has no status word of its own. The runtime reports its
# own query as LostTracking while the object is tracked perfectly well, so the
# sender cannot set `visible` for it; index 2 is therefore always considered
# visible when it has a pose.
OBJECT_INDEX = BIT["waist"]


class Cv1UdpSource(TrackerSource):
    def __init__(self, port=39570, bind="0.0.0.0", mirror_port=None,
                 heartbeat=None, log=print):
        self.roles = ROLES
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)

        self.mirror = None
        if mirror_port:
            self.mirror = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.mirror_addr = ("127.0.0.1", mirror_port)

        self.heartbeat = heartbeat
        self.packets = 0
        self.last_rx = None
        self.visible = 0
        self._poses = {}
        self._last_status_write = 0.0

    # ------------------------------------------------------------- reading

    def poll(self):
        handled = 0
        while True:
            try:
                data, _addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError:
                break

            handled += 1
            self.packets += 1
            self.last_rx = time.monotonic()

            if self.mirror is not None:
                try:
                    self.mirror.sendto(data, self.mirror_addr)
                except OSError:
                    pass

            if self.heartbeat and time.monotonic() - self._last_status_write > 0.25:
                self._last_status_write = time.monotonic()
                try:
                    with open(self.heartbeat, "w") as handle:
                        handle.write(f"{time.time():.3f}\n")
                except OSError:
                    pass

            self._handle(data)

        return handled

    def _handle(self, data):
        if len(data) != PACKET.size:
            return
        try:
            fields = PACKET.unpack(data)
        except struct.error:
            return

        magic, version, _flags, _reserved, _time = fields[:5]
        if magic != MAGIC or version != VERSION:
            return

        self.visible = fields[-1]
        poses = fields[5:5 + 21]

        for index, role in enumerate(ROLES):
            base = index * 7
            quat = (poses[base + 0], poses[base + 1], poses[base + 2], poses[base + 3])
            position = (poses[base + 4], poses[base + 5], poses[base + 6])

            # (0, 0, 0) is what the runtime reports for a device it cannot
            # place. A worn tracker never sits exactly there, and accepting it
            # would drag the tracker to the origin.
            if position == (0.0, 0.0, 0.0):
                continue

            if index != OBJECT_INDEX and not (self.visible & (1 << index)):
                continue

            self._poses[role] = (position, quat)

    # ------------------------------------------------------------- serving

    def pose(self, role):
        return self._poses.get(role)

    def is_visible(self, role):
        index = BIT.get(role)
        if index is None:
            return False
        if role not in self._poses:
            return False
        if index == OBJECT_INDEX:
            return True
        return bool(self.visible & (1 << index))

    def clear(self):
        """Forget everything, for after the transform changes.

        Held poses were recorded in the old frame; carrying them across would
        put a tracker somewhere that means nothing in the new one.
        """
        self._poses.clear()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        if self.mirror is not None:
            try:
                self.mirror.close()
            except OSError:
                pass

    # ------------------------------------------------------------- tooling

    def write_status(self, path):
        """Publish state for a launcher or viewer to read."""
        import json

        data = {
            "time": time.time(),
            "packets": self.packets,
            "age_ms": ((time.monotonic() - self.last_rx) * 1000.0
                       if self.last_rx else None),
            "trackers": {},
        }
        for role in ROLES:
            entry = {"live": self.is_visible(role), "seen": role in self._poses}
            pose = self._poses.get(role)
            if pose:
                entry["pos"] = [round(value, 3) for value in pose[0]]
            data["trackers"][role] = entry

        temporary = path + ".tmp"
        try:
            with open(temporary, "w") as handle:
                json.dump(data, handle)
            os.replace(temporary, path)
        except OSError:
            pass
