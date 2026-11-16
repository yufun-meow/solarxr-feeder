#!/usr/bin/env python3
"""Feed WiVRn three trackers from the CV1 that runs on a Windows box.

The Windows side (cv1-fbt-source) reads the poses from the Oculus runtime and
broadcasts them; this side receives them and hands them to WiVRn over the same
SolarXR socket that solarxr-feed.py uses, so the runtime and the rest of the
setup are unchanged.

    ./cv1-fbt-feeder.py                     # WiVRn must be running

The body parts are the ones this rig was built for: the two Touch controllers
are worn on the feet and the tracked object is worn on the waist.

If a tracker loses sight of the cameras the Windows side keeps sending its last
known pose with `visible` clear, and that pose is published unchanged - a foot
that stops being seen should stay where it was, not jump to the origin.
"""

import argparse
import importlib.util
import json
import os
import signal
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)

MAGIC = 0x54424631
VERSION = 1
PACKET = struct.Struct("<IBBHd" + "7f" * 3 + "HH")

# The Windows side always sends left foot, right foot, waist, in that order.
ROLE_FOR_INDEX = ("left_foot", "right_foot", "waist")

# A SolarXR tracker is identified by (device_id, tracker_num). solarxr-feed.py
# derives these from OpenHMD device indexes; here they are just a fixed, unique
# number per body part, so the three never collide on the runtime's side.
DEVICE_ID_FOR_ROLE = {
    "left_foot": 1,
    "right_foot": 2,
    "waist": 3,
}


def load_feed_module():
    """Import solarxr-feed.py by path: the name has a hyphen in it."""
    path = os.path.join(PARENT, "solarxr-feed.py")
    if not os.path.exists(path):
        raise SystemExit(f"solarxr-feed.py not found at {path}")
    spec = importlib.util.spec_from_file_location("solarxr_feed", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def unpack(data):
    if len(data) < PACKET.size:
        return None
    magic, version, _flags, _reserved, t, *rest = PACKET.unpack_from(data)
    if magic != MAGIC or version != VERSION:
        return None
    poses = [tuple(rest[i * 7:(i + 1) * 7]) for i in range(3)]
    valid, visible = rest[21], rest[22]
    return t, poses, valid, visible


DEFAULT_TRANSFORM = os.path.join(os.path.expanduser("~"), ".config", "cv1-fbt", "transform.json")


def quat_to_matrix(q):
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


def reset_last_known(src):
    """Drop held poses after a calibration change.

    They were recorded in the old frame; carrying them over would put a
    tracker at a place that means nothing in the new one.
    """
    for i in range(3):
        src.last_known[i] = None
    src.poses = [None, None, None]


class Transform:
    """Maps CV1 tracking space onto the headset's.

    The CV1 poses arrive in the Windows room's frame. WiVRn puts SolarXR
    trackers in the headset's own tracking origin, so without this the
    trackers land at unrelated places - correct relative to each other,
    wrong relative to the headset. cv1-fbt-calibrate.py measures the mapping.
    """

    def __init__(self, quat, pos):
        self.quat = list(quat)
        self.pos = list(pos)
        self.matrix = quat_to_matrix(self.quat)

    @classmethod
    def identity(cls):
        return cls([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0])

    @classmethod
    def load(cls, path):
        if not os.path.exists(path):
            return None
        with open(path) as f:
            data = json.load(f)
        r = data["rotation"]
        t = data["translation"]
        return cls([r["x"], r["y"], r["z"], r["w"]], [t["x"], t["y"], t["z"]])

    def apply_pos(self, p):
        r = self.matrix
        return (
            r[0][0] * p[0] + r[0][1] * p[1] + r[0][2] * p[2] + self.pos[0],
            r[1][0] * p[0] + r[1][1] * p[1] + r[1][2] * p[2] + self.pos[1],
            r[2][0] * p[0] + r[2][1] * p[1] + r[2][2] * p[2] + self.pos[2],
        )

    def apply_quat(self, q):
        # An orientation maps the device's own axes into the room's. Moving to
        # the new room means composing the frame rotation in front of it, not
        # conjugating: R left-multiplied gives local -> new room.
        return tuple(quat_mul(self.quat, list(q)))


class UdpSource:
    """The poses as they arrive, in the shape the feeder wants."""

    def __init__(self, port, bind="0.0.0.0", stale_after=1.0, mirror_port=None,
                 transform=None, heartbeat=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Lets the viewer and this feeder listen at the same time.
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        self.sock.bind((bind, port))
        self.sock.setblocking(False)

        self.port = port
        self.stale_after = stale_after
        self.poses = [None, None, None]  # ((px,py,pz), (qx,qy,qz,qw)) or None
        # The last pose each part was ever seen at. A part that goes out of
        # view keeps producing this, because a tracker that stops appearing in
        # the update at all is reset to the origin by the runtime.
        self.last_known = [None, None, None]
        self.visible = 0
        self.packets = 0
        self.last_rx = 0.0
        self.sender = None

        # A copy of every datagram forwarded here, so a viewer can watch the
        # same stream. Two processes cannot share one UDP port: SO_REUSEPORT
        # load-balances between them, it does not duplicate.
        self.transform = transform or Transform.identity()
        # Touched on every received datagram so a launcher can tell whether
        # the Windows side is actually sending.
        self.heartbeat = heartbeat
        self.transform_path = None
        self.transform_mtime = None

        self.mirror = None
        if mirror_port:
            self.mirror = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.mirror_addr = ("127.0.0.1", mirror_port)

    def poll(self):
        if self._maybe_reload():
            self.reload_note = True
        got = 0
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, socket.timeout):
                break
            except OSError:
                break
            parsed = unpack(data)
            if parsed is None:
                continue
            _t, poses, valid, visible = parsed
            got += 1
            self.packets += 1
            self.last_rx = time.monotonic()
            if self.heartbeat:
                try:
                    with open(self.heartbeat, "w") as hb:
                        hb.write(f"{time.time():.3f}\n")
                except OSError:
                    pass
            if self.mirror is not None:
                try:
                    self.mirror.sendto(data, self.mirror_addr)
                except OSError:
                    pass
            self.sender = addr[0]
            self.visible = visible
            for i in range(3):
                p = poses[i]
                # A position of exactly zero is what the sender reports when
                # the cameras lose a device. Holding the previous one is the
                # only way a tracker stays where it was instead of snapping to
                # the origin, and doing it here as well means a sender that
                # lets a zero through cannot drag anything home.
                is_zero = (p[4] == 0.0 and p[5] == 0.0 and p[6] == 0.0)

                if (valid & (1 << i)) and not is_zero:
                    pos = self.transform.apply_pos((p[4], p[5], p[6]))
                    quat = self.transform.apply_quat((p[0], p[1], p[2], p[3]))
                    self.poses[i] = (pos, quat)
                    self.last_known[i] = self.poses[i]
                else:
                    self.poses[i] = None
        return got

    def write_status(self, path):
        """Publish the current state for the launcher window to display.

        A file rather than another socket: the viewer already listens on the
        mirror port, and two readers there would fight over the datagrams.
        """
        data = {
            "time": time.time(),
            "packets": self.packets,
            "age_ms": (time.monotonic() - self.last_rx) * 1000.0 if self.last_rx else None,
            "trackers": {},
        }
        for i, role in enumerate(ROLE_FOR_INDEX):
            p = self.poses[i]
            live = p is not None and (i == 2 or bool(self.visible & (1 << i)))
            entry = {"live": live, "seen": p is not None}
            if p is not None:
                entry["pos"] = [round(v, 3) for v in p[0]]
            else:
                entry["held"] = self.last_known[i] is not None
                if self.last_known[i] is not None:
                    entry["pos"] = [round(v, 3) for v in self.last_known[i][0]]
            data["trackers"][role] = entry
        tmp = path + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(data, f)
            os.replace(tmp, path)
        except OSError:
            pass

    def is_live(self, role):
        """Whether this part is being seen right now, not just held.

        The sender flags visibility for the hands, but has nothing to say about
        the tracked object: the runtime reports that device's own query as
        LostTracking even while its pose is fresh, so the object's visibility
        is taken from the pose being there at all.
        """
        try:
            i = ROLE_FOR_INDEX.index(role)
        except ValueError:
            return False
        if self.poses[i] is None:
            return False
        if i == 2:
            return True
        return bool(self.visible & (1 << i))

    def fresh(self):
        return self.last_rx > 0 and (time.monotonic() - self.last_rx) < self.stale_after

    def trackers(self):
        """[(role, pos, quat)] for all three parts.

        A part with no current pose still reports the last place it was seen,
        so it is always present in the update. Omitting a tracker entirely
        makes the runtime fall back to the origin, which is what made a foot
        jump home whenever its controller lost tracking.
        """
        out = []
        for i, role in enumerate(ROLE_FOR_INDEX):
            p = self.poses[i] or self.last_known[i]
            if p is not None:
                out.append((role, p[0], p[1]))
        return out

    def status(self):
        bits = []
        for i, role in enumerate(ROLE_FOR_INDEX):
            if self.poses[i] is None:
                bits.append(f"{role}=-")
            elif self.visible & (1 << i):
                bits.append(f"{role}=live")
            else:
                bits.append(f"{role}=held")
        age = time.monotonic() - self.last_rx if self.last_rx else None
        link = "never" if age is None else f"{age * 1000:.0f}ms"
        return f"{self.packets} pkts, last {link}  " + " ".join(bits)

    def watch_transform(self, path):
        """Reload the calibration when the file changes.

        Calibration has to run while this feeder is up: WiVRn anchors the
        headset's tracking origin when its session opens, so stopping the
        feeder to free the port would mean restarting WiVRn and throwing the
        calibration away. Reloading in place avoids the whole dance.
        """
        self.transform_path = path
        try:
            self.transform_mtime = os.stat(path).st_mtime
        except OSError:
            self.transform_mtime = None

    def _maybe_reload(self):
        if not self.transform_path:
            return
        try:
            mtime = os.stat(self.transform_path).st_mtime
        except OSError:
            return
        if self.transform_mtime is None or mtime > self.transform_mtime:
            self.transform_mtime = mtime
            fresh = Transform.load(self.transform_path)
            if fresh is not None:
                self.transform = fresh
                reset_last_known(self)
                return True
        return False

    def close(self):
        self.sock.close()
        if self.mirror is not None:
            self.mirror.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen-port", type=int, default=39570)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--hz", type=float, default=60.0,
                    help="how often poses are pushed to WiVRn")
    ap.add_argument("--transform", default=DEFAULT_TRANSFORM,
                    help="calibration written by cv1-fbt-calibrate.py")
    ap.add_argument("--status", default="/tmp/cv1-fbt-status.json",
                    help="state file the launcher reads")
    ap.add_argument("--heartbeat", default="/tmp/cv1-fbt-last-rx",
                    help="file touched on every datagram, for status display")
    ap.add_argument("--mirror-port", type=int, default=39571,
                    help="forward a copy of each datagram to this local port, "
                         "for cv1-fbt-view.py; 0 disables")
    args = ap.parse_args()

    feed = load_feed_module()
    log = feed.log

    xform = Transform.load(args.transform)
    if xform is None:
        log(f"no calibration at {args.transform}: poses go through unchanged.")
        log("Run cv1-fbt-calibrate.py to line the CV1 room up with the headset.")
    else:
        log(f"using calibration from {args.transform}")

    src = UdpSource(args.listen_port, args.bind,
                    mirror_port=(args.mirror_port or None),
                    transform=xform, heartbeat=args.heartbeat)
    src.reload_note = False
    src.watch_transform(args.transform)
    log(f"listening for CV1 poses on udp {args.bind}:{args.listen_port}")

    stop_evt = threading.Event()

    def on_signal(signum, frame):
        stop_evt.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, on_signal)
        except (ValueError, OSError):
            pass

    def serve_input():
        """Accept WiVRn's own device reports on SlimeVRInput.

        WiVRn runs the Monado solarxr feeder, which connects out to this socket
        to tell SlimeVR about the headset and controllers it has. Nothing here
        needs those messages, but the socket has to exist and accept the
        connection or the feeder side errors out.
        """
        path = feed.socket_path("SlimeVRInput")
        if os.path.exists(path):
            try:
                os.unlink(path)
            except OSError:
                pass
        try:
            srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            srv.bind(path)
            os.chmod(path, 0o600)
            srv.listen(4)
        except OSError as e:
            log(f"SlimeVRInput listen failed: {e}")
            return
        log(f"SlimeVRInput up on {path}")

        conns = []
        while not stop_evt.is_set():
            srv.settimeout(0.5)
            try:
                c, _ = srv.accept()
                conns.append(c)
                log("WiVRn attached to SlimeVRInput")
            except socket.timeout:
                pass
            except OSError:
                break
            for c in list(conns):
                try:
                    c.setblocking(False)
                    d = c.recv(65536)
                    if not d and d is not None:
                        c.close()
                        conns.remove(c)
                except BlockingIOError:
                    pass
                except OSError:
                    try:
                        c.close()
                    except OSError:
                        pass
                    conns.remove(c)
        for c in conns:
            try:
                c.close()
            except OSError:
                pass
        srv.close()
        try:
            os.unlink(path)
        except OSError:
            pass

    def serve():
        """Exactly solarxr-feed.py's rpc_server loop, with a UDP source."""
        path = feed.socket_path("SlimeVRRpc")
        if os.path.exists(path):
            try:
                os.unlink(path)
            except OSError:
                pass
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(path)
        os.chmod(path, 0o600)
        srv.listen(1)

        roles = sorted(role for role, _p, _q in [])  # filled per connection
        while not stop_evt.is_set():
            srv.settimeout(0.5)
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            conn_roles = None
            try:
                conn.settimeout(0.5)
                try:
                    conn.recv(65536)
                except socket.timeout:
                    pass

                # The roles, and the trackers themselves, are announced in the
                # settings response: WiVRn enumerates its devices from the
                # DataFeedUpdate carried in that very bundle, so an empty one
                # means "Enumerated 0 SolarXR trackers" and nothing appears.
                src.poll()

                # Always introduce all three. The runtime creates its devices
                # once, from this bundle, so a part that happens to have no
                # pose at the moment of the handshake would otherwise never
                # exist at all. A part without a pose yet is introduced at the
                # last place it was seen, or at the origin if it never was.
                known = {role: (pos, quat) for role, pos, quat in src.trackers()}
                intro = []
                for role in ROLE_FOR_INDEX:
                    pos, quat = known.get(role, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
                    intro.append((role, DEVICE_ID_FOR_ROLE[role], pos, quat))

                conn_roles = sorted({role for role, _d, _p, _q in intro})
                reply = feed.encode_settings_response(conn_roles, intro)
                msg = struct.pack("<I", len(reply) + 4) + reply
                conn.sendall(msg)
                log(f"runtime connected: sent SettingsResponse for {', '.join(conn_roles)}")

                period = 1.0 / args.hz
                stats = feed.FrameStats(status_path=None)
                last_settings = time.monotonic()
                conn.settimeout(0.0)
                last_state = None
                last_beat = time.monotonic()
                last_status_write = 0.0

                while not stop_evt.is_set():
                    try:
                        data = conn.recv(65536)
                        if not data and data is not None:
                            break
                    except (BlockingIOError, socket.timeout):
                        pass
                    except OSError:
                        break

                    if time.monotonic() - last_settings >= 1.0:
                        try:
                            conn.sendall(msg)
                            last_settings = time.monotonic()
                        except OSError:
                            break

                    got = src.poll()
                    if time.monotonic() - last_status_write > 0.3:
                        last_status_write = time.monotonic()
                        src.write_status(args.status)
                    if getattr(src, "reload_note", False):
                        src.reload_note = False
                        log("calibration reloaded")

                    batch = []
                    poses = []
                    for role, pos, quat in src.trackers():
                        batch.append((role, DEVICE_ID_FOR_ROLE[role], pos, quat))
                        poses.append((role, pos))
                    if not batch:
                        time.sleep(period)
                        continue

                    payload = feed.encode_bundle(batch)
                    frame = struct.pack("<I", len(payload) + 4) + payload
                    sent_ok = True
                    try:
                        conn.sendall(frame)
                    except (BlockingIOError, socket.timeout):
                        sent_ok = False
                    except OSError as e:
                        log(f"pose send failed: {e}")
                        break
                    stats.frame(poses, sent_ok, conn_roles or [])

                    # Only the changes are worth printing: a line per second
                    # saying nothing changed buries the lines that matter.
                    now_state = tuple(sorted(role for role, _p in poses))
                    if now_state != last_state or not sent_ok:
                        if now_state:
                            log("tracking: " + ", ".join(
                                f"{r}={('held' if not src.is_live(r) else 'live')}"
                                for r in now_state))
                        else:
                            log("tracking: nothing")
                        last_state = now_state
                    elif time.monotonic() - last_beat > 60.0:
                        last_beat = time.monotonic()
                        log(f"still running, {src.packets} packets received")
                    time.sleep(period)
            except OSError as e:
                log(f"RPC error: {e}")
            finally:
                try:
                    conn.close()
                except OSError:
                    pass

        srv.close()
        try:
            os.unlink(path)
        except OSError:
            pass

    ti = threading.Thread(target=serve_input, daemon=True)
    ti.start()
    t = threading.Thread(target=serve, daemon=True)
    t.start()

    try:
        while not stop_evt.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop_evt.set()
    finally:
        src.close()
        t.join(timeout=2.0)
        ti.join(timeout=1.0)

    log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
