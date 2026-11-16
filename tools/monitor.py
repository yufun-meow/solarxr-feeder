#!/usr/bin/env python3
"""Receive CV1 pose datagrams and show what arrives.

This is a viewer, not the real receiver: it prints the trackers as they come
in so the Windows side can be checked against the Linux side before anything
depends on the format.

Usage:
    ./monitor.py                  # listen on 0.0.0.0:39570
    ./monitor.py --port 39570 --hz 4
    ./monitor.py --raw            # also dump the first packet byte by byte
"""

import argparse
import math
import socket
import struct
import sys
import time

MAGIC = 0x54424631  # "1FBT"
VERSION = 1

# The datagram, little endian, as cv1-fbt-source.c lays it out:
#   uint32 magic, uint8 version, uint8 flags, uint16 reserved,
#   double time_seconds,
#   pose left_foot, pose right_foot, pose waist,
#   uint16 valid, uint16 reserved2
# and each pose is six floats: qx qy qz qw px py pz.
HEADER = struct.Struct("<IBBHd")
POSE = struct.Struct("<7f")
TAIL = struct.Struct("<HH")

PACKET = struct.Struct("<IBBHd" + "7f" * 3 + "HH")

TRACKER_NAMES = ("left_foot", "right_foot", "waist")

# Which foot/waist a packet claims to carry, for the summary line.
VALID_BITS = (0x01, 0x02, 0x04)


def unpack(data):
    """Return (time_seconds, poses, valid) or None if this is not one of ours."""
    if len(data) < PACKET.size:
        return None
    magic, version, flags, reserved, t, *rest = PACKET.unpack_from(data)
    if magic != MAGIC or version != VERSION:
        return None
    poses = []
    for i in range(3):
        poses.append(tuple(rest[i * 7:(i + 1) * 7]))
    valid, visible = rest[21], rest[22]
    return t, poses, valid, visible


def quat_to_euler(qx, qy, qz, qw):
    """Roll, pitch, yaw in degrees. Oculus uses Y-up, -Z forward."""
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=39570)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--hz", type=float, default=2.0, help="how often to print")
    ap.add_argument("--raw", action="store_true", help="dump the first packet")
    ap.add_argument("--quiet", action="store_true", help="only print changes")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Lets the viewer and the feeder listen at the same time.
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)

    print(f"Listening on {args.bind}:{args.port} for cv1-fbt packets.")
    print("Waiting for the first one; press Ctrl-C to stop.\n")

    count = 0
    bad = 0
    last_print = 0.0
    last_summary = None
    first_raw = args.raw
    started = time.monotonic()
    last_seq_t = None
    last_rx = None

    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            if count == 0 and time.monotonic() - started > 3.0:
                print("nothing yet. Is cv1-fbt-source running, and is it on this subnet?")
                started = time.monotonic()
            continue
        except KeyboardInterrupt:
            break

        if first_raw:
            first_raw = False
            print(f"--- first packet from {addr[0]}, {len(data)} bytes ---")
            for off in range(0, min(len(data), 128), 16):
                row = data[off:off + 16]
                hexes = " ".join(f"{b:02x}" for b in row)
                text = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
                print(f"  {off:04x}  {hexes:<48}  {text}")
            print()

        parsed = unpack(data)
        if parsed is None:
            bad += 1
            if bad <= 3:
                print(f"unexpected datagram from {addr[0]}: {len(data)} bytes, "
                      f"first words {data[:8].hex()}")
            continue

        count += 1
        t_sender, poses, valid, visible = parsed
        now = time.monotonic()

        if args.quiet:
            summary = (visible, tuple(tuple(round(v, 2) for v in p[4:7]) for p in poses))
            if summary != last_summary:
                print(f"[{count}] " + ", ".join(
                    f"{TRACKER_NAMES[i]}={'live' if visible & VALID_BITS[i] else 'held'}"
                    for i in range(3)))
                last_summary = summary
            continue

        if now - last_print < 1.0 / args.hz:
            continue
        last_print = now

        rate = 0.0
        if last_rx is not None:
            dt = now - last_rx
            if dt > 0:
                rate = 1.0 / dt
        last_rx = now

        print(f"\n[{count}] from {addr[0]}  sender_t={t_sender:.3f}  ~{rate:.0f} pkt/s")
        for i, name in enumerate(TRACKER_NAMES):
            qx, qy, qz, qw = poses[i][0:4]
            px, py, pz = poses[i][4:7]
            if not (valid & VALID_BITS[i]):
                print(f"   {name:<11} NO    ---")
                continue
            marked = "live" if (visible & VALID_BITS[i]) else "held"
            roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)
            print(f"   {name:<11} {marked}  "
                  f"pos {px:+7.3f} {py:+7.3f} {pz:+7.3f}   "
                  f"rpy {roll:+7.1f} {pitch:+7.1f} {yaw:+7.1f}")

    sock.close()
    print(f"\nReceived {count} packets ({bad} unexpected).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
