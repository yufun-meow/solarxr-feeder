"""A SolarXR feeder: any pose source in, WiVRn/Monado trackers out.

The runtime expects SolarXR trackers over a pair of Unix sockets. This
program provides them, taking poses from a source - by default the UDP
packets from a CV1 rig, described in docs/PACKET.md.

    ./feeder.py --help

Needs `flatbuffers`; nothing else outside the standard library.
"""

import argparse
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from solarxr_feeder.service import SolarXRService       # noqa: E402
from solarxr_feeder.transform import Transform          # noqa: E402

DEFAULT_TRANSFORM = os.path.join(os.path.expanduser("~"),
                                 ".config", "solarxr-feeder", "transform.json")

# A SolarXR tracker is identified by (device_id, tracker_num). These ids only
# have to be unique and stable within a session.
DEVICE_IDS = {"left_foot": 1, "right_foot": 2, "waist": 3,
              "left_hand": 4, "right_hand": 5, "chest": 6}


def log(message):
    print(f"[feeder {time.strftime('%H:%M:%S')}] {message}", flush=True)


def build_source(args):
    if args.source == "cv1":
        from sources.cv1_udp import Cv1UdpSource
        return Cv1UdpSource(
            port=args.port,
            bind=args.bind,
            mirror_port=(args.mirror_port or None),
            heartbeat=args.heartbeat,
            log=log,
        )
    raise SystemExit(f"unknown source '{args.source}'")


def main():
    parser = argparse.ArgumentParser(
        description="Feed any pose source to WiVRn/Monado as SolarXR trackers.")
    parser.add_argument("--source", default="cv1", choices=("cv1",),
                        help="where the poses come from")
    parser.add_argument("--port", type=int, default=39570,
                        help="UDP port the source sends to")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="address to listen on")
    parser.add_argument("--hz", type=float, default=60.0,
                        help="how often poses are pushed to the runtime")
    parser.add_argument("--transform", default=DEFAULT_TRANSFORM,
                        help="mapping from the source's space into the "
                             "headset's, written by tools/calibrate.py")
    parser.add_argument("--mirror-port", type=int, default=39571,
                        help="forward a copy of each datagram here for "
                             "tools/monitor.py; 0 disables")
    parser.add_argument("--heartbeat", default="/tmp/solarxr-feeder-last-rx",
                        help="file touched while data is arriving")
    parser.add_argument("--status", default="",
                        help="optional JSON state file for a launcher")
    args = parser.parse_args()

    transform = Transform.load(args.transform)
    if transform is None:
        log(f"no calibration at {args.transform}: poses pass through unchanged")
        log("run tools/calibrate.py to line the source's room up with the headset")
    else:
        log(f"transform from {args.transform}: heading {transform.yaw_degrees:+.1f}°")

    source = build_source(args)
    service = SolarXRService(source, DEVICE_IDS, transform=transform,
                             hz=args.hz, log=log)

    stop = threading.Event()

    def on_signal(_signum, _frame):
        stop.set()
        service.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, on_signal)
        except (ValueError, OSError):
            pass

    log(f"listening for {args.source} poses on udp {args.bind}:{args.port}")

    def publish_status():
        if not args.status:
            return
        while not stop.is_set():
            source.write_status(args.status)
            stop.wait(0.3)

    if args.status:
        threading.Thread(target=publish_status, daemon=True).start()

    try:
        service.run()
    finally:
        source.close()
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
