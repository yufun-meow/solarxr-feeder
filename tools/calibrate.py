#!/usr/bin/env python3
"""Measure the mapping from a pose source's space into the headset's.

Hold one device from the source against one headset controller and move the
pair around as if they were glued together. Two systems report the same
physical motion in their own coordinates; the fit between the two point sets
is the transform that makes the trackers land correctly.

The one thing this cannot do for you is turn the pair. Carrying it around
without rotating it leaves the fixed offset between the two devices constant
in the room, where a fit will absorb it into the translation: the result is
internally perfect, physically wrong, and different every run. The tool
measures how far the pair was turned and refuses to save a result that came
from too little of it.

    ./calibrate.py --hand left

Needs an OpenXR runtime with the headset controller tracked, and the same
pose source the feeder reads.
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from solarxr_feeder import openxr as xr            # noqa: E402
from solarxr_feeder import solve                   # noqa: E402
from solarxr_feeder.transform import Transform     # noqa: E402

XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED = 18
XR_TYPE_SESSION_BEGIN_INFO = 10
XR_SESSION_STATE_READY = 2
XR_SESSION_STATE_EXITING = 7
XR_SESSION_STATE_LOSS_PENDING = 8
XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO = 2

DEFAULT_OUT = os.path.join(os.path.expanduser("~"),
                           ".config", "solarxr-feeder", "transform.json")


def begin_session(L, instance, session, timeout=10.0):
    """Drive the session to RUNNING.

    Nothing reports poses before this: the runtime keeps the session IDLE,
    actions never become active and device enumeration comes back empty.
    """
    import ctypes

    class Body(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                    ("session", ctypes.c_uint64), ("state", ctypes.c_int),
                    ("varying", ctypes.c_uint64)]

    class BeginInfo(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                    ("viewConfigurationType", ctypes.c_int)]

    L.xrBeginSession.restype = ctypes.c_int
    L.xrBeginSession.argtypes = [ctypes.c_uint64, ctypes.POINTER(BeginInfo)]
    L.xrPollEvent.restype = ctypes.c_int
    L.xrPollEvent.argtypes = [ctypes.c_uint64,
                              ctypes.POINTER(xr.XrEventDataBuffer)]

    def try_begin():
        info = BeginInfo()
        info.type = XR_TYPE_SESSION_BEGIN_INFO
        info.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO
        return L.xrBeginSession(session, ctypes.byref(info)) == 0

    def drain():
        state = None
        for _ in range(64):
            event = xr.XrEventDataBuffer()
            event.type = xr.XR_TYPE_EVENT_DATA_BUFFER
            if L.xrPollEvent(instance, ctypes.byref(event)) != 0:
                break
            if event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
                body = ctypes.cast(ctypes.byref(event),
                                   ctypes.POINTER(Body)).contents
                state = body.state
        return state

    drain()
    # Most runtimes accept this straight away; waiting for a READY event that
    # may never be sent just makes the tool look hung.
    if try_begin():
        return True

    print("等待头显就绪 ...", end="", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = drain()
        if state == XR_SESSION_STATE_READY:
            print(" 就绪", flush=True)
            return try_begin()
        if state in (XR_SESSION_STATE_EXITING, XR_SESSION_STATE_LOSS_PENDING):
            print(" 头显断开了", flush=True)
            return False
        time.sleep(0.05)
    print(" 超时", flush=True)
    return try_begin()


def build_source(args):
    if args.source == "cv1":
        from sources.cv1_udp import Cv1UdpSource
        return Cv1UdpSource(port=args.port, bind=args.bind, mirror_port=None)
    raise SystemExit(f"unknown source '{args.source}'")


def main():
    parser = argparse.ArgumentParser(
        description="Measure the transform from a pose source into the "
                    "headset's tracking space.")
    parser.add_argument("--source", default="cv1", choices=("cv1",))
    parser.add_argument("--port", type=int, default=39571,
                        help="read the source here. The default is the port a "
                             "running feeder mirrors to, so calibration does "
                             "not need the feeder stopped - stopping it would "
                             "force a runtime restart, and the runtime anchors "
                             "the headset's origin when its session opens")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--cv1-role", default="left_foot",
                        help="which source device to hold against the "
                             "controller")
    parser.add_argument("--hand", default="left", choices=("left", "right"),
                        help="which headset controller to hold it against")
    parser.add_argument("--seconds", type=float, default=25.0)
    parser.add_argument("--min-samples", type=int, default=60)
    parser.add_argument("--min-turn", type=float, default=90.0,
                        help="degrees the pair must be turned about the "
                             "vertical for the result to be saved")
    parser.add_argument("--force", action="store_true",
                        help="save even if the pair was barely turned")
    parser.add_argument("--no-prompt", action="store_true")
    parser.add_argument("--out", default=DEFAULT_OUT)
    args = parser.parse_args()

    print()
    print("=" * 66)
    print("  空间对齐标定")
    print("=" * 66)
    print()
    print(f"  要配对的设备:")
    print(f"    A: {args.cv1_role}   (数据源)")
    print(f"    B: 头显 {args.hand} 手柄   (头显追踪)")
    print()
    print("  A 和 B 握紧、当成一个整体移动。")
    print("  关键是转动: 像拧门把手让朝向水平转过 180° 以上,")
    print("  同时走遍房间。只走不转,结果是错的。")
    print()

    source = build_source(args)
    transform = Transform.identity()

    L = xr.load_lib()
    xr.MonadoXR._bind(L)
    instance, system_id, session, stage_space = xr.MonadoXR._ensure_started(L)
    if not begin_session(L, instance, session):
        print("无法开始 OpenXR 会话。VRChat 之类占了会话的程序要关掉。")
        return 1

    reader = object.__new__(xr.MonadoXR)
    reader.L = L
    reader.instance = instance
    reader.session = session
    reader._check = xr.MonadoXR._check
    action = reader._create_pose_action(f"/user/hand/{args.hand}/input/grip/pose")
    reader._attach_once()
    for _ in range(60):
        reader.poll()
        reader._sync_now()
        time.sleep(0.02)

    import ctypes
    stage = stage_space.value if hasattr(stage_space, "value") else stage_space
    L.xrCreateActionSpace.restype = ctypes.c_int
    L.xrCreateActionSpace.argtypes = [
        ctypes.c_uint64, ctypes.POINTER(xr.XrActionSpaceCreateInfo),
        ctypes.POINTER(ctypes.c_uint64)]
    L.xrLocateSpace.restype = ctypes.c_int
    L.xrLocateSpace.argtypes = [
        ctypes.c_uint64, ctypes.c_uint64, ctypes.c_int64,
        ctypes.POINTER(xr.XrSpaceLocation)]

    space_info = xr.XrActionSpaceCreateInfo()
    space_info.type = xr.XR_TYPE_ACTION_SPACE_CREATE_INFO
    space_info.action = action
    space_info.poseInActionSpace.orientation.w = 1.0
    action_space = ctypes.c_uint64(0)
    L.xrCreateActionSpace(session, ctypes.byref(space_info),
                          ctypes.byref(action_space))

    def read_controller():
        # A zero time is rejected by WiVRn, so the current time is used.
        now_ns = int(time.monotonic() * 1e9)
        location = xr.XrSpaceLocation()
        location.type = xr.XR_TYPE_SPACE_LOCATION
        if L.xrLocateSpace(action_space.value, stage, now_ns,
                           ctypes.byref(location)) != 0:
            return None
        flags = location.locationFlags
        if not (flags & xr.XR_SPACE_LOCATION_POSITION_VALID_BIT and
                flags & xr.XR_SPACE_LOCATION_POSITION_TRACKED_BIT):
            return None
        p = location.pose.position
        o = location.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    if not args.no_prompt:
        try:
            input("  准备好后,按回车开始 ... ")
        except EOFError:
            pass
        print()
        for n in (3, 2, 1):
            print(f"  {n} ...", end="", flush=True)
            time.sleep(1.0)
        print(" 开始!\n")

    pairs = []
    orientations = []
    deadline = time.monotonic() + args.seconds
    last_report = 0.0
    tick = 0
    while time.monotonic() < deadline:
        reader.poll()
        reader._sync_now()
        source.poll()

        pose = source.pose(args.cv1_role)
        position_c = pose[0] if pose else None
        got = read_controller()
        position_d = got[0] if got else None
        orientation_c = pose[1] if pose else None

        if orientation_c:
            orientations.append(orientation_c)

        tick += 1
        if position_c is not None and position_d is not None and tick % 2 == 0:
            pairs.append((position_c, position_d))
            if len(pairs) > 40000:
                del pairs[0]

        now = time.monotonic()
        if now - last_report > 0.25:
            last_report = now
            left = deadline - now
            done = int(24 * (1.0 - left / max(args.seconds, 1e-6)))
            bar = "#" * done + "." * (24 - done)
            a_ok = "OK  " if position_c is not None else "LOST"
            b_ok = "OK  " if position_d is not None else "LOST"
            print(f"\r  [{bar}] {left:4.1f}s   {args.cv1_role} {a_ok}   "
                  f"{args.hand} 手柄 {b_ok}   {len(pairs)} 采样",
                  end="", flush=True)

    print("\n")
    source.close()

    if len(pairs) < args.min_samples:
        print(f"只有 {len(pairs)} 对可用样本,需要 {args.min_samples}。")
        print("检查两边是否都在被追踪。")
        return 1

    turned = solve.yaw_spread(orientations) if orientations else 0.0
    print(f"  水平转动 {turned:.0f}°")
    if turned < args.min_turn and not args.force:
        print()
        print("!" * 66)
        print(f"  只水平转了 {turned:.0f}°,这次标定不可信。"
              f"至少 {args.min_turn:.0f}°,最好 180° 以上。")
        print()
        print("  两个设备之间有固定偏移,它会跟着一起转。只有真的转动")
        print("  才能把偏移和位移分开;上下翻动没有用。")
        print("!" * 66)
        return 1

    src = [p[0] for p in pairs]
    dst = [p[1] for p in pairs]
    quat, translation, rms = solve.robust_solve_yaw(src, dst)

    # What the unconstrained fit would have wanted. It should be near zero: a
    # real tilt would mean the two systems disagree about which way is down,
    # which cannot happen when both use gravity.
    free_quat, _ft, free_rms = solve.robust_solve(src, dst)
    free_angle = math.degrees(2.0 * math.acos(min(1.0, abs(free_quat[3]))))
    if free_angle > 1e-6:
        axis_y = abs(free_quat[1]) / math.sqrt(max(1e-12, 1.0 - free_quat[3] ** 2))
        tilt = math.degrees(math.acos(min(1.0, axis_y)))
    else:
        tilt = 0.0

    yaw = math.degrees(2.0 * math.atan2(quat[1], quat[3]))
    print(f"Samples: {len(pairs)}   residual: {rms * 1000:.1f} mm")
    print(f"Heading    {yaw:+.1f}°   (tilt fixed at 0 by gravity)")
    print(f"Translation {translation[0]:+.4f} {translation[1]:+.4f} "
          f"{translation[2]:+.4f} m")
    print(f"  unconstrained fit would have tilted {tilt:.1f}° "
          f"(residual {free_rms * 1000:.1f} mm); that tilt is sensor noise")
    if rms > 0.15:
        print("  That residual is large. The pair may have slipped, or one of")
        print("  them lost tracking during the sweep. Worth running again.")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as handle:
        json.dump({
            "comment": "Maps the source's tracking space onto the headset's. "
                       "Written by tools/calibrate.py; apply with feeder.py.",
            "rotation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]},
            "translation": {"x": translation[0], "y": translation[1],
                            "z": translation[2]},
            "residual_m": rms,
            "heading_only": True,
            "yaw_coverage_deg": turned,
            "unconstrained_tilt_deg": tilt,
            "samples": len(pairs),
            "source": args.source,
            "source_device": args.cv1_role,
            "against_controller": args.hand,
        }, handle, indent=2)

    print(f"\nWritten to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
