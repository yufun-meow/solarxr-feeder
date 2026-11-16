"""End-to-end check: feeder plus a stand-in runtime.

The protocol tests pin the encoded bytes. That is not the same as the service
working: the sockets have to exist in the right order, the reply has to be
framed correctly, and a pose update has to actually arrive. This exercises
all of that with a fake runtime that behaves like WiVRn's SolarXR feeder.
"""

import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(HERE))

from solarxr_feeder import protocol  # noqa: E402

# The reference packet the CV1 reader sends: magic, version, flags, reserved,
# time, three poses, then the valid and visible masks.
CV1_PACKET = struct.Struct("<IBBHd" + "7f" * 3 + "HH")

ROLES = ("left_foot", "right_foot", "waist")


def fake_sender(port, count=240, stop=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i in range(count):
        if stop and stop.is_set():
            break
        t = i / 120.0
        poses = [
            (0.0, 0.0, 0.0, 1.0, -0.3 + 0.1 * t, -1.0, 0.2),
            (0.0, 0.0, 0.0, 1.0, 0.2, -1.0, 0.2 - 0.1 * t),
            (0.0, 0.0, 0.0, 1.0, 0.0, -0.4, 0.0),
        ]
        values = ([0x54424631, 1, 0, 0, t]
                  + [v for pose in poses for v in pose]
                  + [0b111, 0b111])
        sock.sendto(CV1_PACKET.pack(*values), ("127.0.0.1", port))
        time.sleep(1 / 120.0)
    sock.close()


def read_frame(sock, timeout=3.0):
    sock.settimeout(timeout)
    header = b""
    while len(header) < 4:
        chunk = sock.recv(4 - len(header))
        if not chunk:
            raise AssertionError("connection closed before the length prefix")
        header += chunk
    (length,) = struct.unpack("<I", header)
    body = b""
    wanted = length - 4
    while len(body) < wanted:
        chunk = sock.recv(wanted - len(body))
        if not chunk:
            raise AssertionError("connection closed mid-frame")
        body += chunk
    return body


def main():
    udp_port = 39610
    rpc_path = protocol.socket_path("SlimeVRRpc")
    input_path = protocol.socket_path("SlimeVRInput")

    for path in (rpc_path, input_path):
        try:
            os.unlink(path)
        except OSError:
            pass

    status = os.path.join(tempfile.gettempdir(), "solarxr-feeder-test-status.json")
    log_path = os.path.join(tempfile.gettempdir(), "solarxr-feeder-test.log")
    log_file = open(log_path, "w")
    feeder = subprocess.Popen(
        [sys.executable, "-u", os.path.join(ROOT, "feeder.py"),
         "--port", str(udp_port), "--hz", "60",
         "--mirror-port", "0", "--status", status,
         "--transform", "/nonexistent"],
        cwd=ROOT, stdout=log_file, stderr=subprocess.STDOUT, text=True)

    try:
        # The runtime can only connect once the sockets exist.
        deadline = time.time() + 10
        while time.time() < deadline:
            if os.path.exists(rpc_path) and os.path.exists(input_path):
                break
            if feeder.poll() is not None:
                raise AssertionError("feeder exited early")
            time.sleep(0.1)
        else:
            raise AssertionError("sockets never appeared")

        print("  ok  both sockets created")

        # Data has to be flowing, otherwise the introduction uses the origin.
        threading.Thread(target=fake_sender, args=(udp_port,), daemon=True).start()
        time.sleep(0.6)

        rpc = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        rpc.connect(rpc_path)
        rpc.sendall(b"\x00" * 16)          # the runtime sends a request first
        body = read_frame(rpc)
        assert len(body) > 100, "settings reply looks empty"
        for role in ROLES:
            assert role.encode() in body, f"{role} missing from the settings reply"
        print(f"  ok  settings reply, {len(body)} bytes, introduces all three roles")

        # Only after the settings reply does the runtime attach here.
        feed = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        feed.connect(input_path)
        print("  ok  runtime attached to SlimeVRInput")

        # Poses should arrive at roughly the configured rate. Counting frames
        # rather than just waiting for one distinguishes the 60 Hz update
        # stream from the settings reply, which is repeated once a second and
        # would otherwise look like success.
        frames = 0
        deadline = time.time() + 1.5
        while time.time() < deadline:
            try:
                read_frame(rpc, timeout=1.0)
            except socket.timeout:
                break
            frames += 1
        assert frames >= 20, f"only {frames} frames in 1.5 s - poses are not streaming"
        print(f"  ok  {frames} frames in 1.5 s (~{frames / 1.5:.0f} Hz)")

        rpc.close()
        feed.close()

        assert os.path.exists(status), "status file was not written"
        print("  ok  status file written")
    finally:
        feeder.terminate()
        try:
            feeder.wait(timeout=5)
        except subprocess.TimeoutExpired:
            feeder.kill()
        log_file.close()
        with open(log_path) as handle:
            output = handle.read()
        if "Traceback" in output:
            print(output)
            raise AssertionError("feeder crashed")

    print("\nservice end-to-end checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
