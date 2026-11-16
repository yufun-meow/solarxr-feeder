"""The part the runtime talks to: two sockets, and the pose stream.

A source provides poses; this class puts them on the wire in the shape and
cadence the runtime expects, and keeps the connection alive.
"""

import socket
import struct
import threading
import time

from . import protocol
from .transform import Transform


class TrackerSource:
    """What a pose source has to provide.

    Subclasses read from wherever their poses come from and expose the three
    calls below. `poll()` is allowed to be blocking-free; the service calls it
    once per stream frame.
    """

    #: Body parts this source can drive, as SolarXR role names.
    roles = ()

    def poll(self):
        """Take in whatever data has arrived since the last call."""
        raise NotImplementedError

    def pose(self, role):
        """(position, orientation) for a role, or None if it is not tracked.

        Returning None means "no fresh pose this frame". The service holds the
        previous pose rather than dropping the tracker back to the origin,
        because a tracker that teleports to the floor whenever it is briefly
        occluded is worse than one that stands still.
        """
        raise NotImplementedError

    def close(self):
        pass


class SolarXRService:
    def __init__(self, source, device_ids, transform=None, hz=60.0, log=print):
        self.source = source
        self.device_ids = dict(device_ids)
        self.transform = transform or Transform.identity()
        self.hz = hz
        self.log = log
        self._last_pose = {}
        self._stop = threading.Event()

    # ------------------------------------------------------------ sockets

    def _listen(self, name):
        path = protocol.socket_path(name)
        try:
            import os
            if os.path.exists(path):
                os.unlink(path)
        except OSError:
            pass

        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(path)
        import os as _os
        _os.chmod(path, 0o600)
        server.listen(4)
        self.log(f"{name} listening on {path}")
        return server, path

    def _announce(self):
        """Build the settings reply, introducing every role.

        Every role has to appear here even if it has no pose yet: the runtime
        creates its tracker objects from the DataFeedUpdate in this bundle and
        never revisits that decision. A role missing here simply does not
        exist for the rest of the session.
        """
        trackers = []
        for role in self.source.roles:
            device_id = self.device_ids[role]
            entry = self._last_pose.get(role)
            position, orientation = entry if entry else ((0.0, 0.0, 0.0),
                                                         (0.0, 0.0, 0.0, 1.0))
            trackers.append((role, device_id, position, orientation))

        return self._frame(
            protocol.encode_settings_response(self.source.roles, trackers))

    def _update(self):
        trackers = []
        for role in self.source.roles:
            entry = self._last_pose.get(role)
            if entry is None:
                continue
            trackers.append((role, self.device_ids[role], entry[0], entry[1]))
        return self._frame(protocol.encode_update(trackers))

    @staticmethod
    def _frame(payload):
        """Length-prefix a message.

        The length includes the four bytes of the prefix itself. Both the
        settings reply and every pose update go out this way; sending a bare
        flatbuffer makes the reader take its first four bytes as a length and
        wait for a message that never finishes.
        """
        return struct.pack("<I", len(payload) + 4) + payload

    # --------------------------------------------------------------- loop

    def stop(self):
        self._stop.set()

    def _ingest(self):
        """Pull fresh poses and map them into the headset's space."""
        self.source.poll()
        for role in self.source.roles:
            pose = self.source.pose(role)
            if pose is None:
                continue
            position, orientation = pose
            self._last_pose[role] = (
                self.transform.apply_position(position),
                self.transform.apply_orientation(orientation),
            )

    def _serve_input(self):
        """Accept the runtime's own device reports.

        Monado's feeder connects out to this socket to report the headset and
        controllers it knows about. Nothing here consumes those messages, but
        the socket has to exist and accept the connection, or the runtime logs
        "failed to resolve socket path" and refuses to attach.
        """
        server, path = self._listen("SlimeVRInput")
        connections = []
        try:
            while not self._stop.is_set():
                server.settimeout(0.5)
                try:
                    conn, _ = server.accept()
                    connections.append(conn)
                    self.log("runtime attached to SlimeVRInput")
                except socket.timeout:
                    pass
                except OSError:
                    break

                for conn in list(connections):
                    try:
                        conn.setblocking(False)
                        if not conn.recv(65536):
                            conn.close()
                            connections.remove(conn)
                    except BlockingIOError:
                        pass
                    except OSError:
                        try:
                            conn.close()
                        except OSError:
                            pass
                        connections.remove(conn)
        finally:
            for conn in connections:
                try:
                    conn.close()
                except OSError:
                    pass
            server.close()

    def run(self):
        threading.Thread(target=self._serve_input, daemon=True).start()

        server, _path = self._listen("SlimeVRRpc")
        period = 1.0 / self.hz if self.hz > 0 else 0.0

        try:
            while not self._stop.is_set():
                server.settimeout(0.5)
                try:
                    conn, _ = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                self._session(conn, period)
        finally:
            server.close()

    def _session(self, conn, period):
        self.log("runtime connected")
        try:
            conn.settimeout(0.5)
            try:
                conn.recv(65536)
            except socket.timeout:
                pass

            # Poses seen before the handshake make the introduction honest;
            # roles that have never been seen are introduced at the origin.
            self._ingest()
            announcement = self._announce()
            conn.sendall(announcement)
            self.log(f"sent SettingsResponse for {', '.join(self.source.roles)}")

            last_announce = time.monotonic()
            conn.settimeout(0.0)
            while not self._stop.is_set():
                try:
                    if not conn.recv(65536):
                        break
                except (BlockingIOError, socket.timeout):
                    pass
                except OSError:
                    break

                # The runtime is happy to re-read the settings; repeating it
                # makes a reconnect in the middle of a session recover.
                if time.monotonic() - last_announce >= 1.0:
                    conn.sendall(announcement)
                    last_announce = time.monotonic()

                self._ingest()
                try:
                    conn.sendall(self._update())
                except OSError:
                    break
                time.sleep(period)
        except OSError as e:
            self.log(f"connection ended: {e}")
        finally:
            try:
                conn.close()
            except OSError:
                pass
