# SolarXR tracker protocol, as WiVRn and Monado use it

English | [简体中文](PROTOCOL.zh.md)

Everything here was worked out by reading Monado's C sources and by watching
what the runtime accepted. It is written down because none of it is
documented, and every item below was a thing that failed silently first.

## Two sockets, both created by the tracker side

The runtime *connects out*. That is the opposite of what "server" suggests,
and it means the tracker side has to be listening before the runtime starts.

| Path | Direction | Purpose |
|---|---|---|
| `$XDG_RUNTIME_DIR/SlimeVRRpc` | runtime connects here | asks for settings, then waits for the reply |
| `$XDG_RUNTIME_DIR/SlimeVRInput` | runtime connects here | sends its own device reports |

The order is fixed: the runtime attaches to `SlimeVRRpc`, sends a request,
waits for a `SettingsResponse`, and only then attaches to `SlimeVRInput`.

Consequences:

- If `SlimeVRInput` does not exist, the runtime logs `failed to resolve
  socket path` and refuses to attach. It is easy to implement only the RPC
  socket and be confused by that message.
- Stopping and restarting the tracker side leaves the runtime holding
  connections to sockets that no longer exist. It does not reconnect, so the
  runtime has to be restarted as well.

## The settings reply carries the device list

The bundle sent in reply to the settings poll contains both an
`RpcMessageHeader` with a `SettingsResponse` and a `DataFeedUpdate`. The
runtime enumerates its trackers from **that** update, at connection time.

- An empty update gives `Enumerated 0 SolarXR trackers`.
- Sending only the devices that happen to be tracked at that instant means
  the others never exist for the whole session, however many poses arrive
  later.

So: introduce every device, with an arbitrary pose if none is known yet
(the origin works).

## Field order that matters

`MessageBundle` is:

```
table MessageBundle {
  data_feed_msgs: [DataFeedMessageHeader];   // field 0
  rpc_msgs:       [RpcMessageHeader];        // field 1
}
```

Field 0 is the **data feed**. Putting the rpc vector there produces a message
that is still valid flatbuffers, is still parsed, and yields no trackers and
no error. This was got wrong and took a byte-level comparison against a known
working encoder to find.

Other layout facts that are not free to change:

- `DeviceData.trackers` is read at vtable slot 4, i.e. field index 3, so the
  table has to be declared with four slots.
- `RpcMessageHeader` is read at vtable slots 1 and 2, so field index 0 is
  unused and the indices start at 1.
- `TrackerData.status` must be 1 for the pose to be used.

## Framing

Replies are length-prefixed, and the length **includes the four bytes of the
prefix itself**:

```python
conn.sendall(struct.pack("<I", len(payload) + 4) + payload)
```

Getting that off by four leaves the runtime waiting for bytes that never come.

## Pose updates

Each frame sends one `MessageBundle` holding a `DataFeedUpdate` whose
`devices` vector lists every device. The vector is the complete set, not a
delta, so a device omitted from a frame simply keeps its previous pose.

60 Hz is comfortable; the runtime reads whatever arrives.

## The runtime's own socket

Nothing in this project reads `SlimeVRInput`. Monado's feeder writes its
device reports there, and accepting the connection is enough to keep it
happy. Draining it is still necessary - an unread socket eventually fills
and blocks the writer.

## Visibility, and why the tracked object is special

For the hands, the runtime reports status bits that say whether the device is
tracked right now. For the tracked object there is no status word at all: the
only signal is the result of the runtime's own pose query, and that comes back
as `ovrError_LostTracking` even while the object's pose is fresh and moving.

The practical consequence for a source: judge the object by its pose (is the
timestamp set, is the position non-zero), not by the query result. Otherwise
the object is always reported as invisible while clearly working.

## Sanity check

Start the feeder, then start the runtime. A healthy run looks like:

```
[feeder] SlimeVRRpc listening on /run/user/1000/SlimeVRRpc
[feeder] SlimeVRInput listening on /run/user/1000/SlimeVRInput
[feeder] runtime connected
[feeder] sent SettingsResponse for left_foot, right_foot, waist
```

and on the runtime's side:

```
INFO [solarxr_device_create_xdevs] Enumerated 3 SolarXR physical trackers
```

If that says 0, the device list in the settings bundle is empty or in the
wrong field.
