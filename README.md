# solarxr-feeder

English | [简体中文](README.zh.md)

Feed any pose source to WiVRn or Monado as [SolarXR] trackers.

WiVRn and Monado treat body trackers as SolarXR devices, and they discover
them over a pair of Unix sockets. Nothing outside SlimeVR implements the
tracker side of that protocol, and the details are not documented anywhere -
they have to be read out of Monado's C sources. This project is a minimal,
working implementation of that side, so any program that knows where a body
part is can drive trackers in VR on Linux.

It ships with one source: a CV1 rig on a Windows machine, which reads poses
from the official Oculus runtime and broadcasts them over the network. That
is how it was built, but the protocol layer does not care where poses come
from.Therefore, you can adapt it to anything that can track its own position—you just need to write a data source according to the tutorial below.

[SolarXR]: https://github.com/SlimeVR/SolarXR-Protocol

## What it looks like

```
   your hardware            this program                 runtime
  ───────────────          ─────────────               ──────────
                       ┌─────────────────────┐
  CV1 on Windows  ──►  │  source: cv1_udp    │
   (UDP packets)       │        │            │
                       │        ▼            │
                       │  SolarXRService     │  SlimeVRRpc    ──►  WiVRn
                       │   (two sockets)     │  SlimeVRInput  ──►  Monado
                       └─────────────────────┘                       │
                                                                     ▼
                                                                  headset
```

The runtime places the trackers in the headset's tracking origin, which is a
different space from whatever the source uses, so a transform maps one onto
the other. See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the protocol notes
and [docs/PACKET.md](docs/PACKET.md) for the CV1 wire format.

## Requirements

- Python 3.9 or newer
- [`flatbuffers`](https://pypi.org/project/flatbuffers/) - `pip install flatbuffers`
- WiVRn or Monado, with raw SolarXR trackers enabled:
  `SOLARXR_RAW_TRACKERS=true wivrn-server`

## Running

```bash
pip install flatbuffers
./feeder.py
```

By default it listens on UDP 39570 for CV1 packets and pushes poses to the
runtime at 60 Hz.

```
--source cv1           pose source (only cv1 for now)
--port 39570           UDP port the source sends to
--hz 60                how often poses are pushed
--transform PATH       mapping into the headset's space
--mirror-port 39571    copy every packet here for tools/monitor.py
```

The runtime must find the sockets before it starts, so start this first.
If you restart it, restart WiVRn too: the runtime connects once and does not
retry.

### Where the poses come from

The bundled source expects the packets produced by
[docs/reference-reader.c](docs/reference-reader.c), a small Windows program
that reads the Oculus runtime and sends 104-byte datagrams. Build it with
MinGW:

```bash
x86_64-w64-mingw32-gcc -O2 -static -o cv1-fbt-source-win64.exe docs/reference-reader.c -lws2_32
cv1-fbt-source-win64.exe --dest <this machine>
```

You do not have to use it. Anything that can send the same packets works, and
writing a new source is not much work - see below.

## Writing a source

Subclass `TrackerSource` and return poses for the roles you can drive:

```python
from solarxr_feeder.service import TrackerSource

class MyTrackers(TrackerSource):
    roles = ("left_foot", "right_foot", "waist")

    def poll(self):
        ...                     # read whatever arrived

    def pose(self, role):
        return (position, orientation)   # or None if not tracked now
```

Returning `None` means "no fresh pose", and the service holds the previous
one. A tracker that jumps to the floor whenever it is briefly occluded is
worse than one that stays put.

Then add it to `build_source()` in `feeder.py` and set `--source`.

## Calibration

`tools/calibrate.py` measures the transform for you. It needs an OpenXR
runtime with the headset's controller tracked, and the pose source running:

```bash
./tools/calibrate.py --cv1-role left_foot --hand left
```

Hold that source device against that controller, press enter, and move the
pair around for 25 seconds as if they were one object.

The part that is easy to get wrong is **turning** it. Carrying the pair
around without rotating it leaves the fixed offset between the two devices
constant in the room, which a fit absorbs into the translation: the result is
internally perfect, physically wrong, and different on every run. Turn it
like a door handle, through 180° or more, while also covering the room. The
tool measures how far you turned it and refuses to save anything from less
than `--min-turn` (90° by default).

Layering note: hold the pair **together**, not at arm's length. The offset
you cannot avoid is the distance between the two devices' centres, and it
becomes a constant error in the result.

A healthy result:

```
  水平转动 178°
Samples: 24000   residual: 8.3 mm
Heading    -102.0°   (tilt fixed at 0 by gravity)
Translation +0.782 +1.751 -0.444 m
  unconstrained fit would have tilted 33.7° (residual 10.2 mm); that tilt is sensor noise
Written to ~/.config/solarxr-feeder/transform.json
```

`+1.751` in the translation is the eye height the source's origin sits at -
a useful sanity check that the answer is plausible.

## Mapping between spaces

The trackers land in the headset's tracking origin. If your source uses a
different origin - a different room, a different set of cameras - the
trackers will be in the right place relative to each other and the wrong
place relative to everything else.

`--transform` points at a JSON file mapping one onto the other:

```json
{
  "rotation":    { "x": 0.0, "y": -0.777, "z": 0.0, "w": 0.630 },
  "translation": { "x": 0.782, "y": 1.751, "z": -0.444 }
}
```

Without it, poses pass through unchanged, which is only right if the spaces
already agree.

A useful property of this problem: both sides track orientation with
accelerometers, so both agree on which way is down. The rotation between
them is therefore a turn about the vertical axis and nothing else. A
six-degree-of-freedom fit will happily invent a tilt out of sensor noise, and
if the devices are only carried around without being turned, the extra
freedom has nothing to pin it down and the answer changes from run to run.
`Transform.heading_only()` applies that constraint.

## Tests

```bash
python3 tests/test_protocol.py    # encoded bytes
python3 tests/test_solve.py       # the fit, on data with a known answer
python3 tests/test_service.py     # end to end, with a stand-in runtime
```

`test_service.py` creates the real socket paths, so stop any feeder first.
`test_solve.py` needs no hardware: it checks the fit against synthetic data
where the answer is known. Real captures have no ground truth, and a fit that
is merely self-consistent can look perfect and still be wrong.

The tests pin the exact encoded bytes against values taken from an
implementation that has been driving WiVRn successfully. That is deliberate:
a flatbuffers message can be structurally valid and still be rejected by a
reader expecting a particular field order, and the symptom is nothing at all -
no error, no trackers.

## Status

Tested against WiVRn 26.9 on a Quest 3S over a Radeon-less NVIDIA desktop, on
CachyOS, with three trackers. Other runtimes should work; Monado is what WiVRn
uses underneath.

Known rough edges:

- The CV1 source treats the tracked object specially. The runtime reports the
  query for that device as `LostTracking` even while its pose is fresh, so
  there is no reliable visibility flag and the pose itself decides.
- Only the roles listed in `sources/cv1_udp.py` are exercised. Others are
  encoded but untested.

## Licence

MIT. See [LICENSE](LICENSE).
