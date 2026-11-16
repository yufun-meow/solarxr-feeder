# CV1 pose packet format

English | [简体中文](PACKET.zh.md)

The bundled source (`sources/cv1_udp.py`) reads this format. It is sent by
[docs/reference-reader.c](reference-reader.c) as one UDP datagram per frame,
104 bytes, little-endian, at up to 240 Hz.

## Layout

```c
struct cv1fbt_packet {
    unsigned int   magic;         // 0x54424631, "1FBT"
    unsigned char  version;       // 1
    unsigned char  flags;
    unsigned short reserved;
    double         time_seconds;  // ovr_GetTimeInSeconds at sampling

    cv1fbt_pose    left_foot;
    cv1fbt_pose    right_foot;
    cv1fbt_pose    waist;

    unsigned short valid;         // bit i: pose i has been seen
    unsigned short visible;       // bit i: device i is seen right now
};

struct cv1fbt_pose {              // 28 bytes
    float qx, qy, qz, qw;         // orientation
    float px, py, pz;             // position, metres
};
```

`reserved` is a `unsigned short`, not a `unsigned char`. With the padding that
the `double` needs, the header is 16 bytes:

```
0  magic          4
4  version        1
5  flags          1
6  reserved       2
8  time_seconds   8
16 poses          84  (3 x 28)
100 valid         2
102 visible       2
104 total
```

As a Python format string:

```python
PACKET = struct.Struct("<IBBHd" + "7f" * 3 + "HH")
```

## Two kinds of "not there"

The format separates two things that a single flag would conflate:

- **`valid`** - this device has been seen at some point, and its pose is a
  real place it once was
- **`visible`** - the device is being seen right now

A device that leaves the field of view keeps being sent, with `visible`
cleared and its last known pose. That way it holds still instead of snapping
to the origin, which is what a tracker should do.

A pose of exactly `(0, 0, 0)` means the runtime could not place the device at
all. It is filtered out rather than accepted, because a worn tracker never
sits exactly at the origin and accepting it drags the tracker to the floor.

## Bit masks

| bit | device |
|---|---|
| 0 | left foot |
| 1 | right foot |
| 2 | waist |

## The waist has no visibility flag

The reader cannot fill `visible` for the tracked object truthfully. The
runtime reports its own pose query as `LostTracking` even when the object is
tracked perfectly well, so there is nothing to base the flag on. The bundled
source therefore treats the object as visible whenever it has a pose.

A sender is free to set the bit anyway; the receiver ignores it for that
device.

## BTW....

Does anyone else actually spin up a separate Windows machine just to track this thing? I tried a lot of CV1 tracking approaches on Linux and put a lot of effort into getting them to support vr object — they can track, but the quality is so bad it is unusable, so this is the only way. I open-sourced this so that you can write your own source for whatever you want to use as a tracker — so, over to you (laughs).



## Command line of the reference reader

```
cv1-fbt-source.exe [--dest 192.168.1.50] [--port 39570] [--hz 240] [--quiet]
```
