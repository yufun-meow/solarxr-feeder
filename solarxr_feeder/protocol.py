"""SolarXR IPC: the wire format a runtime expects from a tracker source.

WiVRn and Monado look for trackers over a pair of Unix sockets that the
tracker side - this code - has to create. Nothing about this is documented
anywhere upstream; every constant below was worked out by reading Monado's
reader (src/xrt/auxiliary/tracking/solarxr/protocol.c and solarxr_device.c)
and by watching what the runtime accepted.

Two sockets are involved:

    $XDG_RUNTIME_DIR/SlimeVRRpc     the runtime connects here to ask for settings
    $XDG_RUNTIME_DIR/SlimeVRInput   the runtime connects here to receive poses

The order matters. The runtime connects to SlimeVRRpc first, sends a poll
request, and waits for a bundle containing a SettingsResponse. Only after
that does it connect to SlimeVRInput.

Three things are easy to get wrong and each produces the same silent
outcome - no trackers at all:

1.  Both sockets must be created by this side. If SlimeVRInput does not
    exist, the runtime logs "failed to resolve socket path" and gives up.
2.  The SettingsResponse bundle must also carry a DataFeedUpdate that
    introduces every device. The runtime creates its tracker objects from
    that update; an empty one yields "Enumerated 0 SolarXR trackers".
3.  Every tracker has to be introduced, even the ones without a pose yet.
    Sending only the currently-tracked devices means the rest never appear.
"""

import os

import flatbuffers

# ------------------------------------------------------------------ schema

# Body part ids, from SolarXR's BodyPart enum.
BODY_PARTS = {
    "none": 0, "head": 1, "neck": 2, "chest": 3, "waist": 4, "hip": 5,
    "left_upper_leg": 6, "right_upper_leg": 7,
    "left_lower_leg": 8, "right_lower_leg": 9,
    "left_foot": 10, "right_foot": 11,
    "left_hand": 18, "right_hand": 19,
}

# Field index inside SteamVRTrackersSetting for each role we may drive.
TRACKER_SETTING_FIELDS = {
    "waist": 0, "chest": 1,
    "left_foot": 7, "right_foot": 8,
    "left_knee": 9, "right_knee": 10,
    "left_elbow": 11, "right_elbow": 12,
    "left_hand": 13, "right_hand": 14,
}

# DataFeedMessageHeader.message_type values.
MSG_POLL = 1
MSG_START = 2
MSG_UPDATE = 3

# RpcMessageHeader.message_type value for a settings reply.
RPC_SETTINGS_RESPONSE = 7

# flatbuffers slot ids. The schema's fields are declared in this order, and
# Monado reads them by vtable slot, so they are not free to change.
DH_MESSAGE_TYPE = 4
DH_MESSAGE = 6

DFU_DEVICES = 4
DFU_SYNTHETIC = 6
DFU_BONES = 8

# DeviceData is read with trackers at vtable[4] - field index 3 - so the table
# has to be declared with four slots even though we only set two fields.
DD_TRACKERS_FIELD = 4

TD_TRACKER_ID = 4
TD_INFO = 6
TD_STATUS = 8
TD_ROTATION = 10
TD_POSITION = 12
TD_RAW_ANGVEL = 14

# TrackerData.status: 1 means the pose is valid.
STATUS_OK = 1


def socket_path(name):
    """Where the runtime looks for a SolarXR socket.

    Mirrors solarxr_ipc_socket_find() in Monado: the runtime directory first,
    then the SlimeVR data directories. Only the first is implemented here,
    which is where every runtime on Linux looks.
    """
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}"
    return os.path.join(runtime_dir, name)


# ------------------------------------------------------------- encoding


def _tracker_object(b, role, device_id, position, orientation):
    """One TrackerData table."""
    name_off = b.CreateString(role)

    # TrackerInfo { body_part, display_name }
    b.StartObject(8)
    b.PrependByteSlot(1, BODY_PARTS[role] & 0xFF, 0)
    b.PrependUOffsetTRelativeSlot(7, name_off, 0)
    info = b.EndObject()

    # TrackerId { device_id, tracker_num }
    b.StartObject(2)
    b.PrependByteSlot(0, device_id & 0xFF, 0)
    b.PrependByteSlot(1, 0, 0)
    tracker_id = b.EndObject()

    b.StartObject(9)
    b.PrependUOffsetTRelativeSlot(0, tracker_id, 0)
    b.PrependUOffsetTRelativeSlot(1, info, 0)
    b.PrependByteSlot(2, STATUS_OK, 0)

    if position is not None:
        b.Prep(4, 12)
        for value in reversed(position):
            b.PrependFloat32(float(value))
        b.PrependStructSlot(4, b.Offset(), 0)

    if orientation is not None:
        b.Prep(4, 16)
        for value in reversed(orientation):
            b.PrependFloat32(float(value))
        b.PrependStructSlot(3, b.Offset(), 0)

    return b.EndObject()


def _device_data(b, device_id, tracker_offset):
    """Wrap one TrackerData into a DeviceData holding that single tracker."""
    b.StartVector(4, 1, 4)
    b.PrependUOffsetTRelative(tracker_offset)
    trackers_vec = b.EndVector()

    b.StartObject(5)
    b.PrependByteSlot(0, device_id & 0xFF, 0)
    b.PrependUOffsetTRelativeSlot(DD_TRACKERS_FIELD, trackers_vec, 0)
    return b.EndObject()


def _build_devices_interleaved(b, trackers):
    """Tracker and wrapper built together, which is what the settings reply does.

    Purely a layout choice: the two orders produce different bytes but the
    same message. Both are kept because each matches the reference
    implementation byte for byte, so a regression shows up as a failing
    comparison rather than as a runtime that silently enumerates nothing.
    """
    devices = []
    for role, device_id, position, orientation in trackers:
        tracker = _tracker_object(b, role, device_id, position, orientation)
        devices.append(_device_data(b, device_id, tracker))
    return devices


def _build_devices(b, trackers):
    """All TrackerData tables first, then their DeviceData wrappers.

    The order is not arbitrary: flatbuffers appends each finished object to
    the buffer, so interleaving the two kinds produces a different byte
    layout. The layout it produces is still valid, but keeping it identical
    to the implementation this was extracted from means the well-tested
    output is unchanged.
    """
    tracker_offsets = [_tracker_object(b, role, device_id, position, orientation)
                       for role, device_id, position, orientation in trackers]
    return [_device_data(b, device_id, offset)
            for (role, device_id, _position, _orientation), offset
            in zip(trackers, tracker_offsets)]


def encode_update(trackers):
    """Wrap poses into the MessageBundle that carries a DataFeedUpdate.

    `trackers` is a sequence of (role, device_id, position, orientation).
    Every tracker that should exist must be present - the vector is the
    complete set, not a delta.
    """
    b = flatbuffers.Builder(4096)

    devices = _build_devices(b, trackers)

    b.StartVector(4, len(devices), 4)
    for device in reversed(devices):
        b.PrependUOffsetTRelative(device)
    devices_vec = b.EndVector()

    # DataFeedUpdate { devices, synthetic_trackers, bones }
    b.StartObject(3)
    b.PrependUOffsetTRelativeSlot(0, devices_vec, 0)
    update = b.EndObject()

    # DataFeedMessageHeader { message_type, message }
    b.StartObject(2)
    b.PrependByteSlot(0, MSG_UPDATE, 0)
    b.PrependUOffsetTRelativeSlot(1, update, 0)
    header = b.EndObject()

    b.StartVector(4, 1, 4)
    b.PrependUOffsetTRelative(header)
    data_feed_msgs = b.EndVector()

    # MessageBundle { data_feed_msgs (0), rpc_msgs (1) }
    b.StartObject(2)
    b.PrependUOffsetTRelativeSlot(0, data_feed_msgs, 0)
    bundle = b.EndObject()

    b.Finish(bundle)
    return bytes(b.Output())


def encode_settings_response(roles, trackers=()):
    """The reply to the runtime's settings poll.

    `roles` are the body parts this source can drive. `trackers` is the same
    shape as for encode_update and has to introduce every device, because the
    runtime creates its trackers from this bundle rather than from the first
    pose update.
    """
    b = flatbuffers.Builder(2048)

    wanted = set(roles)
    fields = sorted(TRACKER_SETTING_FIELDS[role] for role in wanted
                    if role in TRACKER_SETTING_FIELDS)
    slots = (max(fields) + 1) if fields else 1

    # SteamVRTrackersSetting: one bool per field, declared up to the highest
    # index actually set.
    b.StartObject(slots)
    for index in fields:
        b.PrependBoolSlot(index, True, False)
    trackers_setting = b.EndObject()

    # SettingsResponse { steam_vr_trackers }
    b.StartObject(1)
    b.PrependUOffsetTRelativeSlot(0, trackers_setting, 0)
    response = b.EndObject()

    # RpcMessageHeader { message_type, message }. Monado reads these from
    # vtable[1] and vtable[2], hence the slot indices 1 and 2.
    b.StartObject(3)
    b.PrependByteSlot(1, RPC_SETTINGS_RESPONSE, 0)
    b.PrependUOffsetTRelativeSlot(2, response, 0)
    rpc_header = b.EndObject()

    b.StartVector(4, 1, 4)
    b.PrependUOffsetTRelative(rpc_header)
    rpc_msgs = b.EndVector()

    devices = _build_devices_interleaved(b, trackers)

    if devices:
        b.StartVector(4, len(devices), 4)
        for device in reversed(devices):
            b.PrependUOffsetTRelative(device)
        devices_vec = b.EndVector()

    # DataFeedUpdate introduces the devices. Poses given here only create
    # them; real poses follow on the stream socket.
    b.StartObject(3)
    if devices:
        b.PrependUOffsetTRelativeSlot(0, devices_vec, 0)
    update = b.EndObject()

    b.StartObject(2)
    b.PrependByteSlot(0, MSG_UPDATE, 0)
    b.PrependUOffsetTRelativeSlot(1, update, 0)
    data_feed_header = b.EndObject()

    b.StartVector(4, 1, 4)
    b.PrependUOffsetTRelative(data_feed_header)
    data_feed_msgs = b.EndVector()

    # MessageBundle { data_feed_msgs (0), rpc_msgs (1) }. The order is part of
    # the schema, not a choice: putting the rpc vector in slot 0 makes the
    # runtime read a rpc header where it expects a data feed one, and the
    # handshake fails with no trackers and no error.
    b.StartObject(2)
    b.PrependUOffsetTRelativeSlot(0, data_feed_msgs, 0)
    b.PrependUOffsetTRelativeSlot(1, rpc_msgs, 0)
    bundle = b.EndObject()

    b.Finish(bundle)
    return bytes(b.Output())
