"""Encoding checks.

The hashes below were taken from an implementation that has been driving
WiVRn successfully, so they pin the exact byte layout rather than merely
"something that parses". A flatbuffers message can be semantically correct
and still be rejected by a reader that expects a particular field order, and
that failure looks like nothing at all: no error, no trackers.
"""

import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from solarxr_feeder import protocol


ROLES = ("left_foot", "right_foot", "waist")

THREE_TRACKERS = [
    ("left_foot", 1, (-0.3, -1.0, 0.2), (0.0, 0.0, 0.0, 1.0)),
    ("right_foot", 2, (0.2, -1.0, 0.2), (0.0, 0.0, 0.0, 1.0)),
    ("waist", 3, (0.0, -0.4, 0.0), (0.0, 0.0, 0.0, 1.0)),
]

ONLY_WAIST = [("waist", 3, None, (0.0, 0.0, 0.0, 1.0))]

GOLDEN = {
    "three_trackers": {
        "settings": "272678dabf993e69ca8ef5f77f661e5a3e00a59cb8467db26bbeda79b0149269",
        "update": "b97672b83f2f636553128acf07e82b0bbc2b5e04f17050dd85243778dd15bb45",
    },
    "only_waist": {
        "settings": "9cf5eeeef1f97cc8d37267ea45b50b8d95b5675b789a7236e5810dfad47b5dbc",
        "update": "c41ca6cb1d94f3fe0cb0b8b1c2fc0d793d694a34b255e3be4a54baf60ae9778b",
    },
}

CASES = {"three_trackers": THREE_TRACKERS, "only_waist": ONLY_WAIST}


def _digest(payload):
    return hashlib.sha256(payload).hexdigest()


def test_settings_response_layout():
    for name, trackers in CASES.items():
        payload = protocol.encode_settings_response(ROLES, trackers)
        assert _digest(payload) == GOLDEN[name]["settings"], (
            f"SettingsResponse bytes changed for {name}; a runtime that worked "
            f"before may now enumerate nothing"
        )


def test_update_layout():
    for name, trackers in CASES.items():
        payload = protocol.encode_update(trackers)
        assert _digest(payload) == GOLDEN[name]["update"], (
            f"DataFeedUpdate bytes changed for {name}"
        )


def test_every_role_is_introduced():
    """Every advertised role must also be present as a device.

    The `roles` argument only sets SteamVRTrackersSetting flags. The devices
    themselves come from the tracker list, and the runtime creates them once
    from this bundle; a role omitted there never exists for the rest of the
    session, however many poses arrive later.

    The service fills the list for every role, using the origin for whichever
    one has no pose yet, so this checks the encoder keeps writing the names.
    """
    payload = protocol.encode_settings_response(ROLES, THREE_TRACKERS)
    for role in ROLES:
        assert role.encode() in payload, f"{role} missing from the settings reply"


def test_no_pose_still_introduces_device():
    payload = protocol.encode_update(ONLY_WAIST)
    assert b"waist" in payload


if __name__ == "__main__":
    passed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"  ok  {name}")
            passed += 1
    print(f"\n{passed} checks passed")
