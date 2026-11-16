"""Minimal OpenXR bindings for reading a controller's pose.

Used by the calibration tool, which needs the headset's own tracking of a
controller as the reference to line a source up against.

XR_MND_headless lets a session exist with no graphics API bound, which is
exactly what a pose-only client needs. The runtime is chosen the usual way
through XR_RUNTIME_JSON, so pointing that at WiVRn (or Monado) picks it.

Everything here is ABI: structure layouts, structure type ids and entry point
signatures have to match openxr.h exactly. A wrong structure type makes the
loader reinterpret memory and crash, which is why the ids are written out
rather than guessed.
"""

import ctypes
import ctypes.util
import os
import sys

# ------------------------------------------------------------------ OpenXR ABI

XR_SUCCESS = 0
XR_SESSION_NOT_FOCUSED = 3
XR_EVENT_UNAVAILABLE = -1000103000
XR_INFINITE_DURATION = 0x7FFFFFFFFFFFFFFF
XR_NULL_SYSTEM_ID = 0
XR_MAX_ACTION_NAME_SIZE = 256
XR_MAX_PATH_LENGTH = 256
XR_MAX_STRUCTURE_NAME_SIZE = 64

# Structure types, verified against /usr/include/openxr/openxr.h. Getting one
# of these wrong makes the loader reinterpret the struct and crash.
XR_TYPE_INSTANCE_CREATE_INFO = 3
XR_TYPE_SYSTEM_GET_INFO = 4
XR_TYPE_SYSTEM_PROPERTIES = 5
XR_TYPE_SESSION_CREATE_INFO = 8
XR_TYPE_EVENT_DATA_BUFFER = 16
XR_TYPE_ACTION_STATE_POSE = 27
XR_TYPE_ACTION_SET_CREATE_INFO = 28
XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO = 60
XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING = 51
XR_TYPE_ACTIONS_SYNC_INFO = 61
XR_TYPE_ACTION_CREATE_INFO = 29
XR_TYPE_REFERENCE_SPACE_CREATE_INFO = 37
XR_TYPE_ACTION_SPACE_CREATE_INFO = 38
XR_TYPE_SPACE_LOCATION = 42
XR_TYPE_SPACE_VELOCITY = 43
XR_TYPE_ACTION_STATE_GET_INFO = 58

XR_ACTION_TYPE_POSE_INPUT = 4
XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY = 1
XR_MAX_ACTION_SET_NAME_SIZE = 64
XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE = 128
XR_MAX_ACTION_NAME_SIZE = 64
XR_MAX_LOCALIZED_ACTION_NAME_SIZE = 128
XR_MAX_STRUCTURE_NAME_SIZE = 64
XR_MAX_SYSTEM_NAME_SIZE = 256

XR_REFERENCE_SPACE_TYPE_LOCAL = 2
XR_REFERENCE_SPACE_TYPE_STAGE = 3

XR_SPACE_LOCATION_ORIENTATION_VALID_BIT = 0x00000001
XR_SPACE_LOCATION_POSITION_VALID_BIT = 0x00000002
XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT = 0x00000004
XR_SPACE_LOCATION_POSITION_TRACKED_BIT = 0x00000008


class XrQuaternionf(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float),
                ("z", ctypes.c_float), ("w", ctypes.c_float)]


class XrVector3f(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float),
                ("z", ctypes.c_float)]


class XrPosef(ctypes.Structure):
    _fields_ = [("orientation", XrQuaternionf), ("position", XrVector3f)]


class XrApplicationInfo(ctypes.Structure):
    _fields_ = [
        ("applicationName", ctypes.c_char * 128),
        ("applicationVersion", ctypes.c_uint32),
        ("engineName", ctypes.c_char * 128),
        ("engineVersion", ctypes.c_uint32),
        ("apiVersion", ctypes.c_uint64),
    ]


class XrInstanceCreateInfo(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("next", ctypes.c_void_p),
        ("createFlags", ctypes.c_uint64),
        ("applicationInfo", XrApplicationInfo),
        ("enabledApiLayerCount", ctypes.c_uint32),
        ("enabledApiLayerNames", ctypes.c_void_p),
        ("enabledExtensionCount", ctypes.c_uint32),
        # const char* const* - a pointer, not an inline array.
        ("enabledExtensionNames", ctypes.c_void_p),
    ]


class XrSystemGetInfo(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("formFactor", ctypes.c_int)]


class XrSystemGraphicsProperties(ctypes.Structure):
    _fields_ = [("maxSwapchainImageHeight", ctypes.c_uint32),
                ("maxSwapchainImageWidth", ctypes.c_uint32),
                ("maxLayerCount", ctypes.c_uint32)]


class XrSystemTrackingProperties(ctypes.Structure):
    _fields_ = [("orientationTracking", ctypes.c_int),
                ("positionTracking", ctypes.c_int)]


class XrSystemProperties(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("next", ctypes.c_void_p),
        ("systemId", ctypes.c_uint64), ("vendorId", ctypes.c_uint32),
        ("systemName", ctypes.c_char * XR_MAX_SYSTEM_NAME_SIZE),
        ("graphicsProperties", XrSystemGraphicsProperties),
        ("trackingProperties", XrSystemTrackingProperties),
    ]


class XrSessionCreateInfo(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("createFlags", ctypes.c_uint64), ("systemId", ctypes.c_uint64)]


class XrActionSetCreateInfo(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("next", ctypes.c_void_p),
        ("actionSetName", ctypes.c_char * XR_MAX_ACTION_SET_NAME_SIZE),
        ("localizedActionSetName", ctypes.c_char * XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE),
        ("priority", ctypes.c_uint32),
    ]


class XrActionCreateInfo(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("next", ctypes.c_void_p),
        ("actionName", ctypes.c_char * XR_MAX_ACTION_NAME_SIZE),
        ("actionType", ctypes.c_int),
        ("countSubactionPaths", ctypes.c_uint32),
        ("subactionPaths", ctypes.c_void_p),
        ("localizedActionName", ctypes.c_char * XR_MAX_LOCALIZED_ACTION_NAME_SIZE),
    ]


class XrActionSpaceCreateInfo(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("next", ctypes.c_void_p),
        ("action", ctypes.c_uint64), ("subactionPath", ctypes.c_uint64),
        ("poseInActionSpace", XrPosef),
    ]


class XrActionStateGetInfo(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int), ("next", ctypes.c_void_p),
        ("action", ctypes.c_uint64), ("subactionPath", ctypes.c_uint64),
    ]


class XrActionStatePose(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("isActive", ctypes.c_int)]


class XrSpaceLocation(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("locationFlags", ctypes.c_uint64), ("pose", XrPosef)]


class XrSpaceVelocity(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("velocityFlags", ctypes.c_uint64),
                ("linearVelocity", XrVector3f),
                ("angularVelocity", XrVector3f)]


class XrReferenceSpaceCreateInfo(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("referenceSpaceType", ctypes.c_int),
                ("poseInReferenceSpace", XrPosef)]


class XrActionSetSyncInfo(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("actionSet", ctypes.c_uint64)]


class XrEventDataBuffer(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("varying", ctypes.c_byte * 4000)]


class XrActionSuggestedBinding(ctypes.Structure):
    _fields_ = [("action", ctypes.c_uint64), ("binding", ctypes.c_uint64)]


class XrInteractionProfileSuggestedBinding(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("interactionProfile", ctypes.c_uint64),
                ("countSuggestedBindings", ctypes.c_uint32),
                ("suggestedBindings", ctypes.POINTER(XrActionSuggestedBinding))]


class XrSessionActionSetsAttachInfo(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("countActionSets", ctypes.c_uint32),
                ("actionSets", ctypes.POINTER(ctypes.c_uint64))]


class XrActiveActionSet(ctypes.Structure):
    _fields_ = [("actionSet", ctypes.c_uint64), ("subactionPath", ctypes.c_uint64)]


class XrActionsSyncInfo(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("next", ctypes.c_void_p),
                ("countActiveActionSets", ctypes.c_uint32),
                ("activeActionSets", ctypes.POINTER(XrActiveActionSet))]


def load_lib():
    """Open the OpenXR loader."""
    name = os.environ.get("OPENXR_LOADER")
    if name:
        return ctypes.CDLL(name)
    for candidate in ("libopenxr_loader.so.1", "libopenxr_loader.so",
                      ctypes.util.find_library("openxr_loader")):
        if not candidate:
            continue
        try:
            return ctypes.CDLL(candidate)
        except OSError:
            continue
    sys.exit("could not load the OpenXR loader")


class MonadoXR:
    """Reads one tracked device's pose from an OpenXR runtime.

    Mirrors the OpenHMD helper used elsewhere in this file so the pose source
    can be swapped without touching the SolarXR side.
    """

    # The instance is process-wide and expensive, but several of these objects
    # are created (one per tracker). Share it.
    _shared = None

    def __init__(self, index, runtime=None):
        if runtime:
            os.environ["XR_RUNTIME_JSON"] = runtime
        L = load_lib()
        self.L = L
        self._bind(L)

        self.instance, self.system_id, self.session, self.app_space = \
            MonadoXR._ensure_started(L)

        # Discover the tracker devices once.
        if MonadoXR._shared["paths"] is None:
            MonadoXR._shared["paths"] = self._enumerate_trackers()
        paths = MonadoXR._shared["paths"]
        if index >= len(paths):
            sys.exit(f"OpenXR runtime reports {len(paths)} tracker(s), "
                     f"device {index} requested")
        self.path = paths[index]
        self.name = self.path.rsplit("/", 1)[-1]

        self.action = self._create_pose_action(self.path)

    # ---------------------------------------------------------------- setup

    @staticmethod
    def _bind(L):
        L.xrCreateInstance.restype = ctypes.c_int
        L.xrCreateInstance.argtypes = [ctypes.POINTER(XrInstanceCreateInfo),
                                       ctypes.POINTER(ctypes.c_uint64)]
        L.xrGetSystem.restype = ctypes.c_int
        L.xrGetSystem.argtypes = [ctypes.c_uint64, ctypes.POINTER(XrSystemGetInfo),
                                  ctypes.POINTER(ctypes.c_uint64)]
        L.xrGetSystemProperties.restype = ctypes.c_int
        L.xrGetSystemProperties.argtypes = [ctypes.c_uint64, ctypes.c_uint64,
                                            ctypes.POINTER(XrSystemProperties)]
        L.xrCreateSession.restype = ctypes.c_int
        L.xrCreateSession.argtypes = [ctypes.c_uint64,
                                      ctypes.POINTER(XrSessionCreateInfo),
                                      ctypes.POINTER(ctypes.c_uint64)]
        L.xrCreateActionSet.restype = ctypes.c_int
        L.xrCreateActionSet.argtypes = [ctypes.c_uint64,
                                        ctypes.POINTER(XrActionSetCreateInfo),
                                        ctypes.POINTER(ctypes.c_uint64)]
        L.xrCreateAction.restype = ctypes.c_int
        L.xrCreateAction.argtypes = [ctypes.c_uint64,
                                     ctypes.POINTER(XrActionCreateInfo),
                                     ctypes.POINTER(ctypes.c_uint64)]
        L.xrCreateActionSpace.restype = ctypes.c_int
        L.xrCreateActionSpace.argtypes = [ctypes.c_uint64,
                                          ctypes.POINTER(XrActionSpaceCreateInfo),
                                          ctypes.POINTER(ctypes.c_uint64)]
        L.xrCreateReferenceSpace.restype = ctypes.c_int
        L.xrCreateReferenceSpace.argtypes = [ctypes.c_uint64,
                                             ctypes.POINTER(XrReferenceSpaceCreateInfo),
                                             ctypes.POINTER(ctypes.c_uint64)]
        L.xrStringToPath.restype = ctypes.c_int
        L.xrStringToPath.argtypes = [ctypes.c_uint64, ctypes.c_char_p,
                                     ctypes.POINTER(ctypes.c_uint64)]
        L.xrSuggestInteractionProfileBindings.restype = ctypes.c_int
        L.xrSuggestInteractionProfileBindings.argtypes = [
            ctypes.c_uint64,
            ctypes.POINTER(XrInteractionProfileSuggestedBinding)]
        L.xrAttachSessionActionSets.restype = ctypes.c_int
        L.xrAttachSessionActionSets.argtypes = [
            ctypes.c_uint64,
            ctypes.POINTER(XrSessionActionSetsAttachInfo)]
        L.xrSyncActions.restype = ctypes.c_int
        L.xrSyncActions.argtypes = [ctypes.c_uint64,
                                    ctypes.POINTER(XrActionsSyncInfo)]
        L.xrGetActionStatePose.restype = ctypes.c_int
        L.xrGetActionStatePose.argtypes = [ctypes.c_uint64,
                                           ctypes.POINTER(XrActionStateGetInfo),
                                           ctypes.POINTER(XrActionStatePose)]
        L.xrLocateSpace.restype = ctypes.c_int
        L.xrLocateSpace.argtypes = [ctypes.c_uint64, ctypes.c_uint64,
                                    ctypes.c_int64,
                                    ctypes.POINTER(XrSpaceLocation)]
        L.xrPollEvent.restype = ctypes.c_int
        L.xrPollEvent.argtypes = [ctypes.c_uint64,
                                  ctypes.POINTER(XrEventDataBuffer)]
        L.xrEnumerateInstanceExtensionProperties.restype = ctypes.c_int
        L.xrEnumerateInstanceExtensionProperties.argtypes = [
            ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_void_p]
        L.xrEnumerateInstanceExtensionProperties.restype = ctypes.c_int
        L.xrResultToString.restype = ctypes.c_int
        L.xrResultToString.argtypes = [ctypes.c_uint64, ctypes.c_int,
                                       ctypes.c_char_p]

    @staticmethod
    def _check(L, result, what):
        if result < 0:
            buf = ctypes.create_string_buffer(256)
            try:
                L.xrResultToString(0, result, buf)
                detail = buf.value.decode(errors="replace")
            except Exception:
                detail = ""
            hint = ""
            if what == "xrCreateInstance":
                hint = ("\n\n没有可用的 OpenXR 运行时。检查:"
                        "\n  - 是否设置了 XR_RUNTIME_JSON 指向 WiVRn/Monado"
                        "\n  - 运行时是否正在运行"
                        "\n  - 是否已有别的程序(比如 VRChat)占着会话")
            sys.exit(f"{what} failed: {result} {detail}{hint}")

    @classmethod
    def _ensure_started(cls, L):
        """Create the instance/system/session once, for all tracker objects."""
        if cls._shared is not None:
            return (cls._shared["instance"], cls._shared["system"],
                    cls._shared["session"], cls._shared["space"])

        ci = XrInstanceCreateInfo()
        ci.type = XR_TYPE_INSTANCE_CREATE_INFO
        ci.applicationInfo.applicationName = b"solarxr-feed"
        ci.applicationInfo.applicationVersion = 1
        ci.applicationInfo.engineName = b"solarxr-feed"
        ci.applicationInfo.engineVersion = 1
        ci.applicationInfo.apiVersion = (1 << 48)  # XR_VERSION_1_0

        # Headless lets us skip a graphics binding entirely.
        # Array of extension name pointers; must outlive the call.
        ext_names = (ctypes.c_char_p * 1)(b"XR_MND_headless")
        ci.enabledExtensionCount = 1
        ci.enabledExtensionNames = ctypes.cast(
            ext_names, ctypes.c_void_p).value

        instance = ctypes.c_uint64(0)
        cls._check(L, L.xrCreateInstance(ctypes.byref(ci), ctypes.byref(instance)),
                   "xrCreateInstance")

        sysinfo = XrSystemGetInfo()
        sysinfo.type = XR_TYPE_SYSTEM_GET_INFO
        sysinfo.formFactor = 1  # XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY
        system_id = ctypes.c_uint64(0)
        cls._check(L, L.xrGetSystem(instance.value, ctypes.byref(sysinfo),
                                    ctypes.byref(system_id)), "xrGetSystem")

        props = XrSystemProperties()
        props.type = XR_TYPE_SYSTEM_PROPERTIES
        if L.xrGetSystemProperties(instance.value, system_id.value,
                                   ctypes.byref(props)) == XR_SUCCESS:
            name = bytes(props.systemName).split(b"\x00")[0].decode(errors="replace")
            print(f"[feeder] OpenXR system: {name}", file=sys.stderr)

        sci = XrSessionCreateInfo()
        sci.type = XR_TYPE_SESSION_CREATE_INFO
        sci.next = None          # no graphics binding (headless)
        sci.systemId = system_id.value
        session = ctypes.c_uint64(0)
        cls._check(L, L.xrCreateSession(instance.value, ctypes.byref(sci),
                                        ctypes.byref(session)), "xrCreateSession")

        # Pump events until the session is ready; the runtime will not hand out
        # valid locations before that.
        import time
        deadline = time.time() + 10.0
        state = 0
        while time.time() < deadline:
            ev = XrEventDataBuffer()
            ev.type = XR_TYPE_EVENT_DATA_BUFFER
            while L.xrPollEvent(instance.value, ctypes.byref(ev)) == XR_SUCCESS:
                ev.type = XR_TYPE_EVENT_DATA_BUFFER
            # Session state is not directly readable here, so poll a space
            # instead: once it resolves, the session is usable.
            break

        # A stage space gives a floor-relative frame, matching what the OpenHMD
        # path produced (the room calibration is already floor-level).
        rci = XrReferenceSpaceCreateInfo()
        rci.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO
        rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE
        rci.poseInReferenceSpace.orientation.w = 1.0
        space = ctypes.c_uint64(0)
        cls._check(L, L.xrCreateReferenceSpace(session.value, ctypes.byref(rci),
                                               ctypes.byref(space)),
                   "xrCreateReferenceSpace")

        cls._shared = {"instance": instance.value, "system": system_id.value,
                       "session": session.value, "space": space, "paths": None}
        return (cls._shared["instance"], cls._shared["system"],
                cls._shared["session"], cls._shared["space"])

    def _enumerate_trackers(self):
        """Return the action paths that currently resolve to a tracked device."""
        return self._probe_paths()

    def _probe_paths(self):
        """Find which action paths the runtime reports as active.

        Every candidate action is created first: attaching the action set (done
        on the first sync) makes it immutable, so no action may be created
        afterwards.
        """
        L = self.L
        candidates = [
            "/user/hand/left/input/grip/pose",
            "/user/hand/right/input/grip/pose",
            "/user/hand/left/input/aim/pose",
            "/user/hand/right/input/aim/pose",
        ]
        actions = [(p, self._create_pose_action(p)) for p in candidates]

        self._attach_once()

        found = []
        for path, act in actions:
            st = XrActionStatePose()
            st.type = XR_TYPE_ACTION_STATE_POSE
            gi = XrActionStateGetInfo()
            gi.type = XR_TYPE_ACTION_STATE_GET_INFO
            gi.action = act
            self._sync_now()
            r = L.xrGetActionStatePose(self.session, ctypes.byref(gi),
                                       ctypes.byref(st))
            print(f"  probe {path}: result={r} isActive={st.isActive}",
                  file=sys.stderr)
            if r == XR_SUCCESS and st.isActive:
                found.append(path)
        if not found:
            sys.exit("the OpenXR runtime reported no active trackers")
        return found

    def _create_pose_action(self, path):
        L = self.L
        if "action_set" not in MonadoXR._shared:
            asi = XrActionSetCreateInfo()
            asi.type = XR_TYPE_ACTION_SET_CREATE_INFO
            asi.actionSetName = b"solarxr"
            asi.localizedActionSetName = b"solarxr"
            asi.priority = 0
            aset = ctypes.c_uint64(0)
            self._check(L, L.xrCreateActionSet(self.instance, ctypes.byref(asi),
                                               ctypes.byref(aset)),
                        "xrCreateActionSet")
            MonadoXR._shared["action_set"] = aset.value
            MonadoXR._shared["actions"] = []


        aset = MonadoXR._shared["action_set"]
        aci = XrActionCreateInfo()
        aci.type = XR_TYPE_ACTION_CREATE_INFO
        safe = path.replace("/", "_")
        aci.actionName = safe.encode()[:XR_MAX_ACTION_NAME_SIZE - 1]
        aci.localizedActionName = safe.encode()[:XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1]
        aci.actionType = XR_ACTION_TYPE_POSE_INPUT
        act = ctypes.c_uint64(0)
        self._check(L, L.xrCreateAction(aset, ctypes.byref(aci), ctypes.byref(act)),
                    "xrCreateAction")

        saci = XrActionSpaceCreateInfo()
        saci.type = XR_TYPE_ACTION_SPACE_CREATE_INFO
        saci.action = act.value
        saci.poseInActionSpace.orientation.w = 1.0
        space = ctypes.c_uint64(0)
        self._check(L, L.xrCreateActionSpace(self.session, ctypes.byref(saci),
                                             ctypes.byref(space)),
                    "xrCreateActionSpace")
        MonadoXR._shared["actions"].append(act.value)
        MonadoXR._shared.setdefault("action_paths", []).append((path, act.value))
        self.action_space = space.value
        return act.value

    def _to_path(self, text):
        out = ctypes.c_uint64(0)
        self._check(self.L, self.L.xrStringToPath(self.instance, text.encode(),
                                                  ctypes.byref(out)),
                    f"xrStringToPath({text})")
        return out.value

    def _suggest_bindings(self):
        """Tell the runtime how our actions map onto its devices.

        Without this the actions stay inactive: Monado will not guess which
        input a pose action wants. The Rift driver offers the tracked object
        through the same hand profiles as the controllers.
        """
        if MonadoXR._shared.get("suggested"):
            return
        L = self.L
        pairs = []
        # Only profiles this runtime actually enables; suggesting an
        # unsupported one is an error, not a no-op.
        for prof in ("/interaction_profiles/oculus/touch_controller",
                     "/interaction_profiles/khr/simple_controller"):
            try:
                prof_path = self._to_path(prof)
            except SystemExit:
                continue
            binds = []
            for path, act in MonadoXR._shared["action_paths"]:
                try:
                    binds.append((act, self._to_path(path)))
                except SystemExit:
                    continue
            if not binds:
                continue
            arr = (XrActionSuggestedBinding * len(binds))()
            for i, (act, b) in enumerate(binds):
                arr[i].action = act
                arr[i].binding = b
            info = XrInteractionProfileSuggestedBinding()
            info.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING
            info.interactionProfile = prof_path
            info.countSuggestedBindings = len(binds)
            info.suggestedBindings = arr
            L.xrSuggestInteractionProfileBindings(self.instance, ctypes.byref(info))
        MonadoXR._shared["suggested"] = True

    def _attach_once(self):
        """Attach the action set, which freezes it. Must happen after every
        action has been created, and before any sync or query."""
        if MonadoXR._shared.get("attached"):
            return
        L = self.L
        self._suggest_bindings()
        attach = XrSessionActionSetsAttachInfo()
        attach.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO
        attach.countActionSets = 1
        one = (ctypes.c_uint64 * 1)(MonadoXR._shared["action_set"])
        attach.actionSets = one
        self._check(L, L.xrAttachSessionActionSets(self.session, ctypes.byref(attach)),
                    "xrAttachSessionActionSets")
        MonadoXR._shared["attached"] = True

    def _sync(self, for_action):
        self._attach_once()
        self._sync_now()

    def _sync_now(self):
        L = self.L
        arr = (XrActiveActionSet * 1)()
        arr[0].actionSet = MonadoXR._shared["action_set"]
        si = XrActionsSyncInfo()
        si.type = XR_TYPE_ACTIONS_SYNC_INFO
        si.countActiveActionSets = 1
        si.activeActionSets = arr
        L.xrSyncActions(self.session, ctypes.byref(si))

    # ----------------------------------------------------------- data access

    def poll(self):
        """Service the runtime. Call once per frame, not per device."""
        ev = XrEventDataBuffer()
        ev.type = XR_TYPE_EVENT_DATA_BUFFER
        while self.L.xrPollEvent(self.instance, ctypes.byref(ev)) == XR_SUCCESS:
            ev.type = XR_TYPE_EVENT_DATA_BUFFER

    def update(self):
        """Return (pos, quat, vel, angvel, tracked) for this device."""
        L = self.L
        self._sync(self.action)

        st = XrActionStatePose()
        st.type = XR_TYPE_ACTION_STATE_POSE
        gi = XrActionStateGetInfo()
        gi.type = XR_TYPE_ACTION_STATE_GET_INFO
        gi.action = self.action
        L.xrGetActionStatePose(self.session, ctypes.byref(gi), ctypes.byref(st))
        if not st.isActive:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], \
                   [0.0] * 3, [0.0] * 3, False

        vel = XrSpaceVelocity()
        vel.type = XR_TYPE_SPACE_VELOCITY
        loc = XrSpaceLocation()
        loc.type = XR_TYPE_SPACE_LOCATION
        loc.next = ctypes.cast(ctypes.byref(vel), ctypes.c_void_p)
        L.xrLocateSpace(self.action_space, MonadoXR._shared["space"], 0,
                        ctypes.byref(loc))

        flags = loc.locationFlags
        tracked = bool(flags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)

        p = loc.pose.position
        o = loc.pose.orientation
        pos = [p.x, p.y, p.z]
        quat = [o.x, o.y, o.z, o.w]
        linear = [vel.linearVelocity.x, vel.linearVelocity.y, vel.linearVelocity.z]
        angular = [vel.angularVelocity.x, vel.angularVelocity.y,
                   vel.angularVelocity.z]
        return pos, quat, linear, angular, tracked


if __name__ == "__main__":
    import time
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    d = MonadoXR(n)
    print(f"device {n}: {d.name} ({d.path})")
    for _ in range(200):
        d.poll()
        pos, quat, lin, ang, trk = d.update()
        print(f"  pos {pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}  "
              f"quat {quat[0]:+.3f} {quat[1]:+.3f} {quat[2]:+.3f} {quat[3]:+.3f}  "
              f"{'TRACKED' if trk else 'no pose'}")
        time.sleep(0.2)
