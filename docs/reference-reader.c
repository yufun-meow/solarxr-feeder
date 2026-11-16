/*
 * cv1-fbt-source: read three trackers from the Oculus runtime and send them
 * over the LAN.
 *
 * The CV1 lives on a Windows box, where the official runtime drives its
 * sensors and radio. This program asks that runtime for the poses of the two
 * Touch controllers (worn on the feet) and the tracked object (worn on the
 * waist), and broadcasts them as one UDP datagram per frame. A receiver on
 * the Linux side feeds them to WiVRn as SolarXR trackers.
 *
 * Build (from a Visual Studio developer prompt):
 *     cl /O2 cv1-fbt-source.c /link LibOVRRT64_1.lib
 *
 * Build (MinGW):
 *     x86_64-w64-mingw32-gcc -O2 -o cv1-fbt-source.exe cv1-fbt-source.c \
 *         -L. -lLibOVRRT64_1
 *
 * The SDK's import library is not needed if you would rather load the DLL at
 * run time; see the LoadLibrary path in main().
 *
 * Usage:
 *     cv1-fbt-source.exe [--dest 192.168.1.50] [--port 39570] [--hz 240] [--quiet]
 *
 * With no --dest the datagrams go to 255.255.255.255, which every machine on
 * the same subnet receives.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* The slice of the Oculus ABI this needs, declared by hand so the file */
/* builds without the SDK headers.                                      */
/* ------------------------------------------------------------------ */

typedef int ovrResult;

typedef struct ovrSessionStruct *ovrSession;

typedef struct {
	float x, y, z;
} ovrVector3f;

typedef struct {
	float x, y, z, w;
} ovrQuatf;

typedef struct {
	ovrQuatf Orientation;
	ovrVector3f Position;
} ovrPosef;

/* 88 bytes on x86-64: the pose, four vectors, then a 4-byte pad and a
 * double. Anything reading only ThePose only needs the first 28. */
typedef struct {
	ovrPosef ThePose;
	ovrVector3f AngularVelocity;
	ovrVector3f LinearVelocity;
	ovrVector3f AngularAcceleration;
	ovrVector3f LinearAcceleration;
	char pad0[4];
	double TimeInSeconds;
} ovrPoseStatef;

typedef enum {
	ovrTrackedDevice_None = 0,
	ovrTrackedDevice_HMD = 1,
	ovrTrackedDevice_LTouch = 2,
	ovrTrackedDevice_RTouch = 4,
	ovrTrackedDevice_Touch = 6,
	ovrTrackedDevice_Object0 = 16,
	ovrTrackedDevice_Object1 = 32,
	ovrTrackedDevice_Object2 = 64,
	ovrTrackedDevice_Object3 = 128,
	ovrTrackedDevice_All = 65535,
} ovrTrackedDeviceType;

typedef struct {
	unsigned char Reserved[16];
} ovrGraphicsLuid;

typedef struct {
	unsigned int Flags;
	unsigned int RequestedMinorVersion;
	void *LogCallback;
	unsigned long long UserData;
	unsigned int ConnectionTimeoutMS;
	unsigned int pad0;
} ovrInitParams;

/* ovrStatusBits, from the Oculus SDK. PositionValid is the one that tells a
 * real position from the zero the runtime reports when the cameras lose the
 * device: the IMU keeps OrientationValid set and TimeInSeconds running
 * throughout, so neither of those can be used for this. */
#define OVR_STATUS_ORIENTATION_TRACKED 0x1
#define OVR_STATUS_POSITION_TRACKED 0x2
#define OVR_STATUS_ORIENTATION_VALID 0x4
#define OVR_STATUS_POSITION_VALID 0x8

/* Flags for ovrInitParams.Flags */
#define ovrInit_RequestVersion 0x00000004u

/* Session status bits, as ovrSessionStatus. */
typedef struct {
	char IsVisible;
	char HmdPresent;
	char HmdMounted;
	char DisplayLost;
	char ShouldQuit;
	char ShouldRecenter;
	char HasInputFocus;
	char OverlayPresent;
} ovrSessionStatus;

typedef ovrResult (*fn_Initialize)(const ovrInitParams *);
typedef ovrResult (*fn_Create)(ovrSession *, ovrGraphicsLuid *);
typedef void (*fn_Destroy)(ovrSession);
typedef ovrResult (*fn_GetSessionStatus)(ovrSession, ovrSessionStatus *);
typedef double (*fn_GetTimeInSeconds)(void);
typedef ovrResult (*fn_GetDevicePoses)(ovrSession, ovrTrackedDeviceType *, int, double, ovrPoseStatef *);

/* 312 bytes. Only the head and the two hands are here: the tracked object is
 * not part of this struct and has to be asked for separately. */
typedef struct {
	ovrPoseStatef HeadPose;
	unsigned int StatusFlags;
	char pad0[4];
	ovrPoseStatef HandPoses[2];
	unsigned int HandStatusFlags[2];
	char pad1[4];
	ovrPosef CalibratedOrigin;
} ovrTrackingState;

/* ovrBool is a single byte in the SDK, not an int: getting this wrong shifts
 * every parameter after it. */
typedef char ovrBool;

typedef ovrTrackingState (*fn_GetTrackingState)(ovrSession, double, ovrBool);

/* ------------------------------------------------------------------ */
/* Wire format, sent one datagram per frame.                           */
/* ------------------------------------------------------------------ */

#define CV1FBT_MAGIC 0x54424631u /* "1FBT" */
#define CV1FBT_VERSION 1

/* A pose is absent rather than zero: the runtime zeroes the quaternion to
 * [0,0,0,1] when it has nothing, which is also a legal orientation, so it
 * cannot be told apart from a real one. */
#define CV1FBT_VALID 0x1

typedef struct {
	/* Normalised orientation quaternion, then position in metres, in the
	 * runtime's tracking frame. */
	float qx, qy, qz, qw;
	float px, py, pz;
} cv1fbt_pose;

typedef struct {
	unsigned int magic;
	unsigned char version;
	unsigned char flags;
	unsigned short reserved;

	double time_seconds; /* ovr_GetTimeInSeconds at sampling */

	/* The order is fixed: left foot, right foot, waist. */
	cv1fbt_pose left_foot;
	cv1fbt_pose right_foot;
	cv1fbt_pose waist;

	/* valid: this pose has been seen at some point and holds a real place.
	 * visible: the device is being seen right now. The pose is still sent
	 * when visible is clear - it is the last place it was seen - so a device
	 * that goes out of view holds still rather than vanishing. */
	unsigned short valid;
	unsigned short visible;
} cv1fbt_packet;

/* ------------------------------------------------------------------ */

static volatile LONG g_running = 1;

static BOOL WINAPI
on_ctrl_c(DWORD type)
{
	(void)type;
	InterlockedExchange(&g_running, 0);
	return TRUE;
}

static ovrPoseStatef
normalise(ovrPoseStatef s)
{
	/* A zeroed quaternion means "no pose": the runtime's documented way of
	 * saying it could not produce one. Give it w = 1 so downstream maths
	 * stays finite, and let the caller decide with the valid bits. */
	const float len2 = s.ThePose.Orientation.x * s.ThePose.Orientation.x +
	                   s.ThePose.Orientation.y * s.ThePose.Orientation.y +
	                   s.ThePose.Orientation.z * s.ThePose.Orientation.z +
	                   s.ThePose.Orientation.w * s.ThePose.Orientation.w;
	if (len2 < 1e-8f) {
		s.ThePose.Orientation.x = 0.0f;
		s.ThePose.Orientation.y = 0.0f;
		s.ThePose.Orientation.z = 0.0f;
		s.ThePose.Orientation.w = 1.0f;
		return s;
	}
	const float inv = 1.0f / sqrtf(len2);
	s.ThePose.Orientation.x *= inv;
	s.ThePose.Orientation.y *= inv;
	s.ThePose.Orientation.z *= inv;
	s.ThePose.Orientation.w *= inv;
	return s;
}

static int
pose_is_present(const ovrPoseStatef *s, unsigned int status_flags)
{
	/*
	 * A pose is only worth keeping if it has a position. Three things make it
	 * useless:
	 *
	 *  - TimeInSeconds at zero: the runtime has never had a pose for it.
	 *  - PositionTracked clear: the cameras cannot see it.
	 *  - A position of exactly (0,0,0): what the runtime reports while a
	 *    controller is out of view, and not a place a worn tracker sits.
	 *
	 * Any of them means "no new position", so the last real one is held
	 * rather than overwritten with a zero that would drag the tracker to the
	 * origin.
	 */
	if (s->TimeInSeconds == 0.0) {
		return 0;
	}
	if ((status_flags & OVR_STATUS_POSITION_TRACKED) == 0) {
		return 0;
	}
	const float px = s->ThePose.Position.x;
	const float py = s->ThePose.Position.y;
	const float pz = s->ThePose.Position.z;
	if (px == 0.0f && py == 0.0f && pz == 0.0f) {
		return 0;
	}
	return 1;
}

static void
fill_pose(cv1fbt_pose *out, const ovrPoseStatef *s)
{
	out->qx = s->ThePose.Orientation.x;
	out->qy = s->ThePose.Orientation.y;
	out->qz = s->ThePose.Orientation.z;
	out->qw = s->ThePose.Orientation.w;
	out->px = s->ThePose.Position.x;
	out->py = s->ThePose.Position.y;
	out->pz = s->ThePose.Position.z;
}

static void
usage(const char *argv0)
{
	printf("usage: %s [--dest A.B.C.D] [--port N] [--hz N] [--quiet]\n", argv0);
	printf("  --dest   receiver address, default 255.255.255.255 (broadcast)\n");
	printf("  --port   UDP port, default 39570\n");
	printf("  --hz     datagrams per second, default 240\n");
	printf("  --quiet  only print changes in which trackers are present\n");
}

int
main(int argc, char **argv)
{
	const char *dest = "255.255.255.255";
	int port = 39570;
	int hz = 240;
	int quiet = 0;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--dest") == 0 && i + 1 < argc) {
			dest = argv[++i];
		} else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
			port = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc) {
			hz = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--quiet") == 0) {
			quiet = 1;
		} else {
			usage(argv[0]);
			return 2;
		}
	}
	if (hz < 1 || hz > 1000 || port < 1 || port > 65535) {
		usage(argv[0]);
		return 2;
	}

	/* --- the runtime --- */

	HMODULE lib = LoadLibraryA("LibOVRRT64_1.dll");
	if (lib == NULL) {
		printf("error: could not load LibOVRRT64_1.dll (%lu).\n", GetLastError());
		printf("       Is the Oculus runtime installed?\n");
		return 1;
	}

	fn_Initialize ovr_Initialize = (fn_Initialize)GetProcAddress(lib, "ovr_Initialize");
	fn_Create ovr_Create = (fn_Create)GetProcAddress(lib, "ovr_Create");
	fn_Destroy ovr_Destroy = (fn_Destroy)GetProcAddress(lib, "ovr_Destroy");
	fn_GetTimeInSeconds ovr_GetTimeInSeconds =
	    (fn_GetTimeInSeconds)GetProcAddress(lib, "ovr_GetTimeInSeconds");
	fn_GetSessionStatus ovr_GetSessionStatus =
	    (fn_GetSessionStatus)GetProcAddress(lib, "ovr_GetSessionStatus");
	fn_GetDevicePoses ovr_GetDevicePoses = (fn_GetDevicePoses)GetProcAddress(lib, "ovr_GetDevicePoses");
	fn_GetTrackingState ovr_GetTrackingState =
	    (fn_GetTrackingState)GetProcAddress(lib, "ovr_GetTrackingState");

	if (!ovr_Initialize || !ovr_Create || !ovr_GetTimeInSeconds || !ovr_GetDevicePoses ||
	    !ovr_GetTrackingState) {
		printf("error: the runtime is missing functions this needs.\n");
		return 1;
	}

	ovrInitParams params;
	memset(&params, 0, sizeof(params));
	params.Flags = ovrInit_RequestVersion;
	params.RequestedMinorVersion = 205;

	int r = ovr_Initialize(&params);
	if (r < 0) {
		printf("error: ovr_Initialize failed (%d).\n", r);
		return 1;
	}

	ovrSession session = NULL;
	ovrGraphicsLuid luid;
	memset(&luid, 0, sizeof(luid));
	r = ovr_Create(&session, &luid);
	if (r < 0) {
		printf("error: ovr_Create failed (%d).\n", r);
		printf("       Is the Oculus service running and the headset connected?\n");
		return 1;
	}
	printf("Oculus session open.\n");

	/* --- the socket --- */

	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		printf("error: WSAStartup failed.\n");
		return 1;
	}

	SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET) {
		printf("error: socket() failed (%d).\n", WSAGetLastError());
		return 1;
	}
	BOOL on = TRUE;
	setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char *)&on, sizeof(on));

	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons((unsigned short)port);
	if (InetPtonA(AF_INET, dest, &to.sin_addr) != 1) {
		printf("error: '%s' is not an IPv4 address.\n", dest);
		return 1;
	}

	printf("Sending to %s:%d at %d Hz.\n", dest, port, hz);
	printf("Trackers: left foot (Touch), right foot (Touch), waist (object).\n");
	printf("Press Ctrl-C to stop.\n\n");

	SetConsoleCtrlHandler(on_ctrl_c, TRUE);

	const double period = 1.0 / (double)hz;
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);

	/*
	 * The last good pose per device. A device that goes out of view keeps its
	 * last known place rather than snapping to the origin: the origin is a
	 * real point in the room, and a tracker that teleports there every time a
	 * camera loses sight of it is worse than one that simply stops moving.
	 */
	ovrPoseStatef last_good[3];
	memset(last_good, 0, sizeof(last_good));
	int have_good[3] = {0, 0, 0};

	unsigned long long sent = 0;
	unsigned long long failed = 0;
	int last_present = -1;
	int verbose_once = 1;
	int verbose_done = 0;
	const double loop_start = ovr_GetTimeInSeconds();
	double next_report = loop_start + 1.0;

	while (InterlockedCompareExchange(&g_running, 1, 1) == 1) {
		LARGE_INTEGER begin;
		QueryPerformanceCounter(&begin);

		ovrSessionStatus status;
		memset(&status, 0, sizeof(status));
		if (ovr_GetSessionStatus) {
			ovr_GetSessionStatus(session, &status);
			if (status.ShouldQuit) {
				printf("The runtime asked this to quit.\n");
				break;
			}
		}

		/* Zero asks for the most recent known pose rather than a prediction. */
		const double now = ovr_GetTimeInSeconds();

		/*
		 * The hands come from the tracking state; the tracked object is not in
		 * it and has to be asked for on its own, one device per call - that is
		 * the shape the Oculus runtime is known to answer.
		 */
		ovrTrackingState state;
		memset(&state, 0, sizeof(state));
		state = ovr_GetTrackingState(session, 0.0, 1);

		if (verbose_once && !verbose_done) {
			printf("\n--- raw tracking state ---\n");
			printf("sizeof(ovrTrackingState)=%d sizeof(ovrPoseStatef)=%d\n", (int)sizeof(ovrTrackingState),
			       (int)sizeof(ovrPoseStatef));
			printf("head  pos %.3f %.3f %.3f  q %.3f %.3f %.3f %.3f  t %.3f\n",
			       state.HeadPose.ThePose.Position.x, state.HeadPose.ThePose.Position.y,
			       state.HeadPose.ThePose.Position.z, state.HeadPose.ThePose.Orientation.x,
			       state.HeadPose.ThePose.Orientation.y, state.HeadPose.ThePose.Orientation.z,
			       state.HeadPose.ThePose.Orientation.w, state.HeadPose.TimeInSeconds);
			printf("head  StatusFlags=0x%x\n", state.StatusFlags);
			for (int h = 0; h < 2; h++) {
				printf("hand%d pos %.3f %.3f %.3f  q %.3f %.3f %.3f %.3f  t %.3f  flags=0x%x\n"
				       "      dT=%.4f\n",
				       h, state.HandPoses[h].ThePose.Position.x, state.HandPoses[h].ThePose.Position.y,
				       state.HandPoses[h].ThePose.Position.z,
				       state.HandPoses[h].ThePose.Orientation.x,
				       state.HandPoses[h].ThePose.Orientation.y,
				       state.HandPoses[h].ThePose.Orientation.z,
				       state.HandPoses[h].ThePose.Orientation.w,
				       state.HandPoses[h].TimeInSeconds, state.HandStatusFlags[h],
				       state.HandPoses[h].LinearAcceleration.x);
			}
			printf("--- end ---\n\n");
			verbose_done = 1;
		}

		ovrPoseStatef poses[3];
		memset(poses, 0, sizeof(poses));
		poses[0] = state.HandPoses[0]; /* ovrHand_Left == 0 */
		poses[1] = state.HandPoses[1]; /* ovrHand_Right == 1 */

		ovrTrackedDeviceType object_type = ovrTrackedDevice_Object0;
		memset(&poses[2], 0, sizeof(poses[2]));

		/*
		 * The tracked object is asked for its pose separately from the hands,
		 * and the time it is asked about matters. Passing 0 is documented as
		 * "most recent", but for this device it comes back as
		 * ovrError_LostTracking (-1018) however well the object is being
		 * tracked. The current absolute time works, the same way xrLocateSpace
		 * rejects a zero time on the other side of this setup.
		 */
		r = ovr_GetDevicePoses(session, &object_type, 1, now, &poses[2]);
		if (r < 0) {
			/* Some runtimes only accept the literal 0; try that too before
			 * giving up for this frame. */
			memset(&poses[2], 0, sizeof(poses[2]));
			r = ovr_GetDevicePoses(session, &object_type, 1, 0.0, &poses[2]);
		}

		if (verbose_once && verbose_done) {
			printf("object r=%d pos %.3f %.3f %.3f  q %.3f %.3f %.3f %.3f  t %.3f\n\n", r,
			       poses[2].ThePose.Position.x, poses[2].ThePose.Position.y,
			       poses[2].ThePose.Position.z, poses[2].ThePose.Orientation.x,
			       poses[2].ThePose.Orientation.y, poses[2].ThePose.Orientation.z,
			       poses[2].ThePose.Orientation.w, poses[2].TimeInSeconds);
			verbose_once = 0;
		}

		cv1fbt_packet packet;
		memset(&packet, 0, sizeof(packet));
		packet.magic = CV1FBT_MAGIC;
		packet.version = CV1FBT_VERSION;
		packet.time_seconds = now;

		/*
		 * Each device is judged on its own. r belongs to the single
		 * ovr_GetDevicePoses call for the tracked object, so folding it into
		 * this loop let one failing device - the object reports LostTracking
		 * while it is out of view - mark the hands invalid too, and a foot
		 * that was tracking perfectly well arrived at the receiver as absent.
		 */
		/*
		 * Each device is judged on its own. r belongs to the single
		 * ovr_GetDevicePoses call for the tracked object, so folding it into
		 * this loop let one failing device - the object reports LostTracking
		 * while it is out of view - mark the hands invalid too.
		 */
		int present = 0;
		int seen_now = 0;
		for (int i = 0; i < 3; i++) {
			ovrPoseStatef p = normalise(poses[i]);
			/* The hands carry their status in HandStatusFlags; the tracked
			 * object's pose comes from its own call, and r already says
			 * whether that call produced anything. */
			int ok;
			if (i < 2) {
				ok = pose_is_present(&p, state.HandStatusFlags[i]);
			} else {
				/*
				 * The tracked object has no status word of its own: the only
				 * thing the runtime reports is the result of its own
				 * ovr_GetDevicePoses call, and that comes back as
				 * ovrError_LostTracking (-1018) while the object is tracked
				 * perfectly well - its pose is fresh and moving. Judge the
				 * object by the pose instead; a zero time or a zero position
				 * still means it is not there.
				 */
				ok = (p.TimeInSeconds != 0.0) &&
				     !(p.ThePose.Position.x == 0.0f && p.ThePose.Position.y == 0.0f &&
				       p.ThePose.Position.z == 0.0f);
			}

			if (ok) {
				seen_now |= 1 << i;
				packet.visible |= (unsigned short)(1 << i);
			}

			if (ok) {
				last_good[i] = p;
				have_good[i] = 1;
			}

			/*
			 * Send the last place this device was seen, and say whether it is
			 * being seen now. valid means "this device exists and has a place
			 * to be shown at"; visible means "the cameras can see it right
			 * now".
			 *
			 * valid is set even before the first good pose, so a controller
			 * that was behind something when the program started still shows
			 * up (at the origin) and starts moving the moment it is seen,
			 * rather than being absent from the stream until then.
			 */
			present |= 1 << i;
			packet.valid |= (unsigned short)(CV1FBT_VALID << i);
			const ovrPoseStatef *out = have_good[i] ? &last_good[i] : &p;

			if (i == 0) {
				fill_pose(&packet.left_foot, out);
			} else if (i == 1) {
				fill_pose(&packet.right_foot, out);
			} else {
				fill_pose(&packet.waist, out);
			}
		}

		int sent_ok = 1;
		if (sendto(sock, (const char *)&packet, sizeof(packet), 0, (struct sockaddr *)&to, sizeof(to)) ==
		    SOCKET_ERROR) {
			sent_ok = 0;
			failed++;
		} else {
			sent++;
		}

		if (seen_now != last_present) {
			printf("now seen: left foot %s, right foot %s, waist %s%s   "
			       "(held: %s%s%s)\n",
			       (seen_now & 1) ? "yes" : "no", (seen_now & 2) ? "yes" : "no",
			       (seen_now & 4) ? "yes" : "no", sent_ok ? "" : "  (send failed)",
			       (present & 1) ? "L" : "-", (present & 2) ? "R" : "-",
			       (present & 4) ? "W" : "-");
			last_present = seen_now;
		}

		if (!quiet) {
			if (now >= next_report) {
				printf("\rsent %llu packets, %llu failed, seen %s%s%s   ", sent, failed,
				       (seen_now & 1) ? "L" : "-", (seen_now & 2) ? "R" : "-",
				       (seen_now & 4) ? "W" : "-");
				fflush(stdout);
				next_report = now + 1.0;
			}
		}

		/* Pace the loop. ovr_GetDevicePoses returns the pose as of now, and
		 * running at the runtime's own rate avoids repeating it. */
		LARGE_INTEGER end;
		QueryPerformanceCounter(&end);
		const double spent = (double)(end.QuadPart - begin.QuadPart) / (double)freq.QuadPart;
		if (spent < period) {
			Sleep((DWORD)((period - spent) * 1000.0));
		}
	}

	printf("\n");

	closesocket(sock);
	WSACleanup();

	if (ovr_Destroy) {
		ovr_Destroy(session);
	}

	printf("Stopped after %llu packets (%llu failed).\n", sent, failed);
	return 0;
}
