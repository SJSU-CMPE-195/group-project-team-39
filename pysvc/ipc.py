"""
ipc.py - Shared-memory publisher for Python (vision) -> C++ (gantry).

Wire format (64 bytes, packed, little-endian):

    offset  size  type     field
    ------  ----  -------  ---------------
       0      4   uint32   seq           (seq-lock; odd = writing, even = stable)
       4      4   uint32   ready         (1 when payload is valid)
       8      4   uint32   request_next  (C++ sets to 1 when it wants new data)
      12      4   uint32   kind          (1=DEFEND, 2=IGNORE, 3=NO_TRACK)
      16      8   uint64   timestamp_ns
      24      8   float64  x_mm          (mallet-relative)
      32      8   float64  y_mm
      40      8   float64  vx_mm_s
      48      8   float64  vy_mm_s
      56      4   float32  confidence    (0..1)
      60      4   uint32   version

UNITS: millimeters and millimeters/second on the wire.  This matches the
gantry firmware's own internal units, so the C++ side reads x_mm and uses
it directly with no scaling.

The C++ side must use `#pragma pack(push, 1)` on a struct with the same
layout and call `shm_open("/puck_xy_mm", ...)` followed by `mmap`.

On Linux this uses posix_ipc.  If posix_ipc isn't installed (e.g. you're
prototyping on macOS), the Publisher silently becomes a no-op so your
detection pipeline still runs.
"""

import os
import struct
import time

# ----------------------------------------------------------------------------
# Wire format constants - keep in sync with C++ struct shm_xy
# ----------------------------------------------------------------------------
SHM_NAME      = "/puck_xy_mm"
SHM_SIZE      = 64
WIRE_VERSION  = 2          # bumped from 1 (cm) to 2 (mm) - mismatched runs fail loudly

# Kind enum
KIND_NONE     = 0
KIND_DEFEND   = 1   # puck inside critical zone -> go intercept
KIND_IGNORE   = 2   # puck on opponent half / outside zone -> go to rest
KIND_NO_TRACK = 3   # tracking lost -> go to rest

# Field offsets
_OFF_SEQ          = 0
_OFF_READY        = 4
_OFF_REQUEST_NEXT = 8
_OFF_KIND         = 12
_OFF_TIMESTAMP    = 16
_OFF_X            = 24
_OFF_Y            = 32
_OFF_VX           = 40
_OFF_VY           = 48
_OFF_CONFIDENCE   = 56
_OFF_VERSION      = 60

# Struct packers
_U32 = struct.Struct("<I")
_U64 = struct.Struct("<Q")
_F32 = struct.Struct("<f")
_F64 = struct.Struct("<d")


class Publisher:
    """
    POSIX shared memory publisher.

    Usage:
        pub = Publisher()
        pub.publish(KIND_DEFEND, x_mm=123.0, y_mm=-56.0, vx_mm_s=800.0, vy_mm_s=1200.0)
        pub.close()      # on shutdown
    """

    def __init__(self, name=SHM_NAME):
        self._name      = name
        self._seq       = 0
        self._mm        = None
        self._shm       = None
        self._enabled   = False
        self._posix_ipc = None

        try:
            import posix_ipc
            import mmap
        except ImportError:
            print(f"[ipc] posix_ipc not installed -- publisher disabled "
                  f"(detection still runs). pip install posix_ipc to enable.",
                  flush=True)
            return

        self._posix_ipc = posix_ipc

        # Create or attach to the segment
        try:
            self._shm = posix_ipc.SharedMemory(
                name, posix_ipc.O_CREX, size=SHM_SIZE, mode=0o666)
        except posix_ipc.ExistentialError:
            # Already exists from a previous run -- attach and reuse
            self._shm = posix_ipc.SharedMemory(name)

        # Force world read+write so other containers (running as different
        # users) can open the segment with O_RDWR.
        os.chmod(f'/dev/shm{name}', 0o666)

        self._mm = mmap.mmap(self._shm.fd, self._shm.size)
        self._shm.close_fd()  # mmap keeps the mapping alive without the fd

        # Zero everything, then prime: request_next=1 so C++ doesn't wait
        # forever for "fresh" data, and stamp the wire version.
        self._mm[:] = b"\x00" * SHM_SIZE
        _U32.pack_into(self._mm, _OFF_REQUEST_NEXT, 1)
        _U32.pack_into(self._mm, _OFF_VERSION, WIRE_VERSION)

        self._enabled = True
        print(f"[ipc] publishing on /dev/shm{name} ({SHM_SIZE} bytes, "
              f"wire v{WIRE_VERSION}, units=mm)", flush=True)

    @property
    def enabled(self):
        return self._enabled

    def cxx_requested_next(self):
        """True if C++ has set request_next=1 since the last publish."""
        if not self._enabled:
            return False
        return _U32.unpack_from(self._mm, _OFF_REQUEST_NEXT)[0] == 1

    def publish(self, kind, x_mm=0.0, y_mm=0.0,
                vx_mm_s=0.0, vy_mm_s=0.0, confidence=1.0):
        """Write one record using a seq-lock so C++ can read tear-free.

        All distance fields are millimeters; velocity fields are mm/sec.
        """
        if not self._enabled:
            return

        # Step seq to the next odd number => "write in progress"
        self._seq += 1
        if self._seq % 2 == 0:
            self._seq += 1
        _U32.pack_into(self._mm, _OFF_SEQ, self._seq)

        # Payload
        _U32.pack_into(self._mm, _OFF_KIND,         int(kind))
        _U64.pack_into(self._mm, _OFF_TIMESTAMP,    time.monotonic_ns())
        _F64.pack_into(self._mm, _OFF_X,            float(x_mm))
        _F64.pack_into(self._mm, _OFF_Y,            float(y_mm))
        _F64.pack_into(self._mm, _OFF_VX,           float(vx_mm_s))
        _F64.pack_into(self._mm, _OFF_VY,           float(vy_mm_s))
        _F32.pack_into(self._mm, _OFF_CONFIDENCE,   float(confidence))
        _U32.pack_into(self._mm, _OFF_READY,        1)
        _U32.pack_into(self._mm, _OFF_REQUEST_NEXT, 0)  # consume the request

        # Step seq to the next even number => "stable"
        self._seq += 1
        _U32.pack_into(self._mm, _OFF_SEQ, self._seq)

    def close(self, unlink=True):
        """Close the mapping and (optionally) remove the /dev/shm file."""
        if not self._enabled:
            return
        try:
            if self._mm is not None:
                self._mm.close()
        except Exception:
            pass
        try:
            if unlink and self._posix_ipc is not None:
                self._posix_ipc.unlink_shared_memory(self._name)
        except Exception:
            pass
        self._enabled = False