"""
ipc_handler.py  --  POSIX shared memory + semaphore IPC with cppsvc.

Shared memory layout (32 bytes, little-endian, no implicit padding):

  Offset  Field           Type    Description
  ──────  ─────────────── ──────  ──────────────────────────────────────────
   0      target_x_mm     float   Gantry target X (pysvc -> cppsvc)
   4      target_y_mm     float   Gantry target Y
   8      command         uint8   IDLE/INTERCEPT/PUSH_NORTH/PUSH_OUT
   9      priority        uint8   LOW/MEDIUM/HIGH
  10      _pad1           uint16  (alignment pad)
  12      command_seq     uint32  Incremented per new command
  16      current_x_mm    float   Current mallet X (cppsvc -> pysvc)
  20      current_y_mm    float   Current mallet Y
  24      state           uint8   IDLE/MOVING/HOMING/ERROR
  25      ready           uint8   1 = gantry ready for next command
  26      _pad2           uint16  (alignment pad)
  28      status_seq      uint32  Incremented per status update
  ──────────────────────────────────────────────────────────────────────────
                          Total   32 bytes

Semaphores:
    /airhockey_mutex  -- binary (init=1): protects shm reads/writes
    /airhockey_cmd    -- counting (init=0): signals cppsvc that a command is ready
"""

from __future__ import annotations

import struct
import mmap
from typing import Optional

import posix_ipc

# ── IPC object names ────────────────────────────────────────────────────────────
SHM_NAME   = "/airhockey_shm"
MUTEX_NAME = "/airhockey_mutex"
CMD_NAME   = "/airhockey_cmd"

# ── Struct layout ───────────────────────────────────────────────────────────────
# "=" = standard (native) byte order, no alignment padding
# f f B B H I f f B B H I
IPC_FORMAT = "=ffBBHIffBBHI"
IPC_SIZE   = struct.calcsize(IPC_FORMAT)  # 32

# ── Command values ──────────────────────────────────────────────────────────────
IDLE       = 0
INTERCEPT  = 1
PUSH_NORTH = 2
PUSH_OUT   = 3

# ── Priority values ─────────────────────────────────────────────────────────────
LOW    = 0
MEDIUM = 1
HIGH   = 2

# ── State values ────────────────────────────────────────────────────────────────
STATE_IDLE = 0
MOVING     = 1
HOMING     = 2
ERROR      = 3

_PRIORITY_MAP: dict[int, int] = {
    INTERCEPT:  HIGH,
    PUSH_NORTH: MEDIUM,
    PUSH_OUT:   MEDIUM,
    IDLE:       LOW,
}


class GantryStatus:
    __slots__ = ("current_x_mm", "current_y_mm", "state", "ready", "status_seq")

    def __init__(
        self,
        current_x_mm: float,
        current_y_mm: float,
        state: int,
        ready: int,
        status_seq: int,
    ) -> None:
        self.current_x_mm = current_x_mm
        self.current_y_mm = current_y_mm
        self.state        = state
        self.ready        = ready
        self.status_seq   = status_seq


class IPCHandler:
    def __init__(self) -> None:
        # Create or open the shared memory region.  mode=0o666 must be
        # explicit -- posix_ipc defaults to 0o600, which locks the C++ side
        # out when the two services run as different uids.
        self._shm = posix_ipc.SharedMemory(
            SHM_NAME,
            posix_ipc.O_CREAT,
            mode=0o666,
            size=IPC_SIZE,
        )
        self._mm = mmap.mmap(self._shm.fd, IPC_SIZE)
        self._shm.close_fd()

        # Create or open semaphores (same 0o666 reasoning as above).
        self._mutex = posix_ipc.Semaphore(
            MUTEX_NAME, posix_ipc.O_CREAT, mode=0o666, initial_value=1
        )
        self._cmd = posix_ipc.Semaphore(
            CMD_NAME, posix_ipc.O_CREAT, mode=0o666, initial_value=0
        )

        self._command_seq: int = 0

    # ── Public API ──────────────────────────────────────────────────────────────

    def send_command(
        self,
        command: int,
        target_x_mm: float,
        target_y_mm: float,
        priority: Optional[int] = None,
    ) -> None:
        """Write a command to shared memory and signal cppsvc.

        Priority is inferred from the command type if not supplied.
        """
        if priority is None:
            priority = _PRIORITY_MAP.get(command, LOW)

        self._command_seq += 1

        self._mutex.acquire()
        try:
            fields = list(self._read_raw())
            fields[0] = float(target_x_mm)
            fields[1] = float(target_y_mm)
            fields[2] = int(command)
            fields[3] = int(priority)
            fields[4] = 0                    # _pad1
            fields[5] = self._command_seq
            self._mm.seek(0)
            self._mm.write(struct.pack(IPC_FORMAT, *fields))
        finally:
            self._mutex.release()

        # Signal cppsvc
        self._cmd.release()

    def read_status(self) -> GantryStatus:
        """Read the current gantry status from shared memory (non-blocking)."""
        self._mutex.acquire()
        try:
            fields = self._read_raw()
        finally:
            self._mutex.release()

        # fields: [0]=tx [1]=ty [2]=cmd [3]=pri [4]=pad1 [5]=cseq
        #         [6]=cx [7]=cy [8]=state [9]=ready [10]=pad2 [11]=sseq
        return GantryStatus(
            current_x_mm=fields[6],
            current_y_mm=fields[7],
            state=fields[8],
            ready=fields[9],
            status_seq=fields[11],
        )

    def close(self) -> None:
        """Release all IPC resources (does not unlink -- let cppsvc own lifetime)."""
        try:
            self._mm.close()
        except Exception:
            pass
        try:
            self._mutex.close()
        except Exception:
            pass
        try:
            self._cmd.close()
        except Exception:
            pass

    def __enter__(self) -> "IPCHandler":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Internal helpers ────────────────────────────────────────────────────────

    def _read_raw(self) -> tuple:
        """Read all 32 bytes from shm and unpack.  Caller must hold the mutex."""
        self._mm.seek(0)
        return struct.unpack(IPC_FORMAT, self._mm.read(IPC_SIZE))
