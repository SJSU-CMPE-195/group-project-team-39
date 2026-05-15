# Air Hockey Project -- Agent Handoff

This document captures the current state of the air hockey robot project so the
next contributor can pick it up without reading the full chat history.

---

## System overview

Two services talk over POSIX shared memory + named semaphores:

```
ZED 2i camera --> pysvc (Python, vision + game logic) ---- /airhockey_shm ----> cppsvc (C++, gantry motor control)
                                                            /airhockey_cmd
                                                            /airhockey_mutex
```

- **pysvc** runs on a Torizon Linux container, captures frames from the ZED 2i
  via V4L2, fuses both stereo eyes into one image, applies a TPS remap to
  correct perspective distortion, detects the green puck with a Kalman filter,
  predicts its trajectory using bounce-aware ray casting, and sends intercept
  coordinates over shared memory when the puck is heading into the critical zone.
- **cppsvc** runs on a separate container with GPIO access, drives two iSV57T
  BLDC motors in CoreXY configuration, reads two limit switches for homing,
  and consumes commands from shared memory one at a time.

Hardware target: **Toradex Apalis iMX8** (NXP i.MX8) on an Ixora carrier board.

---

## Repository layout

```
group-project-team-39/
├── pysvc/                       # Python vision service
│   ├── main.py                  # main loop: capture -> remap -> detect -> predict -> command
│   ├── calibration_config.py   # OUTPUT_W/H, SRC_POINTS, mallet origin, PLAY/GANTRY/CRIT rectangles, px/cm (single source)
│   ├── vision_preprocess.py    # TPS remap build + preprocess_frame (shared with test tool)
│   ├── green_puck_detect.py    # HSV green puck centroid (shared with test tool)
│   ├── camera.py                # ZED 2i V4L2 capture wrapper (stereo fusion)
│   ├── ipc_handler.py           # posix_ipc shm + semaphores (writer side)
│   ├── crop_calibrate.py        # one-time tool: calibrate TPS remap + critical zone
│   ├── test_detection.py        # visual test: same remap + detection as main (no IPC)
│   ├── requirements-ipc.txt     # posix_ipc, scipy; cv2/numpy come from Dockerfile
│   └── Dockerfile
├── cppsvc/                      # C++ gantry control service
│   ├── main.cpp                 # init hardware -> home -> IPC command loop
│   ├── CMakeLists.txt
│   ├── docker-compose.yml       # both services + m4loader
│   ├── include/
│   │   ├── gantry.hpp           # CoreXY gantry interface
│   │   ├── iSV57T.hpp           # motor driver
│   │   ├── limitSwitch.hpp      # GPIO limit switch reader
│   │   └── ipc_handler.hpp      # AirHockeyIPC struct + IPCHandler (reader)
│   └── src/                     # implementations of the above
├── docs/                        # this folder
└── stl_designs/                 # mechanical CAD
```

---

## Coordinate systems

All gameplay logic uses a **bottom-left origin** (X right, Y up):

- **Portrait pixel space** (after TPS remap + 90-CW rotation): (0,0) at
  top-left in OpenCV convention, but `MALLET_ORIGIN_PX` marks where gantry
  (0,0) appears in that frame.  `px_to_gantry_mm()` in `main.py` converts
  from pixel coordinates to absolute gantry mm using `MALLET_ORIGIN_PX` and
  `PX_PER_CM_X`/`PX_PER_CM_Y` (values live in [`calibration_config.py`](../pysvc/calibration_config.py), updated from `crop_calibrate.py`).
- **Gantry space**: (0,0) at the south-west limit-switch corner,
  (869, 901) mm at the north-east. Defined in
  [`cppsvc/include/gantry.hpp`](../cppsvc/include/gantry.hpp).

### Camera physical coverage

The camera is mounted so it sees a **larger area than the gantry can reach**.
Approximate physical extents (per the workflow plan):

```
                  TOP_OFFSET    =   0 mm
                ┌───────────────────────────────┐
                │                               │
LEFT_OFFSET     │      GANTRY_X = 869 mm        │   RIGHT_OFFSET
   = 55 mm      │      GANTRY_Y = 901 mm        │      = 55 mm
                │                               │
                └───────────────────────────────┘
                  BOTTOM_OFFSET = 47 mm

Camera total: 979 × 948 mm  (gantry sits inside, ~5.5 cm slack on each side)
```

Those 5.5 cm side strips are what enable **PUSH_NORTH** (edge case 1 in
*Edge cases*): the puck can rest in a region the camera sees but the gantry
cannot reach in X.  The play-area / gantry-range / critical-zone rectangles
defined in [`calibration_config.py`](../pysvc/calibration_config.py)
(`PLAY_*_PX`, `GANTRY_*_PX`, `CRIT_*_PX`) encode this nesting in pixel
space so `main.py` can branch on which region the puck currently sits in.

---

## IPC protocol

### Shared memory (`/airhockey_shm`, 32 bytes)

| Offset | Field          | Type     | Direction        |
| -----: | -------------- | -------- | ---------------- |
|      0 | `target_x_mm`  | float    | py -> cpp        |
|      4 | `target_y_mm`  | float    | py -> cpp        |
|      8 | `command`      | uint8    | py -> cpp        |
|      9 | `priority`     | uint8    | py -> cpp        |
|     12 | `command_seq`  | uint32   | py -> cpp        |
|     16 | `current_x_mm` | float    | cpp -> py        |
|     20 | `current_y_mm` | float    | cpp -> py        |
|     24 | `state`        | uint8    | cpp -> py        |
|     25 | `ready`        | uint8    | cpp -> py        |
|     28 | `status_seq`   | uint32   | cpp -> py        |

The byte layout is also documented at the top of
[`pysvc/ipc_handler.py`](../pysvc/ipc_handler.py) and
[`cppsvc/include/ipc_handler.hpp`](../cppsvc/include/ipc_handler.hpp).
A `static_assert(sizeof(AirHockeyIPC) == 32)` in the C++ header guards against
accidental padding drift; the Python `struct` format `=ffBBHIffBBHI` matches.

### Semaphores

- `/airhockey_mutex` -- binary, init=1, protects shm reads/writes
- `/airhockey_cmd`   -- counting, init=0, posted by pysvc when a new command
  is ready, awaited by cppsvc before reading

### Command values

```
IDLE=0  INTERCEPT=1  PUSH_NORTH=2  PUSH_OUT=3
```

Priority mapping: `INTERCEPT` = HIGH (2), `PUSH_NORTH`/`PUSH_OUT` = MEDIUM (1),
`IDLE` = LOW (0).  Priority is informational today — cppsvc processes commands
strictly in FIFO order (see *Execution model* below).

**Implementation status:** the IPC layer (both sides) and the cppsvc command
loop support all four commands.  In `pysvc/main.py`, only `IDLE` and
`INTERCEPT` are emitted today; the `PUSH_NORTH` / `PUSH_OUT` branches are
specified by the workflow plan ([`docs/air_hockey_ipc_workflow_c7b9b537.plan.md`](air_hockey_ipc_workflow_c7b9b537.plan.md))
and described in *Edge cases* below but not yet wired into the vision loop.

### Execution model (important)

**cppsvc is strictly sequential**: it `sem_wait`s for one command, runs the
full blocking `move_to_coord`, marks `ready=1`, then waits for the next.
There is no preemption or mid-motion update.

### Status feedback (cpp -> py)

Every state transition in cppsvc (HOMING, MOVING, IDLE, ERROR) is written
back to shared memory under `mutex`, with `current_x_mm`, `current_y_mm`,
and a monotonic `status_seq`.  pysvc can read this back at any time via
`IPCHandler.read_status()` — used today for logging, and required by the
PUSH two-step sequences (see *Edge cases*), which wait on `ready == 1`
before sending the follow-up `target_y = 901` command.

---

## pysvc main loop logic

State machine inside [`pysvc/main.py`](../pysvc/main.py):

### Frame preprocessing

1. ZED 2i outputs a 2560×720 side-by-side stereo frame at 60 FPS.
2. Both eyes (each 1280×720) are **pixel-averaged** (fused) into a single
   1280×720 image -- reduces noise compared to using one eye alone.
3. A **Thin-Plate-Spline (TPS) remap** corrects perspective distortion, warping
   the fused image onto a flat OUTPUT_W×OUTPUT_H (510×400) canvas aligned to
   the table edges.
4. A **90-CW rotation** produces a 400×510 portrait frame where the robot
   side is at the bottom (row h-1) and the opponent side is at the top (row 0).

### Detection & tracking

A **Kalman filter** (constant-velocity, 4-state [x, y, vx, vy]) tracks the
green puck across frames.  HSV masking + contour detection provides raw
measurements; a jump-gate rejects outlier detections.

**Low-light hardening (in place):**

- `green_mask_hsv()` in [`pysvc/green_puck_detect.py`](../pysvc/green_puck_detect.py)
  runs **CLAHE** (`clipLimit=2.0, tileGridSize=(8,8)`) on the V channel
  before `cv2.inRange`, so dim frames are histogram-normalised and the
  HSV bounds (`H:[25,95] S/V:[40,255]`) still match.
- `create_kalman()` in [`pysvc/main.py`](../pysvc/main.py) uses
  `processNoiseCov` diag `[1e-2, 1e-2, 1e-3, 1e-3]` and
  `measurementNoiseCov = 1e-1 * I`.  The low velocity-process-noise
  suppresses centroid jitter under poor lighting from being interpreted
  as real velocity, which kept the puck oscillating STILL ↔ MOVING.
  If a real fast shot ever feels sluggish, raise velocity terms back
  toward `1e-2` or drop `measurementNoiseCov` to `5e-2`.
- A debug HUD (`|v|=X.Xpx  still=N  STILL/MOVING`) prints under the FPS
  line when `HEADLESS=0`.  Use it to visually confirm the filter is
  settling under whatever lighting you're tuning.

### Trajectory prediction

When the Kalman-filtered speed exceeds `MIN_SPEED_PX`:
- `trace_path_to_bottom()` casts a bounce-aware ray from the puck's position
  in the direction of its velocity, reflecting off left/right walls, until
  it reaches the bottom row (robot side) or exceeds `MAX_BOUNCES`.
- The predicted bottom-target X is accumulated in a time-windowed
  **prediction buffer**.

### Command decision

The state machine picks an IPC command each evaluation tick, based on whether
the puck is detected and whether it is moving (Kalman `|v|` thresholded by
`STILL_THRESH_PX` for `STILL_CONFIRM_FRAMES` consecutive frames):

```
no puck detected            -> IDLE
puck STILL, outside gantry X-> PUSH_NORTH  (edge case 1, two-step)
puck STILL, inside gantry,
       outside critical zone-> PUSH_OUT    (edge case 2, two-step)
puck STILL, inside critical -> IDLE / hold position
puck MOVING:
   trace_path_to_bottom + convergence buffer:
      predicted X in critical zone -> INTERCEPT
      otherwise                    -> IDLE
```

**Convergence-based commit (INTERCEPT path):**

- The most recent `RECENT_N` buffer samples are checked for spread ≤
  `CONVERGE_SPREAD_PX`.  Only converged predictions are acted on.
- **In critical zone** (target X inside `CRIT_TL_PX[0]`..`CRIT_BR_PX[0]`):
  send `INTERCEPT` at the predicted gantry mm coordinate.
- **Outside critical zone**: send `IDLE` (gantry returns to default position).
- An **anchor/refine deadband** (`ANCHOR_DEADBAND_PX` / `REFINE_DEADBAND_PX`)
  prevents oscillation: once an INTERCEPT is committed, small jitter in the
  predicted target is ignored until the prediction shifts by ≥ 30 px.
- When the buffer empties (puck slows or disappears): send `IDLE`.

### Edge cases (PUSH sequences)

Both PUSH cases are **two-step sequences** emitted by pysvc.  cppsvc itself
makes no distinction between PUSH and INTERCEPT — it just runs
`move_to_coord` to the supplied target — so the "position behind, then
shove north" feel is enforced entirely by pysvc sending two sequenced
commands and waiting on `ready=1` between them.

**Edge case 1 — puck at rest just outside the gantry's X range** (the
~5.5 cm camera-edge zone where the camera sees the puck but the gantry can
not reach its exact X):

1. Send `PUSH_NORTH` with `target_x = 0` (puck on left side) or
   `target_x = 869` (puck on right side), `target_y = puck_y_mm - BEHIND_OFFSET`.
   The mallet positions at the gantry edge, south of the puck.
2. Wait for cppsvc `ready = 1`, then send a follow-up command with
   `target_y = 901` — the mallet rubs against the side of the puck and
   pushes it north out of the camera view.

**Edge case 2 — puck at rest within gantry range, above the critical zone**:

1. Send `PUSH_OUT` with `target_x = puck_x_mm`,
   `target_y = puck_y_mm - BEHIND_OFFSET`.  Mallet positions directly
   south of the puck.
2. Wait for cppsvc `ready = 1`, then send `target_y = 901`.

**Edge case 3 — no puck detected**: send `IDLE` — gantry returns to its
default rest position `(DEFAULT_X_MM, DEFAULT_Y_MM)` = `(434, 0)` mm.

---

## Camera setup

The ZED 2i outputs a side-by-side V4L2 frame at 720p mode
(2560×720 @ 60 FPS).  Both eyes are averaged per-pixel to produce a single
1280×720 fused frame.

[`pysvc/camera.py`](../pysvc/camera.py) contains the `StereoCamera` class.

### One-time TPS calibration

Run on a machine with a display attached and the ZED plugged in:

```bash
cd pysvc
python crop_calibrate.py
```

Two windows open:

**RAW window** -- drag the 6 teal handles wider than the playing surface
(include table rails / margin).  The puck gets clipped at the south edge
if you crop flush with the play area.

**PREVIEW window** -- the TPS-remapped + 90-CW-rotated view.  Four nested
regions, each with two draggable corner handles (inner handles are hard-
clamped to stay inside their parent):

| Handle colour | Region | What to drag it to |
| ------------- | ------ | ------------------- |
| Green  | `PLAY`   | Actual playing-surface edges |
| Yellow | `GANTRY` | Gantry's reachable rectangle |
| Blue   | `CRIT`   | Physical critical zone (55 cm × 35 cm) |
| Orange | `MALLET 0,0` | Where the gantry sits at (0, 0) |

Press **S** to print the config block.  Paste all printed values into
[`pysvc/calibration_config.py`](../pysvc/calibration_config.py) (overwrite
`OUTPUT_W`, `OUTPUT_H`, `SRC_POINTS`, `MALLET_ORIGIN_PX`,
`PLAY_TL_PX`, `PLAY_BR_PX`, `GANTRY_TL_PX`, `GANTRY_BR_PX`,
`CRIT_TL_PX`, `CRIT_BR_PX`, `PX_PER_CM_X`, `PX_PER_CM_Y`).
`main.py` and `test_detection.py` import from that file, so they stay aligned.

To verify detection on the same calibrated view before deploying:

```bash
python test_detection.py
```

### Current calibration values ([`pysvc/calibration_config.py`](../pysvc/calibration_config.py))

```python
# Output dimensions of the TPS-remapped frame (pre-rotation, landscape).
OUTPUT_W, OUTPUT_H = 510, 400

SRC_POINTS = [
    (223.0,   7.0),   # TL
    (639.0,   2.0),   # T-mid
    (975.0,   5.0),   # TR
    (975.0, 714.0),   # BR
    (636.0, 719.0),   # B-mid
    (239.0, 715.0),   # BL
]

# Remapped + 90-CW rotated pixel space (portrait, goal at bottom row).
# Nesting: full preview (TPS crop) ⊇ PLAY ⊇ GANTRY ⊇ CRIT.
# Mallet origin lives inside the gantry rectangle.
MALLET_ORIGIN_PX = (195, 403)

PLAY_TL_PX   = (8,  33)
PLAY_BR_PX   = (398, 472)

GANTRY_TL_PX = (30,  33)
GANTRY_BR_PX = (366, 433)

CRIT_TL_PX   = (30,  348)
CRIT_BR_PX   = (366, 433)

PX_PER_CM_X = 6.1091
PX_PER_CM_Y = 2.4286
```

> **Note:** these values were last set by running `crop_calibrate.py` on
> the rig.  Re-run whenever the camera mount or table position changes.

---

## Running

### Local Python (development)

```bash
pip install opencv-python-headless numpy scipy posix_ipc
python crop_calibrate.py   # calibrate first -> paste into calibration_config.py
python main.py             # start vision loop
```

Set `HEADLESS=1` to suppress the OpenCV display window (required in production
containers that have no display):

```bash
HEADLESS=1 python main.py
```

### On the Apalis (production)

`docker compose up` from `cppsvc/` brings up all three services:
`m4loader` (runs once to load M4 firmware), then `cppsvc` and `pysvc` in
parallel. Both have `ipc: host` so they share the POSIX namespaces.

The image names (`yathien/cppsvc:dev`, `yathien/pysvc:dev`) are hardcoded
and need to be built+pushed manually right now.

### Build / runtime dependencies

**pysvc** (Python): `posix_ipc >= 1.1.0` (in
[`requirements-ipc.txt`](../pysvc/requirements-ipc.txt)), `scipy` (for the
TPS RBF interpolator), plus `opencv-python-headless` and `numpy` which come
from the [`Dockerfile`](../pysvc/Dockerfile).

**cppsvc** (C++):
- `libgpiod` — GPIO access for limit switches
- `pthread` — threading primitives
- `rt` — POSIX realtime extensions (`shm_open`, `sem_open`).  Linked in
  [`cppsvc/CMakeLists.txt`](../cppsvc/CMakeLists.txt) so the IPC layer
  works.

---

## Calibration verification (next-up)

Two related tasks that ensure the pixel → mm pipeline actually lands the
gantry where pysvc thinks it will.  Both should be done together — task 1
is a prerequisite for task 2 producing meaningful results.

### Hardware caveat (assumed origin)

`Gantry::move_to_origin()` in cppsvc is **broken on the current hardware**
(limit-switch wiring issue, not a software bug).  Because of this:

- cppsvc's startup currently skips homing.
- The gantry is assumed to be physically parked at the south-west
  limit-switch corner (gantry mm `(0, 0)`) before `cppsvc` is launched.
- `curr_x` / `curr_y` are initialised to `0.0` at the start of
  [`cppsvc/main.cpp`](../cppsvc/main.cpp) and reported through the IPC
  status fields as such.

Anyone running the gantry must manually push the carriage to the SW corner
before `docker compose up`.  Re-enable `move_to_origin()` once the
limit-switch hardware is fixed.

### Task 1: Align the gantry coordinate system across cppsvc and pysvc

`Gantry` in [`cppsvc/include/gantry.hpp`](../cppsvc/include/gantry.hpp)
defines its coordinate frame with the origin at the bottom-left
(south-west, limit-switch corner) and extents `(869, 901)` mm at the
north-east — X grows right, Y grows up.

`GANTRY_TL_PX` and `GANTRY_BR_PX` in
[`calibration_config.py`](../pysvc/calibration_config.py) are the
pixel-space mirror of that rectangle.  In the portrait + bottom-left-origin
frame they must map exactly to:

| Pixel handle      | Gantry mm  |
| ----------------- | ---------- |
| `GANTRY_TL_PX`    | `(0, 901)` |
| `GANTRY_BR_PX`    | `(869, 0)` |

i.e. `px_to_gantry_mm(*GANTRY_TL_PX)` must return `(0.0, 901.0)` and
`px_to_gantry_mm(*GANTRY_BR_PX)` must return `(869.0, 0.0)`.

**Status: built.** `verify_gantry_calibration()` in
[`pysvc/main.py`](../pysvc/main.py) runs at startup (before the TPS remap
build), using a new unclamped `px_to_gantry_mm_raw()` helper so an
out-of-range pixel maps to out-of-range mm instead of being silently
clipped.  Both corner mm values + deltas are always printed; on any
component exceeding `CALIBRATION_TOL_MM` (default 2.0 mm) the process
exits non-zero.  Set `SKIP_CALIBRATION_CHECK=1` to bypass (dev only).

In [`pysvc/crop_calibrate.py`](../pysvc/crop_calibrate.py), the PREVIEW
window now shows live "GANTRY-TL mm" / "GANTRY-BR mm" readouts with the
expected values and an `OK`/`BAD` tag (BAD lines render in red).  Pressing
**S** still prints the paste block, but prepends a loud `### WARNING ###`
section explaining that pysvc startup will refuse to launch unless both
lines read OK.

> **Heads-up:** the calibration values currently in
> [`calibration_config.py`](../pysvc/calibration_config.py) **fail** this
> assertion — `PX_PER_CM_X = 6.1091` is derived from the 55 cm critical zone
> width but the GANTRY rectangle implies ~3.87 px/cm.  Re-run
> `crop_calibrate.py` on the rig and drag GANTRY / MALLET / CRIT until both
> `GANTRY-TL mm` and `GANTRY-BR mm` lines read OK.

### Task 2: Four-corner movement verification

After task 1 passes, validate end-to-end that the gantry physically reaches
the four corners pysvc commands.  Runs **before** the main vision loop.

**Status: built.** `run_four_corner_check()` in
[`pysvc/main.py`](../pysvc/main.py) executes after `IPCHandler` connect
and before the vision loop.  For each of `SW (0,0)`, `SE (869,0)`,
`NE (869,901)`, `NW (0,901)`:

1. Capture the pre-send `status_seq`.
2. Send `INTERCEPT` to the corner.
3. Poll `read_status()` until `ready == 1` AND `status_seq > pre_seq`
   (so a stale `ready=1` from the previous command doesn't false-pass).
   Per-corner timeout: `CORNER_CHECK_TIMEOUT_S` (default 30 s) → marked
   FAIL on timeout.
4. Operator prompt on stdin:
   `[CORNER] SW: did the gantry reach (0, 0) mm? (y/n/r=retry):`
   - **y** → record success, advance.
   - **n** → record failure, advance.
   - **r** → resend the same coordinate, re-prompt.

After all four, prints a summary table and returns `True` only if every
corner was `y`; main.py exits non-zero otherwise.  On exit (success or
fail) the function sends an `IDLE` to park the gantry at
`(DEFAULT_X_MM, DEFAULT_Y_MM)`.

**Bypass:** `SKIP_CORNER_CHECK=1` (env var) skips the entire routine —
use once the calibration is trusted.

**Design notes:**

- `INTERCEPT` is used rather than `IDLE` so the priority byte matches a
  real movement command.
- Raw mm corners are sent — `MALLET_ORIGIN_PX` is not involved.  Task 1
  validates the pixel pipeline; task 2 is the physical-reach check.

---

## Known gaps / things the next agent might tackle

1. **No mid-motion preemption** -- if pysvc decides to intercept while the
   gantry is mid-IDLE-return, it has to wait. Could split cppsvc into a
   command-receiver thread + a motion-executor thread that supports
   cancellation.
2. **Critical-zone boundaries are hardcoded** -- `CRIT_TL_PX`, `CRIT_BR_PX`
   live in [`calibration_config.py`](../pysvc/calibration_config.py).  Re-run
   `crop_calibrate.py` if the camera mount changes.
3. **Rig calibration is currently off** -- the corner-assertion check
   (see *Calibration verification* above) is wired up and runs at pysvc
   startup, but the values in
   [`calibration_config.py`](../pysvc/calibration_config.py) **fail** it
   today (GANTRY-BR implies ~280 mm instead of 869 mm in X).  Someone with
   physical access to the rig needs to re-run `crop_calibrate.py` until
   both GANTRY corner lines in the PREVIEW window read OK, then paste
   the new block in.  Use `SKIP_CALIBRATION_CHECK=1` / `SKIP_CORNER_CHECK=1`
   for dev work in the meantime.
4. **`Gantry::move_to_origin()` broken on current hardware** -- per the
   *Hardware caveat* note above, cppsvc is *meant* to skip homing and
   assume the carriage is parked at `(0, 0)` mm.  At present
   [`cppsvc/main.cpp`](../cppsvc/main.cpp) still calls `g.move_to_origin()`
   and initialises `curr_x` to `DEFAULT_X_MM` (434) — so the code and the
   doc disagree.  Either guard the homing call behind a flag (e.g.
   `SKIP_HOMING=1` env or compile-time `#ifdef`) and init `curr_x/y = 0.0`,
   or document that the limit-switch fix has been completed.
5. **PUSH emit logic not yet wired** -- the workflow plan defines
   `PUSH_NORTH` / `PUSH_OUT` as two-step sequences for stationary pucks
   (see *Edge cases* above), but `main.py` currently only emits `IDLE` and
   `INTERCEPT`.  Adding the STILL-puck branches + the `ready=1` wait + the
   follow-up `target_y = 901` command is the next vision-loop task.  The
   IPC layer already supports it.  Agreed `BEHIND_OFFSET_MM = 80`
   (mallet radius ~47.5 mm + puck radius ~30 mm + slack).  If the puck
   stops being STILL between step 1 and step 2 of a sequence, abort the
   sequence and re-evaluate the state machine next tick.
6. **No watchdog** -- if pysvc crashes, cppsvc sits at its last commanded
   position.  A per-command timeout in cppsvc or a heartbeat from pysvc
   would help.
7. **Hardcoded image tags** in `cppsvc/docker-compose.yml`. A CI pipeline
   that tags images on commit and updates the compose file would help.
8. **No tests on pysvc**. The geometry helpers (`trace_path_to_bottom`,
   `first_border_intersection`, `px_to_gantry_mm`) are well-isolated and
   easy to unit-test if someone wants to add a `tests/` folder.
9. **No per-command motion profile on cppsvc** -- by design today, cppsvc
   treats every command identically (`move_to_coord` to target), and the
   "position-behind-then-shove-north" feel is encoded in the pysvc two-step
   sequence.  If we ever want a true fast/slow distinction per command
   type, the C++ side needs to branch on `cmd.command` and adjust the
   motion parameters per case.

---

## Quick file map for the next agent

| Looking for...                            | Look in |
| ----------------------------------------- | ------- |
| ... puck HSV / centroid (shared)           | [`pysvc/green_puck_detect.py`](../pysvc/green_puck_detect.py) |
| ... how the puck is detected/tracked      | [`pysvc/main.py`](../pysvc/main.py) (Kalman + `green_puck_detect`) |
| ... how trajectories are predicted        | [`pysvc/main.py`](../pysvc/main.py) (`trace_path_to_bottom`, `first_border_intersection`) |
| ... when/where the gantry should move     | [`pysvc/main.py`](../pysvc/main.py) (convergence-based commit section) |
| ... pixel/mm geometry or offsets          | [`pysvc/calibration_config.py`](../pysvc/calibration_config.py), `px_to_gantry_mm` in [`pysvc/main.py`](../pysvc/main.py) |
| ... the TPS remap / preprocess            | [`pysvc/vision_preprocess.py`](../pysvc/vision_preprocess.py), constants in [`calibration_config.py`](../pysvc/calibration_config.py) |
| ... the calibration tool                  | [`pysvc/crop_calibrate.py`](../pysvc/crop_calibrate.py) |
| ... visual detection test (no IPC)        | [`pysvc/test_detection.py`](../pysvc/test_detection.py) |
| ... the IPC byte layout                   | both [`pysvc/ipc_handler.py`](../pysvc/ipc_handler.py) and [`cppsvc/include/ipc_handler.hpp`](../cppsvc/include/ipc_handler.hpp) |
| ... how the motors actually move          | [`cppsvc/src/gantry.cpp`](../cppsvc/src/gantry.cpp), [`cppsvc/src/iSV57T.cpp`](../cppsvc/src/iSV57T.cpp) |
| ... what command cppsvc does on startup   | [`cppsvc/main.cpp`](../cppsvc/main.cpp) |
| ... what gets deployed where              | [`cppsvc/docker-compose.yml`](../cppsvc/docker-compose.yml), [`pysvc/Dockerfile`](../pysvc/Dockerfile) |

Good luck.
