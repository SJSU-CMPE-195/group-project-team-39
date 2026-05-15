---
name: Air Hockey IPC Workflow
overview: Integrate pysvc (object detection) and cppsvc (gantry control) via POSIX shared memory and semaphores, with a new trajectory predictor, coordinate mapper, and IPC handler classes on both sides. The workflow detects incoming pucks, predicts trajectories with bounce support, checks critical zone intersection, and sends movement commands to the gantry.
todos:
  - id: coord-mapper
    content: "Create pysvc/coordinate_mapper.py: CoordinateMapper class with configurable offsets, pixel<->gantry conversion"
    status: done-deviation
    note: "Folded into px_to_gantry_mm() in pysvc/main.py + constants in calibration_config.py. No separate class."
  - id: puck-detector
    content: "Create pysvc/puck_detector.py: PuckDetector class wrapping detection + tracking (green_mask_hsv, detect_green_puck_center, p0/p1 state, missing_count)"
    status: done-deviation
    note: "Detection lives in pysvc/green_puck_detect.py (functions, not a class). Kalman tracking + missing_count state is inline in pysvc/main.py."
  - id: trajectory-predictor
    content: "Create pysvc/trajectory_predictor.py: TrajectoryPredictor class with bounce-aware ray casting and critical zone intersection test"
    status: done-deviation
    note: "Implemented inline as trace_path_to_bottom() + first_border_intersection() in pysvc/main.py. Approach differs from plan: cast to bottom row and check predicted X (instead of per-segment ray-vs-rect)."
  - id: py-ipc
    content: "Create pysvc/ipc_handler.py: IPCHandler class using posix_ipc for shared memory + semaphores (send_command, read_status, close)"
    status: completed
  - id: cpp-ipc
    content: "Create cppsvc/include/ipc_handler.hpp + cppsvc/src/ipc_handler.cpp: AirHockeyIPC struct, IPCHandler class (read_command, write_status, wait_for_command)"
    status: completed
  - id: py-main
    content: "Modify pysvc/main.py: comment out old code, add new main loop integrating PuckDetector + TrajectoryPredictor + CoordinateMapper + IPCHandler"
    status: in-progress
    note: "Main loop, detection, tracking, and INTERCEPT/IDLE commands are wired. STILL-puck PUSH_NORTH / PUSH_OUT branches (workflow steps 3e Case 1 & 2) are NOT yet implemented."
  - id: cpp-main
    content: "Modify cppsvc/main.cpp: add IPC command loop (home -> default pos -> wait for command -> move_to_coord -> update status)"
    status: completed
  - id: cmake
    content: "Modify cppsvc/CMakeLists.txt: add ipc_handler.cpp source, link rt library"
    status: completed
  - id: push-emit
    content: "Wire PUSH_NORTH / PUSH_OUT two-step sequences into pysvc/main.py per workflow step 3e and Edge Cases 1 & 2 (position behind puck, wait ready=1, send target_y=901)."
    status: pending
    note: "Agreed BEHIND_OFFSET_MM = 80 (mallet r ~47.5mm + puck r ~30mm). If puck stops being STILL between step 1 and step 2, abort sequence and re-evaluate next tick."
  - id: low-light-hardening
    content: "Low-light robustness: CLAHE on V channel in green_mask_hsv(); Kalman processNoiseCov velocity terms lowered to 1e-3, measurementNoiseCov raised to 1e-1; dev HUD shows |v| and still_count under HEADLESS=0."
    status: completed
  - id: calib-assert
    content: "verify_gantry_calibration() in pysvc/main.py asserts GANTRY_TL_PX -> (0,901) mm and GANTRY_BR_PX -> (869,0) mm within CALIBRATION_TOL_MM (2.0). px_to_gantry_mm_raw() helper added so the clamp doesn't mask errors. crop_calibrate.py PREVIEW shows live implied mm with OK/BAD tags + loud warning on S-save."
    status: completed
    note: "Bypass via SKIP_CALIBRATION_CHECK=1 env var. Current calibration_config.py values FAIL the check (PX_PER_CM_X derived from 55cm CRIT zone vs GANTRY implying ~3.87 px/cm) -- rig re-calibration required."
  - id: corner-check
    content: "run_four_corner_check() in pysvc/main.py drives the gantry to SW(0,0), SE(869,0), NE(869,901), NW(0,901) via INTERCEPT, waits ready=1 with status_seq advance + 30s timeout per corner, prompts operator y/n/r, exits non-zero on any fail. Sends IDLE to park afterwards."
    status: completed
    note: "Bypass via SKIP_CORNER_CHECK=1 env var. Runs after IPC connect, before camera loop."
isProject: false
---

# Air Hockey IPC Workflow Plan

## Implementation status (snapshot)

This plan was written as the design doc for the pysvc ↔ cppsvc IPC integration.
Most of it is now built, with some deviations from the original architecture.
See per-todo `note:` fields in the frontmatter for specifics.

**Done:**

- IPC layer on both sides ([`pysvc/ipc_handler.py`](../pysvc/ipc_handler.py),
  [`cppsvc/include/ipc_handler.hpp`](../cppsvc/include/ipc_handler.hpp),
  [`cppsvc/src/ipc_handler.cpp`](../cppsvc/src/ipc_handler.cpp)).
- cppsvc command loop ([`cppsvc/main.cpp`](../cppsvc/main.cpp)): home → default
  pos → wait → `move_to_coord` → status update.
- CMake wiring (`ipc_handler.cpp` + `rt` link).
- pysvc main loop ([`pysvc/main.py`](../pysvc/main.py)): capture → TPS remap →
  Kalman detection → trajectory prediction → convergence buffer → INTERCEPT /
  IDLE commands.
- Low-light hardening: CLAHE V-channel normalisation in `green_mask_hsv()`,
  retuned Kalman covariance (`processNoiseCov` velocity terms = `1e-3`,
  `measurementNoiseCov` = `1e-1`), and a `HEADLESS=0` dev HUD showing
  `|v|` and `still_count`.
- Calibration verification: `verify_gantry_calibration()` asserts the
  GANTRY pixel-corner → mm mapping at pysvc startup;
  `run_four_corner_check()` drives the gantry to the four mm corners and
  prompts the operator to confirm each.  `SKIP_CALIBRATION_CHECK=1` and
  `SKIP_CORNER_CHECK=1` env vars bypass.  See
  [`docs/handoff.md`](handoff.md) → *Calibration verification* for the
  full design.

**Not yet built:**

- `PUSH_NORTH` / `PUSH_OUT` emit logic in pysvc (workflow step 3e, Cases 1 & 2).
  Defined in the IPC protocol and described in *Edge Case Handling* below,
  but `main.py` only emits `IDLE` and `INTERCEPT` today.

**Deviations from this plan:**

- No separate `CoordinateMapper`, `PuckDetector`, `TrajectoryPredictor`
  classes. The functionality is split between
  [`pysvc/green_puck_detect.py`](../pysvc/green_puck_detect.py) (HSV +
  centroid) and inline helpers in `main.py` (`px_to_gantry_mm`,
  `trace_path_to_bottom`, `first_border_intersection`). Calibration
  constants live in
  [`pysvc/calibration_config.py`](../pysvc/calibration_config.py) instead of
  being class fields.
- Trajectory prediction casts to the bottom row (robot side) and checks
  the predicted X against the critical-zone X-range, plus a convergence
  buffer for stability. The plan's per-segment ray-vs-rectangle test
  was not implemented — current approach is simpler and good enough for
  INTERCEPT, but it ignores trajectories that *enter* the critical zone
  laterally without reaching the bottom row.
- `pysvc/calibration_config.py` adds `PLAY_*_PX` and `GANTRY_*_PX` nested
  rectangles (a refinement not in the original plan) so `main.py` can
  branch on which region a STILL puck is in. See
  [`docs/handoff.md`](handoff.md) → *Camera physical coverage*.

For the canonical state-of-the-system overview, see
[`docs/handoff.md`](handoff.md).

---

## Coordinate System

Both services will use a **bottom-left origin** with X right, Y up:

- **Camera pixel space**: (0,0) at bottom-left, (640, 360) at top-right. OpenCV natively uses top-left origin, so all Y values are flipped via `y_cam = (h - 1) - y_opencv`.
- **Gantry space**: (0,0) at bottom-left (south-west, limit switch corner), (869, 901) mm at top-right. Already matches after homing.

### Pixel-to-Gantry Mapping ([`pysvc/coordinate_mapper.py`](pysvc/coordinate_mapper.py) -- new file)

The camera view physically covers a larger area than the gantry range, with configurable offsets:

```
Camera physical extent:
  width  = LEFT_OFFSET + GANTRY_X + RIGHT_OFFSET = 55 + 869 + 55 = 979 mm
  height = BOTTOM_OFFSET + GANTRY_Y + TOP_OFFSET  = 47 + 901 + 0  = 948 mm

Conversion (camera pixel -> gantry mm):
  x_mm = (pixel_x / CAM_WIDTH) * PHYS_WIDTH - LEFT_OFFSET
  y_mm = (pixel_y / CAM_HEIGHT) * PHYS_HEIGHT - BOTTOM_OFFSET

Valid gantry range: 0 <= x_mm <= 869, 0 <= y_mm <= 901
```

All offsets (`LEFT_OFFSET_MM`, `RIGHT_OFFSET_MM`, `BOTTOM_OFFSET_MM`, `TOP_OFFSET_MM`) and camera resolution (`CAM_WIDTH`, `CAM_HEIGHT`) are configurable constants.

The `CoordinateMapper` class provides:

- `pixel_to_gantry_mm(px, py) -> (x_mm, y_mm)` -- converts and returns raw mm (can be outside gantry range)
- `gantry_mm_to_pixel(x_mm, y_mm) -> (px, py)` -- inverse
- `is_in_gantry_range(x_mm, y_mm) -> bool` -- bounds check
- `clamp_to_gantry(x_mm, y_mm) -> (x_mm, y_mm)` -- clamp to nearest valid gantry position

---

## Critical Zone

Defined in **camera pixel space** (bottom-left origin) as a configurable rectangle. Default: full width, from Y=0 to `CRITICAL_ZONE_Y_MAX` (configurable, e.g. 180 px = bottom half). Any predicted trajectory that enters this rectangle triggers a gantry INTERCEPT command.

---

## Files (as built)

The architecture in *Implementation status* above differs from the original
plan (no separate detector / predictor / mapper classes).  Below is what
actually got built.

### pysvc (Python) — created

| File | Status | Purpose |
| ---- | ------ | ------- |
| [`pysvc/ipc_handler.py`](../pysvc/ipc_handler.py)             | done | `IPCHandler` class: posix_ipc shared memory + semaphores (`send_command`, `read_status`, `close`). |
| [`pysvc/green_puck_detect.py`](../pysvc/green_puck_detect.py) | done | `green_mask_hsv()` (CLAHE + HSV inRange + morph) and `detect_green_puck_center()` (contour centroid). Replaces the planned `PuckDetector` class. |
| [`pysvc/vision_preprocess.py`](../pysvc/vision_preprocess.py) | done | `build_remap_maps()` (TPS RBF) + `preprocess_frame()` (remap → 90-CW rotate). Shared by `main.py`, `test_detection.py`, `crop_calibrate.py`. |
| [`pysvc/calibration_config.py`](../pysvc/calibration_config.py)| done | Single source for `OUTPUT_W/H`, `SRC_POINTS`, `MALLET_ORIGIN_PX`, `PLAY/GANTRY/CRIT` nested rectangles, `PX_PER_CM_X/Y`. Replaces the planned `CoordinateMapper` configurable constants. |
| [`pysvc/camera.py`](../pysvc/camera.py)                       | done | `StereoCamera` wrapper around V4L2 + per-pixel stereo fusion. |
| [`pysvc/crop_calibrate.py`](../pysvc/crop_calibrate.py)       | done | One-time GUI tool: drag TPS handles + PLAY/GANTRY/CRIT/MALLET handles, prints config block on **S**. |
| [`pysvc/test_detection.py`](../pysvc/test_detection.py)       | done | Visual detector test (no IPC, no Kalman). Same remap + HSV path as `main.py`. |

### pysvc (Python) — NOT created (folded inline)

| Planned file | Where it lives now |
| ------------ | ------------------ |
| `pysvc/puck_detector.py` (PuckDetector class)             | Detection in `green_puck_detect.py`; Kalman + tracking state inline in `main.py`. |
| `pysvc/trajectory_predictor.py` (TrajectoryPredictor)     | `trace_path_to_bottom()` + `first_border_intersection()` inline in `main.py`. |
| `pysvc/coordinate_mapper.py` (CoordinateMapper class)     | `px_to_gantry_mm()` inline in `main.py`; constants in `calibration_config.py`. |

### cppsvc (C++) — created

| File | Status | Purpose |
| ---- | ------ | ------- |
| [`cppsvc/include/ipc_handler.hpp`](../cppsvc/include/ipc_handler.hpp) | done | `IPCHandler` class header + `AirHockeyIPC` struct + `static_assert(sizeof == 32)`. |
| [`cppsvc/src/ipc_handler.cpp`](../cppsvc/src/ipc_handler.cpp)         | done | shm_open + sem_open setup, read commands, write status. |

### Modified files

| File | Status |
| ---- | ------ |
| [`pysvc/main.py`](../pysvc/main.py)                 | Capture → TPS remap → Kalman detection → trajectory → convergence-based commit → INTERCEPT / IDLE. PUSH branches still pending. |
| [`cppsvc/main.cpp`](../cppsvc/main.cpp)             | done — home → default pos → wait for command → `move_to_coord` → status update. |
| [`cppsvc/CMakeLists.txt`](../cppsvc/CMakeLists.txt) | done — `ipc_handler.cpp` added, `rt` linked. |

`objectdetect_and_trajectory.py` and `object_detect_Apalis.py` remain as-is (legacy reference).

---

## Shared Memory Layout

Named shared memory: `/airhockey_shm`

```
struct AirHockeyIPC {
    // Command (pysvc -> cppsvc)
    float    target_x_mm;     // gantry target X
    float    target_y_mm;     // gantry target Y
    uint8_t  command;          // IDLE=0, INTERCEPT=1, PUSH_NORTH=2, PUSH_OUT=3
    uint8_t  priority;         // LOW=0, MEDIUM=1, HIGH=2
    uint16_t _pad1;
    uint32_t command_seq;      // incremented per new command

    // Status (cppsvc -> pysvc)
    float    current_x_mm;    // current mallet X
    float    current_y_mm;    // current mallet Y
    uint8_t  state;            // IDLE=0, MOVING=1, HOMING=2, ERROR=3
    uint8_t  ready;            // 1 = ready for next command
    uint16_t _pad2;
    uint32_t status_seq;       // incremented per status update
};
// Total: 32 bytes, naturally aligned
```

Priority mapping: INTERCEPT=HIGH(2), PUSH_NORTH=MEDIUM(1), PUSH_OUT=MEDIUM(1), IDLE=LOW(0).

---

## Semaphore Protocol

Two named POSIX semaphores:

- **`/airhockey_mutex`** (binary, init=1): Protects shared memory reads/writes.
- **`/airhockey_cmd`** (counting, init=0): Signals cppsvc that a new command is available.

### Write sequence (pysvc):

1. `sem_wait(mutex)` -- acquire lock
2. Write command fields to shared memory, increment `command_seq`
3. `sem_post(mutex)` -- release lock
4. `sem_post(cmd)` -- signal cppsvc

### Read sequence (cppsvc):

1. `sem_wait(cmd)` -- block until command available
2. `sem_wait(mutex)` -- acquire lock
3. Read command fields, copy locally
4. `sem_post(mutex)` -- release lock
5. Execute movement

### Status update (cppsvc):

1. `sem_wait(mutex)` -- acquire lock
2. Write status fields, increment `status_seq`
3. `sem_post(mutex)` -- release lock

### Status read (pysvc):

1. `sem_wait(mutex)` -- acquire lock
2. Read status fields
3. `sem_post(mutex)` -- release lock

---

## Workflow: pysvc Main Loop

```
1. Initialize: PuckDetector, TrajectoryPredictor, CoordinateMapper, IPCHandler
2. Open camera (configurable resolution/fps)
3. Loop:
   a. Read frame
   b. puck_state = detector.update(frame)   # returns position, velocity, is_still, is_detected
   c. Flip Y for bottom-left origin
   d. If NOT detected:
        - Send IDLE command (gantry returns to default position)
        - continue
   e. If puck is STILL:
        - Convert puck position to gantry mm
        - Case 1: Puck is on the side, outside gantry X range (x_mm < 0 or x_mm > 869)
            -> Send PUSH_NORTH: target_x = clamp(x_mm, 0, 869), target_y = 901 (push to top)
        - Case 2: Puck is within gantry range but outside critical zone
            -> Send PUSH_OUT: target = position behind puck, then push north
        - Case 3: Puck is in critical zone (do nothing special, it's on our side and still)
            -> Send IDLE or hold position
   f. If puck is MOVING:
        - predicted_pt = trajectory.predict(puck_pos, velocity, camera_bounds)
          (handles side-wall bounces, returns predicted endpoint)
        - Check if predicted trajectory intersects critical zone
        - If YES:
            -> Convert intersection point to gantry mm
            -> Clamp to gantry range
            -> Send INTERCEPT command (priority=HIGH)
        - If NO:
            -> Send IDLE (puck heading away from us)
   g. Read cppsvc status for logging/monitoring
```

---

## Workflow: cppsvc Main Loop

```
1. Initialize: gantry (motors, switches), IPCHandler
2. Home gantry: move_to_origin()
3. Move to default position: move_to_coord(434, 0)  // center-bottom
4. Update status: state=IDLE, ready=1
5. Loop:
   a. Wait for command (sem_wait on /airhockey_cmd)
   b. Read command from shared memory
   c. Update status: state=MOVING, ready=0
   d. Execute: move_to_coord(target_x_mm, target_y_mm)
   e. Update status: current_x/y = new position, state=IDLE, ready=1
   f. Loop back to (a)
```

For PUSH commands, the gantry moves to the target position which is pre-calculated by pysvc to be "behind" the puck, then pysvc sends a follow-up coordinate to push through.

---

## Edge Case Handling

### Edge Case 1: Puck at rest on the side, outside gantry X range

The puck is in the 5.5cm offset zone (camera sees it, but gantry can't reach its exact X). The mallet moves to the nearest gantry edge at the puck's Y coordinate minus an offset (to get "behind" it), then pushes north. Since the mallet is at the gantry edge and the puck is just outside, the mallet "rubs against the side" of the puck, pushing it northward out of the camera view.

**pysvc logic:**

1. Detect puck is still and `x_mm < 0` or `x_mm > 869`
2. Set `target_x = 0` (if puck on left side) or `target_x = 869` (if puck on right side)
3. Set `target_y = puck_y_mm - BEHIND_OFFSET` (position behind/south of puck)
4. Send PUSH_NORTH command
5. After gantry reports ready, send follow-up with `target_y = 901` (push north to max)

### Edge Case 2: Puck at rest outside critical zone, within gantry range

The puck is reachable by the gantry but not in the critical zone (it's in the upper portion of the field). Push it north out of the camera view.

**pysvc logic:**

1. Detect puck is still and within gantry range but Y > critical zone top
2. Set `target_x = puck_x_mm`, `target_y = puck_y_mm - BEHIND_OFFSET`
3. Send PUSH_OUT command
4. After gantry reports ready, send follow-up with `target_y = 901` (push north to camera edge)

### Edge Case 3: No puck detected

Send IDLE -- gantry returns to default position (center-bottom: 434, 0).

---

## Trajectory Prediction ([`pysvc/trajectory_predictor.py`](pysvc/trajectory_predictor.py))

New prediction logic with bounce support:

1. From puck position and velocity direction, cast a ray
2. Find first intersection with camera boundary (left, right, top, bottom walls)
3. If hits a **side wall** (left/right): reflect velocity, continue ray from bounce point
4. Repeat for up to `MAX_BOUNCES` (configurable, default 3)
5. At each step, check if the ray segment passes through the critical zone rectangle
6. If any segment enters the critical zone, return the intersection point with the critical zone boundary as the target
7. If the trajectory exits the camera view (hits top wall = opponent's side) without entering the critical zone, return None (no threat)

The critical zone intersection test is a ray-vs-rectangle intersection: check if the ray segment from point A to point B crosses into the rectangle defined by `(0, 0)` to `(CAM_WIDTH, CRITICAL_ZONE_Y_MAX)`.

---

## Dependencies

**pysvc**: `posix_ipc>=1.1.0` (already in `requirements-ipc.txt`), `opencv-python-headless`, `numpy`

**cppsvc**: `libgpiod`, `pthread`, `rt` (for `shm_open`/`sem_open` -- POSIX realtime extensions)
