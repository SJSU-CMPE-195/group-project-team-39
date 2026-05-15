"""
Single source for TPS remap and table-scale calibration.

After running crop_calibrate.py, paste the printed block here (overwrite the
OUTPUT_W/H, SRC_POINTS, mallet/critical-zone, and PX_PER_CM values).
main.py, vision_preprocess.py, crop_calibrate.py, and test_detection.py all read
these values so geometry stays aligned.
"""

from __future__ import annotations

# Output dimensions of the TPS-remapped frame (pre-rotation, landscape).
OUTPUT_W, OUTPUT_H = 510, 400

SRC_POINTS = [
    (168.0, 11.0),   # TL
    (531.0, 12.0),   # T-mid
    (878.0, 13.0),   # TR
    (873.0, 713.0),   # BR
    (541.0, 714.0),   # B-mid
    (178.0, 715.0),   # BL
]

MALLET_ORIGIN_PX = (31, 472)

PLAY_TL_PX   = (0, 34)
PLAY_BR_PX   = (396, 504)

GANTRY_TL_PX = (31, 34)
GANTRY_BR_PX = (370, 472)

CRIT_TL_PX   = (31, 377)
CRIT_BR_PX   = (370, 472)

PX_PER_CM_X = 3.9010
PX_PER_CM_Y = 4.8613

# Distance behind the puck to position the mallet before a PUSH sequence.
# Tune on rig. Derived from mallet radius (~47.5 mm) + puck radius (~30 mm) + slack.
BEHIND_OFFSET_MM = 80