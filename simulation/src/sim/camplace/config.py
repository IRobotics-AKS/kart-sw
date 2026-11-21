"""Every limit and assumption the study uses, in one place.

Frames: kart frame = rear axle centre on the ground, x forward, y left, z up (metres).
Camera axes (OpenCV): z_c along the view, x_c image right, y_c image down.
Angles: yaw = turned outward (+ = to the left), pitch = tilted down (+ = down), in degrees.
"""
import os

import numpy as np

# Paths in the kart-sw checkout (simulation/). The study's data and results live in
# simulation/studies/camera_placement/; the kart is the shared CAD model.
SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
ROOT = os.path.join(SIM_DIR, "studies", "camera_placement")
KART_XML = os.path.join(SIM_DIR, "models", "dreamer", "kart.xml")
TRACKS_DIR = os.path.join(ROOT, "data", "tracks")

# Cameras (datasheet fields of view, degrees: horizontal x vertical).
CAMS = {
    "oakd_s2": (69.0, 55.0),     # Luxonis OAK-D S2, colour camera (IMX378), standard lens
    "oak1_lite": (69.0, 54.0),   # Luxonis OAK-1 Lite (IMX214)
}
CAM_OF = {"front": "oakd_s2", "left": "oak1_lite", "right": "oak1_lite"}
WIDTH_PX = 1280                  # assumed image width for every camera (a common scale for ranking)
GSD_THR_CM = 10.0                # an edge point counts fully when one pixel spans <= 10 cm across the edge

# Kart geometry used by the lap model.
WHEELBASE = 1.04
MID = 0.52                       # the driven line is followed by the mid-wheelbase point
CG = np.array([0.42, 0.0, 0.107])
STEER_BINS = np.array([-0.4363, -0.2, -0.1, 0.0, 0.1, 0.2, 0.4363])   # rad; ends = full lock

# Robustness: every score is re-checked with the kart pitched / rolled (deg).
PERTURB = [(0, 0), (2, 0), (-2, 0), (0, 1), (0, -1)]   # (pitch nose-down, roll)

# Edge bands: along-track distance from the kart centre (m) and their weight in the set score.
BANDS = {"beside": (-1.5, 2.5), "near": (2.5, 6.0), "mid": (6.0, 12.0), "far": (12.0, 25.0)}
WEIGHTS = np.array([0.30, 0.30, 0.25, 0.15])

# Hard limits (pass/fail, no penalties).
X_MIN = 0.0                      # nothing behind the rear axle
Z_MAX = 0.80                     # height cap (m)
CLEAR_MIN = 0.05                 # camera centre at least this far from any kart part (m)
OCC_MAX = 0.15                   # kart may fill at most 15 % of the image at any steering angle
GOOD = 0.98                      # "good" = within 2 % of the best set score

# Kart-blocking ray map.
RES = 0.5                        # deg
AZ = np.arange(-180, 180, RES)
EL = np.arange(-85, 35 + 1e-9, RES)
AZ_WIN = {"front": (-45, 45), "left": (0, 125), "right": (-125, 0)}   # directions a grid camera can look

# Lap model.
FRAME_STEP_M = 1.0               # a kart pose every metre along the driven line
EDGE_STEP_M = 0.25               # edge sample spacing (m)
SPEED = dict(vmax=12.0, alat=8.5, acc=3.5, brk=8.0)

# Named camera sets: (x, y, z, yaw, pitch) per camera.
SETS = {
    "crossbar": {"front": (0.05, 0.0, 0.75, 0.0, 6.25), "left": (0.05, 0.30, 0.75, 60.0, 25.0),
                 "right": (0.05, -0.30, 0.75, -60.0, 25.0)},
    "crossbar65": {"front": (0.05, 0.0, 0.75, 0.0, 6.25), "left": (0.05, 0.30, 0.75, 65.0, 25.0),
                   "right": (0.05, -0.30, 0.75, -65.0, 25.0)},
    "best_purdue": {"front": (0.0, 0.0, 0.725, 0.0, 1.25), "left": (0.0, 0.3, 0.8, 65.0, 25.0),
                    "right": (0.0, -0.3, 0.75, -65.0, 25.0)},
    "previous": {"front": (0.30, 0.0, 0.70, 0.0, 5.0), "left": (0.15, 0.35, 0.65, 60.0, 20.0),
                 "right": (0.15, -0.35, 0.65, -60.0, 20.0)},
    "nose": {"front": (1.10, 0.0, 0.60, 0.0, 7.5), "left": (0.75, 0.5, 0.55, 60.0, 20.0),
             "right": (0.75, -0.5, 0.55, -60.0, 20.0)},
}

# Mounting boxes found on Purdue (scripts/optimize.py, results/summary.json): x0, x1, y0, y1, z0, z1.
BOXES = {
    "front": (0.0, 0.15, -0.10, 0.10, 0.625, 0.80),
    "left": (0.0, 0.30, 0.30, 0.50, 0.45, 0.75),
    "right": (0.0, 0.35, -0.45, -0.30, 0.45, 0.75),
}
