# """
# wall_e_script.py — Subsumption Architecture Controller for the Object Sorting Task
# ====================================================================================
# Architecture (highest → lowest priority):

#   Level 4  Recharge          — Battery critically low; seek charger or spin-scan.
#   Level 3  Sort_and_Compact  — Bumper force detected; identify object, push to bin.
#   Level 2  Seek_Object       — Coloured blob visible; steer toward it.
#   Level 1  Wander            — Default; drive forward with timed random yaw.
#   Level 0  Survive           — Sonar obstacle < threshold; intercepts final command.

# All behaviours are purely reactive: no X/Y tracking, no "memory" of prior actions.
# The only non-sensor state is a single wall-clock timestamp used by Wander to inject
# periodic random turns — this is temporal scheduling, not spatial memory.

# Assumptions
# -----------
# 1.  Charger is marked with a bright-yellow fiducial visible to the top camera.
# 2.  Object identity (green plant vs brown trash) is read from the small (front) camera.
# 3.  Bin locations are inferred live from the top camera each cycle.
# 4.  robot.compress() is called every cycle while brown trash is being pushed; the
#     CoppeliaSim Lua script is assumed to latch the compress signal itself.
# 5.  get_sonar_sensor() returns -1 when no object is in range (per docstring).
# 6.  Bumper reading max > BUMPER_FORCE_THRESHOLD is used as "actively pushing" test.
# 7.  Level 0 (Survive) will suppress Level 3 if the sonar fires while pushing — a
#     known limitation of purely reactive systems. The robot will resume pushing on
#     the next cycle once the obstacle (bin edge) is no longer < SONAR_DANGER_DIST.
# """

# from robots import *
# import time
# import random
# import numpy as np
# from coppeliasim_zmqremoteapi_client import *

# # ─────────────────────────────────────────────────────────────────────────────
# #  CoppeliaSim connection
# # ─────────────────────────────────────────────────────────────────────────────
# client = RemoteAPIClient()
# sim    = client.require("sim")

# # ─────────────────────────────────────────────────────────────────────────────
# #  Actuators & Sensors
# # ─────────────────────────────────────────────────────────────────────────────
# robot              = Robot_OS(sim, DeviceNames.ROBOT_OS)
# top_image_sensor   = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
# small_image_sensor = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)
# left_motor         = Motor(sim, DeviceNames.MOTOR_LEFT_OS,  Direction.CLOCKWISE)
# right_motor        = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)

# # ─────────────────────────────────────────────────────────────────────────────
# #  Tunable Constants
# # ─────────────────────────────────────────────────────────────────────────────
# BATTERY_LOW_THRESHOLD  = 0.1   # % — triggers Recharge
# BUMPER_FORCE_THRESHOLD = 0.6   # N — triggers Sort_and_Compact
# SONAR_DANGER_DIST      = 0.4    # m — triggers Survive (−1 means no reading)
# BLOB_MIN_COVERAGE      = 0.005  # fraction of pixels — minimum blob size to act on
# BLOB_FOUND             = False
# BASE_SPEED   = 3.0              # rad/s forward cruise
# SLOW_SPEED   = 1.5              # rad/s push / approach speed
# TURN_SPEED   = 2.0              # rad/s differential correction amplitude
# WANDER_INTERVAL = 4.0           # seconds between random wander yaw injections

# # ─────────────────────────────────────────────────────────────────────────────
# #  Pixel colour ranges (uint8, 0–255) keyed by semantic label
# # ─────────────────────────────────────────────────────────────────────────────
# _COLOUR_RANGES: dict[str, dict[str, tuple[int, int]]] = {
#     #            R-range        G-range       B-range
#     "green":  {"r": (  0,  80), "g": (100, 255), "b": (  0,  80)},
#     "brown":  {"r": ( 80, 200), "g": ( 40, 100), "b": (  0,  60)},
#     "blue":   {"r": (  0,  80), "g": (  0,  80), "b": (100, 255)},  # blue bin
#     "red":    {"r": (120, 255), "g": (  0,  80), "b": (  0,  80)},  # red bin
#     "yellow": {"r": (120, 255), "g": (120, 255), "b": (  0,  80)},  # charger
# }

# # ─────────────────────────────────────────────────────────────────────────────
# #  Wander temporal state  (timestamp + current turn sign only — no position)
# # ─────────────────────────────────────────────────────────────────────────────
# _wander_next_turn_time: float = time.time()
# _wander_turn_sign:      int   = 1

# # ─────────────────────────────────────────────────────────────────────────────
# #  Vision Helpers
# # ─────────────────────────────────────────────────────────────────────────────
# def _build_mask(image: np.ndarray, colour: str) -> np.ndarray:
#     """Return a boolean mask of pixels matching the given colour label."""
#     t = _COLOUR_RANGES[colour]
#     r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]
#     return (
#         (r >= t["r"][0]) & (r <= t["r"][1]) &
#         (g >= t["g"][0]) & (g <= t["g"][1]) &
#         (b >= t["b"][0]) & (b <= t["b"][1])
#     )


# def detect_blob(image: np.ndarray, colour: str) -> tuple[bool, float, float]:
#     """
#     Locate a colour blob in an image.

#     Returns
#     -------
#     found    : bool   — True if a significant blob was detected.
#     x_offset : float  — Normalised horizontal centroid in [-1, 1];
#                         negative means the blob is left of frame centre.
#     coverage : float  — Fraction of image pixels matching the colour.
#     """
#     mask     = _build_mask(image, colour)
#     H, W     = mask.shape
#     n        = int(np.sum(mask))
#     coverage = n / (H * W)

#     if coverage < BLOB_MIN_COVERAGE:
#         return False, 0.0, 0.0

#     centroid_x = float(np.mean(np.where(mask)[1]))
#     x_offset   = (centroid_x - W / 2.0) / (W / 2.0)   # normalise to [-1, 1]
#     return True, x_offset, coverage


# def steer_toward(x_offset: float, base: float = BASE_SPEED) -> tuple[float, float]:
#     """
#     Convert a horizontal blob offset to differential drive motor commands.

#     Positive x_offset (blob right of centre) → increase left / decrease right
#     → robot turns right to re-centre the blob.

#     Returns (left_speed, right_speed).
#     """
#     correction  = x_offset * TURN_SPEED
#     left_speed  = base + correction
#     right_speed = base - correction
#     return left_speed, right_speed


# # ─────────────────────────────────────────────────────────────────────────────
# #  Subsumption Behaviour Functions
# #  Convention: return (left, right) float tuple when triggered, else None.
# # ─────────────────────────────────────────────────────────────────────────────

# def level4_recharge(battery: float, top_img: np.ndarray) -> tuple[float, float] | None:
#     """
#     LEVEL 4 — Recharge  (highest priority)
#     ----------------------------------------
#     Trigger  : battery < BATTERY_LOW_THRESHOLD %.
#     Behaviour: Steer toward the yellow charger marker if visible in the top
#                camera; otherwise rotate in place to scan the arena for it.
#     Suppresses: Levels 3, 2, 1.
#     """
#     if battery >= BATTERY_LOW_THRESHOLD:
#         return None

#     found, x_offset, _ = detect_blob(top_img, "yellow")
#     if found:
#         # Charger visible — approach it
#         return steer_toward(x_offset, base=SLOW_SPEED)
#     else:
#         # Charger not visible — spin in place to scan
#         return (-TURN_SPEED, TURN_SPEED)


# def level3_sort_and_compact(
#     bumper:    tuple[float, float, float],
#     top_img:   np.ndarray,
#     small_img: np.ndarray,
# ) -> tuple[float, float] | None:
#     """
#     LEVEL 3 — Sort and Compact
#     ---------------------------
#     Trigger  : max bumper force > BUMPER_FORCE_THRESHOLD (actively pushing something).
#     Behaviour:
#       • Green Plant — seek Blue Bin via top camera and push object toward it.
#       • Brown Trash — call robot.compress() to actuate arm joints, then seek
#                       Red Bin and push toward it.
#       • Unknown     — creep forward until object can be identified.
#     Suppresses: Levels 2, 1.
#     """
#     print(bumper[2])
#     if bumper[2] < BUMPER_FORCE_THRESHOLD:
#         return None

#     # Identify the object being pushed using the small (front-facing) camera
#     green_found, _, _ = detect_blob(small_img, "green")
#     brown_found, _, _ = detect_blob(small_img, "brown")

#     if brown_found:
#         robot.compress()          # actuate arm joints every cycle while pushing
#         target_bin_colour = "red"
#     elif green_found:
#         target_bin_colour = "blue"
#     else:
#         # Cannot identify yet — push very slowly and wait for better view
#         return (SLOW_SPEED * 0.5, SLOW_SPEED * 0.5)

#     # Steer toward the target bin using the top (overview) camera
#     bin_found, x_offset, _ = detect_blob(top_img, target_bin_colour)
#     if bin_found:
#         return steer_toward(x_offset, base=SLOW_SPEED)
#     else:
#         # Bin not yet visible — rotate in place while maintaining slight
#         # forward bias so contact is not lost with the object
#         return (-SLOW_SPEED, SLOW_SPEED)


# def level2_seek_object(top_img: np.ndarray) -> tuple[float, float] | None:
#     """
#     LEVEL 2 — Seek Object
#     ----------------------
#     Trigger  : Top camera detects a green or brown blob.
#     Behaviour: Differential steering to centre the blob horizontally and
#                drive toward it. Green plants take priority over brown trash
#                (green blobs are smaller/faster to move and time-critical).
#     Suppresses: Level 1.
#     """
#     global BLOB_FOUND
#     green_found, x_green, _ = detect_blob(top_img, "green")
#     brown_found, x_brown, _ = detect_blob(top_img, "brown")

#     if green_found:
#         print("Greeen")
#         BLOB_FOUND = True
#         return steer_toward(x_green, base=BASE_SPEED)
#     if brown_found:
#         print("Brownnn")
#         BLOB_FOUND = True
#         return steer_toward(x_brown, base=BASE_SPEED)
#     return None


# def level1_wander() -> tuple[float, float]:
#     """
#     LEVEL 1 — Wander  (always active, lowest behavioural priority)
#     ---------------------------------------------------------------
#     Trigger  : Always (returned when no higher level is active).
#     Behaviour: Drive forward. Every WANDER_INTERVAL seconds, pick a new random
#                turn direction and apply it briefly to ensure full arena coverage.

#     Note: _wander_next_turn_time and _wander_turn_sign are the only non-sensor
#     state in the entire architecture. They encode *when to turn next* — a
#     temporal scheduling decision — not the robot's location or history.
#     """
#     global _wander_next_turn_time, _wander_turn_sign, BLOB_FOUND
#     print("Wnder")
#     now = time.time()
#     if now >= _wander_next_turn_time:
#         _wander_turn_sign      = random.choice([-1, 1])
#         _wander_next_turn_time = now + WANDER_INTERVAL

#     # Small yaw correction injected on top of forward cruise
#     yaw_bias = _wander_turn_sign * TURN_SPEED * 0.4
#     return (BASE_SPEED + yaw_bias, BASE_SPEED - yaw_bias)


# def level0_survive(
#     sonar: float,
#     cmd:   tuple[float, float],
# ) -> tuple[float, float]:
#     """
#     LEVEL 0 — Survive (Obstacle Avoidance)
#     ----------------------------------------
#     This is NOT part of the priority cascade — it intercepts the *final*
#     motor command unconditionally, after all other levels have resolved.

#     Trigger  : sonar reading is positive (obstacle detected) and < SONAR_DANGER_DIST.
#                sonar == -1 means no object in range; safe to pass cmd through.
#     Behaviour: Reverse straight to create clearance.  On the next cycle the
#                obstacle will be gone and higher-level behaviours resume normally.
#                A slight asymmetry (right motor faster in reverse) biases the
#                escape to the robot's left to avoid re-entering the same corner.

#     Override : Replaces any command from Levels 4–1 for the duration of the
#                obstacle condition.
#     """
#     # sonar returns -1 when no object is detected (see Robot_OS.get_sonar_sensor)
#     obstacle_present = (sonar >= 0) and (sonar < SONAR_DANGER_DIST) and BLOB_FOUND == False

#     if not obstacle_present:
#         return cmd   # pass through unchanged

#     # Reverse with a left-biased turn to escape the obstacle
#     return (-BASE_SPEED * 0.8, -BASE_SPEED * 1.2)


# # ─────────────────────────────────────────────────────────────────────────────
# #  Arbitration  — the heart of the Subsumption Architecture
# # ─────────────────────────────────────────────────────────────────────────────

# def arbitrate() -> tuple[float, float]:
#     """
#     Single control cycle.

#     1. Refresh all sensors exactly once.
#     2. Evaluate behaviours in descending priority order (4 → 1).
#        The first non-None return wins; all lower levels are suppressed.
#     3. Pass the winning command through Level 0 (Survive) which may
#        override it if an obstacle is detected.
#     4. Return final (left_speed, right_speed).

#     Visual summary of suppression:

#       ┌─────────────────────────────────┐
#       │  L4: Recharge                   │  ← suppresses 3, 2, 1
#       │    └─ L3: Sort_and_Compact      │  ← suppresses 2, 1
#       │         └─ L2: Seek_Object      │  ← suppresses 1
#       │              └─ L1: Wander      │
#       └──────────────┬──────────────────┘
#                      │  final cmd
#       ┌──────────────▼──────────────────┐
#       │  L0: Survive  (always intercepts)│
#       └─────────────────────────────────┘
#     """
#     global BLOB_FOUND
#     # ── 1. Sensor refresh ────────────────────────────────────────────────────
#     top_image_sensor._update_image()
#     small_image_sensor._update_image()

#     battery   = robot.get_battery()
#     bumper    = robot.get_bumper_sensor()
#     # print(bumper)
#     sonar     = robot.get_sonar_sensor()
#     top_img   = top_image_sensor.get_image()
#     small_img = small_image_sensor.get_image()
#     BLOB_FOUND = False
#     # ── 2. Priority cascade ───────────────────────────────────────────────────
#     cmd = level4_recharge(battery, top_img)

#     if cmd is None:
#         cmd = level3_sort_and_compact(bumper, top_img, small_img)

#     if cmd is None:
#         cmd = level2_seek_object(top_img)

#     if cmd is None:
#         cmd = level1_wander()

#     # ── 3. Level 0 intercept ─────────────────────────────────────────────────
#     cmd = level0_survive(sonar, cmd)

#     return cmd


# # ─────────────────────────────────────────────────────────────────────────────
# #  Entry Point
# # ─────────────────────────────────────────────────────────────────────────────
# sim.startSimulation()
# time.sleep(0.5)   # allow physics to settle before first sensor read

# print("[Wall-E] Simulation started — entering subsumption control loop.")
# print(f"         Thresholds: battery={BATTERY_LOW_THRESHOLD}%  "
#       f"bumper={BUMPER_FORCE_THRESHOLD}N  sonar={SONAR_DANGER_DIST}m")


# while True:
#     left_speed, right_speed = arbitrate()
#     left_motor.run(speed=left_speed)
#     right_motor.run(speed=right_speed)
#     time.sleep(0.01)
#     # wait(1)

"""
wall_e_script.py — Subsumption Architecture Controller for the Object Sorting Task
====================================================================================
"""

from robots import *
import time
import random
import numpy as np
from coppeliasim_zmqremoteapi_client import *

# ─────────────────────────────────────────────────────────────────────────────
#  CoppeliaSim connection
# ─────────────────────────────────────────────────────────────────────────────
client = RemoteAPIClient()
sim    = client.require("sim")

# ─────────────────────────────────────────────────────────────────────────────
#  Actuators & Sensors
# ─────────────────────────────────────────────────────────────────────────────
robot              = Robot_OS(sim, DeviceNames.ROBOT_OS)
top_image_sensor   = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
small_image_sensor = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)
left_motor         = Motor(sim, DeviceNames.MOTOR_LEFT_OS,  Direction.CLOCKWISE)
right_motor        = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)

# ─────────────────────────────────────────────────────────────────────────────
#  Tunable Constants
# ─────────────────────────────────────────────────────────────────────────────
BATTERY_LOW_THRESHOLD  = 0.3  # % — triggers Recharge
BUMPER_FORCE_THRESHOLD = 0.14    # N — triggers Sort_and_Compact
SONAR_DANGER_DIST      = 0.4    # m — triggers Survive (−1 means no reading)
BLOB_MIN_COVERAGE      = 0.005  # fraction of pixels — minimum blob size to act on
BLOB_MIN_COVERAGE_BLACK= 0.3  # fraction of pixels — minimum blob size to act on
BLACK_FOUND_TURN_REDUCTION = 0
BIN_COVERAGE           = 0.4
BLACK_APPROACH_BIAS = 0.5
WALL_COVERAGE           = 0.25
BLACK_MIN_COVERAGE     = 0.0025
BLACK_MAX_COVERAGE     = 0.15
BLOB_FOUND             = False
BASE_SPEED             = 3.0    # rad/s forward cruise
SLOW_SPEED             = 1.5    # rad/s push / approach speed
SLOW_TURN_SPEED        = 1.5    # rad/s push / approach speed
TURN_SPEED             = 2.0    # rad/s differential correction amplitude
TURN_SPEED_BAT             = 2.1    # rad/s differential correction amplitude
WANDER_INTERVAL        = 4.0    # seconds between random wander yaw injections

_wander_next_turn_time: float = time.time()
_wander_turn_sign:      int   = 1
# ─────────────────────────────────────────────────────────────────────────────
#  Pixel colour ranges (uint8, 0–255) keyed by semantic label
# ─────────────────────────────────────────────────────────────────────────────
_COLOUR_RANGES: dict[str, dict[str, tuple[int, int]]] = {
    "green":  {"r": (  0,  80), "g": (100, 255), "b": (  0,  80)},
    "brown":  {"r": ( 80, 200), "g": (  30,  80), "b": (  0,  60)},
    # "brown":  {"r": ( 80, 150), "g": ( 40, 100), "b": (  20,  60)},
    "black":  {"r": ( 0, 30), "g":  (0, 30), "b":  (0, 30)},
    "blue":   {"r": (  0,  80), "g": (  0,  140), "b": (100, 255)},
    "red":    {"r": (100,255), "g": (  0,  30), "b": (  0,  30)},
    "yellow": {"r": (120, 255), "g": (120, 255), "b": (  0,  80)},
    "grey": {"r": (120, 190), "g": (120, 190), "b": (120, 190)},
}



# ─────────────────────────────────────────────────────────────────────────────
#  Vision Helpers
# ─────────────────────────────────────────────────────────────────────────────
def _build_mask(image: np.ndarray, colour: str) -> np.ndarray:
    t = _COLOUR_RANGES[colour]
    r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    return (
        (r >= t["r"][0]) & (r <= t["r"][1]) &
        (g >= t["g"][0]) & (g <= t["g"][1]) &
        (b >= t["b"][0]) & (b <= t["b"][1])
    )

def detect_blob(image: np.ndarray, colour: str) -> tuple[bool, float, float]:
    mask     = _build_mask(image, colour)
    H, W     = mask.shape
    n        = int(np.sum(mask))
    coverage = n / (H * W)

    if coverage < BLOB_MIN_COVERAGE:
        return False, 0.0, 0.0

    centroid_x = float(np.mean(np.where(mask)[1]))
    x_offset   = (centroid_x - W / 2.0) / (W / 2.0)
    return True, x_offset, coverage

def detect_void(image: np.ndarray, colour: str) -> tuple[bool, float, float]:
    mask     = _build_mask(image, colour)
    H, W     = mask.shape
    n        = int(np.sum(mask))
    coverage = n / (H * W)

    if coverage < BLACK_MIN_COVERAGE or coverage > BLACK_MAX_COVERAGE:
        return False, 0.0, 0.0

    centroid_x = float(np.mean(np.where(mask)[1]))
    x_offset   = (centroid_x - W / 2.0) / (W / 2.0)
    return True, x_offset, coverage

def detect_blob_black(image: np.ndarray, colour: str) -> tuple[bool, float, float]:
    mask     = _build_mask(image, colour)
    H, W     = mask.shape
    n        = int(np.sum(mask))
    coverage = n / (H * W)

    if coverage < BLOB_MIN_COVERAGE_BLACK - BLACK_FOUND_TURN_REDUCTION:
        return False, 0.0, 0.0

    centroid_x = float(np.mean(np.where(mask)[1]))
    x_offset   = (centroid_x - W / 2.0) / (W / 2.0)
    return True, x_offset, coverage

def detect_bin(image: np.ndarray, colour: str) -> tuple[bool, float, float]:
    mask     = _build_mask(image, colour)
    H, W     = mask.shape
    n        = int(np.sum(mask))
    coverage = n / (H * W)

    if coverage < BIN_COVERAGE:
        return False, 0.0, 0.0

    centroid_x = float(np.mean(np.where(mask)[1]))
    x_offset   = (centroid_x - W / 2.0) / (W / 2.0)
    return True, x_offset, coverage

def detect_wall(image: np.ndarray, colour: str) -> tuple[bool, float, float]:
    mask     = _build_mask(image, colour)
    H, W     = mask.shape
    n        = int(np.sum(mask))
    coverage = n / (H * W)

    if coverage < WALL_COVERAGE:
        return False, 0.0, 0.0

    centroid_x = float(np.mean(np.where(mask)[1]))
    x_offset   = (centroid_x - W / 2.0) / (W / 2.0)
    return True, x_offset, coverage

def steer_toward(x_offset: float, base: float = BASE_SPEED) -> tuple[float, float]:
    correction  = x_offset * TURN_SPEED
    if abs(x_offset) < 0.15:
        return base, base
    left_speed  = base + correction
    right_speed = base - correction
    return left_speed, right_speed

# ─────────────────────────────────────────────────────────────────────────────
#  Subsumption Behaviour Functions
# ─────────────────────────────────────────────────────────────────────────────
def level4_recharge(battery: float, top_img: np.ndarray) -> tuple[float, float] | None:
    if battery >= BATTERY_LOW_THRESHOLD:
        return None
    found, x_offset, _ = detect_bin(top_img, "yellow")
    if found:
        return steer_toward(x_offset, base=SLOW_SPEED)
    return (-TURN_SPEED_BAT, TURN_SPEED_BAT)

def level3_sort_and_compact(
    sonar:    tuple, # Type hint broadened to handle unexpected CoppeliaSim returns
    top_img:   np.ndarray,
    small_img: np.ndarray,
) -> tuple[float, float] | None:
    global BLOB_FOUND, BLACK_FOUND_TURN_REDUCTION
    blue_found, _, _ = detect_bin(top_img, 'blue')
    red_found, _, _ = detect_bin(top_img, "red")
    wall_found, _, _ = detect_wall(top_img, "grey")
    black_found, _, _ = detect_blob_black(top_img[int(len(top_img) *0.95):], "black")
    if sonar > 0.165 and black_found == False and sonar < 0.5:
        return None

    green_found, _, _ = detect_blob(small_img, "green")
    brown_found, _, _ = detect_blob(small_img, "brown")
    print(f"sonar : {sonar}, green: {green_found}, brown: {brown_found}, black: {black_found}")

    if green_found:
        print("Green")
        BLOB_FOUND = True
        if wall_found:
            return (-SLOW_TURN_SPEED, SLOW_TURN_SPEED)
        target_bin_colour = "blue"
    elif black_found :
        print("Black")
        BLOB_FOUND = True
        if wall_found:
            return (-SLOW_TURN_SPEED, SLOW_TURN_SPEED)
        BLACK_FOUND_TURN_REDUCTION = 0
        target_bin_colour = "red"
    elif brown_found:
        print("Brown")
        BLOB_FOUND = True
        robot.compress()
        return (-TURN_SPEED, TURN_SPEED)
    # if black_found:
    else:
        return (SLOW_SPEED * 0.5, SLOW_SPEED * 0.5)

    bin_found, x_offset, _ = detect_blob(top_img[:int(len(top_img) *0.8)], target_bin_colour)
    if bin_found:
        print("Going to ", target_bin_colour)
        return steer_toward(x_offset, base=BASE_SPEED)
    BLACK_FOUND_TURN_REDUCTION = 0
    # BLACK_FOUND_TURN_REDUCTION = 0.75
    return (-SLOW_TURN_SPEED, SLOW_TURN_SPEED * 1.3)

def level2_seek_object(top_img: np.ndarray, small_img: np.ndarray, sonar: tuple) -> tuple[float, float] | None:
    global BLOB_FOUND
    green_found, x_green, _ = detect_blob(top_img, "green")
    brown_found, x_brown, _ = detect_blob(top_img, "brown")
    blue_found, _, _ = detect_bin(top_img, 'blue')
    red_found, _, _ = detect_bin(top_img, "red")
    wall_found, _, _ = detect_wall(top_img, "grey")
    # black_found, x_black, _ = detect_blob(small_img[(int(len(top_img) / 2)): ], "black")
    black_found, x_black, _ = detect_void(top_img[int(len(top_img) *0.3 ):], "black")

    if green_found:
        BLOB_FOUND = True
        return steer_toward(x_green, base=BASE_SPEED)
    if black_found and sonar < 0.5 and not (blue_found or red_found or wall_found):
        BLOB_FOUND = True
        print("Black--")
        biased_x_black = x_black + BLACK_APPROACH_BIAS

        return steer_toward(biased_x_black, base=BASE_SPEED)
    if brown_found:
        BLOB_FOUND = True
        return steer_toward(x_brown, base=BASE_SPEED)
    # if black_found and sonar < 0.5 and not (blue_found or red_found):

    return None

def level1_wander() -> tuple[float, float]:
    global _wander_next_turn_time, _wander_turn_sign
    print("=====Wander=====")
    now = time.time()
    if now >= _wander_next_turn_time:
        _wander_turn_sign      = random.choice([-1, 1])
        _wander_next_turn_time = now + WANDER_INTERVAL

    yaw_bias = _wander_turn_sign * TURN_SPEED * 0.4
    return (BASE_SPEED + yaw_bias, BASE_SPEED - yaw_bias)

def level0_survive(sonar: float, cmd: tuple[float, float], top_img: np.ndarray) -> tuple[float, float]:
    blue_found, _, _ = detect_bin(top_img, 'blue')
    red_found, _, _ = detect_bin(top_img, "red")
    wall_found, _, _ = detect_wall(top_img, "grey")
    black_found, _, _ = detect_void(top_img[int(len(top_img) *0.3 ):], "black")
    obstacle_present = (sonar >= 0) and (sonar < SONAR_DANGER_DIST)  and not BLOB_FOUND and (blue_found or red_found or wall_found) or (black_found and sonar > 0.5) or sonar > 0.55
    if not obstacle_present:
        return cmd
    print("=====Avoid=====")
    print(f"Sonar : {sonar}, BLOB: {BLOB_FOUND}, Blue Bin: {blue_found}, Red Bin: {red_found}, Wall: {wall_found}")
    return (-BASE_SPEED * 1.5, BASE_SPEED * -1.5)

# ─────────────────────────────────────────────────────────────────────────────
#  Arbitration
# ─────────────────────────────────────────────────────────────────────────────
def arbitrate() -> tuple[float, float]:
    global BLOB_FOUND, _last_log_time

    top_image_sensor._update_image()
    small_image_sensor._update_image()

    battery   = robot.get_battery()
    bumper    = robot.get_bumper_sensor()
    sonar     = robot.get_sonar_sensor()
    top_img   = top_image_sensor.get_image()
    small_img = small_image_sensor.get_image()
    BLOB_FOUND = False

    # ---------------------

    cmd = level4_recharge(battery, top_img)
    if cmd is None:
        cmd = level3_sort_and_compact(sonar, top_img, small_img)
    if cmd is None:
        cmd = level2_seek_object(top_img, small_img, sonar)
    if cmd is None:
        cmd = level1_wander()

    cmd = level0_survive(sonar, cmd, top_img)
    return cmd

# ─────────────────────────────────────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────────────────────────────────────
sim.startSimulation()
time.sleep(0.5)

print("[Wall-E] Simulation started — entering subsumption control loop.")

while True:
    left_speed, right_speed = arbitrate()
    left_motor.run(speed=left_speed)
    right_motor.run(speed=right_speed)
    if left_speed == -BASE_SPEED * 1.5 and right_speed == -BASE_SPEED * 1.5:
        time.sleep(0.3)
        left_speed = -BASE_SPEED * 1.5
        right_speed = BASE_SPEED * 1.5
        left_motor.run(speed=left_speed)
        right_motor.run(speed=right_speed)
        time.sleep(0.2)
    # if left_speed == -TURN_SPEED_BAT and right_speed == TURN_SPEED_BAT:
    #     time.sleep(0.1)
    #     left_speed = -BASE_SPEED * 1.5
    #     right_speed = -BASE_SPEED * 1.5
    #     left_motor.run(speed=left_speed)
    #     right_motor.run(speed=right_speed)
    #     time.sleep(0.2)
    time.sleep(0.01)
