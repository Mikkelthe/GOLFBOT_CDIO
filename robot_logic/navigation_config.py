"""Shared navigation geometry and tuning constants.

Keep this module free of Point/dataclass imports so planning, detection, and
demo code can all depend on it without circular imports.
"""

# Confirmed by Object_Tracking.Object_Tracking coordinate conversion.
FIELD_WIDTH_CM = 170.0
FIELD_HEIGHT_CM = 125.0

# Canonical planner warp size. Teammate conversion functions accept these as
# parameters, so robot_logic should pass the same values everywhere.
WARP_W = 1200
WARP_H = 800

# Goal positions are field setup constants. Goal A is the right-side goal used
# by the current planner demos; Goal B is the left-side counterpart.
GOAL_A_X_CM = FIELD_WIDTH_CM - 8.0
GOAL_A_Y_CM = FIELD_HEIGHT_CM / 2.0 + 2.0
GOAL_B_X_CM = 3.0
GOAL_B_Y_CM = FIELD_HEIGHT_CM / 2.0

# Tunable path/pickup safety values.
WALL_SAFETY_MARGIN_CM = 5.0
CROSS_SAFETY_MARGIN_CM = 6.0
CROSS_FALLBACK_RADIUS_CM = 8.0
EDGE_MARGIN_CM = 20.0
PICKUP_OFFSET_CM = 18.0
SAFE_PICKUP_MARGIN_CM = 3.0
WALL_PICKUP_MARGIN_CM = 2.0
PICKUP_APPROACH_DISTANCE_CM = PICKUP_OFFSET_CM

# Robot dimensions are conservative planning values. Navigation.py previously
# drew the robot with a 16 cm radius, so 32 cm is a reasonable starting width.
ROBOT_BODY_WIDTH_CM = 32.0
ROBOT_BODY_LENGTH_CM = 32.0
ROBOT_CLEARANCE_CM = 3.0

# Measured in Navigation/Navigation.py. Do not import that module because it
# runs image/camera/debug code at import time.
CAMERA_HEIGHT_CM = 195.0
ROBOT_MARKER_HEIGHT_CM = 46.0
APPLY_MARKER_PARALLAX_CORRECTION = True

# Current robot_logic ArUco heading calibration.
MARKER_TO_ROBOT_HEADING_OFFSET_DEGREES = 270.0

# Route search tuning.
MAX_EXACT_ROUTE_TARGETS = 7
TURN_PENALTY_CM = 3.0
ROUTE_STRATEGY = "vip_quadrant_then_sweep"
QUADRANT_SWEEP_ORDER = ("top_left", "bottom_left", "bottom_right")
ON_PATH_BALL_RADIUS_CM = 10.0
BALL_AVOID_RADIUS_CM = 7.0
PATH_SMOOTH_SAMPLE_STEP_CM = 2.5
WALL_SIDE_TIE_MARGIN_CM = 6.0
WALL_CLUSTER_DISTANCE_CM = 35.0
WALL_CLUSTER_ORDER_PENALTY_CM = 30.0
