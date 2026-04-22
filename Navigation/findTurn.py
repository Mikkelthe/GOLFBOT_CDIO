import math

import numpy as np


def find_turn(current_heading, point_x, point_y):
    direction_radian = np.atan2(point_y.y - point_x.y, point_y.x - point_x.x)
    target_direction = round(math.degrees(direction_radian))
    turn_flag = ""
    delta_direction = target_direction - current_heading

    # Normalize to [-180, 180]
    delta = (delta_direction + 180) % 360 - 180

    if delta > 0:
        turn_flag = "right"
        turn_angle = delta  # Degrees to turn
    elif delta < 0:
        turn_flag = "left"
        turn_angle = -delta  # Absolute value for magnitude
    else:
        turn_flag = "none"
        turn_angle = 0

    return turn_flag, turn_angle