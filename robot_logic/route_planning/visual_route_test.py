import argparse
from math import cos, radians, sin
from pathlib import Path
import sys

import cv2

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from Object_Tracking.Course_detecter import find_arena
from Object_Tracking.Object_Tracking import find_objects_in_image, px_to_world_cm, world_cm_to_px
from robot_logic.navigation_config import GOAL_A_X_CM, GOAL_A_Y_CM, GOAL_B_X_CM, GOAL_B_Y_CM, WARP_H, WARP_W
from robot_logic.robot_detection.aruco_robot_detector import detect_robot_pose
from robot_logic.route_planning.obstacles import CrossObstacle, build_cross_obstacle
from robot_logic.route_planning.pathfinder import plan_path, smooth_path
from robot_logic.route_planning.route_planner import (
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
    Ball,
    Goal,
    Point,
    choose_route,
    route_quadrant_order,
)


IMAGE_PATH = "Images/captured_image_12.jpg"
OUTPUT_PATH = "robot_logic/route_planning/visual_route_output.jpg"
ROBOT_HEADING_ARROW_CM = 12

# Goal A is fixed at the middle of the outermost right wall.
LEFT_GOAL_POSITION = Point(GOAL_B_X_CM, GOAL_B_Y_CM)
RIGHT_GOAL_POSITION = Point(GOAL_A_X_CM, GOAL_A_Y_CM)
GOAL_A_POSITION = RIGHT_GOAL_POSITION


def to_pixel_point(point: Point) -> tuple[int, int]:
    return world_cm_to_px(point.x, point.y, WARP_W, WARP_H)


def draw_label(image, text: str, point: Point, color: tuple[int, int, int]) -> None:
    x_px, y_px = to_pixel_point(point)
    cv2.putText(
        image,
        text,
        (x_px + 6, y_px - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1,
        cv2.LINE_AA,
    )


def detection_to_point(detection) -> Point:
    if len(detection) >= 4:
        x_px, y_px = detection[2], detection[3]
        x_cm, y_cm = px_to_world_cm(x_px, y_px, WARP_W, WARP_H)
        return Point(x_cm, y_cm)

    return Point(detection[0], detection[1])


def build_balls(orange_balls, white_balls) -> list[Ball]:
    balls: list[Ball] = []

    for index, detection in enumerate(orange_balls):
        balls.append(
            Ball(
                name=f"O{index}",
                position=detection_to_point(detection),
                is_vip=(index == 0),
            )
        )

    for index, detection in enumerate(white_balls):
        balls.append(
            Ball(
                name=f"W{index}",
                position=detection_to_point(detection),
                is_vip=False,
            )
        )

    return balls


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("image_path", nargs="?", default=IMAGE_PATH)
    return parser.parse_args()


def unpack_detections(detection_result):
    if detection_result is None:
        raise RuntimeError("Object detection returned no result")

    if len(detection_result) == 3:
        orange_balls, white_balls, cross_position = detection_result
        return orange_balls, white_balls, cross_position

    if len(detection_result) == 5:
        orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position = detection_result
        return orange_balls + dark_orange_balls, white_balls + shadowywhite_balls, cross_position

    if len(detection_result) >= 9:
        orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position = detection_result[:5]
        return (orange_balls or []) + (dark_orange_balls or []), (white_balls or []) + (shadowywhite_balls or []), cross_position

    raise RuntimeError(f"Unexpected detection result format: {len(detection_result)} values")


def get_cross_point(cross_position) -> Point | None:
    if cross_position is None:
        return None

    if isinstance(cross_position, dict):
        if "center" not in cross_position or cross_position["center"] is None:
            return None
        center_x, center_y = cross_position["center"]
        world_x_cm, world_y_cm = px_to_world_cm(center_x, center_y, WARP_W, WARP_H)
        return Point(world_x_cm, world_y_cm)

    if len(cross_position) >= 2:
        return Point(cross_position[0], cross_position[1])

    return None


def cm_radius_to_px_axes(center: Point, radius_cm: float) -> tuple[int, int]:
    center_x_px, center_y_px = to_pixel_point(center)
    edge_x_cm = min(center.x + radius_cm, FIELD_WIDTH_CM)
    edge_y_cm = min(center.y + radius_cm, FIELD_HEIGHT_CM)
    edge_x_px, _ = to_pixel_point(Point(edge_x_cm, center.y))
    _, edge_y_px = to_pixel_point(Point(center.x, edge_y_cm))
    return max(1, abs(edge_x_px - center_x_px)), max(1, abs(edge_y_px - center_y_px))


def same_point(point_a: Point, point_b: Point) -> bool:
    return abs(point_a.x - point_b.x) < 0.01 and abs(point_a.y - point_b.y) < 0.01


def draw_cross_obstacle(image, cross_position, cross_obstacle: CrossObstacle | None) -> None:
    if isinstance(cross_position, dict):
        if "vertical_box" in cross_position:
            cv2.drawContours(image, [cross_position["vertical_box"]], 0, (0, 0, 180), 2)
        if "horizontal_box" in cross_position:
            cv2.drawContours(image, [cross_position["horizontal_box"]], 0, (0, 0, 220), 2)

    if cross_obstacle is None:
        return

    for arm in cross_obstacle.arms:
        margin = cross_obstacle.safety_margin_cm
        corner_a = to_pixel_point(Point(arm.min_x - margin, arm.max_y + margin))
        corner_b = to_pixel_point(Point(arm.max_x + margin, arm.min_y - margin))
        x1, y1 = corner_a
        x2, y2 = corner_b
        cv2.rectangle(image, (min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2)), (0, 0, 255), 2)

    if cross_obstacle.fallback_center is not None:
        center_px = to_pixel_point(cross_obstacle.fallback_center)
        cv2.circle(image, center_px, 5, (0, 0, 255), -1)
        if not cross_obstacle.arms:
            cross_radius_px = cm_radius_to_px_axes(cross_obstacle.fallback_center, cross_obstacle.fallback_radius_cm)
            cv2.ellipse(image, center_px, cross_radius_px, 0, 0, 360, (0, 0, 255), 2)
        draw_label(image, "Cross", cross_obstacle.fallback_center, (0, 0, 255))


if __name__ == "__main__":
    arguments = parse_arguments()
    image_path = arguments.image_path
    print(f"Using image path: {image_path}")

    raw_image = cv2.imread(image_path)
    if raw_image is None:
        raise FileNotFoundError(f"Could not load image: {image_path}")

    warped_image = find_arena(raw_image, WARP_W, WARP_H)
    if warped_image is None or not hasattr(warped_image, "shape"):
        raise RuntimeError("Could not create warped arena image")

    detection_result = find_objects_in_image(raw_image, WARP_W, WARP_H)
    orange_balls, white_balls, cross_position = unpack_detections(detection_result)
    balls = build_balls(orange_balls, white_balls)

    if not balls:
        raise RuntimeError("No balls detected. Cannot plan route.")

    robot_pose = detect_robot_pose(raw_image, warp_w_px=WARP_W, warp_h_px=WARP_H)

    if robot_pose is None:
        raise RuntimeError("Robot marker not detected. Cannot plan route without robot position.")

    goal_a = Goal(name="Goal A", position=GOAL_A_POSITION)
    cross_obstacle = build_cross_obstacle(cross_position, WARP_W, WARP_H)
    planned_route = choose_route(robot_pose.position, balls, goal_a, cross_obstacle=cross_obstacle)
    raw_path_points: list[Point] = []
    simplified_path_points: list[Point] = []
    failed_astar_segments = 0
    current_point = robot_pose.position

    def extend_path(total_path: list[Point], segment_path: list[Point]) -> None:
        if total_path:
            total_path.extend(segment_path[1:])
        else:
            total_path.extend(segment_path)

    for target in planned_route:
        blocked_points = [other.ball.position for other in planned_route if not other.ball.is_vip] if target.ball.is_vip else None
        segment_path = plan_path(
            current_point,
            target.pickup_point,
            cross_obstacle=cross_obstacle,
            blocked_points=blocked_points,
        )

        if not segment_path:
            failed_astar_segments += 1
            current_point = target.pickup_point
            continue

        simplified_segment = smooth_path(
            segment_path,
            cross_obstacle=cross_obstacle,
            blocked_points=blocked_points,
        )
        extend_path(raw_path_points, segment_path)
        extend_path(simplified_path_points, simplified_segment)

        current_point = target.pickup_point

    goal_path = plan_path(current_point, goal_a.position, cross_obstacle=cross_obstacle)
    if not goal_path:
        failed_astar_segments += 1
    else:
        simplified_goal_path = smooth_path(goal_path, cross_obstacle=cross_obstacle)
        extend_path(raw_path_points, goal_path)
        extend_path(simplified_path_points, simplified_goal_path)

    robot_heading_radians = radians(robot_pose.heading_degrees)
    heading_end_point = Point(
        robot_pose.position.x + ROBOT_HEADING_ARROW_CM * cos(robot_heading_radians),
        robot_pose.position.y + ROBOT_HEADING_ARROW_CM * sin(robot_heading_radians),
    )

    cv2.circle(warped_image, to_pixel_point(robot_pose.position), 6, (255, 0, 0), -1)
    cv2.arrowedLine(
        warped_image,
        to_pixel_point(robot_pose.position),
        to_pixel_point(heading_end_point),
        (255, 0, 0),
        2,
        tipLength=0.25,
    )
    draw_label(warped_image, "Robot", robot_pose.position, (255, 0, 0))

    cv2.circle(warped_image, to_pixel_point(goal_a.position), 8, (0, 200, 255), 2)
    draw_label(warped_image, goal_a.name, goal_a.position, (0, 200, 255))

    draw_cross_obstacle(warped_image, cross_position, cross_obstacle)

    for ball in balls:
        ball_color = (0, 140, 255) if ball.is_vip else (255, 255, 255)
        cv2.circle(warped_image, to_pixel_point(ball.position), 5, ball_color, -1)
        draw_label(warped_image, ball.name, ball.position, ball_color)

    print(f"Orange balls detected: {len(orange_balls)}")
    print(f"White balls detected: {len(white_balls)}")
    print(f"Total route targets: {len(planned_route)}")
    print("All detections are treated as real route targets")
    print(f"Quadrant order used: {route_quadrant_order(planned_route)}")
    print(f"Cross position: {cross_position}")
    print(
        f"Robot pose: ({robot_pose.position.x:.1f}, {robot_pose.position.y:.1f}) | "
        f"heading={robot_pose.heading_degrees:.1f} deg | marker={robot_pose.marker_id}"
    )
    print("Planned route:")

    for index, target in enumerate(planned_route, start=1):
        pickup_point = target.pickup_point
        ball_point = target.ball.position

        print(
            f"{index}. {target.ball.name} | VIP={target.ball.is_vip} | "
            f"pickup=({pickup_point.x:.1f}, {pickup_point.y:.1f}) | "
            f"face={target.face_direction} | wall={target.is_wall_pickup}"
        )

        cv2.circle(warped_image, to_pixel_point(pickup_point), 6, (0, 255, 0), 2)
        draw_label(warped_image, str(index), pickup_point, (0, 255, 0))
        cv2.arrowedLine(
            warped_image,
            to_pixel_point(pickup_point),
            to_pixel_point(ball_point),
            (0, 255, 255),
            1,
            tipLength=0.25,
        )

    for start_point, end_point in zip(simplified_path_points, simplified_path_points[1:]):
        cv2.line(warped_image, to_pixel_point(start_point), to_pixel_point(end_point), (0, 255, 0), 2)

    print("A* pathfinding used: yes")
    print(f"Raw A* path points created: {len(raw_path_points)}")
    print(f"Simplified waypoint count: {len(simplified_path_points)}")
    print(f"A* failed segments: {failed_astar_segments}")

    cv2.imwrite(OUTPUT_PATH, warped_image)
    print(f"Output path: {OUTPUT_PATH}")
