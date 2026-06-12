import argparse
import contextlib
import io
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
import sys

import cv2

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from Object_Tracking.Course_detecter import find_arena
from Object_Tracking.Object_Tracking import find_objects_in_image, px_to_world_cm, world_cm_to_px
from robot_logic.navigation_config import GOAL_A_X_CM, GOAL_A_Y_CM, WARP_H, WARP_W
from robot_logic.robot_detection.aruco_robot_detector import RobotPose, detect_robot_pose
from robot_logic.route_planning.obstacles import CrossObstacle, build_cross_obstacle, straight_line_crosses_obstacle
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


IMAGE_DIR = Path("Images")
DEFAULT_IMAGE_PATH = IMAGE_DIR / "captured_image_12.jpg"
OUTPUT_PATH = Path("robot_logic/route_planning/planner_demo_output.jpg")
ROBOT_HEADING_ARROW_CM = 12

# Goal A is fixed at the middle of the outermost right wall.
GOAL_A_POSITION = Point(GOAL_A_X_CM, GOAL_A_Y_CM)


@dataclass
class ImageAnalysis:
    path: Path
    raw_image: object | None
    warped_image: object | None
    detected_orange_balls: list
    detected_white_balls: list
    orange_balls: list
    white_balls: list
    cross_position: object | None
    cross_point: Point | None
    cross_obstacle: CrossObstacle | None
    robot_pose: RobotPose | None
    balls: list[Ball]
    blocked_direct_segments: int
    score: int
    notes: list[str]


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("image_path", nargs="?", default=str(DEFAULT_IMAGE_PATH))
    return parser.parse_args()


def to_pixel_point(point: Point) -> tuple[int, int]:
    return world_cm_to_px(point.x, point.y, WARP_W, WARP_H)


def detection_to_point(detection) -> Point:
    if len(detection) >= 4:
        x_px, y_px = detection[2], detection[3]
        x_cm, y_cm = px_to_world_cm(x_px, y_px, WARP_W, WARP_H)
        return Point(x_cm, y_cm)

    return Point(detection[0], detection[1])


def unpack_detections(detection_result) -> tuple[list, list, object | None]:
    if detection_result is None:
        return [], [], None

    if len(detection_result) == 3:
        orange_balls, white_balls, cross_position = detection_result
        return orange_balls or [], white_balls or [], cross_position

    if len(detection_result) == 5:
        orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position = detection_result
        orange_candidates = (orange_balls or []) + (dark_orange_balls or [])
        white_candidates = (white_balls or []) + (shadowywhite_balls or [])
        return orange_candidates, white_candidates, cross_position

    if len(detection_result) >= 9:
        orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position = detection_result[:5]
        orange_candidates = (orange_balls or []) + (dark_orange_balls or [])
        white_candidates = (white_balls or []) + (shadowywhite_balls or [])
        return orange_candidates, white_candidates, cross_position

    return [], [], None


def get_cross_point(cross_position) -> Point | None:
    if cross_position is None:
        return None

    if isinstance(cross_position, dict):
        center = cross_position.get("center")
        if center is None:
            return None

        x_cm, y_cm = px_to_world_cm(center[0], center[1], WARP_W, WARP_H)
        return Point(x_cm, y_cm)

    if len(cross_position) >= 2:
        return Point(cross_position[0], cross_position[1])

    return None


def build_balls(orange_balls: list, white_balls: list) -> list[Ball]:
    balls: list[Ball] = []

    for index, detection in enumerate(orange_balls):
        balls.append(Ball(name=f"O{index}", position=detection_to_point(detection), is_vip=(index == 0)))

    for index, detection in enumerate(white_balls):
        balls.append(Ball(name=f"W{index}", position=detection_to_point(detection), is_vip=False))

    return balls


def make_warped_image(raw_image):
    warped_image = find_arena(raw_image, WARP_W, WARP_H)

    if isinstance(warped_image, tuple):
        warped_image = warped_image[0]

    if warped_image is None or not hasattr(warped_image, "shape"):
        return None

    return warped_image


def run_object_detection(raw_image) -> tuple[list, list, object | None]:
    # Teammate detection currently prints debug data; keep demo output readable.
    with contextlib.redirect_stdout(io.StringIO()):
        detection_result = find_objects_in_image(raw_image, WARP_W, WARP_H)

    orange_balls, white_balls, cross_position = unpack_detections(detection_result)
    return orange_balls, white_balls, cross_position


def score_analysis(
    warped_image,
    orange_balls: list,
    white_balls: list,
    cross_position,
    robot_pose: RobotPose | None,
    blocked_direct_segments: int,
) -> tuple[int, list[str]]:
    score = 0
    notes = []

    if warped_image is not None:
        score += 100
        notes.append("arena ok")
    else:
        notes.append("arena missing")

    if robot_pose is not None:
        score += 1000
        notes.append("robot ok")
    else:
        notes.append("robot missing")

    if orange_balls:
        score += 500 + len(orange_balls) * 25
        notes.append(f"{len(orange_balls)} orange")
    else:
        notes.append("0 orange")

    if white_balls:
        score += min(len(white_balls), 8) * 60
        notes.append(f"{len(white_balls)} white")
    else:
        notes.append("0 white")

    if cross_position is not None:
        score += 250
        notes.append("cross ok")
    else:
        notes.append("cross missing")

    if blocked_direct_segments:
        score += 350
        notes.append(f"{blocked_direct_segments} A* obstacle segments")
    else:
        notes.append("0 A* obstacle segments")

    return score, notes


def count_blocked_direct_segments(points: list[Point], cross_obstacle: CrossObstacle | None) -> int:
    if cross_obstacle is None:
        return 0

    return sum(
        1
        for start, end in zip(points, points[1:])
        if straight_line_crosses_obstacle(start, end, cross_obstacle)
    )


def analyze_image(path: Path) -> ImageAnalysis:
    raw_image = cv2.imread(str(path))
    if raw_image is None:
        return ImageAnalysis(path, None, None, [], [], [], [], None, None, None, None, [], 0, -1, ["cannot load image"])

    warped_image = make_warped_image(raw_image)
    detected_orange_balls, detected_white_balls, cross_position = run_object_detection(raw_image)
    cross_point = get_cross_point(cross_position)
    cross_obstacle = build_cross_obstacle(cross_position, WARP_W, WARP_H)
    orange_balls = detected_orange_balls
    white_balls = detected_white_balls
    robot_pose = detect_robot_pose(raw_image, warp_w_px=WARP_W, warp_h_px=WARP_H)
    balls = build_balls(orange_balls, white_balls)
    blocked_direct_segments = 0

    if robot_pose is not None and balls:
        goal_a = Goal(name="Goal A", position=GOAL_A_POSITION)
        route = choose_route(robot_pose.position, balls, goal_a, cross_obstacle=cross_obstacle)
        route_points = [robot_pose.position] + [target.pickup_point for target in route] + [goal_a.position]
        blocked_direct_segments = count_blocked_direct_segments(route_points, cross_obstacle)

    score, notes = score_analysis(
        warped_image,
        orange_balls,
        white_balls,
        cross_position,
        robot_pose,
        blocked_direct_segments,
    )

    return ImageAnalysis(
        path=path,
        raw_image=raw_image,
        warped_image=warped_image,
        detected_orange_balls=detected_orange_balls,
        detected_white_balls=detected_white_balls,
        orange_balls=orange_balls,
        white_balls=white_balls,
        cross_position=cross_position,
        cross_point=cross_point,
        cross_obstacle=cross_obstacle,
        robot_pose=robot_pose,
        balls=balls,
        blocked_direct_segments=blocked_direct_segments,
        score=score,
        notes=notes,
    )


def image_candidates() -> list[Path]:
    extensions = ["*.jpg", "*.jpeg", "*.png"]
    candidates: list[Path] = []

    for extension in extensions:
        candidates.extend(sorted(IMAGE_DIR.glob(extension)))

    return candidates


def choose_best_image() -> ImageAnalysis:
    candidates = image_candidates()
    if not candidates:
        raise RuntimeError("No candidate images found in Images/.")

    analyses = [analyze_image(path) for path in candidates]

    print("Candidate image scores:")
    for analysis in sorted(analyses, key=lambda item: item.score, reverse=True):
        print(f"- {analysis.path} | score={analysis.score} | {', '.join(analysis.notes)}")

    return max(analyses, key=lambda item: item.score)


def draw_label(image, text: str, point: Point, color: tuple[int, int, int], scale: float = 0.55) -> None:
    x_px, y_px = to_pixel_point(point)
    cv2.putText(image, text, (x_px + 6, y_px - 6), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def cm_radius_to_px_axes(center: Point, radius_cm: float) -> tuple[int, int]:
    center_x_px, center_y_px = to_pixel_point(center)
    edge_x_cm = min(center.x + radius_cm, FIELD_WIDTH_CM)
    edge_y_cm = min(center.y + radius_cm, FIELD_HEIGHT_CM)
    edge_x_px, _ = to_pixel_point(Point(edge_x_cm, center.y))
    _, edge_y_px = to_pixel_point(Point(center.x, edge_y_cm))

    return max(1, abs(edge_x_px - center_x_px)), max(1, abs(edge_y_px - center_y_px))


def same_point(point_a: Point, point_b: Point) -> bool:
    return abs(point_a.x - point_b.x) < 0.01 and abs(point_a.y - point_b.y) < 0.01


def draw_cross(image, cross_position, cross_obstacle: CrossObstacle | None) -> None:
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
            radius_px = cm_radius_to_px_axes(cross_obstacle.fallback_center, cross_obstacle.fallback_radius_cm)
            cv2.ellipse(image, center_px, radius_px, 0, 0, 360, (0, 0, 255), 2)

        draw_label(image, "Cross safety", cross_obstacle.fallback_center, (0, 0, 255))


def draw_robot(image, robot_pose: RobotPose) -> None:
    robot_px = to_pixel_point(robot_pose.position)
    heading_radians = radians(robot_pose.heading_degrees)
    heading_end = Point(
        robot_pose.position.x + ROBOT_HEADING_ARROW_CM * cos(heading_radians),
        robot_pose.position.y + ROBOT_HEADING_ARROW_CM * sin(heading_radians),
    )

    cv2.circle(image, robot_px, 8, (255, 0, 0), -1)
    cv2.arrowedLine(image, robot_px, to_pixel_point(heading_end), (255, 0, 0), 3, tipLength=0.25)
    draw_label(image, "Robot", robot_pose.position, (255, 0, 0))


def draw_balls(image, balls: list[Ball]) -> None:
    for ball in balls:
        center_px = to_pixel_point(ball.position)
        color = (0, 140, 255) if ball.is_vip else (255, 255, 255)
        outline = (0, 0, 0) if not ball.is_vip else (0, 90, 180)

        cv2.circle(image, center_px, 8, outline, 2)
        cv2.circle(image, center_px, 5, color, -1)
        draw_label(image, f"{ball.name}{' VIP' if ball.is_vip else ''}", ball.position, color, scale=0.5)


def draw_astar_segment(image, path_points: list[Point]) -> None:
    for start_point, end_point in zip(path_points, path_points[1:]):
        cv2.line(image, to_pixel_point(start_point), to_pixel_point(end_point), (0, 220, 0), 3)

    if len(path_points) >= 2:
        cv2.arrowedLine(
            image,
            to_pixel_point(path_points[-2]),
            to_pixel_point(path_points[-1]),
            (0, 220, 0),
            3,
            tipLength=0.25,
        )


def _extend_path(total_path: list[Point], segment_path: list[Point]) -> None:
    if total_path:
        total_path.extend(segment_path[1:])
    else:
        total_path.extend(segment_path)


def draw_route(
    image,
    robot_pose: RobotPose,
    route,
    goal_a: Goal,
    cross_obstacle: CrossObstacle | None,
) -> tuple[list[Point], list[Point], int]:
    raw_path_points: list[Point] = []
    simplified_path_points: list[Point] = []
    failed_segments = 0
    current_point = robot_pose.position

    for target in route:
        blocked_points = [other.ball.position for other in route if not other.ball.is_vip] if target.ball.is_vip else None
        segment_path = plan_path(
            current_point,
            target.pickup_point,
            cross_obstacle=cross_obstacle,
            blocked_points=blocked_points,
        )

        if not segment_path:
            failed_segments += 1
            cv2.arrowedLine(
                image,
                to_pixel_point(current_point),
                to_pixel_point(target.pickup_point),
                (0, 0, 255),
                2,
                tipLength=0.08,
            )
            current_point = target.pickup_point
            continue

        simplified_segment = smooth_path(
            segment_path,
            cross_obstacle=cross_obstacle,
            blocked_points=blocked_points,
        )
        draw_astar_segment(image, simplified_segment)

        _extend_path(raw_path_points, segment_path)
        _extend_path(simplified_path_points, simplified_segment)

        current_point = target.pickup_point

    goal_path = plan_path(current_point, goal_a.position, cross_obstacle=cross_obstacle)
    if not goal_path:
        failed_segments += 1
        cv2.arrowedLine(
            image,
            to_pixel_point(current_point),
            to_pixel_point(goal_a.position),
            (0, 0, 255),
            2,
            tipLength=0.08,
        )
    else:
        simplified_goal_path = smooth_path(goal_path, cross_obstacle=cross_obstacle)
        draw_astar_segment(image, simplified_goal_path)
        _extend_path(raw_path_points, goal_path)
        _extend_path(simplified_path_points, simplified_goal_path)

    for index, target in enumerate(route, start=1):
        pickup_point = target.pickup_point
        ball_point = target.ball.position
        pickup_px = to_pixel_point(pickup_point)

        cv2.circle(image, pickup_px, 9, (0, 255, 0), 2)
        draw_label(image, str(index), pickup_point, (0, 255, 0), scale=0.65)
        cv2.arrowedLine(image, pickup_px, to_pixel_point(ball_point), (0, 255, 255), 2, tipLength=0.25)

    return raw_path_points, simplified_path_points, failed_segments


def draw_demo(analysis: ImageAnalysis) -> tuple[list, list[Point], list[Point], int]:
    if analysis.warped_image is None:
        raise RuntimeError("Arena could not be detected. Cannot create demo image.")
    if analysis.robot_pose is None:
        raise RuntimeError("Robot marker not detected. Cannot plan route without robot position.")
    if not analysis.balls:
        raise RuntimeError("No balls detected. Cannot plan route.")

    image = analysis.warped_image.copy()
    goal_a = Goal(name="Goal A", position=GOAL_A_POSITION)
    route = choose_route(analysis.robot_pose.position, analysis.balls, goal_a, cross_obstacle=analysis.cross_obstacle)

    draw_cross(image, analysis.cross_position, analysis.cross_obstacle)
    draw_robot(image, analysis.robot_pose)
    draw_balls(image, analysis.balls)

    cv2.circle(image, to_pixel_point(goal_a.position), 10, (0, 200, 255), 3)
    draw_label(image, "Goal A", goal_a.position, (0, 200, 255))

    raw_path_points, simplified_path_points, failed_segments = draw_route(
        image,
        analysis.robot_pose,
        route,
        goal_a,
        analysis.cross_obstacle,
    )

    cv2.putText(
        image,
        "Planner demo: detections -> pickup route -> Goal A",
        (30, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (20, 20, 20),
        2,
        cv2.LINE_AA,
    )

    cv2.imwrite(str(OUTPUT_PATH), image)
    return route, raw_path_points, simplified_path_points, failed_segments


def print_demo_summary(
    analysis: ImageAnalysis,
    route,
    raw_path_points: list[Point],
    simplified_path_points: list[Point],
    failed_segments: int,
) -> None:
    print(f"Chosen image: {analysis.path}")
    print(f"Why chosen: score={analysis.score} | {', '.join(analysis.notes)}")
    print(f"Orange balls detected: {len(analysis.detected_orange_balls)}")
    print(f"White balls detected: {len(analysis.detected_white_balls)}")
    print(f"Total detected balls: {len(analysis.balls)}")
    print(f"Total route targets: {len(route)}")
    print("All detections are treated as real route targets")
    print(f"Cross data: {analysis.cross_position}")

    if analysis.robot_pose is not None:
        print(
            f"Robot pose: ({analysis.robot_pose.position.x:.1f}, {analysis.robot_pose.position.y:.1f}) | "
            f"heading={analysis.robot_pose.heading_degrees:.1f} deg | marker={analysis.robot_pose.marker_id}"
        )

    vip_target = next((target for target in route if target.ball.is_vip), None)
    if vip_target is not None:
        print(f"VIP target: {vip_target.ball.name}")

    print(f"Quadrant order used: {route_quadrant_order(route)}")
    print("Planned route:")
    for index, target in enumerate(route, start=1):
        print(
            f"{index}. {target.ball.name} | VIP={target.ball.is_vip} | "
            f"pickup=({target.pickup_point.x:.1f}, {target.pickup_point.y:.1f}) | "
            f"face={target.face_direction} | wall={target.is_wall_pickup}"
        )

    print("A* pathfinding used: yes")
    print(f"Raw A* path points created: {len(raw_path_points)}")
    print(f"Simplified waypoint count: {len(simplified_path_points)}")
    print(f"A* failed segments: {failed_segments}")
    print(f"Direct route segments crossing cross zone: {analysis.blocked_direct_segments}")
    print(f"Goal A fixed: ({GOAL_A_POSITION.x:.1f}, {GOAL_A_POSITION.y:.1f}) on right side")
    print(f"Output path: {OUTPUT_PATH}")


if __name__ == "__main__":
    arguments = parse_arguments()

    if arguments.image_path:
        analysis = analyze_image(Path(arguments.image_path))
        print(f"Using provided image: {analysis.path}")
        print(f"Image score: {analysis.score} | {', '.join(analysis.notes)}")
    else:
        analysis = choose_best_image()

    route_result, raw_demo_path_points, simplified_demo_path_points, demo_failed_segments = draw_demo(analysis)
    print_demo_summary(
        analysis,
        route_result,
        raw_demo_path_points,
        simplified_demo_path_points,
        demo_failed_segments,
    )
