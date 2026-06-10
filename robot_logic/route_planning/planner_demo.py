import argparse
import contextlib
import io
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path

import cv2

from Object_Tracking.Course_detecter import find_arena
from Object_Tracking.Object_Tracking import find_objects_in_image, px_to_world_cm, world_cm_to_px
from robot_logic.robot_detection.aruco_robot_detector import RobotPose, detect_robot_pose
from robot_logic.route_planning.route_planner import (
    CROSS_SAFETY_RADIUS_CM,
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
    Ball,
    Goal,
    Point,
    build_path_points,
    choose_route,
    distance,
)


IMAGE_DIR = Path("Images")
OUTPUT_PATH = Path("robot_logic/route_planning/planner_demo_output.jpg")
WARP_W = 1200
WARP_H = 800
ROBOT_HEADING_ARROW_CM = 12

# Goal A is assumed to be the right-side opening for this demo.
# Confirm the exact side and coordinate with the team before final robot control.
GOAL_A_POSITION = Point(FIELD_WIDTH_CM - 3.0, FIELD_HEIGHT_CM / 2.0)


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
    robot_pose: RobotPose | None
    balls: list[Ball]
    skipped_cross_zone_balls: int
    detour_count: int
    score: int
    notes: list[str]


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("image_path", nargs="?")
    return parser.parse_args()


def to_pixel_point(point: Point) -> tuple[int, int]:
    return world_cm_to_px(point.x, point.y, WARP_W, WARP_H)


def detection_to_point(detection) -> Point:
    if len(detection) >= 4:
        x_px, y_px = detection[2], detection[3]
        x_cm, y_cm = px_to_world_cm(x_px, y_px, WARP_W, WARP_H)
        return Point(x_cm, y_cm)

    return Point(detection[0], detection[1])


def detection_pixel_center(detection) -> tuple[int, int] | None:
    if len(detection) < 4:
        return None

    return int(detection[2]), int(detection[3])


def deduplicate_detections(detections: list, min_pixel_distance: int = 10) -> list:
    kept = []

    for detection in detections:
        center = detection_pixel_center(detection)

        if center is None:
            kept.append(detection)
            continue

        already_seen = False
        for existing in kept:
            existing_center = detection_pixel_center(existing)
            if existing_center is None:
                continue

            dx = center[0] - existing_center[0]
            dy = center[1] - existing_center[1]
            if dx * dx + dy * dy <= min_pixel_distance * min_pixel_distance:
                already_seen = True
                break

        if not already_seen:
            kept.append(detection)

    return kept


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


def filter_cross_zone_detections(
    orange_balls: list,
    white_balls: list,
    cross_point: Point | None,
) -> tuple[list, list, int]:
    if cross_point is None:
        return orange_balls, white_balls, 0

    kept_orange = []
    kept_white = []
    skipped = 0

    # The cross can be misread as a ball; do not plan pickups inside its safety zone.
    for detection in orange_balls:
        if distance(detection_to_point(detection), cross_point) <= CROSS_SAFETY_RADIUS_CM:
            skipped += 1
        else:
            kept_orange.append(detection)

    for detection in white_balls:
        if distance(detection_to_point(detection), cross_point) <= CROSS_SAFETY_RADIUS_CM:
            skipped += 1
        else:
            kept_white.append(detection)

    return kept_orange, kept_white, skipped


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
    return deduplicate_detections(orange_balls), deduplicate_detections(white_balls), cross_position


def score_analysis(
    warped_image,
    orange_balls: list,
    white_balls: list,
    cross_position,
    robot_pose: RobotPose | None,
    skipped_cross_zone_balls: int,
    detour_count: int,
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

    if skipped_cross_zone_balls:
        notes.append(f"{skipped_cross_zone_balls} skipped near cross")

    if detour_count:
        score += 350
        notes.append(f"{detour_count} detours")
    else:
        notes.append("0 detours")

    return score, notes


def analyze_image(path: Path) -> ImageAnalysis:
    raw_image = cv2.imread(str(path))
    if raw_image is None:
        return ImageAnalysis(path, None, None, [], [], [], [], None, None, None, [], 0, 0, -1, ["cannot load image"])

    warped_image = make_warped_image(raw_image)
    detected_orange_balls, detected_white_balls, cross_position = run_object_detection(raw_image)
    cross_point = get_cross_point(cross_position)
    orange_balls, white_balls, skipped_cross_zone_balls = filter_cross_zone_detections(
        detected_orange_balls,
        detected_white_balls,
        cross_point,
    )
    robot_pose = detect_robot_pose(raw_image, warp_w_px=WARP_W, warp_h_px=WARP_H)
    balls = build_balls(orange_balls, white_balls)
    detour_count = 0

    if robot_pose is not None and balls:
        goal_a = Goal(name="Goal A", position=GOAL_A_POSITION)
        route = choose_route(robot_pose.position, balls, goal_a)
        path_points = build_path_points(robot_pose.position, route, goal_a, cross_point, CROSS_SAFETY_RADIUS_CM)
        detour_count = max(0, len(path_points) - (len(route) + 2))

    score, notes = score_analysis(
        warped_image,
        orange_balls,
        white_balls,
        cross_position,
        robot_pose,
        skipped_cross_zone_balls,
        detour_count,
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
        robot_pose=robot_pose,
        balls=balls,
        skipped_cross_zone_balls=skipped_cross_zone_balls,
        detour_count=detour_count,
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


def draw_cross(image, cross_position, cross_point: Point | None) -> None:
    if isinstance(cross_position, dict):
        if "vertical_box" in cross_position:
            cv2.drawContours(image, [cross_position["vertical_box"]], 0, (0, 0, 180), 2)
        if "horizontal_box" in cross_position:
            cv2.drawContours(image, [cross_position["horizontal_box"]], 0, (0, 0, 220), 2)

    if cross_point is None:
        return

    center_px = to_pixel_point(cross_point)
    radius_px = cm_radius_to_px_axes(cross_point, CROSS_SAFETY_RADIUS_CM)
    cv2.circle(image, center_px, 5, (0, 0, 255), -1)
    cv2.ellipse(image, center_px, radius_px, 0, 0, 360, (0, 0, 255), 2)
    draw_label(image, "Cross safety", cross_point, (0, 0, 255))


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


def draw_route(image, robot_pose: RobotPose, route, goal_a: Goal, cross_point: Point | None) -> list[Point]:
    base_path_points = [robot_pose.position] + [target.pickup_point for target in route] + [goal_a.position]
    path_points = build_path_points(robot_pose.position, route, goal_a, cross_point, CROSS_SAFETY_RADIUS_CM)

    for start_point, end_point in zip(path_points, path_points[1:]):
        cv2.arrowedLine(image, to_pixel_point(start_point), to_pixel_point(end_point), (0, 220, 0), 3, tipLength=0.04)

    for point in path_points[1:-1]:
        if not any(same_point(point, base_point) for base_point in base_path_points):
            cv2.circle(image, to_pixel_point(point), 7, (255, 180, 0), -1)
            draw_label(image, "detour", point, (255, 180, 0), scale=0.5)

    for index, target in enumerate(route, start=1):
        pickup_point = target.pickup_point
        ball_point = target.ball.position
        pickup_px = to_pixel_point(pickup_point)

        cv2.circle(image, pickup_px, 9, (0, 255, 0), 2)
        draw_label(image, str(index), pickup_point, (0, 255, 0), scale=0.65)
        cv2.arrowedLine(image, pickup_px, to_pixel_point(ball_point), (0, 255, 255), 2, tipLength=0.25)

    return path_points


def draw_demo(analysis: ImageAnalysis) -> tuple[list, list[Point]]:
    if analysis.warped_image is None:
        raise RuntimeError("Arena could not be detected. Cannot create demo image.")
    if analysis.robot_pose is None:
        raise RuntimeError("Robot marker not detected. Cannot plan route without robot position.")
    if not analysis.balls:
        raise RuntimeError("No balls detected. Cannot plan route.")

    image = analysis.warped_image.copy()
    goal_a = Goal(name="Goal A", position=GOAL_A_POSITION)
    route = choose_route(analysis.robot_pose.position, analysis.balls, goal_a)

    draw_cross(image, analysis.cross_position, analysis.cross_point)
    draw_robot(image, analysis.robot_pose)
    draw_balls(image, analysis.balls)

    cv2.circle(image, to_pixel_point(goal_a.position), 10, (0, 200, 255), 3)
    draw_label(image, "Goal A temp", goal_a.position, (0, 200, 255))

    path_points = draw_route(image, analysis.robot_pose, route, goal_a, analysis.cross_point)

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
    return route, path_points


def print_demo_summary(analysis: ImageAnalysis, route, path_points: list[Point]) -> None:
    print(f"Chosen image: {analysis.path}")
    print(f"Why chosen: score={analysis.score} | {', '.join(analysis.notes)}")
    print(f"Orange balls detected: {len(analysis.detected_orange_balls)}")
    print(f"White balls detected: {len(analysis.detected_white_balls)}")
    print(f"Usable balls after cross safety filter: {len(analysis.balls)}")
    print(f"Ball detections skipped near cross: {analysis.skipped_cross_zone_balls}")
    print(f"Cross data: {analysis.cross_position}")

    if analysis.robot_pose is not None:
        print(
            f"Robot pose: ({analysis.robot_pose.position.x:.1f}, {analysis.robot_pose.position.y:.1f}) | "
            f"heading={analysis.robot_pose.heading_degrees:.1f} deg | marker={analysis.robot_pose.marker_id}"
        )

    print("Planned route:")
    for index, target in enumerate(route, start=1):
        print(
            f"{index}. {target.ball.name} | VIP={target.ball.is_vip} | "
            f"pickup=({target.pickup_point.x:.1f}, {target.pickup_point.y:.1f}) | "
            f"face={target.face_direction}"
        )

    print(f"Detour points added: {max(0, len(path_points) - (len(route) + 2))}")
    print(f"Goal A placeholder: ({GOAL_A_POSITION.x:.1f}, {GOAL_A_POSITION.y:.1f}) on right side")
    print(f"Output path: {OUTPUT_PATH}")


if __name__ == "__main__":
    arguments = parse_arguments()

    if arguments.image_path:
        analysis = analyze_image(Path(arguments.image_path))
        print(f"Using provided image: {analysis.path}")
        print(f"Image score: {analysis.score} | {', '.join(analysis.notes)}")
    else:
        analysis = choose_best_image()

    route_result, demo_path_points = draw_demo(analysis)
    print_demo_summary(analysis, route_result, demo_path_points)
