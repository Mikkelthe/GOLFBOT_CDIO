from math import cos, radians, sin

import cv2

from Object_Tracking.Course_detecter import find_arena
from Object_Tracking.Object_Tracking import find_objects_in_image, world_cm_to_px
from robot_logic.robot_detection.aruco_robot_detector import detect_robot_pose
from robot_logic.route_planning.route_planner import Ball, Goal, Point, choose_route


IMAGE_PATH = "Images/To_bolde_1.jpg"
OUTPUT_PATH = "robot_logic/route_planning/visual_route_output.jpg"
WARP_W = 800
WARP_H = 1200
CROSS_SAFETY_RADIUS_CM = 15
ROBOT_HEADING_ARROW_CM = 12

# Goal A is fixed by field setup. If goal location changes, it must be detected.
GOAL_A_POSITION = Point(110.0, 160.0)


def to_pixel_point(point: Point) -> tuple[int, int]:
    return world_cm_to_px(point.x, point.y, warp_w_px=WARP_W, warp_h_px=WARP_H)


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


def build_balls(orange_balls, white_balls) -> list[Ball]:
    balls: list[Ball] = []

    for index, detection in enumerate(orange_balls):
        x_cm, y_cm = detection[0], detection[1]
        balls.append(
            Ball(
                name=f"O{index}",
                position=Point(x_cm, y_cm),
                is_vip=(index == 0),
            )
        )

    for index, detection in enumerate(white_balls):
        x_cm, y_cm = detection[0], detection[1]
        balls.append(
            Ball(
                name=f"W{index}",
                position=Point(x_cm, y_cm),
                is_vip=False,
            )
        )

    return balls


if __name__ == "__main__":
    raw_image = cv2.imread(IMAGE_PATH)
    if raw_image is None:
        raise FileNotFoundError(f"Could not load image: {IMAGE_PATH}")

    warped_image = find_arena(raw_image, WARP_W, WARP_H)
    if warped_image is None or not hasattr(warped_image, "shape"):
        raise RuntimeError("Could not create warped arena image")

    orange_balls, white_balls, cross_position = find_objects_in_image(raw_image, WARP_W, WARP_H)
    balls = build_balls(orange_balls, white_balls)
    robot_pose = detect_robot_pose(raw_image, warp_w_px=WARP_W, warp_h_px=WARP_H)

    if robot_pose is None:
        raise RuntimeError("Robot marker not detected. Cannot plan route without robot position.")

    goal_a = Goal(name="Goal A", position=GOAL_A_POSITION)
    planned_route = choose_route(robot_pose.position, balls, goal_a)

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

    if cross_position is not None:
        cross_point = Point(cross_position[0], cross_position[1])
        cross_radius_px = int(CROSS_SAFETY_RADIUS_CM * WARP_W / 120.0)
        cv2.circle(warped_image, to_pixel_point(cross_point), 5, (0, 0, 255), -1)
        cv2.circle(warped_image, to_pixel_point(cross_point), cross_radius_px, (0, 0, 255), 2)
        draw_label(warped_image, "Cross", cross_point, (0, 0, 255))

    for ball in balls:
        ball_color = (0, 140, 255) if ball.is_vip else (255, 255, 255)
        cv2.circle(warped_image, to_pixel_point(ball.position), 5, ball_color, -1)
        draw_label(warped_image, ball.name, ball.position, ball_color)

    path_points = [robot_pose.position]

    print(f"Orange balls detected: {len(orange_balls)}")
    print(f"White balls detected: {len(white_balls)}")
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
            f"face={target.face_direction}"
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

        path_points.append(pickup_point)

    path_points.append(goal_a.position)

    for start_point, end_point in zip(path_points, path_points[1:]):
        cv2.arrowedLine(
            warped_image,
            to_pixel_point(start_point),
            to_pixel_point(end_point),
            (0, 255, 0),
            2,
            tipLength=0.03,
        )

    cv2.imwrite(OUTPUT_PATH, warped_image)
    print(f"Output path: {OUTPUT_PATH}")
