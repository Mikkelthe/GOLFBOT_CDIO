from .Object_Tracking import (
    detect_balls_by_hsv,
    draw_detections_on_warp,
    draw_cross_on_warp
)
from .Course_detecter import (
    find_arena,
    find_red_cross_boxes,
    find_red_cross_center,
    find_red_cross_contour)
import cv2
import numpy as np
from pathlib import Path

if __name__ == "__main__":
    i = 0
    base_path = Path(__file__).resolve().parent
    output_folder = base_path.parent / "Warped_Images"
    output_folder.mkdir(exist_ok=True)
        
    # ---- Court settings ----
    WARP_W, WARP_H = 800, 1200
    COURT_W_CM, COURT_H_CM = 120.0, 180.0
    videodevice = cv2.VideoCapture(1)
    while i < 0:
        i += 1
        

        ret, img = videodevice.read()
        videocapturedimagepath = f"captured_image_{i}.jpg"
        # Show results
        output_path = output_folder / videocapturedimagepath
        cv2.imwrite(str(output_path), img)
        
        warped = find_arena(img, out_w=WARP_W, out_h=WARP_H)
        if warped is None:
            raise RuntimeError("Could not find arena")

        orange_balls, omask = detect_balls_by_hsv(warped, lower=(5,120,120), upper=(25,255,255))
        white_balls, wmask   = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))
        cross_position = find_red_cross_center(warped)
        print(len(orange_balls),len(white_balls))
        vis = warped.copy()
        draw_detections_on_warp(
            vis, orange_balls, "O",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_detections_on_warp(
            vis, white_balls, "W",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_cross_on_warp(
            vis, cross_position,
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        videocapturedimagepath = f"captured_image_changed_{i}.jpg"
        # Show results
        output_path = output_folder / videocapturedimagepath
        cv2.imwrite(str(output_path), vis)
        
    images_folder = base_path.parent / "Images"
    videodevice.release()
    

    

    image_files = list(images_folder.glob("*.jpg"))

    print(f"Found {len(image_files)} images")

    for img_path in image_files:
        print(f"Processing {img_path.name}")

        img = cv2.imread(str(img_path))
        if img is None:
            print("Could not load image")
            continue

        warped = find_arena(img, out_w=WARP_W, out_h=WARP_H)
        if warped is None:
            raise RuntimeError("Could not find arena")

        orange_balls, omask = detect_balls_by_hsv(warped, lower=(5,120,120), upper=(25,255,255))
        white_balls, wmask   = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))
        cross_position = find_red_cross_boxes(warped)
        print(cross_position)
        print(len(orange_balls),len(white_balls))
        vis = warped.copy()
        draw_detections_on_warp(
            vis, orange_balls, "O",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_detections_on_warp(
            vis, white_balls, "W",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_cross_on_warp(
            vis, cross_position,
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )

        # Show results
        output_path = output_folder / img_path.name
        cv2.imwrite(str(output_path), vis)
