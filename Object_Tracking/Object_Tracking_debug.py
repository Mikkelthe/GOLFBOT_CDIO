import time

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
    WARP_W, WARP_H = 1200, 800
    COURT_W_CM, COURT_H_CM = 180.0, 120.0

    videodevice = cv2.VideoCapture(1)
    videodevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    videodevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    time.sleep(5)
    while i < 20:
        i += 1
        print("i got here")

        ret, img = videodevice.read()
        print("i took picture")
        videocapturedimagepath = f"Images/captured_image_{i}.jpg"

        # Show results
        output_path = base_path.parent / videocapturedimagepath
        cv2.imwrite(str(output_path), img)
        print("I saved image")


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

        orange_balls, omask = detect_balls_by_hsv(warped, lower=(10, 70, 215), upper=(55, 255, 255))
        dark_orange_balls, domask = detect_balls_by_hsv(warped, lower=(5, 120, 120), upper=(45, 255, 255))
        white_balls, wmask = detect_balls_by_hsv(warped, lower=(0, 0, 220), upper=(255, 60, 255))
        shadowywhite_balls, sw = detect_balls_by_hsv(warped, lower=(0, 0, 115), upper=(180, 125, 240))

        mask_folder = base_path.parent / "masks_Images"
        mask_folder.mkdir(exist_ok=True)

        omask_folder = base_path.parent / mask_folder / "omask_Images"
        omask_folder.mkdir(exist_ok=True)

        wmask_folder = base_path.parent / mask_folder / "wmask_Images"
        wmask_folder.mkdir(exist_ok=True)

        domask_folder = base_path.parent / mask_folder / "domask_Images"
        domask_folder.mkdir(exist_ok=True)

        swmask_folder = base_path.parent / mask_folder / "swmask_Images"
        swmask_folder.mkdir(exist_ok=True)

        wmaskpath = f"{wmask_folder}/wmasked_image_{img_path.name}.jpg"
        omaskpath = f"{omask_folder}/omasked_image_{img_path.name}.jpg"
        domaskpath = f"{domask_folder}/domasked_image_{img_path.name}.jpg"
        swmaskpath = f"{swmask_folder}/swmasked_image_{img_path.name}.jpg"
        output_path = output_folder / img_path

        cv2.imwrite(str(wmaskpath), wmask)

        cv2.imwrite(str(omaskpath), omask)

        cv2.imwrite(str(domaskpath), domask)

        cv2.imwrite(str(swmaskpath), sw)
        cross_position = find_red_cross_boxes(warped)
        vis = warped.copy()
        draw_detections_on_warp(
            vis, orange_balls, "O",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_detections_on_warp(
            vis, dark_orange_balls, "dO",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_detections_on_warp(
            vis, shadowywhite_balls, "swO",
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
