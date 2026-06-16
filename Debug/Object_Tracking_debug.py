import time
from time import sleep
from Drawer import *
from Object_Tracking.Object_Tracking import ObjectTracker
from Object_Tracking.Course_detecter import CourseDetector
import cv2
import numpy as np
from pathlib import Path
from utils.settings.courtSettings import court_settings


if __name__ == "__main__":
    objectTracker = ObjectTracker()
    courseDetector = CourseDetector()
    start_i = 22
    i = start_i
    base_path = Path(__file__).resolve().parent
    output_folder = base_path.parent / "Warped_Images"
    output_folder.mkdir(exist_ok=True)
    imagecount = 20
        
    # ---- Court settings ----
    WARP_W, WARP_H = court_settings.image_width, court_settings.image_height
    COURT_W_CM, COURT_H_CM = court_settings.court_width, court_settings.court_height

    if i < imagecount:
        videodevice = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        videodevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        videodevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        time.sleep(2)
    start_time = time.time()
    while i < imagecount:
        i += 1
        if i%5 == 0:
            print("move")
            sleep(1)
        ret, img = videodevice.read()
        videocapturedimagepath = f"Images/captured_image_{i}.jpg"

        # Show results
        output_path = base_path.parent / videocapturedimagepath
        cv2.imwrite(str(output_path), img)
        print("I saved image")


    images_folder = base_path.parent / "Images"
    new_time = time.time()
    if imagecount >= start_i:
        videodevice.release()
        print(time.time() - new_time)
        print(time.time() - start_time)

    

    image_files = list(images_folder.glob("*.jpg"))

    print(f"Found {len(image_files)} images")

    accumulated_objects, accumulated_vip_objects = list(), list()
    j=0
    for img_path in image_files:
        print(f"Processing {img_path.name}")

        img = cv2.imread(str(img_path))
        if img is None:
            print("Could not load image")
            continue

        warped = courseDetector.find_arena(img, out_w=WARP_W, out_h=WARP_H)
        if warped is None:
            raise RuntimeError("Could not find arena")
        dilated = cv2.dilate(warped, np.ones((1, 1), np.uint8), iterations=1)
        blurred = cv2.GaussianBlur(dilated, (3,3), 0)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        sharpened = cv2.filter2D(blurred, -1, kernel)

        orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position, omask, domask, wmask, sw, wcenter, ocenter, swcenter, docenter = objectTracker.find_objects_in_image(img, WARP_W, WARP_H)
        rounded_objects, rounded_vip_objects = objectTracker.group_valid_objects(wcenter, ocenter, swcenter, docenter)
        objectTracker.accumulate_valid_objects(accumulated_objects,accumulated_vip_objects,rounded_objects, rounded_vip_objects, j)
        j+=1

        # orange_balls, omask = detect_balls_by_hsv(warped, lower=(0, 40, 140), upper=(40, 255, 255))
        # dark_orange_balls, domask = detect_balls_by_hsv(warped, lower=(5, 120, 120), upper=(30, 255, 255))
        # white_balls, wmask = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 90, 255))
        # shadowywhite_balls, sw = detect_balls_by_hsv(warped, lower=(0, 0, 30), upper=(180, 20, 240))


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
        cross_position = courseDetector.find_red_cross_boxes(warped)

        vis = blurred.copy()
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
    print("time to proccess " + str(len(image_files)) +  " = " + str(time.time() - start_time))
