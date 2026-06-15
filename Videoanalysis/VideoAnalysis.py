from Object_Tracking.Object_Tracking import *
import time
import cv2

run = True

# ---- Court settings ----
WARP_W, WARP_H = 1500, 1000
COURT_W_CM, COURT_H_CM = 170.0, 125.0

videodevice = cv2.VideoCapture(1,cv2.CAP_DSHOW)
videodevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
videodevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

time.sleep(5)

# Warm up camera
for _ in range(20):
    videodevice.read()

while run:
    ret, frame = videodevice.read()

    if not ret:
        print("Could not read frame from camera")
        break


    vis = frame.copy()
    vis = find_arena(vis, WARP_W, WARP_H)

    dilated = cv2.dilate(vis, np.ones((3,3), np.uint8), iterations=1)
    blurred = cv2.medianBlur(dilated, 5)

    orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position, a, b, c, d, e, f, g, h = find_objects_in_image(frame, WARP_W, WARP_H)

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

    cv2.imshow("vis", vis)

    cv2.imshow("blurred", blurred)

    # Press q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        run = False

videodevice.release()
cv2.destroyAllWindows()