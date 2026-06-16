import cv2
from utils.settings.courtSettings import court_settings
def draw_detections_on_warp(
        self,
        warped_bgr,
        detections,
        label_prefix,
        warp_w_px, warp_h_px,
        court_w_cm=125.0, court_h_cm=170.0,
):
    for i, (realx_cm, realy_cm, x_px, y_px, r_px, area, circ) in enumerate(detections):
        # Draw circle + center
        cv2.circle(warped_bgr, (x_px, y_px), r_px, (0, 255, 0), 2)  # outline
        cv2.circle(warped_bgr, (x_px, y_px), 2, (0, 255, 0), -1)  # center dot

        # Label
        text = f"{label_prefix}{i}: ({realx_cm:.1f}cm, {realy_cm:.1f}cm)"
        cv2.putText(
            warped_bgr,
            text,
            (x_px + 10, y_px - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )


def draw_cross_on_warp(
        self,
        img,
        cross_data,
        warp_w_px=court_settings.image_width,
        warp_h_px=court_settings.image_height,
        court_w_cm=court_settings.court_width,
        court_h_cm=court_settings.court_height,
        border_px=100
):
    if cross_data is None:
        return img
    if len(cross_data) != 3:
        return img

    cv2.drawContours(img, [cross_data["vertical_box"]], 0, (0, 255, 0), 2)
    cv2.drawContours(img, [cross_data["horizontal_box"]], 0, (255, 0, 0), 2)

    cx, cy = cross_data["center"]
    cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)

    x_cm, y_cm = self.px_to_world_cm(
        cx,
        cy,
        warp_w_px=warp_w_px,
        warp_h_px=warp_h_px,
        border_px=border_px,
        court_w_cm=court_w_cm,
        court_h_cm=court_h_cm
    )

    label = f"Cross: ({x_cm:.1f}cm, {y_cm:.1f}cm)"
    cv2.putText(
        img,
        label,
        (cx + 10, cy - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    return img