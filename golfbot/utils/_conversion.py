from .settings import court_settings

class Conversion:
    def __init__(self):
        pass
    @staticmethod
    def px_to_world_cm(
            x_px,
            y_px,
            warp_w_px=court_settings.image_width,
            warp_h_px=court_settings.image_height,
            border_px=100,
            court_w_cm=court_settings.court_width,
            court_h_cm=court_settings.court_height
    ):
        court_w_px = warp_w_px - 2 * border_px
        court_h_px = warp_h_px - 2 * border_px

        # Convert image pixel coordinate to court-local pixel coordinate
        x_local_px = x_px - border_px
        y_local_px = y_px - border_px

        cm_per_px_x = court_w_cm / court_w_px
        cm_per_px_y = court_h_cm / court_h_px

        x_cm = x_local_px * cm_per_px_x

        # y = 0 at bottom of court
        y_cm = y_local_px * cm_per_px_y

        return x_cm, y_cm

    # Coordinates
    @staticmethod
    def world_cm_to_px(
            x_cm,
            y_cm,
            img_w_px=court_settings.image_width,
            img_h_px=court_settings.image_height,
            border_px=100,
            court_w_cm=court_settings.court_width,
            court_h_cm=court_settings.court_height
    ):
        court_w_px = img_w_px - 2 * border_px
        court_h_px = img_h_px - 2 * border_px

        cm_per_px_x = court_w_cm / court_w_px
        cm_per_px_y = court_h_cm / court_h_px

        x_local_px = x_cm / cm_per_px_x

        # y_cm is measured from bottom, but image y is measured from top
        y_local_px = y_cm/ cm_per_px_y

        # Add the 50 px border back
        x_px = x_local_px + border_px
        y_px = y_local_px + border_px

        return int(round(x_px)), int(round(y_px))