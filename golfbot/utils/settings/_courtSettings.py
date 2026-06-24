from dataclasses import dataclass

@dataclass(frozen=True)
class CourtSettings:
    court_width: int = 170
    court_height: int = 125
    image_width: int = 1500
    image_height: int = 1000
    camera_height: int = 190
    padding: int = 100
    wall_thickness: int = 5
    wall_clearance_extra_px: int = 45
    closeToBall: int = 24

court_settings = CourtSettings()
