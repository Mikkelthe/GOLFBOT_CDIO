import cv2
import math
import numpy as np
from navigation import Navigation
from tracking import CourseDetector, ObjectTracker
from robot_logic import RoutePlanner
from utils.linalg import Vector2
from utils import Conversion, Angle, Point

class Transform:
    def __init__(self):
        self._position = Vector2(0, 0)
        self._rotation = Angle()

        self._forwardDirection = Vector2(1, 0)

    @property
    def rotation(self) -> Angle:
        return self._rotation

    @rotation.setter
    def rotation(self, value: Angle):
        # This ensures that the angle is always between 0 and 2*pi
        self._rotation.radians = value.radians - 2 * math.pi * (value.radians // (2 * math.pi))
        self._forwardDirection = Vector2(1, 0).rotate(self._rotation)

    @property
    def position(self) -> Vector2:
        return self._position

    @position.setter
    def position(self, vector: Vector2):
        self._position = vector

    @property
    def forwardDirection(self) -> Vector2:
        return self._forwardDirection

    @forwardDirection.setter
    def forwardDirection(self, value: np.ndarray):
        # Normalize vector
        normalized = value / (np.sqrt(value @ value))
        self._forwardDirection = normalized

        # Update the angle
        v = np.array([1, 0])
        signed_angle = np.atan2(v[0] * value[1] - v[1] * value[0],
                                v[0] * value[0] + v[1] * value[1])

        # Change signed angle to be between 0 and 2pi
        if signed_angle < 0:
            signed_angle = 2 * np.pi + signed_angle

        self._rotation = signed_angle

class GolfBotMemory:
    videoDevice: cv2.VideoCapture | None
    def __init__(self):
        self.quadrant = 0
        self.currentBall: Point | None = None
        self.whiteBalls = []
        self.orangeBalls = []
        self.storedBallCount = 0
        self._transform = Transform()
        self.objectTracker = ObjectTracker()
        self.courseDetector = CourseDetector()
        self.navigator = Navigation()
        self.converter = Conversion()
        self.approachPoint = Point(0, 0)
        self.deliveryPoint = Point(0, 0)
        self.router = RoutePlanner()
        self.path = []
        self.point = Point(500, 300)
        self.motorStarted = False

        self.videoDevice = None

        self.arena = None
        self.cross = []
        self.goingToCornerLine = False

    def __del__(self):
        if self.videoDevice:
            self.videoDevice.release()

    @property
    def transform(self):
        return self._transform

    @property
    def pos(self) -> Point:
        pos_vec = self._transform.position
        return Point(pos_vec.x, pos_vec.y)

    @property
    def heading(self) -> Angle:
        return self._transform.rotation

    @property
    def forwardDirection(self) -> Vector2:
        return self._transform.forwardDirection

    def start(self):
        self.videoDevice = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not self.videoDevice:
            raise Exception('Video device not available')
        self.videoDevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.videoDevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        _, img = self.videoDevice.read()
        self.arena = self.courseDetector.find_arena(img)

    def updateTransform(self):
        if not self.videoDevice:
            raise Exception('Video device not available')
        _, img = self.videoDevice.read()
        point, angle = self.objectTracker.find_bot(img)
        if point and angle:
            self._transform.position = Vector2(point.x, point.y)
            self._transform.rotation = Angle(angle)