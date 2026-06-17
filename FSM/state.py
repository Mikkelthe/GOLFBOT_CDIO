from typing import Callable

from Object_Tracking.Object_Tracking import ObjectTracker
from Object_Tracking.Course_detecter import CourseDetector
from Navigation.Controller import Controller
import numpy as np
from utils.Linalg.vector import Vector2
import math
import cv2
from Navigation.Navigation import Navigation
from utils.conversion import Conversion
from utils.point import Point
from robot_logic.route_planning.route_planner import RoutePlanner


# TODO: Use vectors and matricies
class Transform:
    def __init__(self):
        self._position = Vector2(0, 0)
        self._rotation = 0

        self._forwardDirection = Vector2(0, 1)
    
    @property
    def rotation(self):
        return self._rotation
    
    @rotation.setter
    def rotation(self, value):
        self._rotation = value % 2*np.pi # We want the angle to be between 0 and 2pi
        self._forwardDirection = Vector2(0, 1).rotate(self._rotation)
        
    @property
    def position(self):
        return self._position
    
    @position.setter
    def position(self, vector: Vector2):
        self._position = vector

    
    @property
    def forwardDirection(self):
        return self._forwardDirection


    @forwardDirection.setter
    def forwardDirection(self, value: np.ndarray):
        # Normalize vector
        normalized = value / (np.sqrt(value @ value))
        self._forwardDirection = normalized
        
        # Update the angle
        v = np.array([0, 1])
        signedAngle = np.atan2(v[0]*value[1] - v[1]*value[0],
                               v[0]*value[0] + v[1]*value[1])
    
        # Change signed angle to be between 0 and 2pi
        if signedAngle < 0:
            signedAngle = 2*np.pi + signedAngle
        
        self._rotation = signedAngle



class GolfBotMemory:
    def __init__(self):
        self.quadrant = 0
        self.currentBall: Vector2 = None
        self.whiteBalls = []
        self.orangeBalls = []
        self.storedBallCount = 0
        self._transform = Transform()
        self.forwardDirection = [0, 1] # Direction
        self.objectTracker = ObjectTracker()
        self.courseDetector = CourseDetector()
        self.navigator = Navigation()
        self.converter = Conversion()
        self.approachPoint = Point(0,0)
        self.deliveryPoint = Point(0,0)
        self.router = RoutePlanner()
        self.pos = Point(0,0)
        self.heading = 0
        self.path = []

        self.videoDevice = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.videoDevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.videoDevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        _, img = self.videoDevice.read()
        self.arena = self.courseDetector.find_arena(img)
        self.cross = []
        self.goingToCornerLine = False


    @property
    def transform(self):
        return self._transform

    def updateTransform(self):
        _, img = self.videoDevice.read()
        point, angle = self.objectTracker.find_bot(img)
        self._transform.position = Vector2(point.x, point.y)
        self._transform.rotation = math.radians(angle)


class State:
    def __init__(self):
        self.handler = None
        self.transitions = []

    def setHandler(self, handler: Callable[[Controller, GolfBotMemory], None]):
        self.handler = handler

    def addTransition(self, transition):
        self.transitions.append(transition)
        transition.stateFrom = self
    
class Transition:
    stateFrom: State
    def __init__(self, stateFrom: State, stateTo: State):
        stateFrom.addTransition(self)
        self.stateTo = stateTo
        self.conditionHandler = None
    
    def setConditionHandler(self, handler: Callable[[GolfBotMemory], bool]):
        self.conditionHandler = handler 

class StateMachine:
    def __init__(self, memory: GolfBotMemory, controller: Controller, startState: State):
        self.memory = memory
        self.controller = controller
        self.currentState = startState
    
    def run(self):
        while self.currentState:
            self.currentState.handler(self.controller, self.memory)
            currentStateTransitions: list[Transition] = self.currentState.transitions
            for transition in currentStateTransitions:
                if transition.conditionHandler(self.memory):
                    self.currentState = transition.stateTo
                    print(f"Switched state: condition handler {transition.conditionHandler.__name__}")
                    break