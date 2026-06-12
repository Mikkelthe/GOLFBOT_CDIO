from typing import Callable

from Object_Tracking.Course_detecter import find_arena,find_red_cross_boxes
from golfbot import *
import numpy as np
from ..Linalg.vector import Vector2
from ..Linalg.matrix import Matrix22


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
    def __init__(self,img):
        # TODO: Store more stuff in memory
        self.quadrant = 0
        self.currentBall = []
        self.whiteBalls = []
        self.orangeBalls = []
        self.storedBallCount = 0
        self.transform = Transform()
        self.forwardDirection = [0, 1] # Direction
        self.arena = find_arena(img,1500,1000)
        self.cross = 0


class State:
    def __init__(self):
        self.handler = None
        self.transitions = []

    def setHandler(self, handler: Callable[[GolfBotMemory], None]):
        self.handler = handler

    def addTransition(self, transition):
        self.transitions.append(transition)
        self.transitions.stateFrom = self
    
class Transition:
    def __init__(self, stateFrom: State, stateTo: State):
        self.stateFrom = stateFrom
        self.stateTo = stateTo
        self.conditionHandler = None
    
    def setConditionHandler(self, handler: Callable[[GolfBotMemory], bool]):
        self.conditionHandler = handler 

class StateMachine:
    def __init__(self, golfBot, startState: State):
        self.transitions = []
        self.golfBot = golfBot
        self.currentState = startState
    
    def run(self):
        while self.currentState:
            self.currentState.handler(self.golfBot)
            currentStateTransitions = self.currentState.transitions
            for transition in currentStateTransitions:
                if transition.conditionHandler(self.golfBot):
                    self.currentState = transition.stateTo
                    break