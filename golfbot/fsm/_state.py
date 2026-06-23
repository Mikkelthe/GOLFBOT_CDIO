from __future__ import annotations

import time
import math
from typing import Callable

import numpy as np

from navigation import Controller
from utils import Angle
from ._golfbot import GolfBotMemory
from _debug.Drawer import *

class State:
    def __init__(self, handler: Callable[[Controller, GolfBotMemory], None]):
        self.handler = handler
        self.transitions = []

    def addTransition(self, transition: Transition):
        self.transitions.append(transition)
        transition.stateFrom = self
    
class Transition:
    stateFrom: State
    def __init__(self, state_from: State, state_to: State, condition_handler: Callable[[GolfBotMemory], bool] ):
        state_from.addTransition(self)
        self.stateTo = state_to
        self.conditionHandler = condition_handler

class StateMachine:
    def __init__(self, memory: GolfBotMemory, controller: Controller, start_state: State):
        self.memory = memory
        self.controller = controller
        self.currentState = start_state
    
    def run(self, debug = False):
        self.controller.connect()
        self.memory.start()
        while self.currentState:
            self.currentState.handler(self.controller, self.memory)
            current_state_transitions: list[Transition] = self.currentState.transitions
            for transition in current_state_transitions:
                if transition.conditionHandler(self.memory):
                    self.currentState = transition.stateTo
                    print(f"Switched state: condition handler {transition.conditionHandler.__name__}")
                    self.controller.move_dir(Vector2(0.0,0.0))
                    break
            if debug and self.memory.videoDevice:
                _, img = self.memory.videoDevice.read()
                warped = self.memory.courseDetector.find_arena(img)
                #distance_to_point = math.sqrt((self.memory.pos.x - self.memory.point.x)**2 + (self.memory.pos.y - self.memory.point.y)**2)
                heading = self.memory.heading
                robot_pos = Point(self.memory.pos.x, self.memory.pos.y)
                forward_for = Vector2(self.memory.forwardDirection.x, -self.memory.forwardDirection.y)
                point_pos = Point(self.memory.point.x, self.memory.point.y)
                target_dir = (self.memory.navigator.find_turn_2(self.memory.heading, robot_pos, point_pos)
                              .rotate(Angle(-heading.radians))).rotate(Angle(np.radians(-90)))
                warped = draw_vector_on_image(warped, robot_pos, forward_for, length=50, color=(0,0,255))
                warped = draw_vector_on_image(warped, robot_pos, Vector2(point_pos.x, point_pos.y) - robot_pos,
                                              color=(0, 255, 0), length=1)
                warped = draw_vector_on_image(warped, robot_pos, target_dir*50,
                                              color=(255, 255, 0), length=1)
                cv2.imshow("Debug", warped)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break