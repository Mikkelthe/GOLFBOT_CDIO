from __future__ import annotations
from typing import Callable
from navigation import Controller
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
            if debug and self.memory.videoDevice:
                _, img = self.memory.videoDevice.read()
                warped = self.memory.courseDetector.find_arena(img)
                warped = draw_vector_on_image(warped, self.memory.pos, self.memory.forwardDirection)
                warped = draw_vector_on_image(warped, self.memory.pos, Vector2(self.memory.point.x, self.memory.point.y),
                                              color=(0, 255, 0))
                cv2.imshow("Debug", warped)