import json
import io
import time
import math
from pathlib import Path
from navigation import Controller
from ._golfbot import GolfBotMemory
from ._state import State, Transition, StateMachine
from utils.settings import court_settings
from utils.linalg import Vector2
from utils import Point, Angle

class FSMFactory:
    # States
    @staticmethod
    #done
    def detect_balls_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        controller.move_dir(Vector2(0, 0))
        golfbot.whiteBalls, golfbot.orangeBalls, golfbot.cross = golfbot.objectTracker.find_objects_in_image(golfbot.videoDevice)
        print(golfbot.whiteBalls[0])
        return None

    @staticmethod
    #done, currently unused
    def approach_point_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        point = golfbot.currentBall
        if not point:
            return None
        movement_vector = FSMFactory.adjust_vector(FSMFactory.find_approach_vector(golfbot, point))
        #turn_vector = golfbot.navigator.find_turn_2(golfbot.heading,golfbot.pos,golfbot.currentBall)
        controller.move_dir(movement_vector)
        #if golfbot.navigator.find_distance_between_points(golfbot.pos,point) > 20 and abs(turn_vector.x) < 0.035 and turn_vector.y > 0:
        #    controller.move_dir(movement_vector)
        #else:
        #   controller.move_dir(FSMFactory.adjust_vector(movement_vector))
        return None
    #done
    @staticmethod
    def collect_orange_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        pointlist = golfbot.router.plan_best_path(golfbot.pos, golfbot.currentBall, golfbot.cross)
        if len(pointlist) >= 1:
            golfbot.point = golfbot.router.plan_best_path(golfbot.pos, golfbot.currentBall, golfbot.cross)[1]
        if 40 > golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) > 20:
            if not golfbot.motorStarted:
                print("i should turn on")
                controller.turn_on_fan()
                golfbot.motorStarted = True
            movement_vector = FSMFactory.find_approach_vector(golfbot, golfbot.point)
            controller.move_dir(movement_vector*0.25)
        else:
            if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) > 20:
                movement_vector = FSMFactory.find_approach_vector(golfbot, golfbot.point)
                controller.move_dir(movement_vector)
            else:
                turn_vector = golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, golfbot.currentBall)
                turn_vector = Vector2(FSMFactory.adjust_vector(turn_vector).x, 0)
                controller.move_dir(turn_vector)
        return None
    #done
    @staticmethod
    def check_quadrant_state_handler(controller: Controller, golfbot: GolfBotMemory):
        return None
    #done
    @staticmethod
    def find_nearest_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        golfbot.currentBall = golfbot.router.choose_best_next_ball(golfbot.pos, golfbot.whiteBalls, golfbot.cross)
        return None

    #Should be used for both the orange and white ball in state corner. Takes current ball and goes to it using corner strategy
    @staticmethod
    def approach_coordinate_in_corner_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        corner_approach_point = (golfbot.navigator.find_optimal_corner_approach(golfbot.currentBall, golfbot.pos))
        #ToDo Adjust to correct tolerance for approach line
        if golfbot.navigator.find_distance_between_points(golfbot.pos, corner_approach_point) < 2:
            golfbot.goingToCornerLine = False

        if golfbot.goingToCornerLine:
            movement_vector = FSMFactory.find_approach_vector(golfbot, corner_approach_point)
            controller.move_dir(movement_vector)
        elif golfbot.currentBall:
            movement_vector = FSMFactory.find_approach_vector(golfbot, golfbot.currentBall)
            controller.move_dir(movement_vector)

        return None
    
    @staticmethod
    def readjust_state_handler(controller: Controller, golfbot: GolfBotMemory):
        controller.move_dir(Vector2(x=0,y=-1), not_backwards=False)
        return None
    
    @staticmethod
    def approach_white_coordinate_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        pointlist = golfbot.router.plan_best_path(golfbot.pos, golfbot.currentBall, golfbot.cross)
        if len(pointlist) >= 1:
            golfbot.point = golfbot.router.plan_best_path(golfbot.pos, golfbot.currentBall, golfbot.cross)[1]
        if golfbot.navigator.find_distance_between_points(golfbot.pos,
                                                          golfbot.currentBall) < 40 and golfbot.navigator.find_distance_between_points(
            golfbot.pos, golfbot.currentBall) > 20:
            movement_vector = FSMFactory.find_approach_vector(golfbot, golfbot.point)
            controller.move_dir(movement_vector * 0.25)
        else:
            if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) > 20:
                movement_vector = FSMFactory.find_approach_vector(golfbot, golfbot.point)
                controller.move_dir(movement_vector)
            else:
                turn_vector = golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, golfbot.currentBall)
                turn_vector = Vector2(FSMFactory.adjust_vector(turn_vector).x, 0)
                controller.move_dir(turn_vector)

    @staticmethod
    def approach_new_quadrant_state_handler(controller: Controller, golfbot: GolfBotMemory):
        next_quadrant = golfbot.quadrant + 1
        if golfbot.quadrant == 5:
            next_quadrant = 1
        #ToDo: Determine navigation point in next quadrant
        golfbot.quadrant = next_quadrant
        return None

    @staticmethod
    def approach_narrow_goal_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        pointlist = golfbot.router.plan_best_path(golfbot.pos, golfbot.approachPoint, golfbot.cross)
        point = pointlist[0]
        turn_vector = golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, golfbot.currentBall)
        if golfbot.navigator.find_distance_between_points(golfbot.pos, point) > 20 and abs(turn_vector.x) > 0.035 and turn_vector.y > 0:
            turn_vector = Vector2(FSMFactory.adjust_vector(turn_vector).x*0.2, 0)
            controller.move_dir(turn_vector)
        else:
            movement_vector = FSMFactory.find_approach_vector(golfbot, point)
            controller.move_dir(movement_vector)
        return None

    @staticmethod
    def approach_delivery_point_state_handler(controller: Controller, golfbot: GolfBotMemory):
        golfbot.updateTransform()
        turn_vector = golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, golfbot.currentBall)
        if abs(turn_vector.x) > 0.010:
            turn_vector = Vector2(FSMFactory.adjust_vector(turn_vector).x*0.2, 0)
            controller.move_dir(turn_vector)
        else:
            if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.deliveryPoint) > 10:
                if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.deliveryPoint) > 3:
                    controller.move_dir(Vector2(x=0,y=-0.25), not_backwards=False)
                else:
                    controller.move_dir(Vector2(x=0,y=-0.50), not_backwards=False)
            else:
                controller.move_dir(Vector2(x=0,y=-1), not_backwards=False)
        return None

    @staticmethod
    def avoid_obstacle_state_handler(controller: Controller, golfbot: GolfBotMemory):
        #ToDo add obstacle detector somehow from nav
        return None
        
    @staticmethod
    def submit_balls_state_handler(controller: Controller, golfbot: GolfBotMemory):
        controller.turn_off_fan()
        controller.open_door()
        time.sleep(2)
        controller.close_door()
        controller.open_door()
        return None

    # Transitions
    @staticmethod
    def bot_is_gone(golfbot: GolfBotMemory) -> bool:
        _, img = golfbot.videoDevice.read()
        if golfbot.objectTracker.find_bot(img)[0] is None:
            return True
        return False

    @staticmethod
    def white_in_quadrant_transition_handler(golfbot: GolfBotMemory) -> bool:
        _, img = golfbot.videoDevice.read()
        golfbot.whiteBalls, orange_balls, cross_pos = golfbot.objectTracker.find_objects_in_image(golfbot.videoDevice)
        print(golfbot.whiteBalls)
        for ball in golfbot.whiteBalls:
            _,_ = golfbot.converter.px_to_world_cm(ball[0],ball[1])
            print(FSMFactory.is_in_quadrant(ball[0], ball[1], golfbot))
            return FSMFactory.is_in_quadrant(ball[0], ball[1], golfbot)
        return False
    
    @staticmethod
    def obstacle_in_path_transition_handler(golfbot: GolfBotMemory) -> bool:
        return False

    @staticmethod
    def reached_goal_transition_handler(golfbot: GolfBotMemory) -> bool:
        golfbot.updateTransform()
        distance = golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.deliveryPoint)
        if distance < 1:
            return True
        else:
            return False

    @staticmethod
    def reached_approach_point_transition_handler(golfbot: GolfBotMemory) -> bool:
        golfbot.updateTransform()
        distance = golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.deliveryPoint)
        turn_vector = golfbot.navigator.find_turn_2(golfbot.heading,golfbot.pos,golfbot.currentBall)
        if distance > 1 and abs(turn_vector.x) < 0.35 and turn_vector.y > 0:
            return True
        else:
            return False


    @staticmethod
    def failed_to_collect_orange_transition_handler(golfbot: GolfBotMemory) -> bool:
        if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) < 16:
            return True
        return False

    @staticmethod
    def obstacle_avoided_transition_handler(golfbot: GolfBotMemory) -> bool:
        return False

    
    @staticmethod
    def nearest_ball_in_corner_transition_handler(golfbot: GolfBotMemory) -> bool:
        if golfbot.currentBall and FSMFactory.is_in_corner(golfbot, golfbot.currentBall[0], golfbot.currentBall[1]):
            return True
        return False

    @staticmethod
    def nearest_ball_not_in_corner_transition_handler(golfbot: GolfBotMemory) -> bool:
        if not FSMFactory.nearest_ball_in_corner_transition_handler(golfbot):
            return True
        return False
    
    @staticmethod
    def ball_collected_transition_handler(golfbot: GolfBotMemory) -> bool:
        turn_vector = golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, golfbot.currentBall)
        if 20 > golfbot.navigator.find_distance_between_points(golfbot.currentBall, golfbot.pos) > 17 and abs(turn_vector.x) < 0.17 and turn_vector.y > 0:
            return True
        return False

    @staticmethod
    def failed_to_collect_ball_transition_handler(golfbot: GolfBotMemory) -> bool:
        if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) < 16:
            return True
        return False
    
    @staticmethod
    def orange_detected_transition_handler(golfbot: GolfBotMemory) -> bool:
        orange_balls = golfbot.orangeBalls
        if len(orange_balls) > 0:
            golfbot.currentBall = golfbot.orangeBalls[0]
            return True
        else:
            return False

    @staticmethod
    def orange_in_corner_transition_handler(golfbot: GolfBotMemory) -> bool:
        if not golfbot.currentBall:
            return False
        in_corner = FSMFactory.is_in_corner(golfbot, golfbot.currentBall[0], golfbot.currentBall[1])
        if golfbot.orangeBalls[0] == golfbot.currentBall and in_corner:
            golfbot.goingToCornerLine = True
            return True
        return False

    @staticmethod
    def orange_collected_transition_handler(golfbot: GolfBotMemory) -> bool:
        turn_vector = golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, golfbot.currentBall)
        if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) < 19 and abs(turn_vector.x) < 0.35/4 and turn_vector.y > 0:
            return True
        return False
    
    @staticmethod
    def white_detected_transition_handler(golfbot: GolfBotMemory) -> bool:
        white_balls = golfbot.whiteBalls
        if len(white_balls) > 0:
            return True
        else:
            return False

    @staticmethod
    def white_in_corner_transition_handler(golfbot: GolfBotMemory) -> bool:
        if golfbot.currentBall in golfbot.whiteBalls and golfbot.currentBall:
            golfbot.goingToCornerLine = True
            return FSMFactory.is_in_corner(golfbot, golfbot.currentBall[0], golfbot.currentBall[1])
        return False

    @staticmethod
    def white_not_in_quadrant_transition_handler(golfbot: GolfBotMemory) -> bool:
        return not FSMFactory.white_in_quadrant_transition_handler(golfbot)
    
    @staticmethod
    def in_new_quadrant_transition_handler(golfbot: GolfBotMemory) -> bool:
        print(len(golfbot.whiteBalls))
        if FSMFactory.current_quadrant(golfbot) != golfbot.quadrant:
            return True
        return False

    @staticmethod
    def done_readjusting_transition_handler(golfbot: GolfBotMemory) -> bool:
        if (golfbot.currentBall in golfbot.whiteBalls) or (golfbot.currentBall in golfbot.orangeBalls):
            return True
        return False

    @staticmethod
    def none_detected_transition_handler(golfbot: GolfBotMemory) -> bool:
        if FSMFactory.orange_detected_transition_handler(golfbot) or FSMFactory.white_detected_transition_handler(golfbot):
            return False
        else:
            return True
    
    @staticmethod
    def is_in_quadrant(x: int, y: int, golfbot: GolfBotMemory) -> bool:
        cross = golfbot.cross["center"]

        if x <= cross[0] and y < cross[1] and golfbot.quadrant == 1:
            return True
        elif x > cross[0] and y <= cross[1] and golfbot.quadrant == 2:
            return True
        elif x >= cross[0] and y >= cross[1] and golfbot.quadrant == 3:
            return True
        elif x <= cross[0] and y >= cross[1] and golfbot.quadrant == 4:
            return True
        else:
            return False
    
    @staticmethod
    def is_in_corner(golfbot: GolfBotMemory, x: int, y: int) -> bool:
        x,y = golfbot.converter.px_to_world_cm(x,y)
        return FSMFactory.x_coordinate_in_corner(x) and FSMFactory.y_coordinate_in_corner(y)
    
    @staticmethod
    def x_coordinate_in_corner(x: int) -> bool:

        return x<15 or x>court_settings.court_width-15

    @staticmethod
    def y_coordinate_in_corner(y: int) -> bool:
        return y<15 or y>court_settings.court_height-15

    @staticmethod
    def current_quadrant(golfbot: GolfBotMemory) -> int:
        golfbot.updateTransform()
        x, y = golfbot.converter.px_to_world_cm(golfbot.pos.x, golfbot.pos.y)
        point = [x,y]
        cross_center = golfbot.cross["center"]

        if point[0] <= cross_center[0] and point[1] < cross_center[1]:
            return 1
        elif point[0] > cross_center[0] and point[1] <= cross_center[1]:
            return 2
        elif point[0] >= cross_center[0] and point[1] >= cross_center[1]:
            return 3
        elif point[0] <= cross_center[0] and point[1] >= cross_center[1]:
            return 4
        else:
            raise RuntimeError("This should not happen")

    @staticmethod
    def find_approach_vector(golfbot: GolfBotMemory, point: Point) -> Vector2:
        turn_vector = FSMFactory.adjust_vector((golfbot.navigator.find_turn_2(golfbot.heading, golfbot.pos, point)))
        print("turn_vector: " + str(turn_vector))
        return turn_vector

    @staticmethod
    def sign(value: float) -> float:
        if value < 0:
            return -1.0
        elif value > 0:
            return 1.0
        else:
            return 0.0

    @staticmethod
    def adjust_vector(turn_vector: Vector2) -> Vector2:
        if turn_vector.y < 0:
            return Vector2(FSMFactory.sign(turn_vector.x), 0)
        elif abs(turn_vector.x) > 0.3:
            return Vector2(turn_vector.x, 0)
        else:
            return Vector2(0, turn_vector.y)

    @staticmethod
    def create_robot_fsm() -> StateMachine:
        factory = FSMFactory()
        with io.open(str(Path(__file__).parent /  "fsm.json")) as file:
            fsm = json.load(file)

            states: dict[str, dict] = fsm['states']
            state_transitions: dict[str, list] = fsm['stateTransitions']

            state_objects = {}
            transition_objects = []
            def char_to_snake_case(char: str) -> str:
                if char.isupper():
                    return f"_{char.lower()}"
                else:
                    return char
            for name, handler in states.items():
                handler_name: str = states[name]['handler']
                handler_name_snake_case = "".join(list(map(char_to_snake_case, list(handler_name))))
                if hasattr(factory, handler_name):
                    handler_method = getattr(factory, handler_name)
                elif hasattr(factory, handler_name_snake_case):
                    handler_method = getattr(factory, handler_name_snake_case)
                else:
                    raise NameError(f"State handler name \"{handler_name}\" was not found")
                new_state = State(handler_method)
                state_objects[name] = new_state

            for state, transitions in state_transitions.items():
                for transition in transitions:
                    handler_name = transition['handler']
                    handler_name_snake_case = "".join(list(map(char_to_snake_case, list(handler_name))))
                    if hasattr(factory, handler_name):
                        handler_method = getattr(factory, handler_name)
                    elif hasattr(factory, handler_name_snake_case):
                        handler_method = getattr(factory, handler_name_snake_case)
                    else:
                        raise NameError(f"Transition handler name \"{handler_name}\" was not found")

                    next_state = transition['nextState']
                    new_transition = Transition(state_objects[state], state_objects[next_state], handler_method)
                    transition_objects.append(new_transition)

            return StateMachine(GolfBotMemory(),
                                Controller(('172.20.10.7', 6853), ('172.20.10.12', 80)),
                                state_objects[fsm['startState']])