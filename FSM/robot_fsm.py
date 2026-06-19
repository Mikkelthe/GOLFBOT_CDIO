from .state import *
from Navigation.Controller import Controller
from utils.settings.courtSettings import court_settings
import json
import cv2
import io
import time
from pathlib import Path

class FSMFactory:
    # States
    @staticmethod
    #done
    def detectBallsStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        golfBot.arena = golfBot.courseDetector.find_arena(img)
        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(img)
        golfBot.whiteBalls, golfBot.orangeBalls, golfBot.cross = golfBot.objectTracker.find_objects_in_image(golfBot.videoDevice)
        print(golfBot.whiteBalls[0])
        return None

    @staticmethod
    #done, currently unused
    def approachPointStateHandler(controller: Controller, golfBot: GolfBotMemory):
        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(golfBot.videoDevice.read())
        point = golfBot.currentBall
        movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
        if golfBot.navigator.find_distance_between_points(golfBot.pos,point) > 20 and golfBot.navigator.find_turn(golfBot.heading,golfBot.pos,golfBot.currentBall)[1] < 0.035:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
            controller.move_dir(movementVector)

        controller.move_dir(movementVector)
        return None
    #done
    @staticmethod
    def collectOrangeStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        pos, heading = golfBot.objectTracker.find_bot(img)
        if pos is not None:
            golfBot.pos = pos
            golfBot.heading = heading
        starttime = time.time()
        point = golfBot.router.plan_best_path(golfBot.pos, golfBot.currentBall, golfBot.cross)[1]
        endtime = time.time()
        print(starttime-endtime)
        print("orangeball: " + str(golfBot.currentBall.x) + "," + str(golfBot.currentBall.y))
        print("pathpoint: " + str(point.x) + "," + str(point.y))
        print ("distance: " + str(golfBot.navigator.find_distance_between_points(golfBot.pos, golfBot.currentBall)))
        if golfBot.navigator.find_distance_between_points(golfBot.pos, golfBot.currentBall) > 20:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
            controller.move_dir(movementVector)
        else:
            #controller.turn_on_fan()
            if golfBot.navigator.find_turn(golfBot.heading,golfBot.pos,golfBot.currentBall)[0] == "right":
                controller.move_dir(Vector2(-1,0))
            else:
                controller.move_dir(Vector2(1,0))
        return None
    #done
    @staticmethod
    def checkQuadrantStateHandler(controller: Controller, golfBot: GolfBotMemory):
        return None
    #done
    @staticmethod
    def findNearestStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        pos, heading = golfBot.objectTracker.find_bot(img)
        if pos is not None:
            golfBot.pos = pos
            golfBot.heading = heading
        golfBot.currentBall = golfBot.router.choose_best_next_ball(golfBot.pos, golfBot.whiteBalls, golfBot.cross)
        return None

    #Should be used for both the orange and whiteball in state corner. Takes currentball and goes to it using corner strategy
    @staticmethod
    def approachCoordinateInCornerStateHandler(controller: Controller, golfBot: GolfBotMemory):
        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(golfBot.videoDevice.read())
        cornerApproachPoint = Navigation.find_optimal_corner_approach(golfBot.currentBall, golfBot.pos)
        #ToDo Adjust to correct tolerance for approachline
        if golfBot.navigator.find_distance_between_points(golfBot.pos,cornerApproachPoint) < 2:
            golfBot.goingToCornerLine = False

        if golfBot.goingToCornerLine:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, cornerApproachPoint)
            controller.move_dir(movementVector)

        else:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, golfBot.currentBall)
            controller.move_dir(movementVector)

        return None
    
    @staticmethod
    def readjustStateHandler(controller: Controller, golfBot: GolfBotMemory):
        controller.move_dir(Vector2(x=0,y=-1), notbackwards=False)
        return None
    
    @staticmethod
    def approachWhiteCoordinateStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        pos, heading = golfBot.objectTracker.find_bot(img)
        if pos is not None:
            golfBot.pos = pos
            golfBot.heading = heading

        point = golfBot.router.plan_best_path(golfBot.currentBall)[0]
        if golfBot.navigator.find_distance_between_points(golfBot.pos, point) > 20:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
            controller.move_dir(movementVector)
        else:
            if golfBot.navigator.find_turn(golfBot.heading, golfBot.pos, golfBot.currentBall) == "right":
                controller.move_dir(Vector2(1, 0))
            else:
                controller.move_dir(Vector2(-1, 0))
        return None

    @staticmethod
    def approachNewQuadrantStateHandler(controller: Controller, golfBot: GolfBotMemory):
        next_quadrant = golfBot.quadrant+1
        if golfBot.quadrant == 5:
            next_quadrant = 1
        #ToDo: Determine navigation point in next quadrant
        golfBot.quadrant = next_quadrant
        return None

    @staticmethod
    def approachNarrowGoalStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        pos, heading = golfBot.objectTracker.find_bot(img)
        if pos is not None:
            golfBot.pos = pos
            golfBot.heading = heading
        pointlist = golfBot.router.plan_best_path(golfBot.pos, golfBot.approachPoint, golfBot.cross)
        point = pointlist[0]
        flag, angle = golfBot.navigator.find_turn(golfBot.heading, golfBot.pos, golfBot.currentBall)
        if golfBot.navigator.find_distance_between_points(golfBot.pos, point) > 20 and angle > 0.035:
            if flag == "right":
                controller.move_dir(Vector2(x=0.20,y=0))
            if flag == "left":
                controller.move_dir(Vector2(x=-0.20,y=0))
        else:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
            controller.move_dir(movementVector)
        return None

    @staticmethod
    def approachDeliveryPointStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        pos, heading = golfBot.objectTracker.find_bot(img)
        if pos is not None:
            golfBot.pos = pos
            golfBot.heading = heading
        flag, angle = golfBot.navigator.find_turn(golfBot.heading,golfBot.pos,golfBot.deliveryPoint)[0]
        if angle > 0.010:
            if flag == "right":
                controller.move_dir(Vector2(x=0.20,y=0))
            if flag == "left":
                controller.move_dir(Vector2(x=-0.20,y=0))
        else:
            if golfBot.navigator.find_distance_between_points(golfBot.pos, golfBot.deliveryPoint) > 10:
                if golfBot.navigator.find_distance_between_points(golfBot.pos,golfBot.deliveryPoint) > 3:
                    controller.move_dir(Vector2(x=0,y=-0.25), notbackwards=False)
                else:
                    controller.move_dir(Vector2(x=0,y=-0.50), notbackwards=False)
            else:
                controller.move_dir(Vector2(x=0,y=-1), notbackwards=False)
        return None

    @staticmethod
    def avoidObstacleStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDo add obstacle detecter somehow from nav
        return None
        
    @staticmethod
    def submitBallsStateHandler(controller: Controller, golfBot: GolfBotMemory):
        controller.turn_off_fan()
        controller.open_door()
        time.sleep(2)
        controller.close_door()
        controller.open_door()
        return None

    # Transitions
    @staticmethod
    def botIsGone(golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        if golfBot.objectTracker.find_bot(img)[0] is None:
            return True
        return False

    @staticmethod
    def whiteInQuadrantTransitionHandler(golfbot: GolfBotMemory):
        _, img = golfbot.videoDevice.read()
        golfbot.whiteBalls, orangeballs, crosspos = golfbot.objectTracker.find_objects_in_image(golfbot.videoDevice)
        for ball in golfbot.whiteBalls:
            FSMFactory.isInQuadrant(golfbot.converter.cm_to_px(ball[0]),golfbot.converter.cm_to_px(ball[0]),golfbot)
        return False
    
    @staticmethod
    def obstacleInPathTransitionHandler(golfbot: GolfBotMemory):

        return False

    @staticmethod
    def reachedGoalTransitionHandler(golfbot: GolfBotMemory):
        _, img = golfbot.videoDevice.read()
        pos, heading = golfbot.objectTracker.find_bot(img)
        if pos is not None:
            golfbot.pos = pos
            golfbot.heading = heading
        distance = golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.deliveryPoint)
        if distance < 1:
            return True
        else:
            return False

    @staticmethod
    def reachedApproachPointTransitionHandler(golfbot: GolfBotMemory):
        _, img = golfbot.videoDevice.read()
        pos, heading = golfbot.objectTracker.find_bot(img)
        if pos is not None:
            golfbot.pos = pos
            golfbot.heading = heading
        distance = golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.deliveryPoint)

        if distance > 1 and golfbot.navigator.find_turn(golfbot.heading,golfbot.pos,golfbot.currentBall)[1] < 0.35:
            return True
        else:
            return False


    @staticmethod
    def failedToCollectOrangeTransitionHandler(golfbot: GolfBotMemory):
        if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) < 16:
            return True
        return False

    @staticmethod
    def obstacleAvoidedTransitionHandler(golfbot: GolfBotMemory):
        return False

    
    @staticmethod
    def nearestBallInCornerTransitionHandler(golfbot: GolfBotMemory):
        if FSMFactory.IsInCorner(golfbot, golfbot.currentBall[0], golfbot.currentBall[1]):
            return True
        return False

    @staticmethod
    def nearestBallNotInCornerTransitionHandler(golfbot: GolfBotMemory):
        if not FSMFactory.nearestBallInCornerTransitionHandler(golfbot):
            return True
        return False
    
    @staticmethod
    def ballCollectedTransitionHandler(golfbot: GolfBotMemory):
        if golfbot.navigator.find_distance_between_points(golfbot.currentBall,golfbot.pos) < 20 and golfbot.navigator.find_distance_between_points(golfbot.currentBall,golfbot.pos) > 17 and golfbot.navigator.find_turn(golfbot.heading,golfbot.pos,golfbot.currentBall)[1] < 0.17:
            return True
        return False

    @staticmethod
    def failedToCollectBallTransitionHandler(golfbot: GolfBotMemory):
        if golfbot.navigator.find_distance_between_points(golfbot.pos, golfbot.currentBall) < 16:
            return True
        return False
    
    @staticmethod
    def orangeDetectedTransitionHandler(golfBot: GolfBotMemory):
        orangeBalls = golfBot.orangeBalls
        if len(orangeBalls) > 0:
            golfBot.currentBall = golfBot.orangeBalls[0]
            return True
        else:
            return False

    @staticmethod
    def orangeInCornerTransitionHandler(golfBot: GolfBotMemory):
        inCorner = FSMFactory.IsInCorner(golfBot, golfBot.currentBall[0], golfBot.currentBall[1])
        if golfBot.orangeBalls[0] == golfBot.currentBall and inCorner:
            golfBot.goingToCornerLine = True
            return True
        return False

    @staticmethod
    def orangeCollectedTransitionHandler(golfBot: GolfBotMemory):
        if golfBot.navigator.find_distance_between_points(golfBot.pos, golfBot.currentBall) > 20 and golfBot.navigator.find_turn(golfBot.heading,golfBot.pos,golfBot.currentBall)[1] < 0.035:
            return True
        return False
    
    @staticmethod
    def whiteDetectedTransitionHandler(golfBot: GolfBotMemory):
        whiteBalls = golfBot.whiteBalls
        if len(whiteBalls) > 0:
            return True
        else:
            return False

    @staticmethod
    def whiteInCornerTransitionHandler(golfBot: GolfBotMemory):
        if golfBot.currentBall in golfBot.whiteBalls:
            golfBot.goingToCornerLine = True
            return FSMFactory.IsInCorner(golfBot, golfBot.currentBall[0], golfBot.currentBall[1])
        return False

    @staticmethod
    def whiteInQuadrantTransitionHandler(golfBot: GolfBotMemory):
        for ball in golfBot.whiteBalls:
            if FSMFactory.isInQuadrant(ball[0], ball[1], golfBot):
                return True
        return False

    @staticmethod
    def whiteNotInQuadrantTransitionHandler(golfBot: GolfBotMemory):
        return not FSMFactory.whiteInQuadrantTransitionHandler(golfBot)
    
    @staticmethod
    def inNewQuadrantTransitionHandler(golfBot: GolfBotMemory):
        if FSMFactory.currentQuadrant(golfBot) != golfBot.quadrant:
            return True
        return False

    @staticmethod
    def doneReadjustingTransitionHandler(golfBot: GolfBotMemory):
        if (golfBot.currentBall in golfBot.whiteBalls) or (golfBot.currentBall in golfBot.orangeBalls):
            return True
        return False

    @staticmethod
    def noneDetectedTransitionHandler(golfBot: GolfBotMemory):
        if FSMFactory.orangeDetectedTransitionHandler(golfBot) or FSMFactory.whiteDetectedTransitionHandler(golfBot):
            return False
        else:
            return True
    
    @staticmethod
    def isInQuadrant(x,y,golfBot: GolfBotMemory):
        x,y = golfBot.converter.px_to_world_cm(x,y)
        print(golfBot.cross)
        cross = golfBot.cross["center"]
        if x <= cross[0] and y < cross[1] and golfBot.quadrant == 1:
            return True
        elif x > cross[0] and y <= cross[1] and golfBot.quadrant == 2:
            return True
        elif x >= cross[0] and y >= cross[1] and golfBot.quadrant == 3:
            return True
        elif x <= cross[0] and y >= cross[1] and golfBot.quadrant == 4:
            return True
    
    @staticmethod
    def IsInCorner(golfbot: GolfBotMemory, x,y):
        x,y = golfbot.converter.px_to_world_cm(x,y)
        return FSMFactory.xCoordinateinCorner(x) and FSMFactory.yCoordinateinCorner(y)
    
    @staticmethod
    def xCoordinateinCorner(x):

        return x<15 or x>court_settings.court_width-15

    @staticmethod
    def yCoordinateinCorner(y):
        return y<15 or y>court_settings.court_height-15

    @staticmethod
    def currentQuadrant(golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        pos, heading = golfBot.objectTracker.find_bot(img)
        if pos is not None:
            golfBot.pos = pos
            golfBot.heading = heading
        x, y = golfBot.converter.px_to_world_cm(golfBot.pos.x,golfBot.pos.y)
        point = [x,y]
        cross_center = golfBot.cross["center"]

        if point[0] <= cross_center[0] and point[1] < cross_center[1]:
            return 1
        elif point[0] > cross_center[0] and point[1] <= cross_center[1]:
            return 2
        elif point[0] >= cross_center[0] and point[1] >= cross_center[1]:
            return 3
        elif point[0] <= cross_center[0] and point[1] >= cross_center[1]:
            return 4
    @staticmethod
    def findApproachVector(golfBot: GolfBotMemory,botpos, heading, point):
        angle = golfBot.navigator.find_turn(heading, botpos, point)
        print("heading: ", str(heading))

        print("flag" + str(angle[0]))
        if angle[0] == "right":
            turnangle = -angle[1]
        else:
            turnangle = angle[1]
        print("angle" + str(turnangle))
        return Vector2(0,1).rotate(turnangle)

    @staticmethod
    def createRobotFSM():
        factory = FSMFactory()
        fsm = json.load(io.open(str(Path(__file__).parent /  "fsm.json")))

        states: dict[str, dict] = fsm['states']
        stateTransitions: dict[str, list] = fsm['stateTransitions']
        
        stateObjects = {}
        transitionObjects = []
        for name, handler in states.items():
            handlerName = states[name]['handler']
            handlerMethod = getattr(factory, handlerName)
            newState = State()
            newState.setHandler(handlerMethod)
            stateObjects[name] = newState

        for state, transitions in stateTransitions.items():
            for transition in transitions:
                handlerName = transition['handler']
                nextState = transition['nextState']
                handlerMethod = getattr(factory, handlerName)
                newTransition = Transition(stateObjects[state], stateObjects[nextState])
                newTransition.setConditionHandler(handlerMethod)
                transitionObjects.append(newTransition)

        
        return StateMachine(GolfBotMemory(),
                            Controller(('10.248.150.144', 6853), ('172.20.10.12', 80)),
                            stateObjects[fsm['startState']])