from Debug.Videoanalysis.VideoAnalysis import videodevice, orange_balls
from .state import *
from Object_Tracking.Object_Tracking import ObjectTracker
from utils.point import Point
from Navigation.Controller import Controller
from utils.settings.courtSettings import court_settings
import json
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
        return None

    @staticmethod
    #done
    def approachPointStateHandler(controller: Controller, golfBot: GolfBotMemory):
        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(golfBot.videoDevice.read())
        point = golfBot.currentBall
        movementVector = FSMFactory.findApproachVector(golfBot.pos, golfBot.heading, point)
        if golfBot.converter.px_to_world_cm(golfBot.navigator.find_distance_between_points(golfBot.pos,point)) > 20 & golfBot.navigator.find_turn(golfBot.heading,golfBot.pos,golfBot.currentBall)[1] < 0.35:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
            controller.move_dir(movementVector)

        controller.move_dir(movementVector)
        return None
    #done
    @staticmethod
    def collectOrangeStateHandler(controller: Controller, golfBot: GolfBotMemory):

        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(golfBot.videoDevice.read())
        point = golfBot.currentBall
        if golfBot.converter.px_to_world_cm(golfBot.navigator.find_distance_between_points(golfBot.pos,point)) > 20:
            movementVector = FSMFactory.findApproachVector(golfBot, golfBot.pos, golfBot.heading, point)
            controller.move_dir(movementVector)
        else:
            if golfBot.navigator.find_turn(golfBot.heading,golfBot.pos,golfBot.currentBall) == "right":
                controller.move_dir(Vector2(1,0))
            else:
                controller.move_dir(Vector2(-1,0))
        return None
    #done
    @staticmethod
    def checkQuadrantStateHandler(controller: Controller, golfBot: GolfBotMemory):
        return None
    #done
    @staticmethod
    def findNearestStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = golfBot.videoDevice.read()
        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(img)
        golfBot.currentBall = golfBot.router.choose_best_next_ball(golfBot.pos, golfBot.whiteBalls, golfBot.cross)
        return None

    #Should be used for both the orange and whiteball in state corner. Takes currentball and goes to it using corner strategy
    @staticmethod
    def approachCoordinateInCornerStateHandler(controller: Controller, golfBot: GolfBotMemory):
        golfBot.pos, golfBot.heading = golfBot.objectTracker.find_bot(golfBot.videoDevice.read())
        cornerApproachPoint = Navigation.find_optimal_corner_approach(golfBot.currentBall, golfBot.pos)

        if golfBot.pos == cornerApproachPoint:
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
        controller.move_dir(Vector2(x=0,y=-1))
        return None
    
    @staticmethod
    def approachWhiteCoordinateStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDO: Nav to ball
        pos, angle = golfBot.objectTracker.find_bot(golfBot.videoDevice.read())
        points = golfBot.router.plan_best_path(pos, golfBot.currentBall,golfBot.cross)
        for point in points:

            controller.move_dir(point)
        return None

    @staticmethod
    def approachNewQuadrantStateHandler(controller: Controller, golfBot: GolfBotMemory):
        next_quadrant = golfBot.quadrant+1
        if golfBot.quadrant == 5:
            next_quadrant = 1
        #ToDo: Determine navigation point in next quadrant
        return None

    @staticmethod
    def approachNarrowGoalStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = videodevice.read()
        pos, angle = golfBot.objectTracker.find_bot(img)
        golfBot.approachPoint, golfBot.deliveryPoint = golfBot.navigator.find_goal_approach_point()
        golfBot.router.pathToPoint(pos ,golfBot.approachPoint)

        return None

    @staticmethod
    def approachDeliveryPointStateHandler(controller: Controller, golfBot: GolfBotMemory):
        _, img = videodevice.read()
        pos, angle = golfBot.objectTracker.find_bot(img)
        moveVector = golfBot.router.pathToPoint(pos, golfBot.deliveryPoint)
        controller.move_dir(moveVector)
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
        _, img = videodevice.read()
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
        _, img = videodevice.read()
        if golfbot.objectTracker.find_bot(img)[0] == golfbot.deliveryPoint:
            return True
        else:
            return False

    @staticmethod
    def reachedApproachPointTransitionHandler(golfbot: GolfBotMemory):
        _, img = videodevice.read()
        if golfbot.objectTracker.find_bot(img)[0] == golfbot.approachPoint:
            return True
        else:
            return False


    @staticmethod
    def failedToCollectOrangeTransitionHandler(golfbot: GolfBotMemory):
        return False

    @staticmethod
    def obstacleAvoidedTransitionHandler(golfbot: GolfBotMemory):
        return False

    
    @staticmethod
    def nearestBallInCornerTransitionHandler(golfbot: GolfBotMemory):
        return False

    @staticmethod
    def nearestBallNotInCornerTransitionHandler(golfbot: GolfBotMemory):
        return False
    
    @staticmethod
    def ballCollectedTransitionHandler(golfbot: GolfBotMemory):
        if golfbot.converter.px_to_world_cm(golfbot.navigator.find_distance_between_points(golfbot.currentBall,golfbot.pos)) < 20 & golfbot.converter.px_to_world_cm(golfbot.navigator.find_distance_between_points(golfbot.currentBall,golfbot.pos)) > 17 & golfbot.navigator.find_turn(golfbot.heading,golfbot.pos,golfbot.currentBall)[1] < 0.035 & golfbot.navigator.find_turn(golfbot.heading,golfbot.pos,golfbot.currentBall)[1] > -0.035:
            return True
        return False

    @staticmethod
    def failedToCollectBallTransitionHandler(golfbot: GolfBotMemory):
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
        if golfBot.orangeBalls[1] == golfBot.currentBall:
            golfBot.goingToCornerLine = True
            return FSMFactory.IsInCorner(golfBot.currentBall[0], golfBot.currentBall[1])
        return False

    @staticmethod
    def orangeCollectedTransitionHandler(golfBot: GolfBotMemory):
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
            return FSMFactory.IsInCorner(golfBot.currentBall[0], golfBot.currentBall[1])
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
            golfBot.quadrant = FSMFactory.currentQuadrant(golfBot)
            return True
        return False

    @staticmethod
    def doneReadjustingTransitionHandler(golfBot: GolfBotMemory):
        if golfBot.currentBall in golfBot.whiteBalls:
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
        if x <= golfBot.cross[0] and y < golfBot.cross[1] and golfBot.quadrant == 1:
            return True
        elif x > golfBot.cross[0] and y <= golfBot.cross[1] and golfBot.quadrant == 2:
            return True
        elif x >= golfBot.cross[0] and y >= golfBot.cross[1] and golfBot.quadrant == 3:
            return True
        elif x <= golfBot.cross[0] and y >= golfBot.cross[1] and golfBot.quadrant == 4:
            return True
    
    @staticmethod
    def IsInCorner(x,y):
        return FSMFactory.xCoordinateinCorner(x) and FSMFactory.yCoordinateinCorner(y)
    
    @staticmethod
    def xCoordinateinCorner(x):
        return x<15 or x>court_settings.court_width-15

    @staticmethod
    def yCoordinateinCorner(y):
        return y<15 or y>court_settings.court_height-15

    @staticmethod
    def currentQuadrant(golfBot: GolfBotMemory):
        position = golfBot.objectTracker.find_bot(golfBot.arena)
        x, y = golfBot.converter.px_to_world_cm(position.point[0],position.point[1])
        point = [x,y]
        cross_center = golfBot.cross

        if point[0] <= cross_center[0] and point[1] < cross_center[1]:
            return 1
        elif point[0] > cross_center[0] and point[1] <= cross_center[1]:
            return 2
        elif point[0] >= cross_center[0] and point[1] >= cross_center[1]:
            return 3
        elif point[0] <= cross_center[0] and point[1] >= cross_center[1]:
            return 4
    @staticmethod
    def findApproachVector(golfBot: GolfBotMemory,botpos, angle, point):
        golfBot.navigator.find_turn(angle, botpos, point)
        return Vector2(0,1).rotate(angle)

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
                            Controller(('172.20.10.7', 6853), ('172.20.10.7', 80)),
                            stateObjects[fsm['startState']])