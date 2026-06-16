from .state import *
from Object_Tracking.Object_Tracking import find_objects_in_image, px_to_world_cm, find_arena, cm_to_px
from Navigation.Navigation import find_bot, find_optimal_corner_approach, drive_to_point
from utils.point import Point
from Navigation.Controller import Controller
from utils.settings.courtSettings import court_settings

class FSMFactory:
    # States
    @staticmethod
    def detectStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDone: Change the find_objects_in_image, to persistant ball finder
        warp_w = court_settings.image_width
        warp_h = court_settings.image_height
        _, img = golfBot.videoDevice.read()
        golfBot.arena = golfBot.courseDetector.find_arena(img)
        golfBot.whiteBalls, golfBot.orangeBalls, golfBot.cross = golfBot.objectTracker.find_objects_in_image(img,warp_w ,warp_h)
        return None

    @staticmethod
    def collectOrangeStateHandler(controller: Controller, golfBot: GolfBotMemory):
        golfBot.currentBall = golfBot.orangeBalls[0]
        #ToDo: Use algorithm on currentball position to move to it
        transform = golfBot.transform
        robotToBall = golfBot.currentBall - transform.position
        normalized = robotToBall.normalized
        normalized.rotate(-transform.rotation)

        controller.move_dir(normalized)
        return None

    @staticmethod
    def checkQuadrantStateHandler(controller: Controller, golfBot: GolfBotMemory):
        return None

    @staticmethod
    def findNearestStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDo closest ball function

        return None

    #Should be used for both the orange and whiteball in state corner. Takes currentball and goes to it using corner strategy
    @staticmethod
    def approachCoordinateInCornerStateHandler(controller: Controller, golfBot: GolfBotMemory):
        # ToDo: go to line in quadrant
        x = cm_to_px(golfBot.currentBall[0])
        y = cm_to_px(golfBot.currentBall[1])
        ballPoint = Point(x, y)
        a, b, c, = find_bot(golfBot.arena)
        cornerApproachPoint = find_optimal_corner_approach(a, ballPoint)

        # ToDo: go to corner approach point
        if a == cornerApproachPoint:
            golfBot.goingToCornerLine = False
        if golfBot.goingToCornerLine:
            commands = drive_to_point(cornerApproachPoint)
            sendCommmands(commands)
        else:
        #consider splitting states to one to get to the corner, then get the ball. How do i split this?
        # ToDo: go to ball

            commands = drive_to_point(golfBot.currentBall)
            sendCommands(commands)
        return None
    
    @staticmethod
    def readjustStateHandler(controller: Controller, golfBot: GolfBotMemory):
        # ToDo add nav to readjustment
        return None
    
    @staticmethod
    def approachWhiteCoordinateStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDO: Nav to ball
        nav_to_ball(golfBot.currentBall)
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
        #Find point of goal
        #ToDo: fix botholeToCenter to correct distance
        botholeToCenter= 20
        y = court_settings.court_height/2
        x = court_settings.court_width-botholeToCenter-5
        #ToDo: Nav to x,y

        return None

    @staticmethod
    def avoidObstacleStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDo add obstacle detecter somehow from nav
        return None
        
    @staticmethod
    def submitBallsStateHandler(controller: Controller, golfBot: GolfBotMemory):
        #ToDo run Open bothole
        #ToDo run unstuck function
        #ToDo consider rechecking for balls in transitiion from SubmitBalls
        return None

    # Transitions
    @staticmethod
    def botIsGone(golfBot: GolfBotMemory):
        if find_bot(golfBot.arena)[0] is None:
            return True
        return False

    @staticmethod
    def orangeDetectedTransitionHandler(golfBot: GolfBotMemory):
        orangeBalls = golfBot.orangeBalls
        if len(orangeBalls) > 0:
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
            return False
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
    def inNewQuadrantTransitionHandler(golfBot: GolfBotMemory):
        if FSMFactory.currentQuadrant(golfBot) != golfBot.quadrant:
            golfBot.quadrant = FSMFactory.currentQuadrant(golfBot)
            return True
        return False

    @staticmethod
    def doneReadjustingTransitionHandler(golfBot: GolfBotMemory):
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
        elif x > golfBot.cross[0] and y <= golfBot.cross[1] and golfBot.quadrant == 1:
            return True
        elif x >= golfBot.cross[0] and y >= golfBot.cross[1] and golfBot.quadrant == 1:
            return True
        elif x <= golfBot.cross[0] and y >= golfBot.cross[1] and golfBot.quadrant == 1:
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
        warp_w = court_settings.image_width
        warp_h = court_settings.image_height
        position = find_bot(golfBot.arena)
        x, y =px_to_world_cm(position.point[0],position.point[1],warp_w,warp_h)
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
    def createRobotFSM():
        detectState = State()
        detectState.setHandler(FSMFactory.detectStateHandler)

        collectOrangeState = State()
        collectOrangeState.setHandler(FSMFactory.collectOrangeStateHandler)

        checkQuadrantState = State()
        checkQuadrantState.setHandler(FSMFactory.checkQuadrantStateHandler)

        findNearestState = State()
        findNearestState.setHandler(FSMFactory.findNearestStateHandler)

        approachCoordinateInCornerState = State()
        approachCoordinateInCornerState.setHandler(FSMFactory.approachCoordinateInCornerStateHandler)

        readjustState = State()
        readjustState.setHandler(FSMFactory.readjustStateHandler)

        approachWhiteCoordinateState = State()
        approachWhiteCoordinateState.setHandler(FSMFactory.approachWhiteCoordinateStateHandler)

        approachNewQuadrantState = State()
        approachNewQuadrantState.setHandler(FSMFactory.approachNewQuadrantStateHandler)

        approachNarrowGoalState = State()
        approachNarrowGoalState.setHandler(FSMFactory.approachNarrowGoalStateHandler)

        avoidObstacleGoalState = State()
        avoidObstacleGoalState.setHandler(FSMFactory.avoidObstacleStateHandler)

        submitBallsState = State()
        submitBallsState.setHandler(FSMFactory.submitBallsStateHandler)