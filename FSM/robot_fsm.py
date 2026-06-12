from .state import *
from Object_Tracking.Object_Tracking import find_objects_in_image, px_to_world_cm, find_arena, cm_to_px
from Navigation.Navigation import find_bot, find_optimal_corner_approach
from Navigation.point import Point
from settings.courtSettings import court_settings
# States
def detectStateHandler(golfBot: GolfBotMemory):
    #ToDo: Change the find_objects_in_image, to persistant ball finder
    warp_w = court_settings.image_width
    warp_h = court_settings.image_height
    orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position, omask, domask, wmask, sw = find_objects_in_image(img,warp_w ,warp_h)
    golfBot.orangeBalls = orange_balls
    golfBot.whiteBalls = white_balls
    golfBot.cross = cross_position["center"]
    golfBot.arena = find_arena(img,warp_w,warp_h)
    return None

def collectOrangeStateHandler(golfBot: GolfBotMemory):
    golfBot.currentBall = golfBot.orangeBalls[0]
    #ToDo: Use algorithm on currentball position to move to it
    return None

def checkQuadrantStateHandler(golfBot: GolfBotMemory):
    return None

def findNearestStateHandler(golfBot: GolfBotMemory):
    return None

#Should be used for both the orange and whiteball in state corner. Takes currentball and goes to it using corner strategy
def approachCoordinateInCornerStateHandler(golfBot: GolfBotMemory):
    # ToDo: go to line in quadrant
    x = cm_to_px(golfBot.currentBall[0])
    y = cm_to_px(golfBot.currentBall[1])
    ballPoint = Point(x, y)
    a, b, c, = find_bot(golfBot.arena)
    cornerApproachPoint = find_optimal_corner_approach(a, ballPoint)
    # ToDo: go to corner approach point

    # ToDo: Turn to heading for ball

    # ToDo: go to ball
    currentBall = golfBot.currentBall
    return None

def readjustStateHandler(golfBot: GolfBotMemory):
    # ToDo add nav to readjustment
    return None

def approachWhiteCoordinateStateHandler(golfBot: GolfBotMemory):
    #ToDO: Nav to ball
    nav_to_ball(golfBot.currentBall)
    return None

def approachNewQuadrantStateHandler(golfBot: GolfBotMemory):
    next_quadrant = golfBot.quadrant+1
    if golfBot.quadrant == 5:
        next_quadrant = 1
    #ToDo: Determine navigation point in next quadrant
    return None

def approachNarrowGoalStateHandler(golfBot: GolfBotMemory):
    #Find point of goal
    #ToDo: fix botholeToCenter to correct distance
    botholeToCenter= 20
    y = court_settings.court_height/2
    x = court_settings.court_width-botholeToCenter-5
    #ToDo: Nav to x,y

    return None

def avoidObstacleStateHandler(golfBot: GolfBotMemory):
    #ToDo add obstacle detecter somehow from nav
    return None

def submitBallsStateHandler(golfBot: GolfBotMemory):
    #ToDo run Open bothole
    #ToDo run unstuck function
    #ToDo consider rechecking for balls in transitiion from SubmitBalls
    return None

# Transitions
def botIsGone(golfBot: GolfBotMemory):
    if find_bot(golfBot.arena)[0] is None:
        return True
    return False

def orangeDetectedTransitionHandler(golfBot: GolfBotMemory):
    orangeBalls = golfBot.orangeBalls
    if len(orangeBalls) > 0:
        return True
    else:
        return False

def orangeInCornerTransitionHandler(golfBot: GolfBotMemory):
    if golfBot.orangeBalls[1] == golfBot.currentBall:
        return IsInCorner(golfBot.currentBall[0], golfBot.currentBall[1])
    return False

def orangeCollectedTransitionHandler(golfBot: GolfBotMemory):
    return False

def whiteDetectedTransitionHandler(golfBot: GolfBotMemory):
    whiteBalls = golfBot.whiteBalls
    if len(whiteBalls) > 0:
        return False
    else:
        return False

def whiteInCornerTransitionHandler(golfBot: GolfBotMemory):
    if golfBot.currentBall in golfBot.whiteBalls:
        return IsInCorner(golfBot.currentBall[0], golfBot.currentBall[1])
    return False

def whiteInQuadrantTransitionHandler(golfBot: GolfBotMemory):
    for ball in golfBot.whiteBalls:
        if isInQuadrant(ball[0], ball[1], golfBot):
            return True
    return False

def inNewQuadrantTransitionHandler(golfBot: GolfBotMemory):
    if currentQuadrant(golfBot) != golfBot.quadrant:
        golfBot.quadrant = currentQuadrant(golfBot)
        return True
    return False

def doneReadjustingTransitionHandler(golfBot: GolfBotMemory):
    return False

def noneDetectedTransitionHandler(golfBot: GolfBotMemory):
    if orangeDetectedTransitionHandler(golfBot) or whiteDetectedTransitionHandler(golfBot):
        return False
    else:
        return True

def isInQuadrant(x,y,golfBot: GolfBotMemory):
    if x <= golfBot.cross[0] and y < golfBot.cross[1] and golfBot.quadrant == 1:
        return True
    elif x > golfBot.cross[0] and y <= golfBot.cross[1] and golfBot.quadrant == 1:
        return True
    elif x >= golfBot.cross[0] and y >= golfBot.cross[1] and golfBot.quadrant == 1:
        return True
    elif x <= golfBot.cross[0] and y >= golfBot.cross[1] and golfBot.quadrant == 1:
        return True

def IsInCorner(x,y):
    return xCoordinateinCorner(x) and yCoordinateinCorner(y)

def xCoordinateinCorner(x):
    return x<15 or x>court_settings.court_width-15

def yCoordinateinCorner(y):
    return y<15 or y>court_settings.court_height-15

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

def createRobotFSM():
    detectState = State()
    detectState.setHandler(detectStateHandler)

    collectOrangeState = State()
    collectOrangeState.setHandler(collectOrangeStateHandler)

    checkQuadrantState = State()
    checkQuadrantState.setHandler(checkQuadrantStateHandler)

    findNearestState = State()
    findNearestState.setHandler(findNearestStateHandler)

    approachCoordinateInCornerState = State()
    approachCoordinateInCornerState.setHandler(approachCoordinateInCornerStateHandler)

    readjustState = State()
    readjustState.setHandler(readjustStateHandler)

    approachWhiteCoordinateState = State()
    approachWhiteCoordinateState.setHandler(approachWhiteCoordinateStateHandler)

    approachNewQuadrantState = State()
    approachNewQuadrantState.setHandler(approachNewQuadrantStateHandler)

    approachNarrowGoalState = State()
    approachNarrowGoalState.setHandler(approachNarrowGoalStateHandler)

    avoidObstacleGoalState = State()
    avoidObstacleGoalState.setHandler(avoidObstacleStateHandler)

    submitBallsState = State()
    submitBallsState.setHandler(submitBallsStateHandler)