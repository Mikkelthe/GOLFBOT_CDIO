from .state import *
from Object_Tracking.Object_Tracking import find_objects_in_image

# States
def detectStateHandler(golfBot: GolfBot):
    return None

def collectOrangeStateHandler(golfBot: GolfBot):
    return None

def approachOrangeInCornerStateHandler(golfBot: GolfBot):
    return None

def checkQuadrantStateHandler(golfBot: GolfBot):
    return None

def findNearestStateHandler(golfBot: GolfBot):
    return None

def approachCoordinateInCornerStateHandler(golfBot: GolfBot):
    return None

def readjustStateHandler(golfBot: GolfBot):
    return None

def approachWhiteCoordinateStateHandler(golfBot: GolfBot):
    return None

def approachNewQuadrantStateHandler(golfBot: GolfBot):
    return None

def approachNarrowGoalStateHandler(golfBot: GolfBot):
    return None

def avoidObstacleStateHandler(golfBot: GolfBot):
    return None

def submitBallsStateHandler(golfBot: GolfBot):
    return None

# Transitions
def orangeDetectedTransitionHandler(golfBot: GolfBot):
    return False

def orangeInCornerTransitionHandler(golfBot: GolfBot):
    return False

def orangeCollectedTransitionHandler(golfBot: GolfBot):
    return False

def whiteDetectedTransitionHandler(golfBot: GolfBot):
    return False

def whiteInCornerTransitionHandler(golfBot: GolfBot):
    return False

def OrangeCollectedTransitionHandler(golfBot: GolfBot):
    return False

def doneReadjustingTransitionHandler(golfBot: GolfBot):
    return False

def OrangeCollectedTransitionHandler(golfBot: GolfBot):
    return False

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